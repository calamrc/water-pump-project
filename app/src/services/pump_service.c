/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * PumpControlService
 *
 * Owns the pump_controller driver.
 * Subscribes to flow_sample_chan, pump_cmd_chan, flow_event_chan (for watchdog
 * etc), and timer_state_chan (maps timer_status to pure timer_pure_status for
 * demand; timer complete only ever forces off when active).
 */

#include <app/drivers/pump_controller.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "fixed_math.h"
#include "pump/pump_demand.h"
#include "pump_state_machine.h"
#include "services/pump_service.h"
#include "zbus/channels.h"

/* Forward declaration */
static void pump_handle_demand(const struct flow_sample *flow,
                               const struct timer_pure_status *timer);

/* Cached snapshots for cross-trigger re-evaluation (flow updates + timer pubs).
 * Allows timer_state_chan to drive demand using last known flow, and vice-versa.
 */
static struct flow_sample cached_flow;
static struct timer_pure_status cached_timer;

LOG_MODULE_REGISTER(pump_service, CONFIG_APP_LOG_LEVEL);

static enum pump_sm_state current_sm_state = PUMP_SM_STATE_OFF;
/* NOTE: current_sm_state mutated from multiple listener cbs (on_flow_sample,
 * on_pump_cmd, on_flow_event) which run in zbus observer contexts (potentially
 * different threads from pubs). Best-effort K_NO_WAIT + zbus serialization
 * assumed (consistent with safety_service etc). Documented per review Issue 6;
 * add k_mutex if races observed in PR2+.
 */
static bool pump_device_ready = false;
static const struct device *pump_dev;
static int64_t pump_run_start_time = 0; /* when we last entered RUNNING */
static bool last_driver_on_state = false;

/* Helper: publish current status on zbus when something meaningful changed */
static void publish_pump_status_if_changed(bool force) {
  if (!pump_device_ready) {
    return;
  }

  bool driver_on = pump_controller_is_on(pump_dev);
  int64_t runtime = 0;

  if (current_sm_state == PUMP_SM_STATE_RUNNING && pump_run_start_time > 0) {
    runtime = k_uptime_get() - pump_run_start_time;
  }

  bool state_changed = (driver_on != last_driver_on_state) || force;

  if (state_changed) {
    struct pump_status status = {
        .is_on = driver_on,
        .runtime_ms = (uint64_t)runtime,
        .state = (uint8_t)current_sm_state,
        .safety_flags = 0,
    };

    int ret = zbus_chan_pub(&pump_status_chan, &status, K_NO_WAIT);
    if (ret == 0) {
      LOG_INF("[PUMP] status -> on=%d runtime=%lld ms state=%s", driver_on,
              runtime, pump_sm_state_to_str(current_sm_state));
    }
    last_driver_on_state = driver_on;
  }
}

/* Listener: receives flow samples from FlowSensorService via zbus */
static void on_flow_sample(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);

  struct flow_sample sample;
  if (zbus_chan_read(&flow_sample_chan, &sample, K_NO_WAIT) != 0) {
    return;
  }

  if (!pump_device_ready || !sample.valid) {
    return;
  }

  cached_flow = sample;
  pump_handle_demand(&cached_flow, &cached_timer);
}

/* Demand handler: builds pure input (flow + mapped timer snapshot), runs
 * evaluate, processes SM event, drives hardware. Called from both flow and
 * timer listeners (using caches for the other). With the keeper PLATEAU
 * for active no-force cases, re-eval on every timer pub (incl. 1s hb) is
 * safe and does not force off (only complete timer does via SAFETY).
 */

static void pump_handle_demand(const struct flow_sample *flow,
                               const struct timer_pure_status *timer) {
  struct pump_demand_input in = {.flow = flow, .timer = timer};
  struct pump_demand_result res;

  pump_demand_evaluate(&in, current_sm_state, &res);

  enum pump_sm_state next =
      pump_sm_process_event(current_sm_state, res.recommended_event);

  if (next != current_sm_state) {
    LOG_INF("[PUMP SM] %s -> %s (reason=%s flow=%.2f)",
            pump_sm_state_to_str(current_sm_state), pump_sm_state_to_str(next),
            res.reason_str ? res.reason_str : "none", res.flow_rate_used);

    current_sm_state = next;

    /* Drive the real hardware (only if ready; sm_state is updated regardless
     * for consistency with demand decisions).
     */
    if (pump_device_ready && pump_dev) {
      if (pump_sm_is_active(next) && !pump_controller_is_on(pump_dev)) {
        int64_t use_period = (res.updated_plateau_period_us > 0)
                                 ? res.updated_plateau_period_us
                                 : 0;
        (void)pump_controller_turn_on(pump_dev, use_period);
        pump_run_start_time = k_uptime_get();
        LOG_INF("[PUMP] Driver turned ON");
      } else if (!pump_sm_is_active(next) && pump_controller_is_on(pump_dev)) {
        (void)pump_controller_turn_off(pump_dev);
        pump_run_start_time = 0;
        LOG_INF("[PUMP] Driver turned OFF");
      }
    }
  }

  /* Period refresh for 1.5x watchdog while RUNNING. Hoisted outside the
   * (next != current) block so it applies on "keeper" evals (PLATEAU from
   * active no-force cases, including timer heartbeats and flow samples
   * while running) that cause no state change. (turn_on path passes period
   * directly at on-transition.)
   */
  if (current_sm_state == PUMP_SM_STATE_RUNNING &&
      res.updated_plateau_period_us > 0 && pump_device_ready && pump_dev) {
    pump_controller_update_plateau_period(pump_dev,
                                          res.updated_plateau_period_us);
  }

  publish_pump_status_if_changed(false);
}

ZBUS_LISTENER_DEFINE(pump_flow_listener, on_flow_sample);

/* Listener stub for flow_event_chan (e.g. WATCHDOG_TIMEOUT from
 * FlowSensorService contract). In PR1 we attach and receive; actual SM drive
 * (PUMP_SM_EVENT_TIMEOUT) + turn_off in later PR. This sets up the seam for
 * correct watchdog without polluting flow_sample.
 * NOTE: uses K_NO_WAIT inside zbus listener cb (triggered by pub from other
 * thread). Best-effort per existing patterns (safety on_flow_sample etc.);
 * zbus provides some cb serialization but concurrent pubs (flow+cmd+event)
 * could race on current_sm_state without further protection (see review
 * Issue 6; for PR2+ consider mutex or dedicated work/queue if observed).
 */
static void on_flow_event(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);
  struct flow_event ev;
  if (zbus_chan_read(&flow_event_chan, &ev, K_NO_WAIT) != 0) {
    return;
  }
  if (ev.type == FLOW_EVENT_WATCHDOG_TIMEOUT) {
    /* Stub: log for observability in PR1. Real action (timeout event to SM)
     * deferred. */
    LOG_INF(
        "[PUMP] received FLOW_EVENT_WATCHDOG_TIMEOUT (action in follow-up)");
  }
}

ZBUS_LISTENER_DEFINE(pump_flow_event_listener, on_flow_event);

/* Listener for timer_state_chan. Maps zbus timer_status to pure
 * timer_pure_status and re-evals demand (using cached flow if available,
 * or NULL flow for pure timer-off path). Timer complete only turns off
 * (if active); never on.
 */
static void on_timer_state(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);
  if (!pump_device_ready) {
    return;
  }

  struct timer_status ts;
  if (zbus_chan_read(&timer_state_chan, &ts, K_NO_WAIT) != 0) {
    return;
  }

  cached_timer = (struct timer_pure_status){
      .completed = (ts.state == TIMER_STATE_COMPLETED),
      .remaining_sec = ts.remaining_sec,
  };

  /* Re-eval on timer pub (periodic or state change). Use cached flow for
   * context (plateau/period) or allow NULL flow for timer-only force-off.
   * Safe for heartbeats because demand emits keeper (PLATEAU) for active
   * states when no timer complete.
   */
  if (cached_flow.valid) {
    pump_handle_demand(&cached_flow, &cached_timer);
  } else {
    pump_handle_demand(NULL, &cached_timer);
  }
}

ZBUS_LISTENER_DEFINE(pump_timer_listener, on_timer_state);

/* Listener for external pump commands (from UI, Safety, etc.) */
static void on_pump_cmd(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);

  if (!pump_device_ready || !pump_dev) {
    return;
  }

  struct pump_cmd cmd;
  if (zbus_chan_read(&pump_cmd_chan, &cmd, K_NO_WAIT) != 0) {
    return;
  }

  switch (cmd.action) {
  case PUMP_CMD_OFF:
    current_sm_state =
        pump_sm_process_event(current_sm_state, PUMP_SM_EVENT_RESET);
    (void)pump_controller_turn_off(pump_dev);
    pump_run_start_time = 0;
    LOG_INF("[PUMP] External OFF command received");
    break;

  case PUMP_CMD_ON:
    current_sm_state =
        pump_sm_process_event(current_sm_state, PUMP_SM_EVENT_PLATEAU_DETECTED);
    if (pump_sm_is_active(current_sm_state) &&
        !pump_controller_is_on(pump_dev)) {
      (void)pump_controller_turn_on(pump_dev, cmd.value);
      pump_run_start_time = k_uptime_get();
    }
    LOG_INF("[PUMP] External ON command received");
    break;

  case PUMP_CMD_EMERGENCY:
    current_sm_state = PUMP_SM_STATE_ERROR;
    (void)pump_controller_emergency_stop(pump_dev);
    pump_run_start_time = 0;
    LOG_WRN("[PUMP] EMERGENCY command received");
    break;

  default:
    break;
  }

  publish_pump_status_if_changed(true);
}

ZBUS_LISTENER_DEFINE(pump_cmd_listener, on_pump_cmd);

static K_THREAD_STACK_DEFINE(pump_stack, 3072);
static struct k_thread pump_thread_cb;

static void pump_thread(void *a1, void *a2, void *a3) {
  ARG_UNUSED(a1);
  ARG_UNUSED(a2);
  ARG_UNUSED(a3);

  LOG_INF("PumpService started — using pure pump_state_machine + real driver");

  pump_dev = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
  if (device_is_ready(pump_dev)) {
    pump_device_ready = true;
    LOG_INF("Pump controller device ready");
  } else {
    LOG_WRN("Pump controller device not ready");
  }

  /* Dynamically attach listeners */
  int ret =
      zbus_chan_add_obs(&flow_sample_chan, &pump_flow_listener, K_NO_WAIT);
  if (ret < 0 && ret != -EALREADY) {
    LOG_WRN("Failed to attach flow listener (%d)", ret);
  }

  ret = zbus_chan_add_obs(&pump_cmd_chan, &pump_cmd_listener, K_NO_WAIT);
  if (ret < 0 && ret != -EALREADY) {
    LOG_WRN("Failed to attach pump_cmd listener (%d)", ret);
  }

  ret =
      zbus_chan_add_obs(&flow_event_chan, &pump_flow_event_listener, K_NO_WAIT);
  if (ret < 0 && ret != -EALREADY) {
    LOG_WRN("Failed to attach flow_event listener (%d)", ret);
  }

  ret =
      zbus_chan_add_obs(&timer_state_chan, &pump_timer_listener, K_NO_WAIT);
  if (ret < 0 && ret != -EALREADY) {
    LOG_WRN("Failed to attach timer listener (%d)", ret);
  }

  /* Initial status */
  publish_pump_status_if_changed(true);

  while (1) {
    k_sleep(K_SECONDS(10));
  }
}

int pump_service_start(void) {
  k_tid_t tid = k_thread_create(&pump_thread_cb, pump_stack,
                                K_THREAD_STACK_SIZEOF(pump_stack), pump_thread,
                                NULL, NULL, NULL, 4, 0, K_NO_WAIT);
  if (!tid) {
    return -EAGAIN;
  }
  k_thread_name_set(&pump_thread_cb, "pump");
  LOG_INF("PumpService thread created (Phase 2: SM + driver control active)");
  return 0;
}
