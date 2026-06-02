/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * FlowSensorService — owns the YF-S201C driver, performs filtering + plateau
 * analysis, and publishes flow samples on the central zbus channel.
 *
 * This is the canonical producer for all flow data in the new architecture.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/sys/atomic.h>

#include <app/drivers/yf_s201c.h>

#include "fixed_math.h"
#include "flow_processor.h"
#include "pump/flow_stabilization_contract.h"
#include "services/flow_sensor_service.h"
#include "zbus/channels.h"

LOG_MODULE_REGISTER(flow_sensor_service, CONFIG_APP_LOG_LEVEL);

/* Stack sized for sensor work + occasional zbus pub */
#define FLOW_SENSOR_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(flow_sensor_stack, FLOW_SENSOR_STACK_SIZE);
static struct k_thread flow_sensor_thread_cb;

/* Semaphore used by the YF-S201C driver to signal new pulse data (ISR-safe) */
K_SEM_DEFINE(flow_data_sem, 0, 1);

/* Separate semaphore used to wake flow thread from wd_work (for contract gap check in flow thread context only) */
K_SEM_DEFINE(wd_wake_sem, 0, 1);

/* Flow processing context - owns filtering that used to be inside the driver */
static struct flow_processor_ctx flow_ctx;

/* Authoritative flow stabilization contract (single source of truth for
 * original plateau rules) */
static struct flow_contract_state flow_contract;

/* Monotonic sequence for published samples. File-static (was local static
 * inside thread; moved per review for idiomatic scope, still inits once). */
static uint32_t seq = 0;

/* Cache of authoritative pump state from pump_status_chan. Safe default false
 * selects INITIAL_K. Listener callback updates ONLY the cache (per design:
 * minimal, no other side effects here). Uses Zephyr atomic_t for cross-thread
 * safety (written by PumpService's zbus cb thread, read by flow thread).
 * Matches project pattern in input_manager.c (no C11 stdatomic).
 */
static atomic_t current_pump_on_cache = ATOMIC_INIT(0);

/* Watchdog timer + work for 1.5x gap detection per design (PR2).
 * - k_timer one-shot armed to suggested (or 1.5x); expiry only posts work (ISR safe).
 * - k_work_delayable defers execution; handler does contract(now,0) + flow_event pub.
 * - All contract calls (pulse + gap) effectively serialized via flow thread or
 *   work (no concurrent when gaps); listener only touches cache + arm flag.
 * - Arm on good_period_captured (from pulse path) or on-pump-on transition (via flag
 *   checked in flow thread).
 * - Cancel on new pulse or pump off.
 * - last_suggested_us for tracking + on-trans arm fallback.
 */
static atomic_t last_suggested_us = ATOMIC_INIT(0); /* fits in 32-bit (capped at 1s); use atomic for cross workq/flow thread safety on 32-bit (esp32) */
static atomic_t arm_watchdog_pending = ATOMIC_INIT(0);
static struct k_work_delayable wd_work;

/* Forward decls so K_TIMER_DEFINE can reference */
static void wd_timer_expiry(struct k_timer *t);
static void wd_work_handler(struct k_work *w);

K_TIMER_DEFINE(wd_timer, wd_timer_expiry, NULL);

static void on_pump_status_for_contract(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);
  struct pump_status st;
  if (zbus_chan_read(&pump_status_chan, &st, K_NO_WAIT) != 0) {
    return;
  }
  bool was_on = atomic_get(&current_pump_on_cache) != 0;
  bool is_on = st.is_on;
  atomic_set(&current_pump_on_cache, is_on ? 1 : 0);

  if (!was_on && is_on) {
    /* Signal flow thread (owns arm/cancel per design) to arm using last suggested
     * (or fallback). Listener remains minimal (only cache + flag). */
    atomic_set(&arm_watchdog_pending, 1);
    /* No sem give: 1s take timeout in flow thread is sufficient for arming latency */
  } else if (was_on && !is_on) {
    k_timer_stop(&wd_timer);
    (void)k_work_cancel_delayable(&wd_work);
  }
  /* best-effort (K_NO_WAIT + zbus listener cb serialization assumed);
   * see Issue 6 in review for PR2+ queueing if races on high-rate events.
   * cache read in flow thread for contract K selection. */
}

ZBUS_LISTENER_DEFINE(flow_pump_status_listener, on_pump_status_for_contract);

/* k_timer expiry: ONLY schedule the work (never mutate contract or pub from here) */
static void wd_timer_expiry(struct k_timer *t) {
  ARG_UNUSED(t);
  k_work_schedule(&wd_work, K_NO_WAIT);
}

/* Work handler: runs contract with period=0 to let gap logic fire exactly at 1.5x.
 * Pubs lightweight flow_event (not sample) on timeout. In flow/work context.
 * Per design: separate from flow_sample per user decision on Open Q1. */
static void wd_work_handler(struct k_work *w) {
  ARG_UNUSED(w);
  /* Signal the flow thread (owner of contract) to perform the gap check.
   * This ensures *all* contract mutations happen in flow thread context (serial
   * with pulse path), fixing races on contract state and last_pulse_time.
   * k_work is still used (per design) only to defer from timer IRQ to schedulable ctx. */
  k_sem_give(&wd_wake_sem);
}

/* Arm one-shot k_timer for 1.5x suggested (from contract on good capture or trans).
 * Update last_suggested for logging + fallback arm. Cap at MAX per contract. */
static void arm_watchdog(int64_t timeout_us) {
  if (timeout_us <= 0) {
    timeout_us = 1000000LL; /* fallback 1s */
  }
  if (timeout_us > 1000000LL) {
    timeout_us = 1000000LL;
  }
  atomic_set(&last_suggested_us, (atomic_val_t)timeout_us);
  k_timer_start(&wd_timer, K_USEC(timeout_us), K_NO_WAIT);
  LOG_DBG("[FLOW] armed 1.5x watchdog timer for %lld us", timeout_us);
}

/* Cancel any armed watchdog (on new pulse or off transition). */
static void cancel_watchdog(void) {
  k_timer_stop(&wd_timer);
  (void)k_work_cancel_delayable(&wd_work);
  k_sem_reset(&wd_wake_sem); /* discard any pending wd signal from work */
  LOG_DBG("[FLOW] canceled watchdog timer");
}

/* Helper to dedup the flow_event pub + log for WATCHDOG (used from pulse path
 * when contract detects on late sample, and from wd gap path in flow thread).
 * Per review nit. */
static void publish_flow_watchdog_event(int64_t associated_period_us, const char *context) {
  struct flow_event ev = {
      .type = FLOW_EVENT_WATCHDOG_TIMEOUT,
      .timestamp = k_uptime_get(),
      .associated_period_us = associated_period_us,
  };
  (void)zbus_chan_pub(&flow_event_chan, &ev, K_NO_WAIT);
  LOG_INF("[FLOW] watchdog timeout%s published to flow_event_chan",
          context ? context : "");
}

static void flow_sensor_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  LOG_INF("=== FLOWSENSORSERVICE STARTED (zbus) ===");

  const struct device *flow_dev = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
  if (!device_is_ready(flow_dev)) {
    LOG_ERR("Flow sensor device not ready");
    return;
  }

  /* Tell the driver to signal us via semaphore on valid data */
  int ret = yf_s201c_set_data_semaphore(flow_dev, &flow_data_sem);
  if (ret < 0) {
    LOG_ERR("Failed to attach semaphore to flow sensor (%d)", ret);
    return;
  }

  LOG_INF("Flow sensor ready — publishing on flow_sample_chan");

  /* Initialize application-owned flow processor (replaces logic that was in the
   * driver) */
  flow_processor_init(&flow_ctx, 5);

  /* Initialize the pure flow stabilization contract (original master branch
   * rules + reset discipline) */
  flow_contract_init(&flow_contract);
  LOG_INF("Flow stabilization contract initialized (original plateau + 1.5x "
          "watchdog rules)");

  /* Subscribe (after contract init) to pump_status for authoritative
   * pump_is_on. This lets contract use correct K factor (INITIAL 2.0 /
   * NORMAL 3.0) and Phase B. cb updates only cache.
   */
  ret = zbus_chan_add_obs(&pump_status_chan, &flow_pump_status_listener,
                          K_NO_WAIT);
  if (ret < 0 && ret != -EALREADY) {
    LOG_WRN("Failed to attach pump status listener for contract (%d)", ret);
  }

  /* Bootstrap the pump cache immediately after attach (zbus has no replay of prior pubs).
   * PumpService may have pub'd initial status during its init (before this thread's add_obs).
   * This ensures authoritative current_pump_on for contract K selection even if attach races
   * with early pubs (callers use short sleep but we make robust here). */
  struct pump_status st0;
  if (zbus_chan_read(&pump_status_chan, &st0, K_NO_WAIT) == 0) {
    atomic_set(&current_pump_on_cache, st0.is_on ? 1 : 0);
  }

  /* Init work for watchdog (timer already defined with K_TIMER_DEFINE) */
  k_work_init_delayable(&wd_work, wd_work_handler);

  int64_t last_hb = 0;

  while (1) {
    /* Handle wd gap signal first (from k_work, which was posted by timer expiry).
     * This performs the contract(0) *in flow thread context only*, serializing
     * all contract state mutations with the pulse path (fixes concurrent access
     * to analyzer/last_pulse_time etc, and post-cancel pollution of gap base time).
     * Separate sem ensures wd wake doesn't masquerade as pulse data. */
    if (k_sem_take(&wd_wake_sem, K_NO_WAIT) == 0) {
      double now_s = (double)k_uptime_get() / 1000.0;
      bool pump_on = atomic_get(&current_pump_on_cache) != 0;
      struct flow_contract_result p = flow_contract_process_sample(
          &flow_contract, 0, 0, now_s, pump_on);
      if (p.watchdog_timeout) {
        publish_flow_watchdog_event((int64_t)atomic_get(&last_suggested_us), " (via k_work signal to flow thread)");
        k_timer_stop(&wd_timer);
        /* Reset yf buffer (original timeout path) so subsequent get_recent in pulse path
         * doesn't re-feed stale last good period and spuriously re-arm or re-detect. */
        (void)yf_s201c_reset(flow_dev);
      }
    }

    if (k_sem_take(&flow_data_sem, K_MSEC(1000)) == 0) {
      /* New pulse: cancel any armed watchdog (design: cancel on new pulse) + clear wd signal */
      cancel_watchdog();

      /* Authoritative path: thin driver gives raw recent periods.
       * FlowSensorService owns all processing via flow_processor.
       */
      int64_t recent[8];
      int n = yf_s201c_get_recent_periods_us(flow_dev, recent, 8);

      if (n > 0) {
        int64_t filtered_period = recent[0];

        /* Feed every new raw period through the authoritative app-layer filter
         */
        for (int i = 0; i < n; i++) {
          if (recent[i] > 0) {
            filtered_period =
                flow_processor_filter_period(&flow_ctx, recent[i]);
          }
        }

        /* Service owns rate calc + validity policy */
        fixed_t rate = flow_processor_calculate_flow_rate(filtered_period, 450);
        bool valid = (filtered_period > 0) && (n >= 1);

        /* Use the pure flow stabilization contract (original master rules).
         * This gives us: phase-aware K (starting with more sensitive
         * INITIAL_K), explicit reset after plateau, first-sample false,
         * calibration requirement, and (later) 1.5x watchdog awareness.
         */
        double now_s = (double)k_uptime_get() / 1000.0;
        struct flow_contract_result p = flow_contract_process_sample(
            &flow_contract, rate, filtered_period, now_s,
            atomic_get(&current_pump_on_cache) != 0); /* authoritative from pump_status sub (atomic) ->
                                       correct K and Phase B */

        bool plateau = p.plateau_confirmed;

        if (p.analyzer_was_reset) {
          LOG_DBG("[FLOW] Analyzer reset performed by contract");
        }

        /* Track suggested for arming/logging (set on both gap and plateau paths in contract) */
        if (p.suggested_next_timeout_us > 0) {
          atomic_set(&last_suggested_us, (atomic_val_t)p.suggested_next_timeout_us);
        }

        if (p.watchdog_timeout) {
          /* Per design: publish to lightweight flow_event_chan (not to sample).
           * PumpService (and others) will listen; no control bits on
           * measurement samples.
           * Uses K_NO_WAIT best-effort (consistent with all other pubs/listeners
           * in services; zbus cb provides serialization but no full queuing).
           */
          publish_flow_watchdog_event(filtered_period, " (pulse path)");
        }

        struct flow_sample sample = {
            .rate = rate,
            .timestamp = k_uptime_get(),
            .sequence = seq++,
            .valid = valid,
            .plateau_detected = plateau,
            .period_us = filtered_period,
        };

        ret = zbus_chan_pub(&flow_sample_chan, &sample, K_NO_WAIT);
        if (ret == 0) {
          LOG_INF("[FLOW] seq=%u rate=%.2f L/min (valid=%d)", sample.sequence,
                  (double)fixed_to_float(sample.rate), valid);
        } else if (ret != -EBUSY) {
          LOG_WRN("zbus pub failed (%d)", ret);
        }

        /* Arm (or re-arm) on good_period_captured (covers Phase A first plateau
         * too, since contract sets good + suggested even when passed pump_on=false
         * for the demand_event case). */
        if (p.good_period_captured && p.suggested_next_timeout_us > 0) {
          arm_watchdog(p.suggested_next_timeout_us);
        }
      }
    }

    /* Check arm request from pump on-transition (listener sets flag; flow thread
     * owns arm per design). Checked on every wake (up to 1s latency acceptable). */
    if (atomic_get(&arm_watchdog_pending) != 0) {
      atomic_set(&arm_watchdog_pending, 0);
      int64_t t = (atomic_get(&last_suggested_us) > 0) ? (int64_t)atomic_get(&last_suggested_us) : 1000000LL;
      arm_watchdog(t);
    }

    /* Heartbeat when idle */
    int64_t now = k_uptime_get();
    if (now - last_hb > 5000) {
      LOG_INF("[FLOW] Heartbeat — waiting for pulses...");
      last_hb = now;
    }
  }
}

int flow_sensor_service_start(void) {
  k_tid_t tid =
      k_thread_create(&flow_sensor_thread_cb, flow_sensor_stack,
                      K_THREAD_STACK_SIZEOF(flow_sensor_stack),
                      flow_sensor_thread, NULL, NULL, NULL, 3, 0, K_NO_WAIT);
  if (tid == NULL) {
    LOG_ERR("Failed to create FlowSensorService thread");
    return -EAGAIN;
  }

  k_thread_name_set(&flow_sensor_thread_cb, "flow_sensor");
  LOG_INF("FlowSensorService thread created (publishes flow_sample_chan)");
  return 0;
}
