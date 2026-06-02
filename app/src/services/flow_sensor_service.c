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

/* Flow processing context - owns filtering that used to be inside the driver */
static struct flow_processor_ctx flow_ctx;

/* Authoritative flow stabilization contract (single source of truth for
 * original plateau rules) */
static struct flow_contract_state flow_contract;

/* Cache of authoritative pump state from pump_status_chan. Safe default false
 * selects INITIAL_K. Listener callback updates ONLY the cache (per design:
 * minimal, no other side effects here).
 */
static bool current_pump_on_cache = false;

static void on_pump_status_for_contract(const struct zbus_channel *chan) {
  ARG_UNUSED(chan);
  struct pump_status st;
  if (zbus_chan_read(&pump_status_chan, &st, K_NO_WAIT) != 0) {
    return;
  }
  current_pump_on_cache = st.is_on;
}

ZBUS_LISTENER_DEFINE(flow_pump_status_listener, on_pump_status_for_contract);

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

  static uint32_t seq = 0;
  int64_t last_hb = 0;

  while (1) {
    if (k_sem_take(&flow_data_sem, K_MSEC(1000)) == 0) {
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
            current_pump_on_cache); /* authoritative from pump_status sub ->
                                       correct K and Phase B */

        bool plateau = p.plateau_confirmed;

        if (p.analyzer_was_reset) {
          LOG_DBG("[FLOW] Analyzer reset performed by contract");
        }

        if (p.watchdog_timeout) {
          /* Per design: publish to lightweight flow_event_chan (not to sample).
           * PumpService (and others) will listen; no control bits on
           * measurement samples.
           */
          struct flow_event ev = {
              .type = FLOW_EVENT_WATCHDOG_TIMEOUT,
              .timestamp = k_uptime_get(),
              .associated_period_us = filtered_period,
          };
          (void)zbus_chan_pub(&flow_event_chan, &ev, K_NO_WAIT);
          LOG_INF("[FLOW] watchdog timeout published to flow_event_chan");
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
      }
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
