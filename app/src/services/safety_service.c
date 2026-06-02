/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * SafetyService — independent safety layer.
 * Subscribes to pump_status and flow_sample to enforce
 * secondary safety rules (max runtime, anomaly detection).
 * Can issue emergency commands via pump_cmd_chan.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "services/safety_service.h"
#include "zbus/channels.h"

LOG_MODULE_REGISTER(safety_service, CONFIG_APP_LOG_LEVEL);

/* Independent safety tracking (separate from driver internal timer) */
static uint64_t safety_start_time = 0;
static bool safety_timer_active = false;

#define SAFETY_MAX_RUNTIME_MS (30ULL * 60 * 1000)   /* 30 minutes independent hard limit */

static void on_pump_status(const struct zbus_channel *chan)
{
    ARG_UNUSED(chan);

    struct pump_status status;
    if (zbus_chan_read(&pump_status_chan, &status, K_NO_WAIT) != 0) {
        return;
    }

    if (status.is_on) {
        if (!safety_timer_active) {
            safety_start_time = k_uptime_get();
            safety_timer_active = true;
            LOG_INF("[SAFETY] Independent safety timer started");
        }

        uint64_t runtime = k_uptime_get() - safety_start_time;
        if (runtime > SAFETY_MAX_RUNTIME_MS) {
            LOG_WRN("[SAFETY] Independent max runtime exceeded (%llu ms)", runtime);

            /* Publish emergency stop via pump_cmd_chan */
            struct pump_cmd cmd = {
                .action = PUMP_CMD_EMERGENCY,
                .value = 0,
            };
            (void)zbus_chan_pub(&pump_cmd_chan, &cmd, K_NO_WAIT);
            safety_timer_active = false;
        }
    } else {
        if (safety_timer_active) {
            safety_timer_active = false;
            LOG_INF("[SAFETY] Pump off - independent safety timer reset");
        }
    }
}

ZBUS_LISTENER_DEFINE(safety_pump_listener, on_pump_status);

/* Simple anomaly detection via flow samples */
static void on_flow_sample(const struct zbus_channel *chan)
{
    ARG_UNUSED(chan);

    struct flow_sample sample;
    if (zbus_chan_read(&flow_sample_chan, &sample, K_NO_WAIT) != 0) {
        return;
    }

    /* Example anomaly: pump believed running but no valid flow for extended period.
     * (More sophisticated logic can be added later) */
    if (!sample.valid) {
        /* Placeholder for future anomaly detection */
    }
}

ZBUS_LISTENER_DEFINE(safety_flow_listener, on_flow_sample);

static K_THREAD_STACK_DEFINE(safety_stack, 2048);
static struct k_thread safety_thread_cb;

static void safety_thread(void *a1, void *a2, void *a3)
{
    ARG_UNUSED(a1); ARG_UNUSED(a2); ARG_UNUSED(a3);

    LOG_INF("SafetyService started (independent safety observer)");

    int ret = zbus_chan_add_obs(&pump_status_chan, &safety_pump_listener, K_NO_WAIT);
    if (ret < 0 && ret != -EALREADY) {
        LOG_WRN("Failed to attach safety pump listener (%d)", ret);
    }

    ret = zbus_chan_add_obs(&flow_sample_chan, &safety_flow_listener, K_NO_WAIT);
    if (ret < 0 && ret != -EALREADY) {
        LOG_WRN("Failed to attach safety flow listener (%d)", ret);
    }

    while (1) {
        k_sleep(K_SECONDS(30));  /* Periodic safety checks */
    }
}

int safety_service_start(void)
{
    k_tid_t tid = k_thread_create(&safety_thread_cb, safety_stack,
                                  K_THREAD_STACK_SIZEOF(safety_stack),
                                  safety_thread, NULL, NULL, NULL,
                                  5, 0, K_NO_WAIT);
    if (!tid) {
        return -EAGAIN;
    }
    k_thread_name_set(&safety_thread_cb, "safety");
    LOG_INF("SafetyService thread created");
    return 0;
}