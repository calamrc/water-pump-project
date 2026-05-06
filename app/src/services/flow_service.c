/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <app/drivers/yf_s201c.h>
#include "fixed_math.h"
#include "flow_analyzer.h"
#include "services/flow_service.h"

LOG_MODULE_REGISTER(flow_service, CONFIG_APP_LOG_LEVEL);

/* Channel definition (only one place in the project) */
ZBUS_CHAN_DEFINE_WITH_ID(flow_data_chan,
    0x10000001,
    struct flow_sample,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

/* Thread stack and control block */
#define FLOW_SERVICE_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(flow_service_stack, FLOW_SERVICE_STACK_SIZE);
static struct k_thread flow_service_thread_cb;

/* Semaphore for ISR-to-thread signaling (shared with YF-S201C driver) */
extern struct k_sem data_sem;

/**
 * @brief Flow Service thread entry point
 */
static void flow_service_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("=== FLOW SERVICE STARTED ===");

    /* Get flow sensor device */
    const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
    if (!device_is_ready(flow_sensor)) {
        LOG_ERR("Flow sensor device not ready");
        return;
    }

    LOG_INF("Flow sensor device ready");

    /* Configure sensor for event-driven operation */
    int ret = yf_s201c_set_data_semaphore(flow_sensor, &data_sem);
    if (ret < 0) {
        LOG_ERR("Could not configure sensor semaphore (%d)", ret);
        return;
    }

    LOG_INF("Sensor semaphore configured");

    static uint32_t sequence_counter = 0;
    LOG_INF("Flow Service entering main loop - waiting for flow pulses...");

    while (1) {
        /* Wait for sensor data */
        if (k_sem_take(&data_sem, K_MSEC(1000)) == 0) {
            fixed_t flow_rate;
            ret = yf_s201c_get_flow_rate(flow_sensor, &flow_rate);

            if (ret == 0 && yf_s201c_is_data_valid(flow_sensor)) {
                /* Run plateau detection */
                bool plateau = flow_analyzer_detect_plateau(flow_rate, FIXED_PLATEAU_K_FACTOR);

                /* Publish flow sample via zbus */
                struct flow_sample sample = {
                    .rate = flow_rate,
                    .timestamp = k_uptime_get(),
                    .sequence = sequence_counter++
                };

                ret = zbus_chan_pub(&flow_data_chan, &sample, K_NO_WAIT);
                if (ret == 0) {
                    LOG_INF("[FLOW] seq=%u rate=%.2f L/min plateau=%s",
                            sample.sequence,
                            (double)fixed_to_float(flow_rate),
                            plateau ? "YES" : "NO");
                } else {
                    LOG_WRN("Failed to publish flow data (%d)", ret);
                }
        } else {
            LOG_DBG("Invalid or no flow data");
        }

        /* Heartbeat every 5 seconds when no flow */
        static int64_t last_heartbeat = 0;
        int64_t now = k_uptime_get();
        if (now - last_heartbeat > 5000) {
            LOG_INF("[FLOW] Heartbeat - waiting for pulses...");
            last_heartbeat = now;
        }
    }
}
}

/**
 * @brief Initialize and start the Flow Service
 */
int flow_service_start(void)
{
    k_tid_t tid = k_thread_create(&flow_service_thread_cb,
                                  flow_service_stack,
                                  K_THREAD_STACK_SIZEOF(flow_service_stack),
                                  flow_service_thread,
                                  NULL, NULL, NULL,
                                  3, 0, K_NO_WAIT);

    if (tid == NULL) {
        LOG_ERR("Failed to create Flow Service thread");
        return -EAGAIN;
    }

    k_thread_name_set(&flow_service_thread_cb, "flow_service");
    LOG_INF("Flow Service thread created");

    return 0;
}
