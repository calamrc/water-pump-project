/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/pump_controller.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <app_version.h>
#include "error_handler.h"
#include "thread_manager.h"
#include "ui_manager.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

// Semaphore for event-driven sensor data signaling is now defined in thread_manager.c

/**
 * @brief Main application entry point for Zephyr Water Pump control system
 *
 * @return Application exit code (0 for normal operation, never returns in production)
 */
int main(void)
{
    int ret;

    LOG_INF("Zephyr Water Pump Application %s", APP_VERSION_STRING);

    // Initialize error handler first (critical for other modules)
    ret = error_handler_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize error handler (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_SYSTEM_CRITICAL_FAILURE);
    }

    // Initialize UI manager (display and input)
    ret = ui_manager_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize UI manager (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_SYSTEM_CRITICAL_FAILURE);
    }

    // Get pump controller device reference (driver initializes itself)
    const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
    if (!device_is_ready(pump)) {
        LOG_ERR("Pump controller device not ready");
        return ERROR_REPORT_CRITICAL(ERROR_PUMP_GPIO_FAILED);
    }

    // Create and start all threads
    ret = thread_manager_create_all_threads();
    if (ret < 0) {
        LOG_ERR("Failed to create threads (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_SYSTEM_CRITICAL_FAILURE);
    }

    // Monitor system health (this function never returns)
    thread_manager_monitor_health();

    return 0;
}
