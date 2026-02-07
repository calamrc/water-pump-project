/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/pump_controller.h>
#include <app/drivers/yf_s201c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <app_version.h>
#include <string.h>
#include <math.h>
#include "fixed_math.h"
#include "error_handler.h"
#include "flow_analyzer.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

// Semaphore for event-driven sensor data signaling
K_SEM_DEFINE(data_sem, 0, 1);



=======
>>>>>>> Stashed changes
/**
 * @brief Main application entry point for Zephyr Water Pump control system
 *
 * @return Application exit code (0 for normal operation, never returns in production)
 */
int main(void)
{
    int64_t initial_plateau_period = 0;
    int64_t latest_plateau_period = 0;
    int ret;

    LOG_INF("Zephyr Water Pump Application %s", APP_VERSION_STRING);

    // Initialize error handler first (critical for other modules)
    ret = error_handler_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize error handler (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_SYSTEM_CRITICAL_FAILURE);
    }

    // Get flow sensor device reference (driver initializes itself)
    const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
    if (!device_is_ready(flow_sensor)) {
        LOG_ERR("Flow sensor device not ready");
        return ERROR_REPORT_CRITICAL(ERROR_SENSOR_INIT_FAILED);
    }

    // Configure sensor for event-driven operation
    ret = yf_s201c_set_data_semaphore(flow_sensor, &data_sem);
    if (ret < 0) {
        LOG_ERR("Could not configure sensor semaphore (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_SENSOR_INIT_FAILED);
    }

    // Initialize flow analyzer
    ret = flow_analyzer_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize flow analyzer (%d)", ret);
        return ERROR_REPORT_CRITICAL(ERROR_FLOW_CALCULATION_ERROR);
    }

    // Get pump controller device reference (driver initializes itself)
    const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
    if (!PUMP_CONTROLLER_DT_CHECK(DT_NODELABEL(pump_controller))) {
        LOG_ERR("Pump controller device not ready");
        return ERROR_REPORT_CRITICAL(ERROR_PUMP_GPIO_FAILED);
    }

    while (1) {
        bool current_pump_on = pump_controller_is_on(pump);

        LOG_DBG("Waiting on semaphore (pump_on: %d)", current_pump_on);

        int64_t start_wait_ms = k_uptime_get();
        int64_t timeout_us = (!current_pump_on) ? -1LL : (latest_plateau_period * 1.5);
        k_timeout_t timeout = (!current_pump_on) ? K_FOREVER : K_USEC(MIN(timeout_us, MAX_TIMEOUT_US));

        if (k_sem_take(&data_sem, timeout) == 0) {
            int64_t end_wait_ms = k_uptime_get();

            LOG_DBG("Semaphore taken after %lld ms", end_wait_ms - start_wait_ms);

            fixed_t flow_rate_fixed;
            ret = yf_s201c_get_flow_rate(flow_sensor, &flow_rate_fixed);
            if (ret < 0) {
                LOG_ERR("Failed to get flow rate (%d)", ret);
                continue;
            }
            float flow_rate_lpm = fixed_to_float(flow_rate_fixed);

            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            if (yf_s201c_is_data_valid(flow_sensor)) {
                bool plateau_detected = flow_analyzer_detect_plateau(flow_rate_fixed, !pump_controller_is_on(pump) ? FIXED_PLATEAU_INITIAL_K_FACTOR : FIXED_PLATEAU_K_FACTOR);

                if (plateau_detected) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)",
                            flow_rate_lpm, fixed_to_float(flow_analyzer_get_noise_std()),
                            fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, flow_analyzer_get_noise_std())));

                    int64_t current_period;
                    ret = yf_s201c_get_current_period(flow_sensor, &current_period);
                    if (ret < 0) {
                        LOG_ERR("Failed to get current period (%d)", ret);
                        continue;
                    }

                    if (!(current_pump_on && current_period < initial_plateau_period)) {
                        latest_plateau_period = current_period;
                    }

                    pump_controller_update_plateau_period(pump, latest_plateau_period);
                    flow_analyzer_reset();

                    if (!pump_controller_is_on(pump) && current_period > 0) {
                        ret = pump_controller_turn_on(pump, latest_plateau_period);
                        if (ret < 0) {
                            LOG_ERR("Failed to turn on pump (%d)", ret);
                        } else {
                            initial_plateau_period = current_period;
                        }
                    }
                }
            }
        } else {
            int64_t end_wait_ms = k_uptime_get();

            LOG_DBG("Timeout after %lld ms, resetting flow state", end_wait_ms - start_wait_ms);

            yf_s201c_reset(flow_sensor);
            flow_analyzer_reset();

            if (pump_controller_is_on(pump)) {
                LOG_INF("Plateau period expired, turning off pump");
                ret = pump_controller_turn_off(pump);
                if (ret < 0) {
                    LOG_ERR("Failed to turn off pump on timeout (%d)", ret);
                } else {
                    initial_plateau_period = 0;
                    latest_plateau_period = 0;
                }
            }
        }
    }

    return 0;
}
