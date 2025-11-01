/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <app_version.h>
#include <string.h>
#include <math.h>
#include "fixed_math.h"
#include "sensor_manager.h"
#include "flow_analyzer.h"
#include "pump_controller.h"
#include "error_handler.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define FLOW_THRESHOLD_L_PER_MIN 0.1f // Minimum flow rate to consider active
#define PUMP_ON_DEBOUNCE_MS 3000 // Delay in ms before allowing pump to turn on

#define PLATEAU_CONFIRM_COUNT 2 // Consecutive small differences to confirm plateau
#define PLATEAU_WINDOW_SIZE 5 // Sliding window for calibration
#define PLATEAU_MIN_SLOPE 0.01f // Minimum slope to assume linear phase (L/min per sample)
#define PLATEAU_K_FACTOR 3.0f // Threshold multiplier (3-sigma for Gaussian noise)
#define PLATEAU_INITIAL_K_FACTOR 2.0f // Threshold multiplier (1-sigma for Gaussian noise)

#define PUMP_SAFETY_TIMEOUT_MIN 5 // Max pump runtime in minutes
#define MAX_TIMEOUT_US 1000000LL // 1 second cap for timeout

K_SEM_DEFINE(data_sem, 0, 1);

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

    ret = error_handler_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize error handler (%d)", ret);
        return 0;
    }

    ret = sensor_manager_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize sensor manager (%d)", ret);
        return 0;
    }

    ret = flow_analyzer_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize flow analyzer (%d)", ret);
        return 0;
    }

    ret = pump_controller_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize pump controller (%d)", ret);
        return 0;
    }

    while (1) {
        bool current_pump_on = pump_controller_is_on();

        LOG_DBG("Waiting on semaphore (pump_on: %d)", current_pump_on);

        int64_t start_wait_ms = k_uptime_get();
        int64_t timeout_us = (!current_pump_on) ? -1LL : (latest_plateau_period * 1.5);
        k_timeout_t timeout = (!current_pump_on) ? K_FOREVER : K_USEC(MIN(timeout_us, MAX_TIMEOUT_US));

        if (k_sem_take(&data_sem, timeout) == 0) {
            int64_t end_wait_ms = k_uptime_get();

            LOG_DBG("Semaphore taken after %lld ms", end_wait_ms - start_wait_ms);

            fixed_t flow_rate_fixed = sensor_manager_get_flow_rate();
            float flow_rate_lpm = fixed_to_float(flow_rate_fixed);

            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            if (sensor_manager_is_data_valid()) {
                bool plateau_detected = flow_analyzer_detect_plateau(flow_rate_fixed, !pump_controller_is_on() ? FIXED_PLATEAU_INITIAL_K_FACTOR : FIXED_PLATEAU_K_FACTOR);

                if (plateau_detected) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)",
                            flow_rate_lpm, fixed_to_float(flow_analyzer_get_noise_std()),
                            fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, flow_analyzer_get_noise_std())));

                    int64_t current_period = sensor_manager_get_current_period();

                    if (!(current_pump_on && current_period < initial_plateau_period)) {
                        latest_plateau_period = current_period;
                    }

                    pump_controller_update_plateau_period(latest_plateau_period);
                    flow_analyzer_reset();

                    if (!pump_controller_is_on() && current_period > 0) {
                        ret = pump_controller_turn_on(latest_plateau_period);
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

            sensor_manager_reset();
            flow_analyzer_reset();

            if (pump_controller_is_on()) {
                LOG_INF("Plateau period expired, turning off pump");
                ret = pump_controller_turn_off();
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
