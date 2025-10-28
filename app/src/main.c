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

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define FLOW_THRESHOLD_L_PER_MIN 0.1f // Minimum flow rate to consider active
#define PUMP_ON_DEBOUNCE_MS 3000 // Delay in ms before allowing pump to turn on

#define PLATEAU_CONFIRM_COUNT 2 // Consecutive small differences to confirm plateau
#define PLATEAU_WINDOW_SIZE 5 // Sliding window for calibration
#define PLATEAU_MIN_SLOPE 0.01f // Minimum slope to assume linear phase (L/min per sample)
#define PLATEAU_K_FACTOR 3.0f // Threshold multiplier (3-sigma for Gaussian noise)

#define PUMP_SAFETY_TIMEOUT_MIN 5 // Max pump runtime in minutes
#define MAX_TIMEOUT_US 1000000LL // 1 second cap for timeout

K_SEM_DEFINE(data_sem, 0, 1);

// Thread safety mutexes
K_MUTEX_DEFINE(pump_state_mutex);

static const struct device *gpio_dev;

static struct k_timer safety_timer;
static bool pump_on = false;

/**
 * @brief Safety timer callback to prevent pump from running indefinitely
 *
 * Automatically shuts down the pump if it has been running continuously
 * for the maximum allowed time (PUMP_SAFETY_TIMEOUT_MIN minutes).
 * This provides hardware protection against system failures that could
 * leave the pump running unattended.
 *
 * @param timer_id Timer identifier (unused)
 */
static void safety_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);

    // Protect pump state access
    k_mutex_lock(&pump_state_mutex, K_FOREVER);
    if (pump_on) {
        pump_on = false;
        k_mutex_unlock(&pump_state_mutex);

        int ret = gpio_pin_set(gpio_dev, 22, 1);

        if (ret < 0) {
            LOG_ERR("Safety: Could not set pump relay to OFF (%d)", ret);
        } else {
            LOG_INF("Safety: Pump turned OFF (max runtime exceeded)");
        }
    } else {
        k_mutex_unlock(&pump_state_mutex);
    }
}





/**
 * @brief Main application entry point for Zephyr Water Pump control system
 *
 * Initializes the intelligent water pump application with the following sequence:
 * 1. System initialization and logging setup
 * 2. GPIO hardware configuration (flow sensor on GPIO 23, pump relay on GPIO 22)
 * 3. Safety timer setup for maximum runtime protection
 * 4. Interrupt service routine registration for real-time flow monitoring
 *
 * The main control loop implements demand-based pumping with two operational phases:
 *
 * **Phase 1: Sensor Monitoring (Pump OFF)**
 * - Waits indefinitely on data semaphore from ISR
 * - Processes flow measurements and detects plateaus
 * - Activates pump when stable flow conditions are achieved
 *
 * **Phase 2: Pump Runtime Control (Pump ON)**
 * - Implements timeout-based monitoring (1.5x plateau period)
 * - Continues plateau detection during pump operation
 * - Automatically shuts down pump on timeout or state changes
 *
 * The application achieves intelligent, demand-driven water pumping that:
 * - Prevents pump activation during turbulent flow periods
 * - Optimizes pump runtime based on actual demand
 * - Provides multiple layers of safety and error protection
 * - Adapts to varying flow conditions through statistical analysis
 *
 * @return Application exit code (0 for normal operation, never returns in production)
 */
int main(void)
{
    int64_t initial_plateau_period = 0;
    int64_t latest_plateau_period = 0;
    int ret;

    LOG_INF("Zephyr Water Pump Application %s", APP_VERSION_STRING);

    // Initialize sensor manager
    ret = sensor_manager_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize sensor manager (%d)", ret);
        return 0;
    }

    // Initialize flow analyzer
    ret = flow_analyzer_init();
    if (ret < 0) {
        LOG_ERR("Could not initialize flow analyzer (%d)", ret);
        return 0;
    }

    k_timer_init(&safety_timer, safety_timer_handler, NULL);

    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return 0;
    }

    ret = gpio_pin_configure(gpio_dev, 22, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Could not configure pump relay GPIO (%d)", ret);
        return 0;
    }

    ret = gpio_pin_set(gpio_dev, 22, 1);
    if (ret < 0) {
        LOG_ERR("Could not set initial pump relay state (%d)", ret);
        return 0;
    }

    while (1) {
        // Protect pump state access
        k_mutex_lock(&pump_state_mutex, K_FOREVER);
        bool current_pump_on = pump_on;
        k_mutex_unlock(&pump_state_mutex);

        LOG_DBG("Waiting on semaphore (pump_on: %d, timeout_us: %lld)", current_pump_on, current_pump_on ? latest_plateau_period * 1.5 : -1LL);

        int64_t start_wait_ms = k_uptime_get();
        int64_t timeout_us = latest_plateau_period * 1.5;
        k_timeout_t timeout = (!current_pump_on) ? K_FOREVER : K_USEC(MIN(timeout_us, MAX_TIMEOUT_US));

        if (k_sem_take(&data_sem, timeout) == 0) {
            int64_t end_wait_ms = k_uptime_get();
            LOG_DBG("Semaphore taken after %lld ms", end_wait_ms - start_wait_ms);

            // Get flow rate from sensor manager
            fixed_t flow_rate_fixed = sensor_manager_get_flow_rate();
            float flow_rate_lpm = fixed_to_float(flow_rate_fixed);
            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            if (sensor_manager_is_data_valid()) {

                bool plateau_detected = flow_analyzer_detect_plateau(flow_rate_fixed);

                if (plateau_detected) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)", flow_rate_lpm, fixed_to_float(flow_analyzer_get_noise_std()), fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, flow_analyzer_get_noise_std())));

                    // Get current period from sensor manager
                    int64_t current_period = sensor_manager_get_current_period();

                    // Protect pump state check
                    k_mutex_lock(&pump_state_mutex, K_FOREVER);
                    if (!(pump_on && current_period < initial_plateau_period))
                        latest_plateau_period = current_period;
                    k_mutex_unlock(&pump_state_mutex);

                    // Reset flow analyzer state
                    flow_analyzer_reset();

                    // Protect pump state check and update
                    k_mutex_lock(&pump_state_mutex, K_FOREVER);
                    if (!pump_on) {
                        // Unlock temporarily to do GPIO operations
                        k_mutex_unlock(&pump_state_mutex);

                        initial_plateau_period = current_period;

                        ret = gpio_pin_set(gpio_dev, 22, 0);
                        if (ret < 0) {
                            LOG_ERR("Could not set pump relay to ON (%d)", ret);
                        } else {
                            // Protect pump state update
                            k_mutex_lock(&pump_state_mutex, K_FOREVER);
                            pump_on = true;
                            k_mutex_unlock(&pump_state_mutex);

                            LOG_INF("Pump turned ON");
                            k_timer_start(&safety_timer, K_MINUTES(PUMP_SAFETY_TIMEOUT_MIN), K_NO_WAIT);
                        }
                    } else {
                        k_mutex_unlock(&pump_state_mutex);
                    }
                }
            }
        } else {
            int64_t end_wait_ms = k_uptime_get();

            LOG_DBG("Timeout after %lld ms, resetting flow state", end_wait_ms - start_wait_ms);

            // Reset sensor manager state on timeout
            sensor_manager_reset();

            // Protect pump state check and update
            k_mutex_lock(&pump_state_mutex, K_FOREVER);
            if (pump_on) {
                // Unlock to do GPIO operations
                k_mutex_unlock(&pump_state_mutex);

                ret = gpio_pin_set(gpio_dev, 22, 1); // OFF
                if (ret < 0) {
                    LOG_ERR("Could not set pump relay to OFF (%d)", ret);
                } else {
                    // Protect pump state update
                    k_mutex_lock(&pump_state_mutex, K_FOREVER);
                    pump_on = false;
                    k_mutex_unlock(&pump_state_mutex);

                    LOG_INF("Pump turned OFF");
                    k_timer_stop(&safety_timer);
                }
            } else {
                k_mutex_unlock(&pump_state_mutex);
            }

            // Reset flow analyzer state on timeout
            flow_analyzer_reset();
        }
    }

    return 0;
}
