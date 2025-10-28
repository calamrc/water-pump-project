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

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define YF_S201C_PULSES_PER_LITER 450 // YF-S201C specification
#define FLOW_THRESHOLD_L_PER_MIN 0.1f // Minimum flow rate to consider active
#define PUMP_ON_DEBOUNCE_MS 3000 // Delay in ms before allowing pump to turn on
#define MIN_PERIOD_US 100 // Minimum valid period to filter noise/debounce

#define PLATEAU_CONFIRM_COUNT 2 // Consecutive small differences to confirm plateau
#define PLATEAU_WINDOW_SIZE 5 // Sliding window for calibration
#define PLATEAU_MIN_SLOPE 0.01f // Minimum slope to assume linear phase (L/min per sample)
#define PLATEAU_K_FACTOR 3.0f // Threshold multiplier (3-sigma for Gaussian noise)

#define CONSECUTIVE_INVALID_THRESHOLD 5 // Threshold to reset buffer on invalids
#define STALE_PERIOD_THRESHOLD_MS 200 // Threshold for stale period data
#define SEM_GIVE_MIN_INTERVAL_MS 10 // Min interval for semaphore gives
#define PUMP_SAFETY_TIMEOUT_MIN 5 // Max pump runtime in minutes
#define MAX_TIMEOUT_US 1000000LL // 1 second cap for timeout

K_SEM_DEFINE(data_sem, 0, 1);

// Thread safety mutexes
K_MUTEX_DEFINE(period_buffer_mutex);
K_MUTEX_DEFINE(flow_buffer_mutex);
K_MUTEX_DEFINE(pump_state_mutex);

// Forward declarations
static void data_processing_work_handler(struct k_work *work);

// Workqueue for deferring ISR processing
K_WORK_DEFINE(data_processing_work, data_processing_work_handler);

static volatile int64_t period_buffer[5] = {0};
static volatile int64_t last_pulse_ticks = 0;
static volatile int64_t period_us = 0;
static volatile int valid_periods = 0;
static volatile int buffer_index = 0;

// ISR timestamp data for workqueue processing
static volatile int64_t isr_current_period_us = 0;
static volatile bool isr_valid_update = false;

static fixed_t flow_buffer[PLATEAU_WINDOW_SIZE] = {0};
static fixed_t flow_slope = 0;
static fixed_t noise_std = 0;
static fixed_t prev_flow = FIXED_MIN; // Use FIXED_MIN as sentinel value instead of NAN
static int flow_buffer_index = 0;
static int flow_diff_count = 0;

static const struct device *gpio_dev;

static struct k_timer safety_timer;
static int64_t last_valid_update_ms = 0;
static int64_t last_sem_give_ms = 0;
static int consecutive_invalid = 0;
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
 * @brief Calculate median value from an array of int64_t values
 *
 * Performs a simple bubble sort on a copy of the input array and returns
 * the middle element. Used for robust measurement filtering in the presence
 * of outliers.
 *
 * @param arr Pointer to array of int64_t values
 * @param size Number of elements in the array (must be odd for true median)
 * @return The median value from the sorted array
 */
static int64_t get_median(int64_t *arr, int size) {
    int64_t sorted[5];

    memcpy(sorted, arr, size * sizeof(int64_t));

    // Simple bubble sort (small array, so efficiency is not critical)
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                int64_t temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    return sorted[size / 2];
}

/**
 * @brief GPIO interrupt service routine for YF-S201C flow sensor
 *
 * Minimal ISR that only captures pulse timing and defers processing to workqueue.
 * This ensures ultra-low latency (~2μs) for critical ISR path by moving all
 * computational work (median calculation, filtering, buffer management) to
 * the workqueue thread.
 *
 * @param port GPIO device that triggered the interrupt
 * @param cb GPIO callback structure
 * @param pins Bitmask of pins that triggered the interrupt
 */
static void flow_sensor_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    int64_t current_ticks = k_uptime_ticks();

    if (last_pulse_ticks > 0) {
        int64_t diff = current_ticks - last_pulse_ticks;

        if (diff <= 0 || diff > 1000000000LL) {
            return;
        }

        int64_t current_period_us = k_ticks_to_us_floor64((k_ticks_t)diff);

        if (current_period_us > MIN_PERIOD_US) {
            // Store data for workqueue processing
            isr_current_period_us = current_period_us;
            isr_valid_update = true;

            // Submit work item for deferred processing
            k_work_submit(&data_processing_work);
        } else {
            // Invalid period - submit work for invalid handling
            isr_valid_update = false;
            k_work_submit(&data_processing_work);
        }
    }

    last_pulse_ticks = current_ticks;
}

/**
 * @brief Workqueue handler for deferred ISR processing
 *
 * Processes flow sensor data captured by ISR at lower priority to maintain
 * real-time ISR responsiveness. Handles all computationally intensive tasks:
 * - Period validation and filtering
 * - Median-based outlier rejection
 * - Buffer management and circular indexing
 * - Semaphore signaling to main thread
 * - Invalid period counting and buffer reset
 *
 * @param work Work item pointer (unused, from workqueue framework)
 */
static void data_processing_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (isr_valid_update) {
        // Valid period - process with median filtering
        consecutive_invalid = 0;
        int64_t current_period_us = isr_current_period_us;

        // Protect period buffer and related variables
        k_mutex_lock(&period_buffer_mutex, K_FOREVER);

        if (valid_periods >= 5) {
            int64_t median = get_median((int64_t *)period_buffer, 5);

            // Check if current period is significantly different (using 1.5 factor for softer rejection)
            if (current_period_us < (median / 1.5) || current_period_us > (median * 1.5)) {
                period_us = median; // Use median if outlier
                period_buffer[buffer_index] = median;
            } else {
                period_us = current_period_us; // Use current period if within bounds
                period_buffer[buffer_index] = current_period_us;
            }
        } else {
            period_us = current_period_us;
            period_buffer[buffer_index] = current_period_us;

            if (valid_periods < 5) {
                valid_periods++;
            }
        }

        buffer_index = (buffer_index + 1) % 5;
        k_mutex_unlock(&period_buffer_mutex);

        last_valid_update_ms = k_uptime_get();

        // Sem give only on valid update
        int64_t now_ms = k_uptime_get();
        if (now_ms - last_sem_give_ms > SEM_GIVE_MIN_INTERVAL_MS) {
            k_sem_give(&data_sem);
            last_sem_give_ms = now_ms;
        }
    } else {
        // Invalid period handling
        consecutive_invalid++;

        if (consecutive_invalid >= CONSECUTIVE_INVALID_THRESHOLD) {
            // Protect period buffer reset
            k_mutex_lock(&period_buffer_mutex, K_FOREVER);
            valid_periods = 0;
            memset((void *)period_buffer, 0, sizeof(period_buffer));
            k_mutex_unlock(&period_buffer_mutex);
        }
    }

    // Reset ISR flags
    isr_valid_update = false;
    isr_current_period_us = 0;
}

/**
 * @brief Perform statistical calibration for plateau detection using fixed-point arithmetic
 *
 * Analyzes the current flow buffer to determine slope and noise characteristics.
 * This calibration enables adaptive thresholding:
 * - If slope is significant (> FIXED_PLATEAU_MIN_SLOPE), assumes linear trend and
 *   computes noise as residual standard deviation from linear fit
 * - If slope is negligible, skips noise calculation (plateau assumption)
 *
 * All calculations performed in Q16.16 fixed-point for memory efficiency.
 *
 * Updates global noise_std and flow_slope variables for use in plateau detection.
 */
static void calibrate_plateau(void) {
    if (flow_buffer_index != 0) // Calibrate only when buffer full
        return;

    // Compute differences for slope using fixed-point arithmetic
    fixed_t sum_diffs = 0;

    for (int i = 0; i < PLATEAU_WINDOW_SIZE - 1; i++) {
        fixed_t diff = fixed_sub(flow_buffer[(flow_buffer_index + i + 1) % PLATEAU_WINDOW_SIZE],
                                flow_buffer[(flow_buffer_index + i) % PLATEAU_WINDOW_SIZE]);
        sum_diffs = fixed_add(sum_diffs, diff);
    }

    flow_slope = fixed_div_int(sum_diffs, PLATEAU_WINDOW_SIZE - 1);

    // Check if slope is significant using fixed-point comparison
    if (fixed_gt(fixed_abs(flow_slope), FIXED_PLATEAU_MIN_SLOPE)) {
        // Predict linear values
        fixed_t predicted[PLATEAU_WINDOW_SIZE];

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            fixed_t slope_term = fixed_mul_int(flow_slope, i);
            predicted[i] = fixed_add(flow_buffer[0], slope_term);
        }

        // Compute residuals and noise std using fixed-point
        fixed_t sum_sq_res = 0;

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            fixed_t residual = fixed_sub(flow_buffer[i], predicted[i]);
            sum_sq_res = fixed_add(sum_sq_res, fixed_mul(residual, residual));
        }

        // Calculate standard deviation
        fixed_t mean_sq_res = fixed_div_int(sum_sq_res, PLATEAU_WINDOW_SIZE);
        noise_std = fixed_sqrt(fixed_abs(mean_sq_res));
    } else {
        // Slope negligible - assume plateau, no noise calculation needed
        noise_std = 0;
    }

    LOG_DBG("Calibration complete, noise_std: %.4f, flow_slope: %.4f",
            fixed_to_float(noise_std), fixed_to_float(flow_slope));
}

/**
 * @brief Core plateau detection algorithm using fixed-point statistical analysis
 *
 * Implements sophisticated flow stability detection with adaptive thresholding:
 * - Maintains sliding window buffer of recent flow measurements in Q16.16 format
 * - Performs statistical calibration when buffer fills using fixed-point math
 * - Uses noise-adaptive epsilon threshold (3-sigma rule) in fixed-point
 * - Requires consecutive stable measurements for plateau confirmation
 * - Handles first measurement case and buffer wraparound
 *
 * This algorithm distinguishes between:
 * - Stable plateaus (minimal flow variation within noise bounds)
 * - Linear trends (significant slope requiring calibration)
 * - Noise-induced fluctuations (statistically bounded)
 *
 * All operations performed in Q16.16 fixed-point arithmetic for memory efficiency.
 *
 * @param flow_rate_fixed Current flow rate measurement in fixed-point L/min
 * @return true if plateau detected, false otherwise
 */
static bool detect_plateau(fixed_t flow_rate_fixed) {
    LOG_DBG("detect_plateau called with flow_rate: %.3f, buffer_index: %d",
            fixed_to_float(flow_rate_fixed), flow_buffer_index);

    // Add to circular buffer
    flow_buffer[flow_buffer_index] = flow_rate_fixed;
    flow_buffer_index = (flow_buffer_index + 1) % PLATEAU_WINDOW_SIZE;

    LOG_DBG("Added to buffer, new buffer_index: %d", flow_buffer_index);

    if (flow_buffer_index == 0) { // Buffer full, calibrate
        LOG_DBG("Buffer full, triggering calibration");
        calibrate_plateau();
    }

    if (fixed_eq(prev_flow, FIXED_MIN)) { // First value (using FIXED_MIN as sentinel)
        LOG_DBG("First flow value, setting prev_flow: %.3f", fixed_to_float(flow_rate_fixed));

        prev_flow = flow_rate_fixed;
        return false;
    }

    // Calculate delta and epsilon using fixed-point arithmetic
    fixed_t delta = fixed_abs(fixed_sub(flow_rate_fixed, prev_flow));
    fixed_t epsilon = fixed_mul(FIXED_PLATEAU_K_FACTOR, noise_std);

    // Use fallback epsilon if no calibration
    if (fixed_eq(noise_std, 0)) {
        epsilon = FIXED_EPSILON_FALLBACK;
    }

    LOG_DBG("Calculated delta: %.4f, epsilon: %.4f, prev_flow: %.3f",
            fixed_to_float(delta), fixed_to_float(epsilon), fixed_to_float(prev_flow));

    if (fixed_lt(delta, epsilon)) {
        flow_diff_count++;
        LOG_DBG("Delta < epsilon, flow_diff_count: %d", flow_diff_count);

        if (flow_diff_count >= PLATEAU_CONFIRM_COUNT) {
            LOG_DBG("Plateau detected (flow_diff_count >= %d)", PLATEAU_CONFIRM_COUNT);
            return true; // Plateau detected
        }
    } else {
        LOG_DBG("Delta >= epsilon, resetting flow_diff_count");
        flow_diff_count = 0;
    }

    prev_flow = flow_rate_fixed;

    LOG_DBG("Updated prev_flow: %.3f", fixed_to_float(prev_flow));

    return false;
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
    struct gpio_callback flow_callback;
    int64_t initial_plateau_period = 0;
    int64_t latest_plateau_period = 0;
    int ret;

    LOG_INF("Zephyr Water Pump Application %s", APP_VERSION_STRING);

    k_timer_init(&safety_timer, safety_timer_handler, NULL);

    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return 0;
    }

    ret = gpio_pin_configure(gpio_dev, 23, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Could not configure flow sensor GPIO (%d)", ret);
        return 0;
    }

    ret = gpio_pin_interrupt_configure(gpio_dev, 23, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("Could not enable GPIO interrupt (%d)", ret);
        return 0;
    }

    gpio_init_callback(&flow_callback, flow_sensor_isr, BIT(23));
    ret = gpio_add_callback(gpio_dev, &flow_callback);
    if (ret < 0) {
        LOG_ERR("Could not add GPIO callback (%d)", ret);
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

            if (k_uptime_get() - last_valid_update_ms > STALE_PERIOD_THRESHOLD_MS) {
                period_us = 0;
            }

            float flow_rate_lpm = (period_us > 0) ? (60.0f * 1000000.0f) / (period_us * YF_S201C_PULSES_PER_LITER) : 0.0f;
            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            if (flow_rate_lpm > 0.0f && period_us > 0) {
                // Convert flow rate to fixed-point for plateau detection
                fixed_t flow_rate_fixed = fixed_from_float(flow_rate_lpm);

                // Protect flow buffer access in plateau detection
                k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
                bool plateau_detected = detect_plateau(flow_rate_fixed);
                k_mutex_unlock(&flow_buffer_mutex);

                if (plateau_detected) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)", flow_rate_lpm, fixed_to_float(noise_std), fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, noise_std)));

                    // Protect pump state check
                    k_mutex_lock(&pump_state_mutex, K_FOREVER);
                    if (!(pump_on && period_us < initial_plateau_period))
                        latest_plateau_period = period_us;
                    k_mutex_unlock(&pump_state_mutex);

                    // Protect flow buffer reset
                    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
                    flow_buffer_index = 0;
                    prev_flow = FIXED_MIN;
                    flow_diff_count = 0;
                    noise_std = 0;
                    flow_slope = 0;
                    memset(flow_buffer, 0, sizeof(flow_buffer));
                    k_mutex_unlock(&flow_buffer_mutex);

                    // Protect pump state check and update
                    k_mutex_lock(&pump_state_mutex, K_FOREVER);
                    if (!pump_on) {
                        // Unlock temporarily to do GPIO operations
                        k_mutex_unlock(&pump_state_mutex);

                        initial_plateau_period = period_us;

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

            // Protect period buffer reset
            k_mutex_lock(&period_buffer_mutex, K_FOREVER);
            period_us = 0;
            valid_periods = 0;
            buffer_index = 0;
            memset((void *)period_buffer, 0, sizeof(period_buffer));
            k_mutex_unlock(&period_buffer_mutex);

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

            // Protect flow buffer reset in timeout section
            k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
            flow_buffer_index = 0;
            prev_flow = FIXED_MIN;
            flow_diff_count = 0;
            noise_std = 0;
            flow_slope = 0;
            memset(flow_buffer, 0, sizeof(flow_buffer));
            k_mutex_unlock(&flow_buffer_mutex);
        }
    }

    return 0;
}
