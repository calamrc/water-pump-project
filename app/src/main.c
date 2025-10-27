/*
 * Copyright (c) 2025 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <app_version.h>
#include <string.h>
#include <math.h>

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

static const struct device *gpio_dev;

static volatile int64_t period_buffer[5] = {0};
static volatile int64_t last_pulse_ticks = 0;
static volatile int64_t period_us = 0;
static volatile int valid_periods = 0;
static volatile int buffer_index = 0;

static float flow_buffer[PLATEAU_WINDOW_SIZE] = {0};
static float flow_slope = 0.0f;
static float noise_std = 0.0f;
static float prev_flow = NAN;
static int flow_buffer_index = 0;
static int flow_diff_count = 0;

static struct k_timer safety_timer;
static int64_t last_valid_update_ms = 0;
static int64_t last_sem_give_ms = 0;
static int consecutive_invalid = 0;
static bool pump_on = false;

static void safety_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);

    if (pump_on) {
        int ret = gpio_pin_set(gpio_dev, 22, 1);

        if (ret < 0) {
            LOG_ERR("Safety: Could not set pump relay to OFF (%d)", ret);
        } else {
            pump_on = false;

            LOG_INF("Safety: Pump turned OFF (max runtime exceeded)");
        }
    }
}

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
            consecutive_invalid = 0;

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
            last_valid_update_ms = k_uptime_get();

            // Sem give only on valid update
            int64_t now_ms = k_uptime_get();

            if (now_ms - last_sem_give_ms > SEM_GIVE_MIN_INTERVAL_MS) {
                k_sem_give(&data_sem);
                last_sem_give_ms = now_ms;
            }
        } else {
            consecutive_invalid++;

            if (consecutive_invalid >= CONSECUTIVE_INVALID_THRESHOLD) {
                valid_periods = 0;

                memset((void *)period_buffer, 0, sizeof(period_buffer));
            }
        }
    }

    last_pulse_ticks = current_ticks;
}

static void calibrate_plateau(void) {
    if (flow_buffer_index != 0) // Calibrate only when buffer full
        return;

    // Compute differences for slope
    float diffs[PLATEAU_WINDOW_SIZE - 1];
    float sum_diffs = 0.0f;

    for (int i = 0; i < PLATEAU_WINDOW_SIZE - 1; i++) {
        diffs[i] = flow_buffer[(flow_buffer_index + i + 1) % PLATEAU_WINDOW_SIZE] - flow_buffer[(flow_buffer_index + i) % PLATEAU_WINDOW_SIZE];
        sum_diffs += diffs[i];
    }

    flow_slope = sum_diffs / (PLATEAU_WINDOW_SIZE - 1);

    if (fabsf(flow_slope) > PLATEAU_MIN_SLOPE) { // Assume linear if slope significant
        // Predict linear values
        float predicted[PLATEAU_WINDOW_SIZE];

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            predicted[i] = flow_buffer[0] + flow_slope * i;
        }

        // Compute residuals and noise std
        float sum_sq_res = 0.0f;

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            float res = flow_buffer[i] - predicted[i];
            sum_sq_res += res * res;
        }

        noise_std = sqrtf(sum_sq_res / PLATEAU_WINDOW_SIZE); // Approx population std
    }

    LOG_DBG("Calibration complete, noise_std: %.4f, flow_slope: %.4f", noise_std, flow_slope);
}

static bool detect_plateau(float flow_rate) {
    LOG_DBG("detect_plateau called with flow_rate: %.3f, buffer_index: %d", flow_rate, flow_buffer_index);

    // Add to circular buffer
    flow_buffer[flow_buffer_index] = flow_rate;
    flow_buffer_index = (flow_buffer_index + 1) % PLATEAU_WINDOW_SIZE;

    LOG_DBG("Added to buffer, new buffer_index: %d", flow_buffer_index);

    if (flow_buffer_index == 0) { // Buffer full, calibrate
        LOG_DBG("Buffer full, triggering calibration");
        calibrate_plateau();
    }

    if (isnan(prev_flow)) { // First value
        LOG_DBG("First flow value, setting prev_flow: %.3f", flow_rate);

        prev_flow = flow_rate;
        return false;
    }

    float delta = fabsf(flow_rate - prev_flow);
    float epsilon = PLATEAU_K_FACTOR * noise_std;

    if (noise_std == 0.0f)
        epsilon = 0.01f; // Fallback if no calibration

    LOG_DBG("Calculated delta: %.4f, epsilon: %.4f, prev_flow: %.3f", delta, epsilon, prev_flow);

    if (delta < epsilon) {
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

    prev_flow = flow_rate;

    LOG_DBG("Updated prev_flow: %.3f", prev_flow);

    return false;
}

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
        LOG_DBG("Waiting on semaphore (pump_on: %d, timeout_us: %lld)", pump_on, pump_on ? latest_plateau_period * 1.5 : -1LL);

        int64_t start_wait_ms = k_uptime_get();
        int64_t timeout_us = latest_plateau_period * 1.5;
        k_timeout_t timeout = (!pump_on) ? K_FOREVER : K_USEC(MIN(timeout_us, MAX_TIMEOUT_US));

        if (k_sem_take(&data_sem, timeout) == 0) {
            int64_t end_wait_ms = k_uptime_get();
            LOG_DBG("Semaphore taken after %lld ms", end_wait_ms - start_wait_ms);

            if (k_uptime_get() - last_valid_update_ms > STALE_PERIOD_THRESHOLD_MS) {
                period_us = 0;
            }

            float flow_rate_lpm = (period_us > 0) ? (60.0f * 1000000.0f) / (period_us * YF_S201C_PULSES_PER_LITER) : 0.0f;
            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            if (flow_rate_lpm > 0.0f && period_us > 0) {
                if (detect_plateau(flow_rate_lpm)) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.2f, epsilon: %.2f)", flow_rate_lpm, noise_std, PLATEAU_K_FACTOR * noise_std);

                    if (!(pump_on && period_us < initial_plateau_period))
                        latest_plateau_period = period_us;

                    flow_buffer_index = 0;
                    prev_flow = NAN;
                    flow_diff_count = 0;
                    noise_std = 0.0f;
                    flow_slope = 0.0f;

                    memset(flow_buffer, 0, sizeof(flow_buffer));

                    if (!pump_on) {
                        initial_plateau_period = period_us;

                        ret = gpio_pin_set(gpio_dev, 22, 0);
                        if (ret < 0) {
                            LOG_ERR("Could not set pump relay to ON (%d)", ret);
                        } else {
                            pump_on = true;
                            LOG_INF("Pump turned ON");
                            k_timer_start(&safety_timer, K_MINUTES(PUMP_SAFETY_TIMEOUT_MIN), K_NO_WAIT);
                        }
                    }
                }
            }
        } else {
            int64_t end_wait_ms = k_uptime_get();

            LOG_DBG("Timeout after %lld ms, resetting flow state", end_wait_ms - start_wait_ms);

            period_us = 0;
            valid_periods = 0;
            buffer_index = 0;

            memset((void *)period_buffer, 0, sizeof(period_buffer));

            if (pump_on) {
                ret = gpio_pin_set(gpio_dev, 22, 1); // OFF
                if (ret < 0) {
                    LOG_ERR("Could not set pump relay to OFF (%d)", ret);
                } else {
                    pump_on = false;
                    LOG_INF("Pump turned OFF");
                    k_timer_stop(&safety_timer);
                }
            }

            flow_buffer_index = 0;
            prev_flow = NAN;
            flow_diff_count = 0;
            noise_std = 0.0f;
            flow_slope = 0.0f;

            memset(flow_buffer, 0, sizeof(flow_buffer));
        }
    }

    return 0;
}