/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>
#include <zephyr/sys/util.h>

#include <app/drivers/blink.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

// YF-S201C flow sensor constants
#define YF_S201C_PULSES_PER_LITER 450  // YF-S201C specification
#define FLOW_THRESHOLD_L_PER_MIN 0.1f   // Minimum flow rate to consider active
#define MIN_PERIOD_US 100               // Minimum valid period to filter noise/debounce
#define PUMP_ON_DEBOUNCE_MS 3000       // Delay in ms before allowing pump to turn on

// Plateau detection constants
#define PLATEAU_WINDOW_SIZE 10          // Sliding window for calibration
#define PLATEAU_K_FACTOR 3.0f          // Threshold multiplier (3-sigma for Gaussian noise)
#define PLATEAU_CONFIRM_COUNT 2        // Consecutive small differences to confirm plateau
#define PLATEAU_MIN_SLOPE 0.01f        // Minimum slope to assume linear phase (L/min per sample)

K_SEM_DEFINE(data_sem, 0, 1);

// Global variables for flow measurement (volatile for ISR/main sharing)
static volatile int64_t period_us = 0;
static volatile int64_t last_pulse_ticks = 0;
static const struct device *gpio_dev;  // Global for ISR access
static volatile int64_t period_buffer[5] = {0};
static volatile int buffer_index = 0;
static volatile int valid_periods = 0;

// Global variables for plateau detection
static float flow_buffer[PLATEAU_WINDOW_SIZE] = {0};
static int flow_buffer_index = 0;
static float prev_flow = NAN;  // Initialize to invalid
static float noise_std = 0.0f;
static float flow_slope = 0.0f;
static int flow_diff_count = 0;
static bool plateau_detected = false;

// GPIO callback for flow sensor pulse detection
static void flow_sensor_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    int64_t current_ticks = k_uptime_ticks();
    if (last_pulse_ticks > 0) {
        int64_t diff = current_ticks - last_pulse_ticks;
        int64_t current_period_us = k_ticks_to_us_floor64((k_ticks_t)diff);
        if (current_period_us > MIN_PERIOD_US) {
            if (valid_periods >= 5) {
                // Calculate average from buffer
                int64_t sum = 0;
                for (int i = 0; i < 5; i++) {
                    sum += period_buffer[i];
                }
                int64_t average = sum / 5;

                // Check if current period is significantly different (using 1.5 factor for softer rejection)
                if (current_period_us < (average / 1.5) || current_period_us > (average * 1.5)) {
                    period_us = average;
                    period_buffer[buffer_index] = average;
                } else {
                    period_us = current_period_us;
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
        }
    }

    last_pulse_ticks = current_ticks;
    k_sem_give(&data_sem);
}

// Function to calibrate noise and slope for plateau detection
static void calibrate_plateau(void) {
    if (flow_buffer_index != 0) return;  // Calibrate only when buffer full
    

    LOG_DEBUG("Calibrating plateau detection");
    

    // Compute differences for slope
    float diffs[PLATEAU_WINDOW_SIZE - 1];
    float sum_diffs = 0.0f;
    for (int i = 0; i < PLATEAU_WINDOW_SIZE - 1; i++) {
        diffs[i] = flow_buffer[(flow_buffer_index + i + 1) % PLATEAU_WINDOW_SIZE] -
                   flow_buffer[(flow_buffer_index + i) % PLATEAU_WINDOW_SIZE];
        sum_diffs += diffs[i];
    }
    flow_slope = sum_diffs / (PLATEAU_WINDOW_SIZE - 1);
    LOG_DEBUG("Calculated flow_slope: %.4f", flow_slope);
    

    if (fabsf(flow_slope) > PLATEAU_MIN_SLOPE) {  // Assume linear if slope significant
        LOG_DEBUG("Slope significant, calculating noise_std");
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
        noise_std = sqrtf(sum_sq_res / PLATEAU_WINDOW_SIZE);  // Approx population std
        LOG_DEBUG("Calculated noise_std: %.4f", noise_std);
    } else {
        LOG_DEBUG("Slope not significant, keeping noise_std at 0");
    }
}

// Function to process flow rate for plateau detection
static bool detect_plateau(float flow_rate) {
    // Add to circular buffer
    flow_buffer[flow_buffer_index] = flow_rate;
    flow_buffer_index = (flow_buffer_index + 1) % PLATEAU_WINDOW_SIZE;
    LOG_DEBUG("Added flow %.2f to buffer, flow_buffer_index %d", flow_rate, flow_buffer_index);

    if (flow_buffer_index == 0) {  // Buffer full, calibrate
        LOG_DEBUG("Buffer full, calibrating");
        calibrate_plateau();
    }
    

    if (isnan(prev_flow)) {  // First value
        LOG_DEBUG("First flow value: %.2f, initializing", flow_rate);
        prev_flow = flow_rate;
        return false;
    }
    

    float delta = fabsf(flow_rate - prev_flow);
    float epsilon = PLATEAU_K_FACTOR * noise_std;
    if (noise_std == 0.0f) epsilon = 0.01f;  // Fallback if no calibration
    LOG_DEBUG("Checking plateau: flow=%.2f, prev=%.2f, delta=%.4f, epsilon=%.4f", flow_rate, prev_flow, delta, epsilon);
    

    if (delta < epsilon) {
        flow_diff_count++;
        LOG_DEBUG("Delta < epsilon, flow_diff_count now %d", flow_diff_count);
        if (flow_diff_count >= PLATEAU_CONFIRM_COUNT) {
            LOG_DEBUG("Plateau confirmed at %.2f L/min", flow_rate);
            return true;  // Plateau detected
        }
    } else {
        LOG_DEBUG("Delta >= epsilon, resetting flow_diff_count");
        flow_diff_count = 0;
    }
    

    prev_flow = flow_rate;
    LOG_DEBUG("Returning false, no plateau");
    return false;
}

int main(void)
{
    int ret;
    const struct device *blink;
    struct gpio_callback flow_callback;
    bool pump_on = false;
    int64_t last_off_time = 0;

    printk("Zephyr Water Pump Application %s\n", APP_VERSION_STRING);

    // Initialize blink LED
    blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (!device_is_ready(blink)) {
        LOG_ERR("Blink LED not ready");
        return 0;
    }

    ret = blink_set_period_ms(blink, 1000000);  // Very slow blink initially
    if (ret < 0) {
        LOG_ERR("Could not start LED blinking (%d)", ret);
        return 0;
    }

    // Get GPIO device for flow sensor and pump relay
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return 0;
    }

    // Configure flow sensor pin (GPIO 23) for interrupt with pull-up
    ret = gpio_pin_configure(gpio_dev, 23, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Could not configure flow sensor GPIO (%d)", ret);
        return 0;
    }

    // Enable GPIO interrupt for flow sensor pin
    ret = gpio_pin_interrupt_configure(gpio_dev, 23, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("Could not enable GPIO interrupt (%d)", ret);
        return 0;
    }

    // Set up GPIO callback for flow sensor
    gpio_init_callback(&flow_callback, flow_sensor_isr, BIT(23));
    ret = gpio_add_callback(gpio_dev, &flow_callback);
    if (ret < 0) {
        LOG_ERR("Could not add GPIO callback (%d)", ret);
        return 0;
    }

    // Configure pump relay pin (GPIO 22) as output
    ret = gpio_pin_configure(gpio_dev, 22, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Could not configure pump relay GPIO (%d)", ret);
        return 0;
    }

    // Initially turn pump OFF (active low relay, high = OFF)
    ret = gpio_pin_set(gpio_dev, 22, 1);
    if (ret < 0) {
        LOG_ERR("Could not set initial pump relay state (%d)", ret);
        return 0;
    }

    while (1) {
        k_timeout_t timeout = (period_us == 0) ? K_FOREVER : K_USEC(period_us * 2);
        if (k_sem_take(&data_sem, timeout) != 0) {
            // Timeout: no flow
            period_us = 0;
            valid_periods = 0;
            buffer_index = 0;
            memset((void *)period_buffer, 0, sizeof(period_buffer));
            if (pump_on) {
                ret = gpio_pin_set(gpio_dev, 22, 1);  // OFF
                if (ret < 0) {
                    LOG_ERR("Could not set pump relay to OFF (%d)", ret);
                } else {
                    pump_on = false;
                    last_off_time = k_uptime_get();
                }
            }
            blink_set_period_ms(blink, 1000000);  // Slow blink for no flow
            // Reset plateau detection state
            flow_buffer_index = 0;
            prev_flow = NAN;
            flow_diff_count = 0;
            plateau_detected = false;
            noise_std = 0.0f;
            flow_slope = 0.0f;
            memset(flow_buffer, 0, sizeof(flow_buffer));
        } else {
            // Pulse detected: calculate flow rate
            float flow_rate_lpm = (period_us > 0) ? (60.0f * 1000000.0f) / (period_us * YF_S201C_PULSES_PER_LITER) : 0.0f;
            LOG_INF("Flow rate: %.2f L/min", flow_rate_lpm);

            // Check for plateau
            if (!plateau_detected && flow_rate_lpm > 0.0f) {  // Only check if flow is non-zero
                if (detect_plateau(flow_rate_lpm)) {
                    LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.2f, epsilon: %.2f)",
                            flow_rate_lpm, noise_std, PLATEAU_K_FACTOR * noise_std);
                    plateau_detected = true;
                }
            }

            if (flow_rate_lpm >= FLOW_THRESHOLD_L_PER_MIN && plateau_detected) {
                int64_t current_time = k_uptime_get();
                if (!pump_on && (current_time - last_off_time >= PUMP_ON_DEBOUNCE_MS)) {
