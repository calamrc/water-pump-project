/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(sensor_manager);

#include "sensor_manager.h"

/* ============================================================================
 * Module Variables (Static to this module)
 * ============================================================================ */

// Period buffer for flow sensor measurements
volatile int64_t period_buffer[5] = {0};
volatile int64_t last_pulse_ticks = 0;
volatile int64_t period_us = 0;
volatile int valid_periods = 0;
volatile int buffer_index = 0;

// ISR-to-workqueue communication variables
volatile int64_t isr_current_period_us = 0;
volatile bool isr_valid_update = false;

// Sensor configuration constants
const uint32_t YF_S201C_PULSES_PER_LITER = 450;
const uint32_t MIN_PERIOD_US = 100;
const int STALE_PERIOD_THRESHOLD_MS = 200;
const int SEM_GIVE_MIN_INTERVAL_MS = 10;
const int CONSECUTIVE_INVALID_THRESHOLD = 5;

// GPIO device reference
static const struct device *gpio_dev;

// Statistics for validation
static volatile int64_t last_valid_update_ms = 0;
static volatile int64_t last_sem_give_ms = 0;
static int consecutive_invalid = 0;

// Thread safety
K_MUTEX_DEFINE(sensor_period_mutex);

// Forward declarations for workqueue
static void sensor_work_handler(struct k_work *work);

// ISR and workqueue setup
static struct gpio_callback flow_callback;
static K_WORK_DEFINE(sensor_work, sensor_work_handler);

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Calculate median value from the period buffer
 *
 * @param arr Pointer to array of int64_t values
 * @param size Number of elements in the array (must be odd for true median)
 * @return The median value from the sorted array
 */
static int64_t sensor_get_median(int64_t *arr, int size) {
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
 */
static void sensor_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
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
            k_work_submit(&sensor_work);
        } else {
            // Invalid period - submit work for invalid handling
            isr_valid_update = false;
            k_work_submit(&sensor_work);
        }
    }

    last_pulse_ticks = current_ticks;
}

/**
 * @brief Workqueue handler for deferred ISR processing
 */
static void sensor_work_handler(struct k_work *work)
{
    if (isr_valid_update) {
        // Valid period - process with median filtering
        consecutive_invalid = 0;
        int64_t current_period_us = isr_current_period_us;

        // Protect period buffer and related variables
        k_mutex_lock(&sensor_period_mutex, K_FOREVER);

        if (valid_periods >= 5) {
            int64_t median = sensor_get_median((int64_t *)period_buffer, 5);

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
        k_mutex_unlock(&sensor_period_mutex);

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
            k_mutex_lock(&sensor_period_mutex, K_FOREVER);
            valid_periods = 0;
            memset((void *)period_buffer, 0, sizeof(period_buffer));
            k_mutex_unlock(&sensor_period_mutex);
        }
    }

    // Reset ISR flags
    isr_valid_update = false;
    isr_current_period_us = 0;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int sensor_manager_init(void)
{
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    // Configure flow sensor GPIO (input with pull-up)
    int ret = gpio_pin_configure(gpio_dev, 23, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Could not configure flow sensor GPIO (%d)", ret);
        return ret;
    }

    // Enable interrupt on falling edge
    ret = gpio_pin_interrupt_configure(gpio_dev, 23, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("Could not enable GPIO interrupt (%d)", ret);
        return ret;
    }

    // Add callback
    gpio_init_callback(&flow_callback, sensor_isr, BIT(23));
    ret = gpio_add_callback(gpio_dev, &flow_callback);
    if (ret < 0) {
        LOG_ERR("Could not add GPIO callback (%d)", ret);
        return ret;
    }

    LOG_INF("Sensor manager initialized successfully");
    return 0;
}

fixed_t sensor_manager_get_flow_rate(void)
{
    // Thread-safe access to period_us
    k_mutex_lock(&sensor_period_mutex, K_FOREVER);
    int64_t current_period = period_us;
    k_mutex_unlock(&sensor_period_mutex);

    if (current_period > 0) {
        // Flow rate calculation: (60 seconds/min) * (1e6 us/second) / (period_us * pulses_per_liter)
        float flow_rate_lpm = (60.0f * 1000000.0f) / (current_period * YF_S201C_PULSES_PER_LITER);
        return fixed_from_float(flow_rate_lpm);
    } else {
        return 0;
    }
}

bool sensor_manager_is_data_valid(void)
{
    int64_t now = k_uptime_get();

    // Data is valid if we have a recent valid update and no consecutive invalids
    if (now - last_valid_update_ms > STALE_PERIOD_THRESHOLD_MS) {
        return false;
    }

    if (consecutive_invalid >= CONSECUTIVE_INVALID_THRESHOLD) {
        return false;
    }

    k_mutex_lock(&sensor_period_mutex, K_FOREVER);
    bool has_valid_period = (valid_periods >= 1 && period_us > 0);
    k_mutex_unlock(&sensor_period_mutex);

    return has_valid_period;
}

int64_t sensor_manager_get_current_period(void)
{
    // Thread-safe access to period_us
    k_mutex_lock(&sensor_period_mutex, K_FOREVER);
    int64_t current_period = period_us;
    k_mutex_unlock(&sensor_period_mutex);

    return current_period;
}

void sensor_manager_reset(void)
{
    k_mutex_lock(&sensor_period_mutex, K_FOREVER);

    // Reset all sensor state
    valid_periods = 0;
    buffer_index = 0;
    period_us = 0;
    consecutive_invalid = 0;
    last_valid_update_ms = 0;
    last_sem_give_ms = 0;
    memset((void *)period_buffer, 0, sizeof(period_buffer));

    k_mutex_unlock(&sensor_period_mutex);

    // Reset ISR state
    last_pulse_ticks = 0;
    isr_valid_update = false;
    isr_current_period_us = 0;

    LOG_INF("Sensor manager reset complete");
}

int sensor_manager_cleanup(void)
{
    // Disable interrupt
    int ret = gpio_pin_interrupt_configure(gpio_dev, 23, GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Could not disable GPIO interrupt (%d)", ret);
        return ret;
    }

    // Remove callback
    gpio_remove_callback(gpio_dev, &flow_callback);

    // Cancel any pending work
    k_work_cancel(&sensor_work);

    sensor_manager_reset();

    LOG_INF("Sensor manager cleanup complete");
    return 0;
}
