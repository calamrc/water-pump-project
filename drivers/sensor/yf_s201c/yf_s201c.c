/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "yf_s201c.h"

#define DT_DRV_COMPAT aygu_yf_s201c

LOG_MODULE_REGISTER(yf_s201c);

/* Fixed-point math definitions */
typedef int32_t fixed_t;
#define FIXED_SCALE_F (65536.0f)

static inline fixed_t fixed_from_float(float f) {
    return (fixed_t)(f * FIXED_SCALE_F + (f >= 0.0f ? 0.5f : -0.5f));
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Calculate median value from the period buffer
 */
static int64_t yf_s201c_get_median(int64_t *arr, int size) {
    int64_t sorted[5];

    memcpy(sorted, arr, size * sizeof(int64_t));

    // Simple bubble sort
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
 * @brief GPIO interrupt service routine
 */
void yf_s201c_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    const struct device *dev = CONTAINER_OF(cb, struct yf_s201c_data, gpio_cb)->dev; // Need to add dev to data
    struct yf_s201c_data *data = dev->data;

    int64_t current_ticks = k_uptime_ticks();

    if (data->last_pulse_ticks > 0) {
        int64_t diff = current_ticks - data->last_pulse_ticks;

        if (diff <= 0 || diff > 1000000000LL) {
            return;
        }

        int64_t current_period_us = k_ticks_to_us_floor64((k_ticks_t)diff);

        if (current_period_us > data->min_period_us) {
            // Store data for workqueue processing
            data->isr_current_period_us = current_period_us;
            data->isr_valid_update = true;

            // Submit work item only if not already pending
            if (!k_work_is_pending(&data->work)) {
                k_work_submit(&data->work);
            }
        } else {
            // Invalid period
            data->isr_valid_update = false;
            // Submit work item only if not already pending
            if (!k_work_is_pending(&data->work)) {
                k_work_submit(&data->work);
            }
        }
    }

    data->last_pulse_ticks = current_ticks;
}

/**
 * @brief Workqueue handler for deferred ISR processing
 */
void yf_s201c_work_handler(struct k_work *work)
{
    struct yf_s201c_data *data = CONTAINER_OF(work, struct yf_s201c_data, work);

    if (data->isr_valid_update) {
        // Valid period
        data->consecutive_invalid = 0;
        int64_t current_period_us = data->isr_current_period_us;

        // Protect period buffer
        k_mutex_lock(&data->mutex, K_FOREVER);

if (data->valid_periods >= data->buffer_size) {
			int64_t median = yf_s201c_get_median((int64_t *)data->period_buffer, data->buffer_size);

			// Outlier rejection: use median for smoothed output,
			// but always write actual measurement to buffer so median
			// can evolve with flow rate changes.
			if (current_period_us < (median / 1.5) || current_period_us > (median * 1.5)) {
				data->period_us = median;
			} else {
				data->period_us = current_period_us;
			}
			data->period_buffer[data->buffer_index] = current_period_us;
		} else {
			data->period_us = current_period_us;
			data->period_buffer[data->buffer_index] = current_period_us;

			if (data->valid_periods < data->buffer_size) {
				data->valid_periods++;
			}
		}

        data->buffer_index = (data->buffer_index + 1) % data->buffer_size;
        k_mutex_unlock(&data->mutex);

        data->last_valid_update_ms = k_uptime_get();
        data->state = YF_S201C_RUNNING;

        // Signal semaphore if configured for event-driven operation
        if (data->data_sem != NULL) {
            k_sem_give(data->data_sem);
        }
    } else {
        // Invalid period
        data->consecutive_invalid++;

        if (data->consecutive_invalid >= data->consecutive_invalid_threshold) {
            // Reset buffer
            k_mutex_lock(&data->mutex, K_FOREVER);
            data->valid_periods = 0;
            memset((void *)data->period_buffer, 0, sizeof(data->period_buffer));
            k_mutex_unlock(&data->mutex);

            data->state = YF_S201C_ERROR;
            LOG_WRN("YF-S201C: Consecutive invalid periods, buffer reset");
        }
    }

    // Reset ISR flags
    data->isr_valid_update = false;
    data->isr_current_period_us = 0;
}

/* ============================================================================
 * Device Driver Implementation
 * ============================================================================ */

static int yf_s201c_init(const struct device *dev)
{
    const struct yf_s201c_config *config = dev->config;
    struct yf_s201c_data *data = dev->data;

    // Copy config to data for runtime access
    data->pulses_per_liter = config->pulses_per_liter;
    data->min_period_us = config->min_period_us;
    data->buffer_size = config->buffer_size;
    data->stale_threshold_ms = config->stale_threshold_ms;
    data->consecutive_invalid_threshold = config->consecutive_invalid_threshold;

    // Initialize mutex
    k_mutex_init(&data->mutex);

    // Configure GPIO
    if (!device_is_ready(config->gpio.port)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&config->gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Could not configure GPIO (%d)", ret);
        return ret;
    }

    // Enable interrupt
    ret = gpio_pin_interrupt_configure_dt(&config->gpio, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("Could not enable GPIO interrupt (%d)", ret);
        return ret;
    }

    // Setup callback
    gpio_init_callback(&data->gpio_cb, yf_s201c_isr, BIT(config->gpio.pin));
    ret = gpio_add_callback(config->gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Could not add GPIO callback (%d)", ret);
        return ret;
    }

    // Initialize workqueue
    k_work_init(&data->work, yf_s201c_work_handler);

    data->state = YF_S201C_READY;
    data->dev = dev;

    LOG_INF("YF-S201C sensor initialized successfully");
    return 0;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int yf_s201c_set_data_semaphore(const struct device *dev, struct k_sem *sem)
{
    struct yf_s201c_data *data = dev->data;

    if (data->state == YF_S201C_INIT) {
        return -ENOTSUP;
    }

    data->data_sem = sem;
    LOG_INF("YF-S201C semaphore configured for event-driven operation");

    return 0;
}

int yf_s201c_get_flow_rate(const struct device *dev, fixed_t *flow_rate)
{
    const struct yf_s201c_config *config = dev->config;
    struct yf_s201c_data *data = dev->data;

    if (data->state == YF_S201C_INIT) {
        return -ENOTSUP;
    }

    // Thread-safe access
    k_mutex_lock(&data->mutex, K_FOREVER);
    int64_t current_period = data->period_us;
    k_mutex_unlock(&data->mutex);

    if (current_period > 0 && current_period <= INT32_MAX) {
        // Overflow check
        if (current_period * config->pulses_per_liter > INT64_MAX / 1000000) {
            LOG_WRN("Flow rate calculation would overflow");
            *flow_rate = 0;
            return -ERANGE;
        }
        // Flow rate: (60 * 1e6) / (period_us * pulses_per_liter)
        float flow_rate_lpm = (60.0f * 1000000.0f) / (current_period * config->pulses_per_liter);
        *flow_rate = fixed_from_float(flow_rate_lpm);
        return 0;
    } else {
        *flow_rate = 0;
        return 0;
    }
}

bool yf_s201c_is_data_valid(const struct device *dev)
{
    struct yf_s201c_data *data = dev->data;

    if (data->state == YF_S201C_INIT || data->state == YF_S201C_ERROR) {
        return false;
    }

    int64_t now = k_uptime_get();

    // Check staleness
    if (now - data->last_valid_update_ms > data->stale_threshold_ms) {
        return false;
    }

    // Check consecutive invalid
    if (data->consecutive_invalid >= data->consecutive_invalid_threshold) {
        return false;
    }

    k_mutex_lock(&data->mutex, K_FOREVER);
    bool has_valid = (data->valid_periods >= 1 && data->period_us > 0);
    k_mutex_unlock(&data->mutex);

    return has_valid;
}

int yf_s201c_get_current_period(const struct device *dev, int64_t *period_us)
{
    struct yf_s201c_data *data = dev->data;

    if (data->state == YF_S201C_INIT) {
        return -ENOTSUP;
    }

    // Thread-safe access
    k_mutex_lock(&data->mutex, K_FOREVER);
    *period_us = data->period_us;
    k_mutex_unlock(&data->mutex);

    return 0;
}

int yf_s201c_reset(const struct device *dev)
{
    struct yf_s201c_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);

    // Reset all state
    data->valid_periods = 0;
    data->buffer_index = 0;
    data->period_us = 0;
    data->consecutive_invalid = 0;
    data->last_valid_update_ms = 0;
    memset((void *)data->period_buffer, 0, sizeof(data->period_buffer));

    k_mutex_unlock(&data->mutex);

    // Reset ISR state
    data->last_pulse_ticks = 0;
    data->isr_valid_update = false;
    data->isr_current_period_us = 0;

    data->state = YF_S201C_READY;

    LOG_INF("YF-S201C sensor reset complete");
    return 0;
}

/* ============================================================================
 * Device Declaration
 * ============================================================================ */

#define YF_S201C_DEFINE(inst) \
    static struct yf_s201c_config yf_s201c_config_##inst = { \
        .gpio = GPIO_DT_SPEC_INST_GET(inst, gpios), \
        .pulses_per_liter = DT_INST_PROP(inst, pulses_per_liter), \
        .min_period_us = DT_INST_PROP(inst, min_period_us), \
        .buffer_size = DT_INST_PROP(inst, buffer_size), \
        .stale_threshold_ms = DT_INST_PROP(inst, stale_threshold_ms), \
        .consecutive_invalid_threshold = DT_INST_PROP(inst, consecutive_invalid_threshold), \
    }; \
    \
    static struct yf_s201c_data yf_s201c_data_##inst = { \
        .state = YF_S201C_INIT, \
        .data_sem = NULL, \
    }; \
    \
    DEVICE_DT_INST_DEFINE(inst, \
                         yf_s201c_init, \
                         NULL, \
                         &yf_s201c_data_##inst, \
                         &yf_s201c_config_##inst, \
                         POST_KERNEL, \
                         CONFIG_YF_S201C_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(YF_S201C_DEFINE)