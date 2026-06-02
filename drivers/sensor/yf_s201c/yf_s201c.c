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
 * @brief Store a new accepted raw period into the recent-period ring buffer.
 *
 * Driver is intentionally thin: it only performs basic electrical sanity
 * (min_period check in ISR, consecutive-invalid ERROR state machine).
 * All median filtering, outlier rejection, flow-rate calculation and
 * validity policy now live in FlowSensorService + flow_processor (app layer).
 */
static void yf_s201c_process_period(struct yf_s201c_data *data, int64_t period_us)
{
	k_mutex_lock(&data->mutex, K_FOREVER);

	/* Simple ring append of the raw accepted period (no median/outlier here) */
	data->period_buffer[data->buffer_index] = period_us;
	data->period_us = period_us;
	data->buffer_index = (data->buffer_index + 1) % data->buffer_size;

	if (data->valid_periods < data->buffer_size) {
		data->valid_periods++;
	}

	k_mutex_unlock(&data->mutex);
}

/**
 * @brief Workqueue handler for deferred ISR processing
 */
void yf_s201c_work_handler(struct k_work *work)
{
	struct yf_s201c_data *data = CONTAINER_OF(work, struct yf_s201c_data, work);

	if (data->isr_valid_update) {
		data->consecutive_invalid = 0;
		int64_t current_period_us = data->isr_current_period_us;

		yf_s201c_process_period(data, current_period_us);

		data->last_valid_update_ms = k_uptime_get();
		data->state = YF_S201C_RUNNING;

		if (data->data_sem != NULL) {
			k_sem_give(data->data_sem);
		}
	} else {
		data->consecutive_invalid++;

		if (data->consecutive_invalid >= data->consecutive_invalid_threshold) {
			k_mutex_lock(&data->mutex, K_FOREVER);
			data->valid_periods = 0;
			memset((void *)data->period_buffer, 0, sizeof(data->period_buffer));
			k_mutex_unlock(&data->mutex);

			data->state = YF_S201C_ERROR;
			LOG_WRN("YF-S201C: Consecutive invalid periods, buffer reset");
		}
	}

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
	/* Deprecated during driver thinning.
	 * Flow rate calculation now lives in FlowSensorService + flow_processor.
	 * Kept for compatibility during the refactor.
	 */
	ARG_UNUSED(dev);
	if (flow_rate) {
		*flow_rate = 0;
	}
	LOG_WRN("yf_s201c_get_flow_rate is deprecated. Use yf_s201c_get_recent_periods_us() + flow_processor instead.");
	return -ENOTSUP;
}

bool yf_s201c_is_data_valid(const struct device *dev)
{
	/* Deprecated during driver thinning.
	 * Validity policy now lives in FlowSensorService.
	 */
	ARG_UNUSED(dev);
	LOG_WRN("yf_s201c_is_data_valid is deprecated. Validity logic moved to application layer.");
	return false;
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

/* New thin API function (introduced during driver refactoring) */
int yf_s201c_get_recent_periods_us(const struct device *dev,
				   int64_t *periods_us,
				   size_t len)
{
	struct yf_s201c_data *data = dev->data;

	if (!periods_us || len == 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	size_t available = MIN(len, (size_t)data->valid_periods);
	size_t start = (data->buffer_index + data->buffer_size - available) % data->buffer_size;

	for (size_t i = 0; i < available; i++) {
		periods_us[i] = data->period_buffer[(start + i) % data->buffer_size];
	}

	k_mutex_unlock(&data->mutex);

	return (int)available;
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