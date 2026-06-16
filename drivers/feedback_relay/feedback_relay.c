/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file feedback_relay.c
 * @brief Feedback relay driver implementation (auditory click feedback)
 *
 * Controls a relay whose coil click provides auditory feedback during
 * the last 10 seconds of the countdown timer. Simplified GPIO output
 * driver — no complex state machine or safety timer required.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <app/drivers/feedback_relay.h>

#define DT_DRV_COMPAT aygu_feedback_relay

LOG_MODULE_REGISTER(feedback_relay, CONFIG_APP_LOG_LEVEL);

/* Configuration from device tree */
struct feedback_relay_config {
    struct gpio_dt_spec relay;
};

/* Runtime data */
struct feedback_relay_data {
    struct k_mutex mutex;
    bool current_state;               /* true = ON (coil energized) */
    const struct device *dev;         /* back-pointer for work handler */
    struct k_work_delayable off_work; /* delayed work to turn relay OFF */
};

/* Forward declarations */
static int feedback_relay_on_impl(const struct device *dev);
static int feedback_relay_off_impl(const struct device *dev);
static int feedback_relay_pulse_impl(const struct device *dev, uint32_t duration_ms);
static int feedback_relay_click_impl(const struct device *dev);
static int feedback_relay_get_state_impl(const struct device *dev, bool *is_on);

/* Driver API structure (definition from public header) */
static const struct feedback_relay_driver_api feedback_relay_api = {
    .on = feedback_relay_on_impl,
    .off = feedback_relay_off_impl,
    .pulse = feedback_relay_pulse_impl,
    .click = feedback_relay_click_impl,
    .get_state = feedback_relay_get_state_impl,
};

/* Helper: Set relay GPIO state using open-drain + module pull-up.
 *
 * Because the relay module is 5V logic and we drive from 3.3V GPIO,
 * we use open-drain mode:
 *   turn_on = true  → drive LOW  (0) → relay ON
 *   turn_on = false → Hi-Z       (1) → module's 5V pull-up wins → relay OFF
 */
static int set_relay_state(const struct gpio_dt_spec *relay, bool turn_on)
{
    return gpio_pin_set_dt(relay, turn_on ? 0 : 1);
}

/* Work handler: turns the relay OFF after a scheduled pulse duration */
static void feedback_relay_off_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct feedback_relay_data *data = CONTAINER_OF(dwork, struct feedback_relay_data, off_work);

    if (data->dev == NULL) {
        return;
    }

    const struct feedback_relay_config *config = data->dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    int ret = set_relay_state(&config->relay, false);
    if (ret == 0) {
        data->current_state = false;
        LOG_DBG("Feedback relay OFF (delayed work)");
    } else {
        LOG_ERR("Failed to turn feedback relay OFF from work handler (%d)", ret);
    }
    k_mutex_unlock(&data->mutex);
}

/* Driver API implementations */
static int feedback_relay_on_impl(const struct device *dev)
{
    const struct feedback_relay_config *config = dev->config;
    struct feedback_relay_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);

    int ret = set_relay_state(&config->relay, true);
    if (ret == 0) {
        data->current_state = true;
        LOG_DBG("Feedback relay ON");
    } else {
        LOG_ERR("Failed to turn feedback relay ON (%d)", ret);
    }

    k_mutex_unlock(&data->mutex);
    return ret;
}

static int feedback_relay_off_impl(const struct device *dev)
{
    const struct feedback_relay_config *config = dev->config;
    struct feedback_relay_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);

    int ret = set_relay_state(&config->relay, false);
    if (ret == 0) {
        data->current_state = false;
        LOG_DBG("Feedback relay OFF");
    } else {
        LOG_ERR("Failed to turn feedback relay OFF (%d)", ret);
    }

    k_mutex_unlock(&data->mutex);
    return ret;
}

static int feedback_relay_pulse_impl(const struct device *dev, uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return -EINVAL;
    }

    struct feedback_relay_data *data = dev->data;

    /* Cancel any previously scheduled OFF work (new pulse overrides) */
    k_work_cancel_delayable(&data->off_work);

    int ret = feedback_relay_on_impl(dev);
    if (ret < 0) {
        return ret;
    }

    /* Schedule the OFF action asynchronously */
    k_work_schedule(&data->off_work, K_MSEC(duration_ms));

    return 0;
}

static int feedback_relay_click_impl(const struct device *dev)
{
    /* 50ms pulse produces a distinct, short click without excessive coil heating */
    return feedback_relay_pulse_impl(dev, 50);
}

static int feedback_relay_get_state_impl(const struct device *dev, bool *is_on)
{
    if (is_on == NULL) {
        return -EINVAL;
    }

    struct feedback_relay_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    *is_on = data->current_state;
    k_mutex_unlock(&data->mutex);

    return 0;
}

/* Driver initialization */
static int feedback_relay_init(const struct device *dev)
{
    const struct feedback_relay_config *config = dev->config;
    struct feedback_relay_data *data = dev->data;

    LOG_INF("Initializing feedback relay driver");

    /* Check GPIO readiness */
    if (!gpio_is_ready_dt(&config->relay)) {
        LOG_ERR("GPIO device not ready for feedback relay");
        return -ENODEV;
    }

    /* Configure GPIO as output, start inactive (OFF) */
    int ret = gpio_pin_configure_dt(&config->relay, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Could not configure feedback relay GPIO (%d)", ret);
        return ret;
    }

    /* Initialize mutex */
    k_mutex_init(&data->mutex);

    /* Initialize delayed work for async pulse OFF */
    k_work_init_delayable(&data->off_work, feedback_relay_off_work_handler);
    data->dev = dev;

    /* Ensure relay starts OFF */
    ret = set_relay_state(&config->relay, false);
    if (ret < 0) {
        LOG_ERR("Could not set initial feedback relay state (%d)", ret);
        return ret;
    }

    data->current_state = false;

    LOG_INF("Feedback relay driver initialized successfully");
    return 0;
}

/* Device instance definitions */
#define FEEDBACK_RELAY_DEFINE(inst) \
    static struct feedback_relay_data feedback_relay_data_##inst; \
    static const struct feedback_relay_config feedback_relay_config_##inst = { \
        .relay = GPIO_DT_SPEC_INST_GET(inst, gpios), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, feedback_relay_init, NULL, \
                          &feedback_relay_data_##inst, &feedback_relay_config_##inst, \
                          POST_KERNEL, CONFIG_FEEDBACK_RELAY_INIT_PRIORITY, \
                          &feedback_relay_api);

DT_INST_FOREACH_STATUS_OKAY(FEEDBACK_RELAY_DEFINE)
