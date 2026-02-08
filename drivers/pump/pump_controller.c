/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "pump_controller.h"

LOG_MODULE_REGISTER(pump_controller);

/* State transition table - copied from original implementation */
static const pump_state_t state_transition_table[PUMP_STATE_COUNT][PUMP_EVENT_COUNT] = {
    /* OFF -> */ {
        PUMP_STATE_STARTING,     /* PLATEAU_DETECTED: valid start */
        PUMP_STATE_OFF,          /* TIMEOUT: already off, ignore */
        PUMP_STATE_OFF,          /* SAFETY_TIMEOUT: already off, ignore */
        PUMP_STATE_ERROR,        /* ERROR_DETECTED: enter error state */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: enter maintenance */
        PUMP_STATE_OFF,          /* MAINTENANCE_EXIT: not in maintenance, ignore */
        PUMP_STATE_OFF           /* RESET: stay off */
    },
    /* STARTING -> */ {
        PUMP_STATE_RUNNING,      /* PLATEAU_DETECTED: transition to running */
        PUMP_STATE_TIMEOUT,      /* TIMEOUT: startup timeout */
        PUMP_STATE_ERROR,        /* SAFETY_TIMEOUT: unexpected safety timeout */
        PUMP_STATE_ERROR,        /* ERROR_DETECTED: startup error */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: maintenance during startup */
        PUMP_STATE_OFF,          /* MAINTENANCE_EXIT: not in maintenance */
        PUMP_STATE_OFF           /* RESET: abort startup */
    },
    /* RUNNING -> */ {
        PUMP_STATE_RUNNING,      /* PLATEAU_DETECTED: update plateau, stay running */
        PUMP_STATE_TIMEOUT,      /* TIMEOUT: runtime timeout */
        PUMP_STATE_OFF,          /* SAFETY_TIMEOUT: emergency stop */
        PUMP_STATE_ERROR,        /* ERROR_DETECTED: runtime error */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: enter maintenance */
        PUMP_STATE_RUNNING,      /* MAINTENANCE_EXIT: not in maintenance, ignore */
        PUMP_STATE_OFF           /* RESET: immediate stop */
    },
    /* TIMEOUT -> */ {
        PUMP_STATE_OFF,          /* PLATEAU_DETECTED: unexpected, go to safe state */
        PUMP_STATE_TIMEOUT,      /* TIMEOUT: stay in timeout */
        PUMP_STATE_OFF,          /* SAFETY_TIMEOUT: emergency timeout */
        PUMP_STATE_ERROR,        /* ERROR_DETECTED: error during timeout */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: maintenance instead */
        PUMP_STATE_TIMEOUT,      /* MAINTENANCE_EXIT: not in maintenance */
        PUMP_STATE_OFF           /* RESET: force to off */
    },
    /* ERROR -> */ {
        PUMP_STATE_ERROR,        /* PLATEAU_DETECTED: ignore in error state */
        PUMP_STATE_ERROR,        /* TIMEOUT: ignore in error state */
        PUMP_STATE_OFF,          /* SAFETY_TIMEOUT: forced shutdown */
        PUMP_STATE_ERROR,        /* ERROR_DETECTED: already in error */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: enter maintenance */
        PUMP_STATE_ERROR,        /* MAINTENANCE_EXIT: not in maintenance */
        PUMP_STATE_OFF           /* RESET: clear error, go to off */
    },
    /* MAINTENANCE -> */ {
        PUMP_STATE_MAINTENANCE,  /* PLATEAU_DETECTED: ignore in maintenance */
        PUMP_STATE_MAINTENANCE,  /* TIMEOUT: ignore in maintenance */
        PUMP_STATE_MAINTENANCE,  /* SAFETY_TIMEOUT: ignore in maintenance */
        PUMP_STATE_MAINTENANCE,  /* ERROR_DETECTED: ignore in maintenance */
        PUMP_STATE_MAINTENANCE,  /* MAINTENANCE_ENTER: already in maintenance */
        PUMP_STATE_OFF,          /* MAINTENANCE_EXIT: resume to off state */
        PUMP_STATE_OFF           /* RESET: force to off */
    }
};

/* Forward declarations */
static void safety_timer_handler(struct k_timer *timer);
static int execute_state_transition(struct pump_data *data, const struct pump_config *config, pump_state_t new_state);
static int set_relay_state(const struct gpio_dt_spec *relay, bool turn_on);

/* Driver API function declarations */
static int pump_controller_turn_on_impl(const struct device *dev, int64_t plateau_period_us);
static int pump_controller_turn_off_impl(const struct device *dev);
static bool pump_controller_is_on_impl(const struct device *dev);
static void pump_controller_update_plateau_period_impl(const struct device *dev, int64_t period_us);
static bool pump_controller_safety_check_impl(const struct device *dev);
static void pump_controller_reset_impl(const struct device *dev);
static int pump_controller_emergency_stop_impl(const struct device *dev);
static int pump_controller_get_state_impl(const struct device *dev, struct pump_state_info *state);
static int pump_controller_set_config_impl(const struct device *dev, const struct pump_config *config);

/* Driver API structure */
static const struct pump_controller_driver_api pump_controller_api = {
    .turn_on = pump_controller_turn_on_impl,
    .turn_off = pump_controller_turn_off_impl,
    .is_on = pump_controller_is_on_impl,
    .update_plateau_period = pump_controller_update_plateau_period_impl,
    .safety_check = pump_controller_safety_check_impl,
    .reset = pump_controller_reset_impl,
    .emergency_stop = pump_controller_emergency_stop_impl,
    .get_state = pump_controller_get_state_impl,
    .set_config = pump_controller_set_config_impl,
};

/* Helper function to set relay state - GPIO API handles active low/high automatically */
static int set_relay_state(const struct gpio_dt_spec *relay, bool turn_on)
{
    /* GPIO API automatically handles polarity inversion based on DT flags
     * For active low relay: turn_on=true -> physical LOW (API inverts 1->0)
     * For active high relay: turn_on=true -> physical HIGH (API keeps 1->1)
     */
    return gpio_pin_set_dt(relay, turn_on ? 1 : 0);
}

/* Safety timer handler */
static void safety_timer_handler(struct k_timer *timer)
{
    /* Get device from timer user data */
    struct pump_data *data = k_timer_user_data_get(timer);

    LOG_WRN("Safety timer expired - executing emergency pump shutdown");

    k_mutex_lock(&data->mutex, K_FOREVER);
    int ret = execute_state_transition(data, NULL, PUMP_STATE_OFF);
    k_mutex_unlock(&data->mutex);

    if (ret < 0) {
        LOG_ERR("Critical: Safety timer failed to shut down pump (%d)", ret);
    }
}

/* State transition execution */
static int execute_state_transition(struct pump_data *data, const struct pump_config *config, pump_state_t new_state)
{
    pump_state_t old_state = data->current_state;
    int ret = 0;

    /* Validate state bounds */
    if (new_state >= PUMP_STATE_COUNT) {
        LOG_ERR("Invalid state transition attempt to %d", new_state);
        return -EINVAL;
    }

    /* Prevent no-op transitions */
    if (new_state == old_state) {
        LOG_DBG("No-op state transition to %s", (new_state == PUMP_STATE_OFF) ? "OFF" :
                (new_state == PUMP_STATE_STARTING) ? "STARTING" :
                (new_state == PUMP_STATE_RUNNING) ? "RUNNING" :
                (new_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
                (new_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE");
        return 0;
    }

    LOG_INF("State transition: %s -> %s",
             (old_state == PUMP_STATE_OFF) ? "OFF" :
             (old_state == PUMP_STATE_STARTING) ? "STARTING" :
             (old_state == PUMP_STATE_RUNNING) ? "RUNNING" :
             (old_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
             (old_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE",
             (new_state == PUMP_STATE_OFF) ? "OFF" :
             (new_state == PUMP_STATE_STARTING) ? "STARTING" :
             (new_state == PUMP_STATE_RUNNING) ? "RUNNING" :
             (new_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
             (new_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE");

    /* Exit actions for current state */
    switch (old_state) {
    case PUMP_STATE_STARTING:
        /* Startup incomplete */
        break;

    case PUMP_STATE_RUNNING:
        /* Stop safety timer */
        k_timer_stop(&data->safety_timer);
        /* Force pump relay OFF */
        if (config && gpio_is_ready_dt(&config->relay)) {
            ret = set_relay_state(&config->relay, false);
            if (ret < 0) {
                LOG_ERR("Failed to set pump relay OFF during transition (%d)", ret);
            }
        }
        break;

    case PUMP_STATE_TIMEOUT:
        /* Timeout cleanup */
        break;

    case PUMP_STATE_ERROR:
        /* Error state cleanup */
        break;

    case PUMP_STATE_MAINTENANCE:
        /* Maintenance mode exit */
        break;

    default:
        /* OFF state has no exit actions */
        break;
    }

    /* Update state tracking */
    data->previous_state = old_state;
    data->current_state = new_state;
    data->state_entry_time = sys_timepoint_calc(K_NO_WAIT);

    /* Entry actions for new state */
    switch (new_state) {
    case PUMP_STATE_STARTING:
        /* Initialize startup process */
        break;

    case PUMP_STATE_RUNNING:
        /* Start pump and safety timer */
        if (config && gpio_is_ready_dt(&config->relay)) {
            ret = set_relay_state(&config->relay, true);
            if (ret < 0) {
                LOG_ERR("Failed to set pump relay ON during transition (%d)", ret);
                /* Emergency transition to error state */
                data->current_state = PUMP_STATE_ERROR;
                return ret;
            }
            k_timer_start(&data->safety_timer, K_MINUTES(config->safety_timeout_min), K_NO_WAIT);
        }
        break;

    case PUMP_STATE_TIMEOUT:
        /* Enter timeout state - pump should already be off */
        break;

    case PUMP_STATE_ERROR:
        /* Enter error state */
        /* Stop safety timer to prevent conflicts */
        k_timer_stop(&data->safety_timer);
        break;

    case PUMP_STATE_MAINTENANCE:
        /* Enter maintenance mode */
        k_timer_stop(&data->safety_timer);
        break;

    default:
        /* OFF state entry action: ensure pump is off */
        break;
    }

    LOG_INF("State transition completed to %s", (new_state == PUMP_STATE_OFF) ? "OFF" :
             (new_state == PUMP_STATE_STARTING) ? "STARTING" :
             (new_state == PUMP_STATE_RUNNING) ? "RUNNING" :
             (new_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
             (new_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE");

    return ret;
}

/* Process state transition event */
static int process_event(struct pump_data *data, const struct pump_config *config, pump_event_t event)
{
    if (event >= PUMP_EVENT_COUNT) {
        LOG_ERR("Invalid event %d", event);
        return -EINVAL;
    }

    pump_state_t current_state = data->current_state;
    pump_state_t target_state = state_transition_table[current_state][event];

    if (target_state >= PUMP_STATE_COUNT) {
        LOG_WRN("Invalid transition %s + event %d",
                (current_state == PUMP_STATE_OFF) ? "OFF" :
                (current_state == PUMP_STATE_STARTING) ? "STARTING" :
                (current_state == PUMP_STATE_RUNNING) ? "RUNNING" :
                (current_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
                (current_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE", event);
        return -EPERM;
    }

    LOG_DBG("Processing event %d in state %s -> target %s", event,
             (current_state == PUMP_STATE_OFF) ? "OFF" :
             (current_state == PUMP_STATE_STARTING) ? "STARTING" :
             (current_state == PUMP_STATE_RUNNING) ? "RUNNING" :
             (current_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
             (current_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE",
             (target_state == PUMP_STATE_OFF) ? "OFF" :
             (target_state == PUMP_STATE_STARTING) ? "STARTING" :
             (target_state == PUMP_STATE_RUNNING) ? "RUNNING" :
             (target_state == PUMP_STATE_TIMEOUT) ? "TIMEOUT" :
             (target_state == PUMP_STATE_ERROR) ? "ERROR" : "MAINTENANCE");

    return execute_state_transition(data, config, target_state);
}

/* Driver initialization */
static int pump_controller_init(const struct device *dev)
{
    const struct pump_config *config = dev->config;
    struct pump_data *data = dev->data;

    LOG_INF("Initializing pump controller driver");

    /* Check GPIO readiness */
    if (!gpio_is_ready_dt(&config->relay)) {
        LOG_ERR("GPIO device not ready for pump controller");
        return -ENODEV;
    }

    /* Configure GPIO pin */
    int ret = gpio_pin_configure_dt(&config->relay, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Could not configure pump relay GPIO (%d)", ret);
        return ret;
    }

    /* Initialize mutex */
    k_mutex_init(&data->mutex);

    /* Initialize timer with user data pointing to device data */
    k_timer_init(&data->safety_timer, safety_timer_handler, NULL);
    k_timer_user_data_set(&data->safety_timer, data);

    /* Initialize state machine context */
    data->current_state = PUMP_STATE_OFF;
    data->previous_state = PUMP_STATE_OFF;
    data->initial_plateau_period = 0;
    data->latest_plateau_period = 0;
    data->safety_systems_active = true;
    data->state_entry_time = sys_timepoint_calc(K_NO_WAIT);

    /* Ensure pump starts OFF */
    ret = set_relay_state(&config->relay, false);
    if (ret < 0) {
        LOG_ERR("Could not set initial pump relay state (%d)", ret);
        return ret;
    }

    LOG_INF("Pump controller driver initialized successfully");
    return 0;
}

/* Driver API implementations */
static int pump_controller_turn_on_impl(const struct device *dev, int64_t plateau_period_us)
{
    if (plateau_period_us <= 0) {
        return -EINVAL;
    }

    const struct pump_config *config = dev->config;
    struct pump_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);

    /* Validate current state allows startup */
    if (data->current_state != PUMP_STATE_OFF && data->current_state != PUMP_STATE_STARTING) {
        k_mutex_unlock(&data->mutex);
        LOG_WRN("Cannot turn on pump from state %d", data->current_state);
        return -EPERM;
    }

    /* Update plateau tracking */
    data->initial_plateau_period = plateau_period_us;
    data->latest_plateau_period = plateau_period_us;

    k_mutex_unlock(&data->mutex);

    /* Process plateau detected event to start pump */
    k_mutex_lock(&data->mutex, K_FOREVER);
    int ret = process_event(data, config, PUMP_EVENT_PLATEAU_DETECTED);
    k_mutex_unlock(&data->mutex);

    return ret;
}

static int pump_controller_turn_off_impl(const struct device *dev)
{
    struct pump_data *data = dev->data;
    const struct pump_config *config = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    int ret = process_event(data, config, PUMP_EVENT_RESET);
    k_mutex_unlock(&data->mutex);

    return ret;
}

static bool pump_controller_is_on_impl(const struct device *dev)
{
    struct pump_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    bool is_on = (data->current_state == PUMP_STATE_RUNNING);
    k_mutex_unlock(&data->mutex);

    return is_on;
}

static void pump_controller_update_plateau_period_impl(const struct device *dev, int64_t period_us)
{
    if (period_us <= 0) {
        return;
    }

    struct pump_data *data = dev->data;
    const struct pump_config *config = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    data->latest_plateau_period = period_us;

    /* If currently running, this could trigger plateau update */
    if (data->current_state == PUMP_STATE_RUNNING) {
        process_event(data, config, PUMP_EVENT_PLATEAU_DETECTED);
    }
    k_mutex_unlock(&data->mutex);
}

static bool pump_controller_safety_check_impl(const struct device *dev)
{
    struct pump_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    bool safety_ok = data->safety_systems_active;
    k_mutex_unlock(&data->mutex);

    return safety_ok;
}

static void pump_controller_reset_impl(const struct device *dev)
{
    struct pump_data *data = dev->data;
    const struct pump_config *config = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    process_event(data, config, PUMP_EVENT_RESET);
    k_mutex_unlock(&data->mutex);
}

static int pump_controller_emergency_stop_impl(const struct device *dev)
{
    LOG_WRN("Emergency stop requested");

    struct pump_data *data = dev->data;
    const struct pump_config *config = dev->config;

    k_mutex_lock(&data->mutex, K_FOREVER);
    int ret = execute_state_transition(data, config, PUMP_STATE_OFF);
    k_mutex_unlock(&data->mutex);

    return ret;
}

static int pump_controller_get_state_impl(const struct device *dev, struct pump_state_info *state)
{
    if (!state) {
        return -EINVAL;
    }

    struct pump_data *data = dev->data;

    k_mutex_lock(&data->mutex, K_FOREVER);
    state->current_state = data->current_state;
    state->previous_state = data->previous_state;
    state->initial_plateau_period = data->initial_plateau_period;
    state->latest_plateau_period = data->latest_plateau_period;
    state->safety_systems_active = data->safety_systems_active;
    state->state_entry_time = data->state_entry_time;
    k_mutex_unlock(&data->mutex);

    return 0;
}

static int pump_controller_set_config_impl(const struct device *dev, const struct pump_config *config)
{
    /* Configuration is read-only after initialization */
    /* Could be extended to allow runtime configuration changes */
    return -ENOTSUP;
}

/* Device instance definitions */
#define DT_DRV_COMPAT aygu_pump_controller

#define PUMP_CONTROLLER_DEFINE(inst) \
    static struct pump_data pump_data_##inst; \
    static const struct pump_config pump_config_##inst = { \
        .relay = GPIO_DT_SPEC_INST_GET(inst, relay_gpios), \
        .safety_timeout_min = DT_INST_PROP(inst, safety_timeout_min), \
    }; \
    \
    DEVICE_DT_INST_DEFINE(inst, pump_controller_init, NULL, \
                         &pump_data_##inst, &pump_config_##inst, \
                         POST_KERNEL, CONFIG_PUMP_CONTROLLER_INIT_PRIORITY, &pump_controller_api);

DT_INST_FOREACH_STATUS_OKAY(PUMP_CONTROLLER_DEFINE)