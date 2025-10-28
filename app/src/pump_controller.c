/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <app_version.h>
#include "pump_controller.h"
#include "error_handler.h"

LOG_MODULE_REGISTER(pump_controller, CONFIG_APP_LOG_LEVEL);

/* ============================================================================
 * Pump State Machine Definition
 * ============================================================================ */

/**
 * @brief Pump operational states
 *
 * Defines all possible states the pump can be in during operation.
 * State transitions must follow defined rules and guard conditions.
 */
typedef enum {
    PUMP_STATE_OFF = 0,      /**< Pump is completely off, available for activation */
    PUMP_STATE_STARTING,     /**< Pump is starting up, initializing systems */
    PUMP_STATE_RUNNING,      /**< Pump is actively running and operational */
    PUMP_STATE_TIMEOUT,      /**< Pump timed out and transitioning to safe shutdown */
    PUMP_STATE_ERROR,        /**< Pump encountered an error, requires manual intervention */
    PUMP_STATE_MAINTENANCE,  /**< Pump is in maintenance mode, inhibited from operation */
    PUMP_STATE_COUNT         /**< Total number of pump states (for bounds checking) */
} pump_state_t;

/**
 * @brief State transition events
 *
 * Events that trigger state changes in the pump controller.
 */
typedef enum {
    PUMP_EVENT_PLATEAU_DETECTED = 0,  /**< New plateau detected in flow */
    PUMP_EVENT_TIMEOUT,                /**< Runtime timeout exceeded */
    PUMP_EVENT_SAFETY_TIMEOUT,         /**< Safety timer expired */
    PUMP_EVENT_ERROR_DETECTED,         /**< System or hardware error detected */
    PUMP_EVENT_MAINTENANCE_ENTER,      /**< Enter maintenance mode */
    PUMP_EVENT_MAINTENANCE_EXIT,       /**< Exit maintenance mode */
    PUMP_EVENT_RESET,                  /**< Reset system state */
    PUMP_EVENT_COUNT                   /**< Total number of transition events */
} pump_event_t;

/**
 * @brief Pump controller context
 *
 * Holds all state and runtime information for pump control.
 */
typedef struct {
    pump_state_t current_state;        /**< Current operational state */
    pump_state_t previous_state;       /**< Previous state for recovery */
    int64_t initial_plateau_period;    /**< Plateau period when pump started */
    int64_t latest_plateau_period;     /**< Latest plateau period detected */
    bool safety_systems_active;        /**< Safety mechanisms status */
    k_timepoint_t state_entry_time;    /**< When current state was entered */
} pump_controller_ctx_t;

/* ============================================================================
 * External Variable Definitions (Phase 3 compatibility)
 * ============================================================================ */

bool pump_on = false;
int64_t initial_plateau_period = 0;
int64_t latest_plateau_period = 0;
const struct device *gpio_dev = NULL;
const int64_t PUMP_SAFETY_TIMEOUT_MIN = 5;

/* ============================================================================
 * Internal Variables
 * ============================================================================ */

static pump_controller_ctx_t pump_ctx;
static struct k_timer safety_timer;
static K_MUTEX_DEFINE(pump_mutex);

/* ============================================================================
 * State Transition Table
 *
 * Defines valid transitions between states, indexed by [current_state][event].
 * Returns target state, or PUMP_STATE_COUNT if transition is invalid.
 * ============================================================================ */

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

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

static void safety_timer_handler(struct k_timer *timer_id);
static int execute_state_transition(pump_state_t new_state);
static const char *pump_state_to_string(pump_state_t state);
static const char *pump_event_to_string(pump_event_t event);

/* ============================================================================
 * Safety Timer Handler
 * ============================================================================ */

/**
 * @brief Safety timer callback to prevent pump from running indefinitely
 *
 * This critical safety mechanism automatically shuts down the pump if it has
 * been running continuously for the maximum allowed time. It provides hardware
 * protection against system failures that could leave the pump running unattended.
 *
 * @param timer_id Timer identifier (unused)
 */
static void safety_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);

    LOG_WRN("Safety timer expired - executing emergency pump shutdown");

    k_mutex_lock(&pump_mutex, K_FOREVER);
    int ret = execute_state_transition(PUMP_STATE_OFF);
    k_mutex_unlock(&pump_mutex);

    if (ret < 0) {
        LOG_ERR("Critical: Safety timer failed to shut down pump (%d)", ret);
        /* Last resort: direct GPIO manipulation if state machine fails */
        gpio_pin_set(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, 1);
    }
}

/* ============================================================================
 * State Persistence Functions - TODO for Phase 3 Completion
 * ============================================================================ */

/**
 * TODO: State persistence implementation requires CONFIG_SETTINGS=y in prj.conf
 *
 * When settings subsystem is properly configured, implement:
 *
 * typedef struct {
 *     pump_state_t current_state;
 *     int64_t initial_plateau_period;
 *     int64_t latest_plateau_period;
 *     uint32_t state_timestamp;
 *     uint32_t checksum;
 * } pump_state_persistence_t;
 *
 * #define PUMP_SETTINGS_KEY "pump/state"
 *
 * static int save_pump_state(void) { ... }  // Use settings_save_one()
 * static int load_pump_state(void) { ... }  // Use settings_get()
 *
 * Add calls in init: settings_subsys_init(), load_pump_state()
 * Add calls in transition: save_pump_state() for significant state changes
 */

/* ============================================================================
 * State Machine Engine
 * ============================================================================ */

/**
 * @brief Convert pump state to human-readable string
 *
 * @param state Pump state to convert
 * @return String representation of the state
 */
static const char *pump_state_to_string(pump_state_t state)
{
    static const char *state_strings[] = {
        "OFF", "STARTING", "RUNNING", "TIMEOUT", "ERROR", "MAINTENANCE"
    };

    if (state >= PUMP_STATE_COUNT) {
        return "INVALID";
    }

    return state_strings[state];
}

/**
 * @brief Convert pump event to human-readable string
 *
 * @param event Pump event to convert
 * @return String representation of the event
 */
static const char *pump_event_to_string(pump_event_t event)
{
    static const char *event_strings[] = {
        "PLATEAU_DETECTED", "TIMEOUT", "SAFETY_TIMEOUT", "ERROR_DETECTED",
        "MAINTENANCE_ENTER", "MAINTENANCE_EXIT", "RESET"
    };

    if (event >= PUMP_EVENT_COUNT) {
        return "INVALID";
    }

    return event_strings[event];
}

/**
 * @brief Execute state transition
 *
 * Performs the actual state change and associated actions. This function
 * must be called with pump_mutex held.
 *
 * @param new_state Target state to transition to
 * @return 0 on success, negative error code on failure
 */
static int execute_state_transition(pump_state_t new_state)
{
    pump_state_t old_state = pump_ctx.current_state;
    int ret = 0;

    /* Validate state bounds */
    if (new_state >= PUMP_STATE_COUNT) {
        LOG_ERR("Invalid state transition attempt to %d", new_state);
        ERROR_REPORT_CRITICAL(ERROR_PUMP_INVALID_STATE);
        return -EINVAL;
    }

    /* Prevent no-op transitions */
    if (new_state == old_state) {
        LOG_DBG("No-op state transition to %s", pump_state_to_string(new_state));
        return 0;
    }

    LOG_INF("State transition: %s -> %s", pump_state_to_string(old_state),
             pump_state_to_string(new_state));

    /* Exit actions for current state */
    switch (old_state) {
    case PUMP_STATE_STARTING:
        /* Startup incomplete */
        break;

    case PUMP_STATE_RUNNING:
        /* Stop safety timer */
        k_timer_stop(&safety_timer);
        pump_on = false;
        /* Force pump relay OFF */
        ret = gpio_pin_set(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, 1);
        if (ret < 0) {
            LOG_ERR("Failed to set pump relay OFF during transition (%d)", ret);
        }
        break;

    case PUMP_STATE_TIMEOUT:
        /* Timeout cleanup */
        pump_on = false;
        break;

    case PUMP_STATE_ERROR:
        /* Error state cleanup */
        pump_on = false;
        break;

    case PUMP_STATE_MAINTENANCE:
        /* Maintenance mode exit */
        break;

    default:
        /* OFF state has no exit actions */
        break;
    }

    /* Update state tracking */
    pump_ctx.previous_state = old_state;
    pump_ctx.current_state = new_state;
    pump_ctx.state_entry_time = sys_timepoint_calc(K_NO_WAIT);

    /* Entry actions for new state */
    switch (new_state) {
    case PUMP_STATE_STARTING:
        /* Initialize startup process */
        pump_on = false;
        break;

    case PUMP_STATE_RUNNING:
        /* Start pump and safety timer */
        ret = gpio_pin_set(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, 0); // ON
        if (ret < 0) {
            LOG_ERR("Failed to set pump relay ON during transition (%d)", ret);
            /* Emergency transition to error state */
            pump_ctx.current_state = PUMP_STATE_ERROR;
            return ret;
        }
        pump_on = true;
        k_timer_start(&safety_timer, K_MINUTES(PUMP_SAFETY_TIMEOUT_MIN), K_NO_WAIT);
        break;

    case PUMP_STATE_TIMEOUT:
        /* Enter timeout state - pump should already be off */
        pump_on = false;
        break;

    case PUMP_STATE_ERROR:
        /* Enter error state */
        pump_on = false;
        /* Stop safety timer to prevent conflicts */
        k_timer_stop(&safety_timer);
        break;

    case PUMP_STATE_MAINTENANCE:
        /* Enter maintenance mode */
        pump_on = false;
        k_timer_stop(&safety_timer);
        break;

    default:
        /* OFF state entry action: ensure pump is off */
        pump_on = false;
        break;
    }

    LOG_INF("State transition completed to %s", pump_state_to_string(new_state));

    /* TODO: State persistence - add save_pump_state() call here when
     * settings subsystem is enabled
     */

    return ret;
}

/**
 * @brief Process state transition event
 *
 * Main entry point for triggering state machine events. Validates the transition
 * and executes it if valid.
 *
 * @param event Event triggering the state transition
 * @return 0 on success, negative error code on failure
 */
static int process_event(pump_event_t event)
{
    if (event >= PUMP_EVENT_COUNT) {
        LOG_ERR("Invalid event %d", event);
        return -EINVAL;
    }

    k_mutex_lock(&pump_mutex, K_FOREVER);

    pump_state_t current_state = pump_ctx.current_state;
    pump_state_t target_state = state_transition_table[current_state][event];

    if (target_state >= PUMP_STATE_COUNT) {
        k_mutex_unlock(&pump_mutex);
        LOG_WRN("Invalid transition %s + %s", pump_state_to_string(current_state),
                 pump_event_to_string(event));
        return -EPERM;
    }

    LOG_DBG("Processing event %s in state %s -> target %s",
             pump_event_to_string(event), pump_state_to_string(current_state),
             pump_state_to_string(target_state));

    int ret = execute_state_transition(target_state);
    k_mutex_unlock(&pump_mutex);

    return ret;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int pump_controller_init(void)
{
    int ret;

    /* TODO: State persistence requires CONFIG_SETTINGS=y in prj.conf
     * and implementation of flash-based storage functions
     *
     * ret = settings_subsys_init();
     * if (ret < 0) {
     *     LOG_WRN("Settings subsystem initialization failed (%d)", ret);
     * }
     * load_pump_state();
     */

    /* Initialize state machine context */
    pump_ctx.current_state = PUMP_STATE_OFF;
    pump_ctx.previous_state = PUMP_STATE_OFF;
    pump_ctx.initial_plateau_period = 0;
    pump_ctx.latest_plateau_period = 0;
    pump_ctx.safety_systems_active = true;
    pump_ctx.state_entry_time = sys_timepoint_calc(K_NO_WAIT);

    /* Get GPIO device */
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready for pump controller");
        return -ENODEV;
    }

    /* Configure pump relay GPIO */
    ret = gpio_pin_configure(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Could not configure pump relay GPIO (%d)", ret);
        ERROR_REPORT_CRITICAL(ERROR_GPIO_CONFIG_FAILED);
        return ret;
    }

    /* Ensure pump starts OFF */
    ret = gpio_pin_set(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, 1);
    if (ret < 0) {
        LOG_ERR("Could not set initial pump relay state (%d)", ret);
        ERROR_REPORT_CRITICAL(ERROR_GPIO_SET_FAILED);
        return ret;
    }

    /* Initialize safety timer */
    k_timer_init(&safety_timer, safety_timer_handler, NULL);

    LOG_INF("Pump controller initialized (state: %s)",
             pump_state_to_string(pump_ctx.current_state));

    /* If we recovered a running state, we may need to check if pump should restart */
    /* For safety, we require manual restart after power cycle */
    if (pump_ctx.current_state == PUMP_STATE_RUNNING) {
        LOG_WRN("Recovered RUNNING state - requiring manual restart for safety");
        pump_ctx.current_state = PUMP_STATE_ERROR; /* Force to error state */
    }

    return 0;
}

int pump_controller_turn_on(int64_t plateau_period_us)
{
    if (plateau_period_us <= 0) {
        return -EINVAL;
    }

    k_mutex_lock(&pump_mutex, K_FOREVER);

    /* Validate current state allows startup */
    if (pump_ctx.current_state != PUMP_STATE_OFF &&
        pump_ctx.current_state != PUMP_STATE_STARTING) {
        k_mutex_unlock(&pump_mutex);
        LOG_WRN("Cannot turn on pump from state %s",
                 pump_state_to_string(pump_ctx.current_state));
        return -EPERM;
    }

    /* Update plateau tracking */
    pump_ctx.initial_plateau_period = plateau_period_us;
    pump_ctx.latest_plateau_period = plateau_period_us;
    initial_plateau_period = plateau_period_us; // External compatibility
    latest_plateau_period = plateau_period_us;

    k_mutex_unlock(&pump_mutex);

    /* Process plateau detected event to start pump */
    return process_event(PUMP_EVENT_PLATEAU_DETECTED);
}

int pump_controller_turn_off(void)
{
    /* Use reset event to force safe shutdown */
    return process_event(PUMP_EVENT_RESET);
}

bool pump_controller_is_on(void)
{
    k_mutex_lock(&pump_mutex, K_FOREVER);
    bool is_on = (pump_ctx.current_state == PUMP_STATE_RUNNING);
    k_mutex_unlock(&pump_mutex);

    return is_on;
}

void pump_controller_update_plateau_period(int64_t period_us)
{
    if (period_us <= 0) {
        return;
    }

    k_mutex_lock(&pump_mutex, K_FOREVER);
    pump_ctx.latest_plateau_period = period_us;
    latest_plateau_period = period_us; // External compatibility

    /* If currently running, this could trigger plateau update */
    if (pump_ctx.current_state == PUMP_STATE_RUNNING) {
        /* Process plateau detected to potentially update timer */
        process_event(PUMP_EVENT_PLATEAU_DETECTED);
    }
    k_mutex_unlock(&pump_mutex);
}

bool pump_controller_safety_check(void)
{
    k_mutex_lock(&pump_mutex, K_FOREVER);
    bool safety_ok = pump_ctx.safety_systems_active;
    k_mutex_unlock(&pump_mutex);

    return safety_ok;
}

void pump_controller_reset(void)
{
    /* Force reset event */
    process_event(PUMP_EVENT_RESET);
}

int pump_controller_emergency_stop(void)
{
    LOG_WRN("Emergency stop requested");

    k_mutex_lock(&pump_mutex, K_FOREVER);
    int ret = execute_state_transition(PUMP_STATE_OFF);
    k_mutex_unlock(&pump_mutex);

    return ret;
}

int pump_controller_cleanup(void)
{
    LOG_INF("Cleaning up pump controller");

    /* Stop timer and ensure pump is off */
    k_timer_stop(&safety_timer);
    pump_on = false;

    if (gpio_dev != NULL) {
        gpio_pin_set(gpio_dev, PUMP_CONTROLLER_RELAY_PIN, 1);
    }

    /* Reset state */
    k_mutex_lock(&pump_mutex, K_FOREVER);
    pump_ctx.current_state = PUMP_STATE_OFF;
    pump_ctx.previous_state = PUMP_STATE_OFF;
    pump_ctx.safety_systems_active = false;
    k_mutex_unlock(&pump_mutex);

    return 0;
}
