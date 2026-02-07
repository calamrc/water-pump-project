/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "safety_monitor.h"
#include "error_handler.h"

LOG_MODULE_REGISTER(safety_monitor, CONFIG_APP_LOG_LEVEL);

/* ============================================================================
 * Internal Constants and Limits
 * ============================================================================ */

#define SAFETY_MONITOR_MAX_CALLBACKS 4
#define SAFETY_MONITOR_CHECK_STACK_SIZE 1024
#define SAFETY_MONITOR_CHECK_PRIORITY -2

/* ============================================================================
 * Internal Data Structures
 * ============================================================================ */

/**
 * @brief Safety check callback registration
 */
typedef struct {
    safety_check_callback_t callback;
    void *user_data;
    bool active;
} safety_check_registration_t;

/**
 * @brief Safety violation callback registration
 */
typedef struct {
    safety_violation_callback_t callback;
    void *user_data;
    bool active;
} safety_violation_registration_t;

/**
 * @brief Safety monitor context
 */
typedef struct {
    safety_monitor_config_t config;
    safety_statistics_t stats;
    safety_status_t current_status;
    bool initialized;
    bool paused;
    k_tid_t check_thread;
    struct k_thread check_thread_data;
    k_thread_stack_t check_stack[SAFETY_MONITOR_CHECK_STACK_SIZE];
} safety_monitor_ctx_t;

/* ============================================================================
 * Internal Variables
 * ============================================================================ */

static safety_monitor_ctx_t safety_ctx;
static safety_check_registration_t check_callbacks[SAFETY_CHECK_COUNT];
static safety_violation_registration_t violation_callbacks[SAFETY_MONITOR_MAX_CALLBACKS];

static K_MUTEX_DEFINE(safety_mutex);
static K_TIMER_DEFINE(safety_check_timer, NULL, NULL);

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Update safety status based on violations
 */
static void update_safety_status(void)
{
    uint32_t total_violations = safety_ctx.stats.failed_checks;

    if (total_violations >= safety_ctx.config.emergency_threshold) {
        safety_ctx.current_status = SAFETY_STATUS_EMERGENCY;
    } else if (total_violations >= safety_ctx.config.critical_violation_threshold) {
        safety_ctx.current_status = SAFETY_STATUS_CRITICAL;
    } else if (total_violations > 0) {
        safety_ctx.current_status = SAFETY_STATUS_DEGRADED;
    } else {
        safety_ctx.current_status = SAFETY_STATUS_HEALTHY;
    }
}

/**
 * @brief Execute a single safety check
 */
static int execute_safety_check(safety_check_type_t check_type,
                               safety_check_result_t *result)
{
    if (check_type >= SAFETY_CHECK_COUNT || !result) {
        return -EINVAL;
    }

    memset(result, 0, sizeof(safety_check_result_t));
    result->check_type = check_type;
    result->timestamp = k_uptime_get_32();

    int64_t start_time = k_uptime_get();

    /* Execute the check callback */
    if (check_callbacks[check_type].active) {
        int ret = check_callbacks[check_type].callback(
            check_type, result, check_callbacks[check_type].user_data);

        if (ret < 0) {
            result->passed = false;
            result->violation = SAFETY_VIOLATION_SOFTWARE_ERROR;
            snprintf(result->description, sizeof(result->description),
                    "Check callback failed: %d", ret);
        }
    } else {
        /* No callback registered for this check type */
        result->passed = true; /* Consider as passed if not implemented */
        result->violation = SAFETY_VIOLATION_NONE;
        snprintf(result->description, sizeof(result->description),
                "Check not implemented");
    }

    result->response_time_ms = k_uptime_get() - start_time;

    /* Check for timeout */
    if (result->response_time_ms > SAFETY_MONITOR_TIMEOUT_MS) {
        result->passed = false;
        result->violation = SAFETY_VIOLATION_TIMEOUT;
        snprintf(result->description, sizeof(result->description),
                "Check timed out: %d ms", (int)result->response_time_ms);
    }

    return 0;
}

/**
 * @brief Safety check thread function
 */
static void safety_check_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true) {
        k_timer_status_sync(&safety_check_timer);

        if (safety_ctx.paused) {
            continue;
        }

        k_mutex_lock(&safety_mutex, K_FOREVER);

        /* Perform all safety checks */
        safety_check_result_t results[SAFETY_CHECK_COUNT];
        uint32_t check_count = 0;

        for (int i = 0; i < SAFETY_CHECK_COUNT; i++) {
            if (execute_safety_check((safety_check_type_t)i, &results[check_count]) == 0) {
                check_count++;
            }
        }

        /* Update statistics */
        safety_ctx.stats.total_checks += check_count;
        safety_ctx.stats.last_check_time = k_uptime_get_32();

        for (uint32_t i = 0; i < check_count; i++) {
            if (results[i].passed) {
                safety_ctx.stats.passed_checks++;
            } else {
                safety_ctx.stats.failed_checks++;
                safety_ctx.stats.critical_violations +=
                    (results[i].violation >= SAFETY_VIOLATION_HARDWARE_FAILURE);

                /* Notify violation callbacks */
                for (int j = 0; j < SAFETY_MONITOR_MAX_CALLBACKS; j++) {
                    if (violation_callbacks[j].active) {
                        uint8_t severity = (results[i].violation >= SAFETY_VIOLATION_HARDWARE_FAILURE) ? 4 : 2;
                        violation_callbacks[j].callback(
                            results[i].violation, severity, results[i].description,
                            violation_callbacks[j].user_data);
                    }
                }
            }
        }

        /* Update status */
        update_safety_status();

        /* Emergency action if needed */
        if (safety_ctx.current_status == SAFETY_STATUS_EMERGENCY &&
            safety_ctx.config.emergency_stop_enabled) {
            safety_ctx.stats.emergency_events++;
            LOG_ERR("EMERGENCY SAFETY STOP TRIGGERED");

            /* This would trigger system-wide emergency stop */
            /* Implementation depends on system architecture */
        }

        k_mutex_unlock(&safety_mutex);

        /* Schedule next check */
        k_timer_start(&safety_check_timer,
                     K_MSEC(safety_ctx.config.check_interval_ms), K_NO_WAIT);
    }
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int safety_monitor_init(void)
{
    if (safety_ctx.initialized) {
        return -EALREADY;
    }

    /* Initialize context */
    memset(&safety_ctx, 0, sizeof(safety_monitor_ctx_t));
    memset(check_callbacks, 0, sizeof(check_callbacks));
    memset(violation_callbacks, 0, sizeof(violation_callbacks));

    /* Set default configuration */
    safety_monitor_create_default_config(&safety_ctx.config);

    /* Create safety check thread */
    safety_ctx.check_thread = k_thread_create(&safety_ctx.check_thread_data,
                                             safety_ctx.check_stack,
                                             sizeof(safety_ctx.check_stack),
                                             safety_check_thread,
                                             NULL, NULL, NULL,
                                             SAFETY_MONITOR_CHECK_PRIORITY,
                                             0, K_NO_WAIT);

    if (!safety_ctx.check_thread) {
        LOG_ERR("Failed to create safety check thread");
        return -EAGAIN;
    }

    safety_ctx.initialized = true;
    safety_ctx.current_status = SAFETY_STATUS_HEALTHY;

    LOG_INF("Safety monitor initialized with %d ms interval",
            safety_ctx.config.check_interval_ms);

    /* Start the first check */
    k_timer_start(&safety_check_timer,
                 K_MSEC(safety_ctx.config.check_interval_ms), K_NO_WAIT);

    return 0;
}

int safety_monitor_configure(const safety_monitor_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);

    /* Validate configuration */
    if (config->check_interval_ms < SAFETY_MONITOR_MAX_INTERVAL_MS &&
        config->check_interval_ms > 0) {
        safety_ctx.config = *config;
    } else {
        k_mutex_unlock(&safety_mutex);
        return -EINVAL;
    }

    k_mutex_unlock(&safety_mutex);

    LOG_INF("Safety monitor reconfigured");
    return 0;
}

int safety_monitor_register_check(safety_check_type_t check_type,
                                 safety_check_callback_t callback,
                                 void *user_data)
{
    if (check_type >= SAFETY_CHECK_COUNT || !callback) {
        return -EINVAL;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);

    check_callbacks[check_type].callback = callback;
    check_callbacks[check_type].user_data = user_data;
    check_callbacks[check_type].active = true;

    k_mutex_unlock(&safety_mutex);

    LOG_DBG("Safety check registered for type %d", check_type);
    return 0;
}

int safety_monitor_register_violation_handler(safety_violation_callback_t callback,
                                             void *user_data)
{
    if (!callback) {
        return -EINVAL;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);

    for (int i = 0; i < SAFETY_MONITOR_MAX_CALLBACKS; i++) {
        if (!violation_callbacks[i].active) {
            violation_callbacks[i].callback = callback;
            violation_callbacks[i].user_data = user_data;
            violation_callbacks[i].active = true;
            k_mutex_unlock(&safety_mutex);
            LOG_DBG("Safety violation handler registered at slot %d", i);
            return i;
        }
    }

    k_mutex_unlock(&safety_mutex);
    return -ENOMEM;
}

int safety_monitor_check_now(safety_check_result_t *results, uint32_t max_results)
{
    if (!results || max_results == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);

    uint32_t result_count = 0;
    for (int i = 0; i < SAFETY_CHECK_COUNT && result_count < max_results; i++) {
        if (execute_safety_check((safety_check_type_t)i, &results[result_count]) == 0) {
            result_count++;
        }
    }

    k_mutex_unlock(&safety_mutex);

    return result_count;
}

safety_status_t safety_monitor_get_status(void)
{
    k_mutex_lock(&safety_mutex, K_FOREVER);
    safety_status_t status = safety_ctx.current_status;
    k_mutex_unlock(&safety_mutex);

    return status;
}

int safety_monitor_get_statistics(safety_statistics_t *stats)
{
    if (!stats) {
        return -EINVAL;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);
    *stats = safety_ctx.stats;
    k_mutex_unlock(&safety_mutex);

    return 0;
}

bool safety_monitor_check_available(safety_check_type_t check_type)
{
    if (check_type >= SAFETY_CHECK_COUNT) {
        return false;
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);
    bool available = check_callbacks[check_type].active;
    k_mutex_unlock(&safety_mutex);

    return available;
}

int safety_monitor_emergency_stop(void)
{
    LOG_WRN("Emergency stop requested");

    k_mutex_lock(&safety_mutex, K_FOREVER);
    safety_ctx.current_status = SAFETY_STATUS_EMERGENCY;
    safety_ctx.stats.emergency_events++;
    k_mutex_unlock(&safety_mutex);

    /* Implementation would trigger system-wide emergency stop */
    /* This depends on the specific system architecture */

    return 0;
}

int safety_monitor_reset_statistics(void)
{
    k_mutex_lock(&safety_mutex, K_FOREVER);
    memset(&safety_ctx.stats, 0, sizeof(safety_statistics_t));
    k_mutex_unlock(&safety_mutex);

    LOG_INF("Safety statistics reset");
    return 0;
}

int safety_monitor_get_last_results(safety_check_result_t *results,
                                   uint32_t max_results, uint32_t *count)
{
    /* This would require storing last results - simplified implementation */
    if (!results || !count || max_results == 0) {
        return -EINVAL;
    }

    *count = 0;
    return 0; /* Not implemented in this basic version */
}

int safety_monitor_pause(void)
{
    k_mutex_lock(&safety_mutex, K_FOREVER);
    safety_ctx.paused = true;
    k_mutex_unlock(&safety_mutex);

    LOG_INF("Safety monitoring paused");
    return 0;
}

int safety_monitor_resume(void)
{
    k_mutex_lock(&safety_mutex, K_FOREVER);
    safety_ctx.paused = false;
    k_mutex_unlock(&safety_mutex);

    LOG_INF("Safety monitoring resumed");
    return 0;
}

bool safety_monitor_is_paused(void)
{
    k_mutex_lock(&safety_mutex, K_FOREVER);
    bool paused = safety_ctx.paused;
    k_mutex_unlock(&safety_mutex);

    return paused;
}

int safety_monitor_cleanup(void)
{
    LOG_INF("Cleaning up safety monitor");

    k_timer_stop(&safety_check_timer);

    if (safety_ctx.check_thread) {
        k_thread_abort(safety_ctx.check_thread);
    }

    k_mutex_lock(&safety_mutex, K_FOREVER);
    safety_ctx.initialized = false;
    k_mutex_unlock(&safety_mutex);

    return 0;
}

const char *safety_status_to_string(safety_status_t status)
{
    static const char *status_strings[] = {
        "HEALTHY", "DEGRADED", "CRITICAL", "EMERGENCY"
    };

    if (status <= SAFETY_STATUS_EMERGENCY) {
        return status_strings[status];
    }
    return "UNKNOWN";
}

const char *safety_check_type_to_string(safety_check_type_t check_type)
{
    static const char *type_strings[] = {
        "HARDWARE", "SOFTWARE", "ENVIRONMENTAL", "COMMUNICATION", "POWER"
    };

    if (check_type < SAFETY_CHECK_COUNT) {
        return type_strings[check_type];
    }
    return "UNKNOWN";
}

const char *safety_violation_to_string(safety_violation_t violation)
{
    static const char *violation_strings[] = {
        "NONE", "TIMEOUT", "OUT_OF_BOUNDS", "HARDWARE_FAILURE",
        "SOFTWARE_ERROR", "RESOURCE_EXHAUSTED", "CONFIGURATION_ERROR"
    };

    if (violation <= SAFETY_VIOLATION_CONFIGURATION_ERROR) {
        return violation_strings[violation];
    }
    return "UNKNOWN";
}

int safety_monitor_create_default_config(safety_monitor_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    config->check_interval_ms = SAFETY_MONITOR_DEFAULT_INTERVAL_MS;
    config->enable_hardware_checks = true;
    config->enable_software_checks = true;
    config->enable_environmental_checks = false;
    config->enable_communication_checks = false;
    config->emergency_stop_enabled = true;
    config->critical_violation_threshold = 3;
    config->emergency_threshold = 5;

    return 0;
}