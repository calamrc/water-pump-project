/*
 * Copyright (c) 2025 Ramon Cristopher Calm
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <stdio.h>
#include "error_handler.h"

LOG_MODULE_REGISTER(error_handler, CONFIG_APP_LOG_LEVEL);

/* ============================================================================
 * Internal Constants and Limits
 * ============================================================================ */

#define ERROR_HANDLER_MAX_CALLBACKS 8
#define ERROR_HANDLER_MAX_RECOVERY_CALLBACKS 4
#define ERROR_CONTEXT_POOL_SIZE 4

/* ============================================================================
 * Internal Data Structures
 * ============================================================================ */

/**
 * @brief Error handler callback registration
 */
typedef struct {
    error_handler_callback_t callback;
    void *user_data;
    int priority;
    bool active;
} error_handler_registration_t;

/**
 * @brief Error recovery callback registration
 */
typedef struct {
    error_code_t pattern;
    error_recovery_callback_t callback;
    void *user_data;
    bool active;
} error_recovery_registration_t;

/**
 * @brief Error context pool entry
 */
typedef struct {
    error_context_t context;
    bool in_use;
} error_context_entry_t;

/**
 * @brief Error handler statistics
 */
typedef struct {
    uint32_t info_count;
    uint32_t warning_count;
    uint32_t error_count;
    uint32_t critical_count;
    uint32_t fatal_count;
    uint32_t last_error_time;
} error_statistics_t;

/* ============================================================================
 * Internal Variables
 * ============================================================================ */

static error_handler_registration_t error_callbacks[ERROR_HANDLER_MAX_CALLBACKS];
static error_recovery_registration_t recovery_callbacks[ERROR_HANDLER_MAX_RECOVERY_CALLBACKS];
static error_context_entry_t context_pool[ERROR_CONTEXT_POOL_SIZE];
static error_statistics_t error_stats;

static error_context_t last_error_context;
static bool error_context_valid = false;

static K_MUTEX_DEFINE(error_mutex);

/* ============================================================================
 * Error Code String Mappings
 * ============================================================================ */

/**
 * @brief Error code to string mapping structure
 */
typedef struct {
    error_code_t code;
    const char *string;
} error_code_mapping_t;

/* Dense array of all error code mappings */
static const error_code_mapping_t error_code_mappings[] = {
    /* Common/System Errors */
    {ERROR_SUCCESS, "SUCCESS"},

    /* GPIO Errors */
    {ERROR_GPIO_INVALID_PIN, "GPIO_INVALID_PIN"},
    {ERROR_GPIO_CONFIG_FAILED, "GPIO_CONFIG_FAILED"},
    {ERROR_GPIO_SET_FAILED, "GPIO_SET_FAILED"},
    {ERROR_GPIO_GET_FAILED, "GPIO_GET_FAILED"},
    {ERROR_GPIO_DEVICE_NOT_READY, "GPIO_DEVICE_NOT_READY"},

    /* Sensor Errors */
    {ERROR_SENSOR_INIT_FAILED, "SENSOR_INIT_FAILED"},
    {ERROR_SENSOR_READ_FAILED, "SENSOR_READ_FAILED"},
    {ERROR_SENSOR_DATA_INVALID, "SENSOR_DATA_INVALID"},
    {ERROR_SENSOR_TIMEOUT, "SENSOR_TIMEOUT"},
    {ERROR_SENSOR_CALIBRATION_FAILED, "SENSOR_CALIBRATION_FAILED"},

    /* Pump Controller Errors */
    {ERROR_PUMP_INVALID_STATE, "PUMP_INVALID_STATE"},
    {ERROR_PUMP_GPIO_FAILED, "PUMP_GPIO_FAILED"},
    {ERROR_PUMP_SAFETY_TIMEOUT, "PUMP_SAFETY_TIMEOUT"},
    {ERROR_PUMP_INVALID_PERIOD, "PUMP_INVALID_PERIOD"},
    {ERROR_PUMP_STATE_PERSISTENCE_FAILED, "PUMP_STATE_PERSISTENCE_FAILED"},

    /* Flow Analyzer Errors */
    {ERROR_FLOW_INVALID_DATA, "FLOW_INVALID_DATA"},
    {ERROR_FLOW_CALCULATION_ERROR, "FLOW_CALCULATION_ERROR"},
    {ERROR_FLOW_STATISTICS_ERROR, "FLOW_STATISTICS_ERROR"},
    {ERROR_FLOW_BUFFER_OVERFLOW, "FLOW_BUFFER_OVERFLOW"},

    /* Configuration Errors */
    {ERROR_CONFIG_INVALID_VALUE, "CONFIG_INVALID_VALUE"},
    {ERROR_CONFIG_DEPENDENCY_FAILED, "CONFIG_DEPENDENCY_FAILED"},
    {ERROR_CONFIG_VALIDATION_FAILED, "CONFIG_VALIDATION_FAILED"},
    {ERROR_CONFIG_CORRUPTION_DETECTED, "CONFIG_CORRUPTION_DETECTED"},

    /* Memory Errors */
    {ERROR_MEMORY_ALLOCATION_FAILED, "MEMORY_ALLOCATION_FAILED"},
    {ERROR_MEMORY_BUFFER_OVERFLOW, "MEMORY_BUFFER_OVERFLOW"},
    {ERROR_MEMORY_NULL_POINTER, "MEMORY_NULL_POINTER"},
    {ERROR_MEMORY_LEAK_DETECTED, "MEMORY_LEAK_DETECTED"},

    /* System Errors */
    {ERROR_SYSTEM_TIMEOUT, "SYSTEM_TIMEOUT"},
    {ERROR_SYSTEM_RESOURCE_EXHAUSTED, "SYSTEM_RESOURCE_EXHAUSTED"},
    {ERROR_SYSTEM_CRITICAL_FAILURE, "SYSTEM_CRITICAL_FAILURE"},
    {ERROR_SYSTEM_RECOVERY_FAILED, "SYSTEM_RECOVERY_FAILED"},

    /* Terminator */
    {ERROR_UNKNOWN, "UNKNOWN_ERROR"}
};

static const char *severity_strings[] = {
    "INFO",
    "WARNING",
    "ERROR",
    "CRITICAL",
    "FATAL"
};

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Get free slot in error handler callback array
 *
 * @return Index of free slot, or -1 if no slots available
 */
static int get_free_callback_slot(void)
{
    for (int i = 0; i < ERROR_HANDLER_MAX_CALLBACKS; i++) {
        if (!error_callbacks[i].active) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Get free slot in error recovery callback array
 *
 * @return Index of free slot, or -1 if no slots available
 */
static int get_free_recovery_slot(void)
{
    for (int i = 0; i < ERROR_HANDLER_MAX_RECOVERY_CALLBACKS; i++) {
        if (!recovery_callbacks[i].active) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Allocate error context from pool
 *
 * @param context Pointer to receive allocated context
 * @return Index of allocated context, or -1 if pool full
 */
static int allocate_error_context(error_context_t **context)
{
    for (int i = 0; i < ERROR_CONTEXT_POOL_SIZE; i++) {
        if (!context_pool[i].in_use) {
            context_pool[i].in_use = true;
            memset(&context_pool[i].context, 0, sizeof(error_context_t));
            *context = &context_pool[i].context;
            return i;
        }
    }
    return -1;
}

/**
 * @brief Free error context back to pool
 *
 * @param context_index Index of context to free
 */
static void free_error_context(int context_index)
{
    if (context_index >= 0 && context_index < ERROR_CONTEXT_POOL_SIZE) {
        context_pool[context_index].in_use = false;
    }
}

/**
 * @brief Update error statistics based on severity
 *
 * @param severity Error severity level
 */
static void update_error_statistics(error_severity_t severity)
{
    k_mutex_lock(&error_mutex, K_FOREVER);

    switch (severity) {
    case ERROR_SEVERITY_INFO:
        error_stats.info_count++;
        break;
    case ERROR_SEVERITY_WARNING:
        error_stats.warning_count++;
        break;
    case ERROR_SEVERITY_ERROR:
        error_stats.error_count++;
        break;
    case ERROR_SEVERITY_CRITICAL:
        error_stats.critical_count++;
        break;
    case ERROR_SEVERITY_FATAL:
        error_stats.fatal_count++;
        break;
    }

    error_stats.last_error_time = k_uptime_get_32();
    k_mutex_unlock(&error_mutex);
}

/**
 * @brief Default error handler callback
 *
 * Provides basic logging and severity-based response.
 *
 * @param context Error context information
 * @param user_data Unused
 * @return 0 on success, negative error code on failure
 */
static int default_error_handler(const error_context_t *context, void *user_data)
{
    ARG_UNUSED(user_data);

    switch (context->severity) {
    case ERROR_SEVERITY_INFO:
        LOG_INF("Error [%s]: %s in %s:%s:%d",
                error_severity_to_string(context->severity),
                error_code_to_string(context->code),
                context->module, context->function, context->line);
        break;

    case ERROR_SEVERITY_WARNING:
        LOG_WRN("Error [%s]: %s in %s:%s:%d",
                error_severity_to_string(context->severity),
                error_code_to_string(context->code),
                context->module, context->function, context->line);
        break;

    case ERROR_SEVERITY_ERROR:
        LOG_ERR("Error [%s]: %s in %s:%s:%d - %s",
                error_severity_to_string(context->severity),
                error_code_to_string(context->code),
                context->module, context->function, context->line,
                context->recoverable ? "Recoverable" : "Non-recoverable");
        break;

    case ERROR_SEVERITY_CRITICAL:
        LOG_ERR("CRITICAL ERROR [%s]: %s in %s:%s:%d - Immediate attention required!",
                error_severity_to_string(context->severity),
                error_code_to_string(context->code),
                context->module, context->function, context->line);
        break;

    case ERROR_SEVERITY_FATAL:
        LOG_ERR("FATAL ERROR [%s]: %s in %s:%s:%d - System shutdown required!",
                error_severity_to_string(context->severity),
                error_code_to_string(context->code),
                context->module, context->function, context->line);
        /* In a real system, this might trigger emergency shutdown */
        break;

    default:
        LOG_WRN("Unknown severity error: %s in %s:%s:%d",
                error_code_to_string(context->code),
                context->module, context->function, context->line);
        break;
    }

    return 0;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int error_handler_init(void)
{
    /* Initialize error statistics */
    memset(&error_stats, 0, sizeof(error_statistics_t));
    memset(&last_error_context, 0, sizeof(error_context_t));

    /* Initialize callback arrays */
    memset(error_callbacks, 0, sizeof(error_callbacks));
    memset(recovery_callbacks, 0, sizeof(recovery_callbacks));
    memset(context_pool, 0, sizeof(context_pool));

    /* Register default error handler */
    int ret = error_handler_register(default_error_handler, NULL, 100);
    if (ret < 0) {
        LOG_ERR("Failed to register default error handler: %d", ret);
        return ret;
    }

    LOG_INF("Error handler initialized successfully");
    return 0;
}

int error_report(error_code_t code, error_severity_t severity,
                 const char *module, const char *function, int line,
                 bool recoverable, const char *recovery_action,
                 void *custom_data, uint32_t custom_data_size)
{
    if (!module || !function) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);

    /* Validate error code */
    if (code == ERROR_SUCCESS) {
        k_mutex_unlock(&error_mutex);
        return 0; /* No error to report */
    }

    /* Allocate error context */
    error_context_t *context;
    int context_index = allocate_error_context(&context);
    if (context_index < 0) {
        LOG_WRN("Error context pool full, dropping error %s", error_code_to_string(code));
        k_mutex_unlock(&error_mutex);
        return -ENOMEM;
    }

    /* Populate error context */
    context->code = code;
    context->severity = severity;
    context->module = module;
    context->function = function;
    context->line = line;
    context->timestamp = k_uptime_get_32();
    context->thread_id = k_current_get();

    /* Gather environmental context */
    context->uptime_ms = k_uptime_get_32();

    /* Stack usage measurement (disabled - requires Zephyr stack symbols) */
    context->stack_usage = 0; /* TODO: Enable when stack symbols available */

    /* Recovery information */
    context->recoverable = recoverable;
    context->recovery_action = recovery_action ? recovery_action : "Unknown";

    /* Custom data (deep copy if needed) */
    if (custom_data && custom_data_size > 0) {
        context->custom_data = custom_data;
        context->custom_data_size = custom_data_size;
    } else {
        context->custom_data = NULL;
        context->custom_data_size = 0;
    }

    /* Save as last error context */
    memcpy(&last_error_context, context, sizeof(error_context_t));
    error_context_valid = true;

    /* Update statistics */
    update_error_statistics(severity);

    /* Execute error handler callbacks */
    int ret = 0;
    for (int i = 0; i < ERROR_HANDLER_MAX_CALLBACKS; i++) {
        if (error_callbacks[i].active) {
            int callback_ret = error_callbacks[i].callback(context, error_callbacks[i].user_data);
            if (callback_ret < 0) {
                LOG_WRN("Error callback %d failed: %d", i, callback_ret);
                ret = callback_ret; /* Return last error */
            }
        }
    }

    /* Check if automatic recovery should be attempted */
    if (context->recoverable && error_recovery_available(context->code)) {
        error_attempt_recovery(context);
    }

    /* Handle fatal errors */
    if (severity == ERROR_SEVERITY_FATAL) {
        LOG_ERR("Fatal error detected - system should shut down");
        /* In a real system, this might call system reset or emergency shutdown */
    }

    k_mutex_unlock(&error_mutex);
    free_error_context(context_index);

    return ret;
}

int error_handler_register(error_handler_callback_t callback,
                          void *user_data, int priority)
{
    if (!callback) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);

    int slot = get_free_callback_slot();
    if (slot < 0) {
        k_mutex_unlock(&error_mutex);
        return -ENOMEM; /* No free slots */
    }

    error_callbacks[slot].callback = callback;
    error_callbacks[slot].user_data = user_data;
    error_callbacks[slot].priority = priority;
    error_callbacks[slot].active = true;

    k_mutex_unlock(&error_mutex);

    LOG_DBG("Error handler registered at slot %d", slot);
    return slot; /* Return slot as handler ID */
}

int error_handler_unregister(int handler_id)
{
    if (handler_id < 0 || handler_id >= ERROR_HANDLER_MAX_CALLBACKS) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);
    error_callbacks[handler_id].active = false;
    k_mutex_unlock(&error_mutex);

    LOG_DBG("Error handler unregistered from slot %d", handler_id);
    return 0;
}

int error_recovery_register(error_code_t error_pattern,
                           error_recovery_callback_t callback,
                           void *user_data)
{
    if (!callback) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);

    int slot = get_free_recovery_slot();
    if (slot < 0) {
        k_mutex_unlock(&error_mutex);
        return -ENOMEM; /* No free slots */
    }

    recovery_callbacks[slot].pattern = error_pattern;
    recovery_callbacks[slot].callback = callback;
    recovery_callbacks[slot].user_data = user_data;
    recovery_callbacks[slot].active = true;

    k_mutex_unlock(&error_mutex);

    LOG_DBG("Error recovery callback registered at slot %d", slot);
    return slot;
}

bool error_recovery_available(error_code_t code)
{
    k_mutex_lock(&error_mutex, K_FOREVER);

    for (int i = 0; i < ERROR_HANDLER_MAX_RECOVERY_CALLBACKS; i++) {
        if (recovery_callbacks[i].active &&
            error_matches_pattern(code, recovery_callbacks[i].pattern)) {
            k_mutex_unlock(&error_mutex);
            return true;
        }
    }

    k_mutex_unlock(&error_mutex);
    return false;
}

int error_attempt_recovery(const error_context_t *context)
{
    if (!context) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);

    for (int i = 0; i < ERROR_HANDLER_MAX_RECOVERY_CALLBACKS; i++) {
        if (recovery_callbacks[i].active &&
            error_matches_pattern(context->code, recovery_callbacks[i].pattern)) {

            LOG_INF("Attempting error recovery for %s", error_code_to_string(context->code));

            int ret = recovery_callbacks[i].callback(context, recovery_callbacks[i].user_data);
            if (ret == 0) {
                LOG_INF("Error recovery successful for %s", error_code_to_string(context->code));
            } else {
                LOG_WRN("Error recovery failed (%d) for %s", ret, error_code_to_string(context->code));
            }

            k_mutex_unlock(&error_mutex);
            return ret;
        }
    }

    k_mutex_unlock(&error_mutex);
    return -ENOENT; /* No recovery handler found */
}

int error_get_last_context(error_context_t *context)
{
    if (!context) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);

    if (!error_context_valid) {
        k_mutex_unlock(&error_mutex);
        return -ENOENT; /* No valid last error */
    }

    memcpy(context, &last_error_context, sizeof(error_context_t));
    k_mutex_unlock(&error_mutex);

    return 0;
}

int error_get_statistics(uint32_t *info_count, uint32_t *warning_count,
                        uint32_t *error_count, uint32_t *critical_count,
                        uint32_t *fatal_count)
{
    if (!info_count || !warning_count || !error_count ||
        !critical_count || !fatal_count) {
        return -EINVAL;
    }

    k_mutex_lock(&error_mutex, K_FOREVER);
    *info_count = error_stats.info_count;
    *warning_count = error_stats.warning_count;
    *error_count = error_stats.error_count;
    *critical_count = error_stats.critical_count;
    *fatal_count = error_stats.fatal_count;
    k_mutex_unlock(&error_mutex);

    return 0;
}

int error_reset_statistics(void)
{
    k_mutex_lock(&error_mutex, K_FOREVER);
    memset(&error_stats, 0, sizeof(error_statistics_t));
    k_mutex_unlock(&error_mutex);

    LOG_INF("Error statistics reset");
    return 0;
}

bool error_system_health_check(void)
{
    k_mutex_lock(&error_mutex, K_FOREVER);
    bool healthy = (error_stats.critical_count == 0 && error_stats.fatal_count == 0);
    k_mutex_unlock(&error_mutex);

    return healthy;
}

int error_handler_cleanup(void)
{
    k_mutex_lock(&error_mutex, K_FOREVER);

    /* Clear all callbacks */
    memset(error_callbacks, 0, sizeof(error_callbacks));
    memset(recovery_callbacks, 0, sizeof(recovery_callbacks));
    memset(context_pool, 0, sizeof(context_pool));

    k_mutex_unlock(&error_mutex);

    LOG_INF("Error handler cleanup completed");
    return 0;
}

const char *error_code_to_string(error_code_t code)
{
    /* Handle special case */
    if (code == ERROR_UNKNOWN) {
        return "UNKNOWN_ERROR";
    }

    /* Look up in mapping table */
    for (size_t i = 0; i < ARRAY_SIZE(error_code_mappings); i++) {
        if (error_code_mappings[i].code == code) {
            return error_code_mappings[i].string;
        }
    }

    return "INVALID_ERROR_CODE";
}

const char *error_severity_to_string(error_severity_t severity)
{
    if (severity < ARRAY_SIZE(severity_strings)) {
        return severity_strings[severity];
    }
    return "UNKNOWN_SEVERITY";
}

uint8_t error_get_module(error_code_t code)
{
    return (code >> 28) & 0x0F;
}

uint8_t error_get_category(error_code_t code)
{
    return (code >> 24) & 0x0F;
}

bool error_matches_pattern(error_code_t code, error_code_t pattern)
{
    /* If pattern is a base value, match module/category */
    if (pattern & 0x0FFFFFFF) {
        return (code & pattern) == pattern;
    }

    /* Exact match for specific error codes */
    return code == pattern;
}
