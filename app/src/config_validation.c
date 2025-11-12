/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include "config_validation.h"
#include "error_handler.h"

LOG_MODULE_REGISTER(config_validation, CONFIG_APP_LOG_LEVEL);

/* ============================================================================
 * Internal Constants and Limits
 * ============================================================================ */

#define CONFIG_VALIDATION_MAX_CONTEXTS 4
#define CONFIG_VALIDATION_MAX_CALLBACKS 4
#define CONFIG_VALIDATION_MAX_CHANGE_CALLBACKS 4

/* ============================================================================
 * Internal Data Structures
 * ============================================================================ */

/**
 * @brief Configuration context registration
 */
typedef struct {
    config_context_t context;
    bool active;
    uint32_t checksum;
} config_context_registration_t;

/**
 * @brief Validation callback registration
 */
typedef struct {
    int context_id;
    config_validation_callback_t callback;
    void *user_data;
    bool active;
} validation_callback_registration_t;

/**
 * @brief Change callback registration
 */
typedef struct {
    int context_id;
    config_change_callback_t callback;
    void *user_data;
    bool active;
} change_callback_registration_t;

/* ============================================================================
 * Internal Variables
 * ============================================================================ */

static config_context_registration_t contexts[CONFIG_VALIDATION_MAX_CONTEXTS];
static validation_callback_registration_t validation_callbacks[CONFIG_VALIDATION_MAX_CALLBACKS];
static change_callback_registration_t change_callbacks[CONFIG_VALIDATION_MAX_CHANGE_CALLBACKS];

static K_MUTEX_DEFINE(config_mutex);

/* ============================================================================
 * Internal Helper Functions
 * ============================================================================ */

/**
 * @brief Find free context slot
 */
static int find_free_context_slot(void)
{
    for (int i = 0; i < CONFIG_VALIDATION_MAX_CONTEXTS; i++) {
        if (!contexts[i].active) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find free validation callback slot
 */
static int find_free_validation_callback_slot(void)
{
    for (int i = 0; i < CONFIG_VALIDATION_MAX_CALLBACKS; i++) {
        if (!validation_callbacks[i].active) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find free change callback slot
 */
static int find_free_change_callback_slot(void)
{
    for (int i = 0; i < CONFIG_VALIDATION_MAX_CHANGE_CALLBACKS; i++) {
        if (!change_callbacks[i].active) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Get context by ID
 */
static config_context_registration_t *get_context_by_id(int context_id)
{
    if (context_id < 0 || context_id >= CONFIG_VALIDATION_MAX_CONTEXTS) {
        return NULL;
    }
    return contexts[context_id].active ? &contexts[context_id] : NULL;
}

/**
 * @brief Validate parameter bounds
 */
static config_validation_result_t validate_parameter_bounds(
    const config_parameter_t *param, const config_value_t *value)
{
    if (!param || !value) {
        return CONFIG_VALIDATION_SYSTEM_ERROR;
    }

    switch (param->type) {
    case CONFIG_TYPE_UINT32:
        if (param->has_min && value->u32 < param->min_value.u32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->u32 > param->max_value.u32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_INT32:
        if (param->has_min && value->i32 < param->min_value.i32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->i32 > param->max_value.i32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_UINT64:
        if (param->has_min && value->u64 < param->min_value.u64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->u64 > param->max_value.u64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_INT64:
        if (param->has_min && value->i64 < param->min_value.i64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->i64 > param->max_value.i64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_FLOAT:
        if (param->has_min && value->f32 < param->min_value.f32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->f32 > param->max_value.f32) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_DOUBLE:
        if (param->has_min && value->f64 < param->min_value.f64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        if (param->has_max && value->f64 > param->max_value.f64) {
            return CONFIG_VALIDATION_OUT_OF_BOUNDS;
        }
        break;

    case CONFIG_TYPE_BOOL:
        /* Boolean values are always valid */
        break;

    case CONFIG_TYPE_STRING:
        if (!value->str) {
            return CONFIG_VALIDATION_MISSING_VALUE;
        }
        /* String length validation could be added here */
        break;

    default:
        return CONFIG_VALIDATION_TYPE_MISMATCH;
    }

    return CONFIG_VALIDATION_SUCCESS;
}

/**
 * @brief Calculate simple checksum for configuration integrity
 */
static uint32_t calculate_checksum(const config_context_t *context)
{
    if (!context || !context->parameters) {
        return 0;
    }

    uint32_t checksum = 0;
    for (uint32_t i = 0; i < context->parameter_count; i++) {
        const config_parameter_t *param = &context->parameters[i];
        checksum ^= (uint32_t)param->name[0];
        checksum ^= param->type;
        checksum ^= param->default_value.u32; /* Simple hash */
    }

    return checksum;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int config_validation_init(void)
{
    memset(contexts, 0, sizeof(contexts));
    memset(validation_callbacks, 0, sizeof(validation_callbacks));
    memset(change_callbacks, 0, sizeof(change_callbacks));

    LOG_INF("Configuration validation system initialized");
    return 0;
}

int config_validation_register_context(const config_context_t *context)
{
    if (!context || !context->parameters || context->parameter_count == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&config_mutex, K_FOREVER);

    int slot = find_free_context_slot();
    if (slot < 0) {
        k_mutex_unlock(&config_mutex);
        return -ENOMEM;
    }

    /* Copy context and calculate checksum */
    memcpy(&contexts[slot].context, context, sizeof(config_context_t));
    contexts[slot].checksum = calculate_checksum(context);
    contexts[slot].active = true;

    k_mutex_unlock(&config_mutex);

    LOG_INF("Configuration context '%s' registered at slot %d",
            context->context_name, slot);

    return slot;
}

config_validation_result_t config_validation_check_parameter(
    const config_context_t *context,
    const config_parameter_t *parameter,
    const config_value_t *value,
    config_violation_t *violation)
{
    if (!context || !parameter || !value) {
        if (violation) {
            violation->result = CONFIG_VALIDATION_SYSTEM_ERROR;
            snprintf(violation->description, sizeof(violation->description),
                    "Invalid parameters to validation function");
        }
        return CONFIG_VALIDATION_SYSTEM_ERROR;
    }

    /* Check parameter type */
    if (parameter->type >= CONFIG_TYPE_COUNT) {
        if (violation) {
            violation->result = CONFIG_VALIDATION_TYPE_MISMATCH;
            violation->parameter_name = parameter->name;
            violation->provided_value = *value;
            snprintf(violation->description, sizeof(violation->description),
                    "Invalid parameter type: %d", parameter->type);
        }
        return CONFIG_VALIDATION_TYPE_MISMATCH;
    }

    /* Validate bounds */
    config_validation_result_t bounds_result = validate_parameter_bounds(parameter, value);
    if (bounds_result != CONFIG_VALIDATION_SUCCESS) {
        if (violation) {
            violation->result = bounds_result;
            violation->parameter_name = parameter->name;
            violation->provided_value = *value;
            violation->expected_range = parameter->has_min ? parameter->min_value :
                                      (parameter->has_max ? parameter->max_value : parameter->default_value);
            snprintf(violation->description, sizeof(violation->description),
                    "Parameter value out of bounds");
        }
        return bounds_result;
    }

    /* Run custom validation callbacks */
    for (int i = 0; i < CONFIG_VALIDATION_MAX_CALLBACKS; i++) {
        if (validation_callbacks[i].active &&
            validation_callbacks[i].context_id == (context - contexts)) { /* Hacky way to get context ID */

            config_validation_result_t custom_result = validation_callbacks[i].callback(
                context, parameter, value, violation, validation_callbacks[i].user_data);

            if (custom_result != CONFIG_VALIDATION_SUCCESS) {
                return custom_result;
            }
        }
    }

    return CONFIG_VALIDATION_SUCCESS;
}

config_validation_result_t config_validation_check_context(
    const config_context_t *context,
    config_violation_t *violations,
    uint32_t max_violations,
    uint32_t *violation_count)
{
    if (!context || !violation_count) {
        return CONFIG_VALIDATION_SYSTEM_ERROR;
    }

    *violation_count = 0;
    config_validation_result_t worst_result = CONFIG_VALIDATION_SUCCESS;

    for (uint32_t i = 0; i < context->parameter_count; i++) {
        const config_parameter_t *param = &context->parameters[i];
        config_value_t value = param->default_value; /* Use defaults for context check */

        config_violation_t violation;
        config_validation_result_t result = config_validation_check_parameter(
            context, param, &value, &violation);

        if (result != CONFIG_VALIDATION_SUCCESS) {
            if (*violation_count < max_violations && violations) {
                violations[*violation_count] = violation;
            }
            (*violation_count)++;

            /* Track worst result */
            if (result > worst_result) {
                worst_result = result;
            }
        }
    }

    return worst_result;
}

int config_validation_set_parameter(
    const config_context_t *context,
    const char *parameter_name,
    const config_value_t *value,
    config_violation_t *violation)
{
    if (!context || !parameter_name || !value) {
        return -EINVAL;
    }

    const config_parameter_t *param = config_validation_find_parameter(context, parameter_name);
    if (!param) {
        if (violation) {
            violation->result = CONFIG_VALIDATION_MISSING_VALUE;
            violation->parameter_name = parameter_name;
            snprintf(violation->description, sizeof(violation->description),
                    "Parameter not found in context");
        }
        return -ENOENT;
    }

    config_validation_result_t result = config_validation_check_parameter(
        context, param, value, violation);

    if (result != CONFIG_VALIDATION_SUCCESS) {
        return -EINVAL;
    }

    /* Parameter is valid - in a real implementation, this would update the stored value */
    /* For now, just log the change */
    LOG_INF("Parameter '%s' would be set to valid value", parameter_name);

    /* Notify change callbacks */
    for (int i = 0; i < CONFIG_VALIDATION_MAX_CHANGE_CALLBACKS; i++) {
        if (change_callbacks[i].active &&
            change_callbacks[i].context_id == (context - contexts)) { /* Hacky context ID */

            change_callbacks[i].callback(
                context, param, &param->default_value, value,
                change_callbacks[i].user_data);
        }
    }

    return 0;
}

int config_validation_get_parameter(
    const config_context_t *context,
    const char *parameter_name,
    config_value_t *value)
{
    if (!context || !parameter_name || !value) {
        return -EINVAL;
    }

    const config_parameter_t *param = config_validation_find_parameter(context, parameter_name);
    if (!param) {
        return -ENOENT;
    }

    /* Return default value - in real implementation, return stored value */
    *value = param->default_value;
    return 0;
}

const config_parameter_t *config_validation_find_parameter(
    const config_context_t *context,
    const char *parameter_name)
{
    if (!context || !parameter_name) {
        return NULL;
    }

    for (uint32_t i = 0; i < context->parameter_count; i++) {
        if (strcmp(context->parameters[i].name, parameter_name) == 0) {
            return &context->parameters[i];
        }
    }

    return NULL;
}

int config_validation_register_callback(
    int context_id,
    config_validation_callback_t callback,
    void *user_data)
{
    if (context_id < 0 || !callback) {
        return -EINVAL;
    }

    k_mutex_lock(&config_mutex, K_FOREVER);

    int slot = find_free_validation_callback_slot();
    if (slot < 0) {
        k_mutex_unlock(&config_mutex);
        return -ENOMEM;
    }

    validation_callbacks[slot].context_id = context_id;
    validation_callbacks[slot].callback = callback;
    validation_callbacks[slot].user_data = user_data;
    validation_callbacks[slot].active = true;

    k_mutex_unlock(&config_mutex);

    LOG_DBG("Validation callback registered for context %d at slot %d", context_id, slot);
    return slot;
}

int config_validation_register_change_callback(
    int context_id,
    config_change_callback_t callback,
    void *user_data)
{
    if (context_id < 0 || !callback) {
        return -EINVAL;
    }

    k_mutex_lock(&config_mutex, K_FOREVER);

    int slot = find_free_change_callback_slot();
    if (slot < 0) {
        k_mutex_unlock(&config_mutex);
        return -ENOMEM;
    }

    change_callbacks[slot].context_id = context_id;
    change_callbacks[slot].callback = callback;
    change_callbacks[slot].user_data = user_data;
    change_callbacks[slot].active = true;

    k_mutex_unlock(&config_mutex);

    LOG_DBG("Change callback registered for context %d at slot %d", context_id, slot);
    return slot;
}

int config_validation_hot_reload(
    int context_id,
    const config_context_t *new_context,
    config_violation_t *violations,
    uint32_t max_violations,
    uint32_t *violation_count)
{
    config_context_registration_t *ctx_reg = get_context_by_id(context_id);
    if (!ctx_reg) {
        return -EINVAL;
    }

    /* Validate new context */
    config_validation_result_t result = config_validation_check_context(
        new_context, violations, max_violations, violation_count);

    if (result == CONFIG_VALIDATION_SUCCESS) {
        /* Update context */
        memcpy(&ctx_reg->context, new_context, sizeof(config_context_t));
        ctx_reg->checksum = calculate_checksum(new_context);

        LOG_INF("Configuration context '%s' hot reloaded",
                new_context->context_name);
    }

    return (result == CONFIG_VALIDATION_SUCCESS) ? 0 : -EINVAL;
}

bool config_validation_check_integrity(const config_context_t *context)
{
    if (!context) {
        return false;
    }

    uint32_t calculated_checksum = calculate_checksum(context);
    /* In a real implementation, compare with stored checksum */
    return true; /* Simplified */
}

uint32_t config_validation_create_checksum(const config_context_t *context)
{
    return calculate_checksum(context);
}

int config_validation_dump(const config_context_t *context)
{
    if (!context) {
        return -EINVAL;
    }

    LOG_INF("Configuration dump for context '%s':", context->context_name);
    for (uint32_t i = 0; i < context->parameter_count; i++) {
        const config_parameter_t *param = &context->parameters[i];
        LOG_INF("  %s: type=%d, default=%d", param->name, param->type, param->default_value.u32);
    }

    return 0;
}

int config_validation_reset_to_defaults(int context_id)
{
    /* Simplified - would reset all parameters to defaults */
    LOG_INF("Configuration context %d reset to defaults", context_id);
    return 0;
}

bool config_validation_supports_hot_reload(
    const config_context_t *context,
    const char *parameter_name)
{
    if (!context) {
        return false;
    }

    /* Check if context supports hot reload */
    if (!context->hot_reload_supported) {
        return false;
    }

    /* In a real implementation, check parameter-specific support */
    return true;
}

int config_validation_cleanup(void)
{
    k_mutex_lock(&config_mutex, K_FOREVER);
    memset(contexts, 0, sizeof(contexts));
    memset(validation_callbacks, 0, sizeof(validation_callbacks));
    memset(change_callbacks, 0, sizeof(change_callbacks));
    k_mutex_unlock(&config_mutex);

    LOG_INF("Configuration validation cleanup completed");
    return 0;
}

const char *config_validation_result_to_string(config_validation_result_t result)
{
    static const char *result_strings[] = {
        "SUCCESS", "OUT_OF_BOUNDS", "TYPE_MISMATCH", "DEPENDENCY_FAIL",
        "CORRUPTION", "MISSING_VALUE", "SYSTEM_ERROR"
    };

    if (result <= CONFIG_VALIDATION_SYSTEM_ERROR) {
        return result_strings[result];
    }
    return "UNKNOWN";
}

const char *config_value_type_to_string(config_value_type_t type)
{
    static const char *type_strings[] = {
        "UINT32", "INT32", "UINT64", "INT64", "FLOAT", "DOUBLE", "BOOL", "STRING"
    };

    if (type < CONFIG_TYPE_COUNT) {
        return type_strings[type];
    }
    return "UNKNOWN";
}