/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONFIG_VALIDATION_H
#define CONFIG_VALIDATION_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration Types and Constants
 * ============================================================================ */

/**
 * @brief Configuration parameter data types
 */
typedef enum {
    CONFIG_TYPE_UINT32 = 0,   /**< 32-bit unsigned integer */
    CONFIG_TYPE_INT32,        /**< 32-bit signed integer */
    CONFIG_TYPE_UINT64,       /**< 64-bit unsigned integer */
    CONFIG_TYPE_INT64,        /**< 64-bit signed integer */
    CONFIG_TYPE_FLOAT,        /**< Single precision float */
    CONFIG_TYPE_DOUBLE,       /**< Double precision float */
    CONFIG_TYPE_BOOL,         /**< Boolean value */
    CONFIG_TYPE_STRING,       /**< Null-terminated string */
    CONFIG_TYPE_COUNT         /**< Total number of types */
} config_value_type_t;

/* ============================================================================
 * Configuration Value Union
 * ============================================================================ */

/**
 * @brief Union to hold different types of configuration values
 */
typedef union {
    uint32_t u32;             /**< Unsigned 32-bit integer */
    int32_t i32;              /**< Signed 32-bit integer */
    uint64_t u64;             /**< Unsigned 64-bit integer */
    int64_t i64;              /**< Signed 64-bit integer */
    float f32;                /**< Single precision float */
    double f64;               /**< Double precision float */
    bool b;                   /**< Boolean value */
    const char *str;          /**< String pointer */
} config_value_t;

/* ============================================================================
 * Configuration Parameter Definition
 * ============================================================================ */

/**
 * @brief Configuration parameter descriptor
 */
typedef struct {
    const char *name;                  /**< Parameter name/identifier */
    config_value_type_t type;          /**< Data type of the parameter */
    config_value_t default_value;      /**< Default value */
    config_value_t min_value;          /**< Minimum allowed value */
    config_value_t max_value;          /**< Maximum allowed value */
    bool has_min;                      /**< Whether minimum value is enforced */
    bool has_max;                      /**< Whether maximum value is enforced */
    const char *description;           /**< Human-readable description */
    const char *units;                 /**< Units of measurement */
} config_parameter_t;

/* ============================================================================
 * Configuration Validation Result
 * ============================================================================ */

/**
 * @brief Result of configuration validation
 */
typedef enum {
    CONFIG_VALIDATION_SUCCESS = 0,    /**< Configuration is valid */
    CONFIG_VALIDATION_OUT_OF_BOUNDS,  /**< Value exceeds allowed range */
    CONFIG_VALIDATION_TYPE_MISMATCH,  /**< Wrong data type for parameter */
    CONFIG_VALIDATION_DEPENDENCY_FAIL,/**< Parameter dependencies not satisfied */
    CONFIG_VALIDATION_CORRUPTION,     /**< Configuration appears corrupted */
    CONFIG_VALIDATION_MISSING_VALUE,  /**< Required parameter is missing */
    CONFIG_VALIDATION_SYSTEM_ERROR    /**< Validation system error */
} config_validation_result_t;

/* ============================================================================
 * Configuration Violation Structure
 * ============================================================================ */

/**
 * @brief Details about a configuration violation
 */
typedef struct {
    config_validation_result_t result; /**< Validation result code */
    const char *parameter_name;         /**< Name of the problematic parameter */
    config_value_t provided_value;      /**< Value that was provided */
    config_value_t expected_range;      /**< Expected value/range information */
    const char *description;            /**< Human-readable error description */
    bool is_critical;                   /**< Whether this violation is critical */
} config_violation_t;

/* ============================================================================
 * Configuration Context Structure
 * ============================================================================ */

/**
 * @brief Runtime configuration context
 */
typedef struct {
    const config_parameter_t *parameters;  /**< Array of parameter definitions */
    uint32_t parameter_count;              /**< Number of parameters in array */
    const char *context_name;              /**< Name of configuration context */
    bool hot_reload_supported;             /**< Whether hot reloading is allowed */
    uint32_t checksum;                     /**< Configuration integrity checksum */
} config_context_t;

/* ============================================================================
 * Callback Function Types
 * ============================================================================ */

/**
 * @brief Configuration validation callback
 *
 * Allows custom validation logic beyond basic bounds checking.
 *
 * @param context Configuration context being validated
 * @param parameter Parameter being validated
 * @param value Value being checked
 * @param violation Pointer to store violation details if validation fails
 * @param user_data User-provided context data
 * @return CONFIG_VALIDATION_SUCCESS if valid, error code otherwise
 */
typedef config_validation_result_t (*config_validation_callback_t)(
    const config_context_t *context,
    const config_parameter_t *parameter,
    const config_value_t *value,
    config_violation_t *violation,
    void *user_data);

/**
 * @brief Configuration change callback
 *
 * Called when a configuration parameter changes value.
 *
 * @param context Configuration context
 * @param parameter Parameter that changed
 * @param old_value Previous value
 * @param new_value New value
 * @param user_data User-provided context data
 * @return 0 on success, negative error code on failure
 */
typedef int (*config_change_callback_t)(
    const config_context_t *context,
    const config_parameter_t *parameter,
    const config_value_t *old_value,
    const config_value_t *new_value,
    void *user_data);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize configuration validation system
 *
 * Sets up the configuration validation infrastructure.
 *
 * @return 0 on success, negative error code on failure
 */
int config_validation_init(void);

/**
 * @brief Register a configuration context
 *
 * Registers a set of configuration parameters for validation.
 *
 * @param context Pointer to configuration context descriptor
 * @return Context ID on success, negative error code on failure
 */
int config_validation_register_context(const config_context_t *context);

/**
 * @brief Validate a configuration parameter value
 *
 * Performs bounds checking and custom validation on a parameter.
 *
 * @param context Configuration context
 * @param parameter Parameter definition
 * @param value Value to validate
 * @param violation Pointer to store violation details (can be NULL)
 * @return CONFIG_VALIDATION_SUCCESS if valid, error code otherwise
 */
config_validation_result_t config_validation_check_parameter(
    const config_context_t *context,
    const config_parameter_t *parameter,
    const config_value_t *value,
    config_violation_t *violation);

/**
 * @brief Validate entire configuration context
 *
 * Validates all parameters in a configuration context.
 *
 * @param context Configuration context to validate
 * @param violations Array to store violation details
 * @param max_violations Maximum violations to store
 * @param violation_count Pointer to receive actual violation count
 * @return CONFIG_VALIDATION_SUCCESS if all valid, first error code otherwise
 */
config_validation_result_t config_validation_check_context(
    const config_context_t *context,
    config_violation_t *violations,
    uint32_t max_violations,
    uint32_t *violation_count);

/**
 * @brief Set a parameter value with validation
 *
 * Attempts to set a parameter value after validating it.
 *
 * @param context Configuration context
 * @param parameter_name Name of parameter to set
 * @param value New value to set
 * @param violation Pointer to store violation if validation fails
 * @return 0 on success, negative error code on failure
 */
int config_validation_set_parameter(
    const config_context_t *context,
    const char *parameter_name,
    const config_value_t *value,
    config_violation_t *violation);

/**
 * @brief Get the current value of a configuration parameter
 *
 * @param context Configuration context
 * @param parameter_name Name of parameter to get
 * @param value Pointer to receive the current value
 * @return 0 on success, negative error code on failure
 */
int config_validation_get_parameter(
    const config_context_t *context,
    const char *parameter_name,
    config_value_t *value);

/**
 * @brief Find a parameter definition by name
 *
 * @param context Configuration context
 * @param parameter_name Name to search for
 * @return Pointer to parameter definition, or NULL if not found
 */
const config_parameter_t *config_validation_find_parameter(
    const config_context_t *context,
    const char *parameter_name);

/**
 * @brief Register a custom validation callback
 *
 * Allows registration of custom validation logic for specific contexts.
 *
 * @param context_id Context ID to register callback for
 * @param callback Validation callback function
 * @param user_data User-provided context data
 * @return Callback ID on success, negative error code on failure
 */
int config_validation_register_callback(
    int context_id,
    config_validation_callback_t callback,
    void *user_data);

/**
 * @brief Register a configuration change callback
 *
 * Callback is invoked when parameter values change.
 *
 * @param context_id Context ID to register callback for
 * @param callback Change callback function
 * @param user_data User-provided context data
 * @return Callback ID on success, negative error code on failure
 */
int config_validation_register_change_callback(
    int context_id,
    config_change_callback_t callback,
    void *user_data);

/**
 * @brief Perform hot reload of configuration
 *
 * Updates configuration values that support hot reloading.
 *
 * @param context_id Context to reload
 * @param new_context New configuration values
 * @param violations Array to store violations from invalid parameters
 * @param max_violations Maximum violations to store
 * @param violation_count Actual violation count
 * @return 0 on success, negative error code on failure
 */
int config_validation_hot_reload(
    int context_id,
    const config_context_t *new_context,
    config_violation_t *violations,
    uint32_t max_violations,
    uint32_t *violation_count);

/**
 * @brief Check configuration integrity
 *
 * Verifies that configuration has not been corrupted.
 *
 * @param context Configuration context to check
 * @return true if configuration appears valid, false if corrupted
 */
bool config_validation_check_integrity(const config_context_t *context);

/**
 * @brief Create configuration checksum
 *
 * Generates a checksum for configuration integrity verification.
 *
 * @param context Configuration context
 * @return Checksum value
 */
uint32_t config_validation_create_checksum(const config_context_t *context);

/**
 * @brief Dump configuration to debug output
 *
 * Prints current configuration values for debugging.
 *
 * @param context Configuration context to dump
 * @return 0 on success, negative error code on failure
 */
int config_validation_dump(const config_context_t *context);

/**
 * @brief Reset configuration to defaults
 *
 * Resets all parameters in a context to their default values.
 *
 * @param context_id Context ID to reset
 * @return 0 on success, negative error code on failure
 */
int config_validation_reset_to_defaults(int context_id);

/**
 * @brief Check if parameter supports hot reloading
 *
 * @param context Configuration context
 * @param parameter_name Parameter to check
 * @return true if hot reloading is supported, false otherwise
 */
bool config_validation_supports_hot_reload(
    const config_context_t *context,
    const char *parameter_name);

/**
 * @brief Cleanup configuration validation resources
 *
 * Should be called during system shutdown.
 *
 * @return 0 on success, negative error code on failure
 */
int config_validation_cleanup(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert validation result to human-readable string
 *
 * @param result Validation result to convert
 * @return String representation of result
 */
const char *config_validation_result_to_string(config_validation_result_t result);

/**
 * @brief Convert configuration type to human-readable string
 *
 * @param type Configuration type to convert
 * @return String representation of type
 */
const char *config_value_type_to_string(config_value_type_t type);

/**
 * @brief Create config_value_t from various types
 *
 * Convenience macros for creating configuration values.
 *
 * @param val Value to convert
 * @param type Type of the value
 * @return config_value_t structure
 */
#define CONFIG_VALUE_U32(val)  ((config_value_t){ .u32 = (val) })
#define CONFIG_VALUE_I32(val)  ((config_value_t){ .i32 = (val) })
#define CONFIG_VALUE_U64(val)  ((config_value_t){ .u64 = (val) })
#define CONFIG_VALUE_I64(val)  ((config_value_t){ .i64 = (val) })
#define CONFIG_VALUE_F32(val)  ((config_value_t){ .f32 = (val) })
#define CONFIG_VALUE_F64(val)  ((config_value_t){ .f64 = (val) })
#define CONFIG_VALUE_BOOL(val) ((config_value_t){ .b = (val) })
#define CONFIG_VALUE_STR(val)  ((config_value_t){ .str = (val) })

/**
 * @brief Get value as specific type
 *
 * Convenience macros for extracting values from config_value_t.
 *
 * @param val config_value_t structure
 * @return Value as requested type
 */
#define CONFIG_GET_U32(val)   ((val).u32)
#define CONFIG_GET_I32(val)   ((val).i32)
#define CONFIG_GET_U64(val)   ((val).u64)
#define CONFIG_GET_I64(val)   ((val).i64)
#define CONFIG_GET_F32(val)   ((val).f32)
#define CONFIG_GET_F64(val)   ((val).f64)
#define CONFIG_GET_BOOL(val)  ((val).b)
#define CONFIG_GET_STR(val)   ((val).str)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_VALIDATION_H */
