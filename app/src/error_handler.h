/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Error Code Definitions
 * ============================================================================ */

/**
 * @brief System-wide error codes
 *
 * Error codes follow a hierarchical structure:
 * - Bits 31-28: Module identifier (4 bits)
 * - Bits 27-24: Error category (4 bits)
 * - Bits 23-0: Specific error code (24 bits)
 */
typedef enum {
    /* Common/System Errors (Module 0x0) */
    ERROR_SUCCESS = 0x00000000,                    /**< No error */

    /* GPIO Module Errors (Module 0x1) */
    ERROR_GPIO_BASE = 0x10000000,
    ERROR_GPIO_INVALID_PIN = 0x10000001,           /**< Invalid GPIO pin requested */
    ERROR_GPIO_CONFIG_FAILED = 0x10000002,         /**< GPIO pin configuration failed */
    ERROR_GPIO_SET_FAILED = 0x10000003,            /**< GPIO pin set operation failed */
    ERROR_GPIO_GET_FAILED = 0x10000004,            /**< GPIO pin read operation failed */
    ERROR_GPIO_DEVICE_NOT_READY = 0x10000005,      /**< GPIO device not initialized */

    /* Sensor Module Errors (Module 0x2) */
    ERROR_SENSOR_BASE = 0x20000000,
    ERROR_SENSOR_INIT_FAILED = 0x20000001,         /**< Sensor initialization failed */
    ERROR_SENSOR_READ_FAILED = 0x20000002,         /**< Sensor read operation failed */
    ERROR_SENSOR_DATA_INVALID = 0x20000003,        /**< Sensor returned invalid data */
    ERROR_SENSOR_TIMEOUT = 0x20000004,             /**< Sensor operation timed out */
    ERROR_SENSOR_CALIBRATION_FAILED = 0x20000005,  /**< Sensor calibration failed */

    /* Pump Controller Module Errors (Module 0x3) */
    ERROR_PUMP_BASE = 0x30000000,
    ERROR_PUMP_INVALID_STATE = 0x30000001,         /**< Invalid pump state transition */
    ERROR_PUMP_GPIO_FAILED = 0x30000002,           /**< Pump GPIO operation failed */
    ERROR_PUMP_SAFETY_TIMEOUT = 0x30000003,        /**< Safety timer expired */
    ERROR_PUMP_INVALID_PERIOD = 0x30000004,        /**< Invalid plateau period provided */
    ERROR_PUMP_STATE_PERSISTENCE_FAILED = 0x30000005, /**< State save/load failed */

    /* Flow Analyzer Module Errors (Module 0x4) */
    ERROR_FLOW_BASE = 0x40000000,
    ERROR_FLOW_INVALID_DATA = 0x40000001,          /**< Invalid flow data for analysis */
    ERROR_FLOW_CALCULATION_ERROR = 0x40000002,     /**< Flow calculation failed */
    ERROR_FLOW_STATISTICS_ERROR = 0x40000003,      /**< Statistical computation error */
    ERROR_FLOW_BUFFER_OVERFLOW = 0x40000004,       /**< Flow buffer capacity exceeded */

    /* Configuration Module Errors (Module 0x5) */
    ERROR_CONFIG_BASE = 0x50000000,
    ERROR_CONFIG_INVALID_VALUE = 0x50000001,       /**< Configuration value out of bounds */
    ERROR_CONFIG_DEPENDENCY_FAILED = 0x50000002,   /**< Configuration dependency not met */
    ERROR_CONFIG_VALIDATION_FAILED = 0x50000003,   /**< Runtime configuration validation failed */
    ERROR_CONFIG_CORRUPTION_DETECTED = 0x50000004, /**< Configuration data corruption */

    /* Memory Module Errors (Module 0x6) */
    ERROR_MEMORY_BASE = 0x60000000,
    ERROR_MEMORY_ALLOCATION_FAILED = 0x60000001,   /**< Memory allocation failed */
    ERROR_MEMORY_BUFFER_OVERFLOW = 0x60000002,     /**< Buffer write exceeded capacity */
    ERROR_MEMORY_NULL_POINTER = 0x60000003,        /**< Null pointer accessed */
    ERROR_MEMORY_LEAK_DETECTED = 0x60000004,       /**< Memory leak detected */

    /* General System Errors (Module 0xF) */
    ERROR_SYSTEM_BASE = 0xF0000000,
    ERROR_SYSTEM_TIMEOUT = 0xF0000001,             /**< System operation timed out */
    ERROR_SYSTEM_RESOURCE_EXHAUSTED = 0xF0000002,  /**< System resource limit exceeded */
    ERROR_SYSTEM_CRITICAL_FAILURE = 0xF0000003,    /**< Critical system failure */
    ERROR_SYSTEM_RECOVERY_FAILED = 0xF0000004,     /**< Error recovery mechanism failed */

    /* Special Values */
    ERROR_UNKNOWN = 0xFFFFFFFF                       /**< Unknown or unspecified error */
} error_code_t;

/* ============================================================================
 * Error Severity Levels
 * ============================================================================ */

/**
 * @brief Error severity classification
 *
 * Determines how the system should respond to errors.
 */
typedef enum {
    ERROR_SEVERITY_INFO = 0,      /**< Informational - no action required */
    ERROR_SEVERITY_WARNING,       /**< Warning - log and monitor */
    ERROR_SEVERITY_ERROR,         /**< Error - requires attention, may continue */
    ERROR_SEVERITY_CRITICAL,      /**< Critical - immediate action required */
    ERROR_SEVERITY_FATAL          /**< Fatal - system shutdown required */
} error_severity_t;

/* ============================================================================
 * Error Context Structure
 * ============================================================================ */

/**
 * @brief Error context information
 *
 * Captures comprehensive information about error conditions
 * for debugging, logging, and recovery purposes.
 */
typedef struct {
    error_code_t code;                    /**< Error code identifier */
    error_severity_t severity;            /**< Error severity level */
    const char *module;                   /**< Module where error occurred */
    const char *function;                 /**< Function where error occurred */
    int line;                             /**< Line number where error occurred */
    uint32_t timestamp;                   /**< Timestamp when error occurred */
    k_tid_t thread_id;                    /**< Thread ID where error occurred */

    /* Environmental context */
    uint32_t uptime_ms;                   /**< System uptime when error occurred */
    uint32_t free_memory;                 /**< Available heap memory */
    uint32_t stack_usage;                 /**< Current stack usage */

    /* Recovery information */
    bool recoverable;                     /**< Whether error is recoverable */
    const char *recovery_action;          /**< Suggested recovery action */

    /* Optional custom context */
    void *custom_data;                    /**< Module-specific error context */
    uint32_t custom_data_size;            /**< Size of custom data */
} error_context_t;

/* ============================================================================
 * Callback Function Types
 * ============================================================================ */

/**
 * @brief Error handler callback function type
 *
 * Called when errors are reported for processing.
 *
 * @param context Error context information
 * @param user_data User-provided context data
 * @return 0 on success, negative error code on failure
 */
typedef int (*error_handler_callback_t)(const error_context_t *context, void *user_data);

/**
 * @brief Error recovery callback function type
 *
 * Called when attempting to recover from errors.
 *
 * @param context Error context information
 * @param user_data User-provided context data
 * @return 0 if recovery successful, negative error code if recovery failed
 */
typedef int (*error_recovery_callback_t)(const error_context_t *context, void *user_data);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize the error handler module
 *
 * Sets up error handling infrastructure and registers default handlers.
 *
 * @return 0 on success, negative error code on failure
 */
int error_handler_init(void);

/**
 * @brief Report an error with full context
 *
 * Captures comprehensive error information and triggers appropriate
 * response mechanisms based on severity.
 *
 * @param code Error code
 * @param severity Error severity level
 * @param module Module name string
 * @param function Function name string
 * @param line Line number
 * @param recoverable Whether error is recoverable
 * @param recovery_action Suggested recovery action string
 * @param custom_data Optional custom context data
 * @param custom_data_size Size of custom data
 * @return 0 on success, negative error code on failure
 */
int error_report(error_code_t code, error_severity_t severity,
                 const char *module, const char *function, int line,
                 bool recoverable, const char *recovery_action,
                 void *custom_data, uint32_t custom_data_size);

/**
 * @brief Report an error with minimal context (convenience macro)
 *
 * @param code Error code
 * @param severity Error severity level
 * @param recoverable Whether error is recoverable
 * @return 0 on success, negative error code on failure
 */
#define ERROR_REPORT(code, severity, recoverable) \
    error_report(code, severity, __FILE__, __func__, __LINE__, \
                recoverable, NULL, NULL, 0)

/**
 * @brief Report a recoverable error (convenience macro)
 *
 * @param code Error code
 * @return 0 on success, negative error code on failure
 */
#define ERROR_REPORT_RECOVERABLE(code) \
    ERROR_REPORT(code, ERROR_SEVERITY_ERROR, true)

/**
 * @brief Report a critical error (convenience macro)
 *
 * @param code Error code
 * @return 0 on success, negative error code on failure
 */
#define ERROR_REPORT_CRITICAL(code) \
    ERROR_REPORT(code, ERROR_SEVERITY_CRITICAL, false)

/**
 * @brief Register an error handler callback
 *
 * Allows modules to register custom error handling callbacks
 * for specific error patterns or all errors.
 *
 * @param callback Error handler callback function
 * @param user_data User-provided context data
 * @param priority Callback priority (lower numbers = higher priority)
 * @return Handler ID on success, negative error code on failure
 */
int error_handler_register(error_handler_callback_t callback,
                          void *user_data, int priority);

/**
 * @brief Unregister an error handler callback
 *
 * @param handler_id Handler ID returned by error_handler_register
 * @return 0 on success, negative error code on failure
 */
int error_handler_unregister(int handler_id);

/**
 * @brief Set recovery callback for specific error patterns
 *
 * @param error_pattern Error code pattern (can use wildcards with ERROR_*_BASE)
 * @param callback Recovery callback function
 * @param user_data User-provided context data
 * @return 0 on success, negative error code on failure
 */
int error_recovery_register(error_code_t error_pattern,
                           error_recovery_callback_t callback,
                           void *user_data);

/**
 * @brief Check if error recovery is available for given error code
 *
 * @param code Error code to check
 * @return true if recovery is available, false otherwise
 */
bool error_recovery_available(error_code_t code);

/**
 * @brief Attempt error recovery for given error context
 *
 * @param context Error context information
 * @return 0 if recovery successful, negative error code if recovery failed
 */
int error_attempt_recovery(const error_context_t *context);

/**
 * @brief Get last reported error context
 *
 * @param context Pointer to receive error context
 * @return 0 on success, negative error code on failure
 */
int error_get_last_context(error_context_t *context);

/**
 * @brief Get error statistics
 *
 * @param info_error_count Number of informational errors
 * @param warning_count Number of warning errors
 * @param error_count Number of error level errors
 * @param critical_count Number of critical errors
 * @param fatal_count Number of fatal errors
 * @return 0 on success, negative error code on failure
 */
int error_get_statistics(uint32_t *info_error_count, uint32_t *warning_count,
                        uint32_t *error_count, uint32_t *critical_count,
                        uint32_t *fatal_count);

/**
 * @brief Reset error statistics
 *
 * @return 0 on success, negative error code on failure
 */
int error_reset_statistics(void);

/**
 * @brief Check system error health status
 *
 * @return true if error status is healthy, false if critical errors present
 */
bool error_system_health_check(void);

/**
 * @brief Cleanup error handler resources
 *
 * Should be called during system shutdown.
 *
 * @return 0 on success, negative error code on failure
 */
int error_handler_cleanup(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert error code to human-readable string
 *
 * @param code Error code to convert
 * @return String representation of error code
 */
const char *error_code_to_string(error_code_t code);

/**
 * @brief Convert error severity to human-readable string
 *
 * @param severity Error severity level
 * @return String representation of severity level
 */
const char *error_severity_to_string(error_severity_t severity);

/**
 * @brief Extract module identifier from error code
 *
 * @param code Error code
 * @return Module identifier (top 4 bits)
 */
uint8_t error_get_module(error_code_t code);

/**
 * @brief Extract category from error code
 *
 * @param code Error code
 * @return Category identifier (bits 27-24)
 */
uint8_t error_get_category(error_code_t code);

/**
 * @brief Check if error code matches pattern
 *
 * @param code Error code to check
 * @param pattern Pattern to match (can use ERROR_*_BASE values)
 * @return true if code matches pattern, false otherwise
 */
bool error_matches_pattern(error_code_t code, error_code_t pattern);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_HANDLER_H */
