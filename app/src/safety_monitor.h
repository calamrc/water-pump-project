/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Safety Monitor Constants
 * ============================================================================ */

/** Default safety check interval in milliseconds */
#define SAFETY_MONITOR_DEFAULT_INTERVAL_MS 5000

/** Maximum allowed safety check interval in milliseconds */
#define SAFETY_MONITOR_MAX_INTERVAL_MS 30000

/** Safety check timeout threshold in milliseconds */
#define SAFETY_MONITOR_TIMEOUT_MS 1000

/* ============================================================================
 * Safety Status Enumeration
 * ============================================================================ */

/**
 * @brief Overall system safety status
 */
typedef enum {
    SAFETY_STATUS_HEALTHY = 0,   /**< All systems operating normally */
    SAFETY_STATUS_DEGRADED,      /**< Some systems degraded but operational */
    SAFETY_STATUS_CRITICAL,      /**< Critical safety issues detected */
    SAFETY_STATUS_EMERGENCY      /**< Emergency stop required */
} safety_status_t;

/* ============================================================================
 * Safety Check Types
 * ============================================================================ */

/**
 * @brief Types of safety checks that can be performed
 */
typedef enum {
    SAFETY_CHECK_HARDWARE = 0,   /**< Hardware component verification */
    SAFETY_CHECK_SOFTWARE,       /**< Software integrity checks */
    SAFETY_CHECK_ENVIRONMENTAL,  /**< Environmental condition monitoring */
    SAFETY_CHECK_COMMUNICATION,  /**< Communication system health */
    SAFETY_CHECK_POWER,          /**< Power system monitoring */
    SAFETY_CHECK_COUNT           /**< Total number of check types */
} safety_check_type_t;

/* ============================================================================
 * Safety Violation Types
 * ============================================================================ */

/**
 * @brief Types of safety violations that can be detected
 */
typedef enum {
    SAFETY_VIOLATION_NONE = 0,           /**< No violation */
    SAFETY_VIOLATION_TIMEOUT,             /**< Operation exceeded time limit */
    SAFETY_VIOLATION_OUT_OF_BOUNDS,       /**< Value exceeded safe limits */
    SAFETY_VIOLATION_HARDWARE_FAILURE,    /**< Hardware component failure */
    SAFETY_VIOLATION_SOFTWARE_ERROR,      /**< Software execution error */
    SAFETY_VIOLATION_RESOURCE_EXHAUSTED,  /**< System resources depleted */
    SAFETY_VIOLATION_CONFIGURATION_ERROR, /**< Invalid configuration detected */
    SAFETY_VIOLATION_COUNT                /**< Total number of violation types */
} safety_violation_t;

/* ============================================================================
 * Safety Check Result Structure
 * ============================================================================ */

/**
 * @brief Result of a safety check operation
 */
typedef struct {
    safety_check_type_t check_type;       /**< Type of check performed */
    bool passed;                          /**< Whether the check passed */
    safety_violation_t violation;         /**< Type of violation if any */
    uint32_t timestamp;                   /**< Timestamp when check was performed */
    int32_t response_time_ms;             /**< Time taken to perform check */
    char description[64];                 /**< Human-readable description */
} safety_check_result_t;

/* ============================================================================
 * Safety Monitor Configuration
 * ============================================================================ */

/**
 * @brief Safety monitor configuration parameters
 */
typedef struct {
    uint32_t check_interval_ms;           /**< Interval between safety checks */
    bool enable_hardware_checks;          /**< Enable hardware-specific checks */
    bool enable_software_checks;          /**< Enable software integrity checks */
    bool enable_environmental_checks;     /**< Enable environmental monitoring */
    bool emergency_stop_enabled;          /**< Enable automatic emergency stop */
    uint32_t critical_violation_threshold; /**< Violations to trigger critical status */
    uint32_t emergency_threshold;         /**< Violations to trigger emergency stop */
} safety_monitor_config_t;

/* ============================================================================
 * Safety Statistics Structure
 * ============================================================================ */

/**
 * @brief Safety monitoring statistics
 */
typedef struct {
    uint32_t total_checks;                /**< Total safety checks performed */
    uint32_t passed_checks;               /**< Number of checks that passed */
    uint32_t failed_checks;               /**< Number of checks that failed */
    uint32_t critical_violations;         /**< Number of critical violations */
    uint32_t emergency_events;            /**< Number of emergency stops triggered */
    uint32_t uptime_seconds;              /**< Total uptime of safety monitoring */
    uint32_t last_check_time;             /**< Timestamp of last safety check */
} safety_statistics_t;

/* ============================================================================
 * Callback Function Types
 * ============================================================================ */

/**
 * @brief Safety check callback function type
 *
 * Performs a specific safety check and returns the result.
 *
 * @param check_type Type of safety check to perform
 * @param result Pointer to store the check result
 * @param user_data User-provided context data
 * @return 0 on success, negative error code on failure
 */
typedef int (*safety_check_callback_t)(safety_check_type_t check_type,
                                      safety_check_result_t *result,
                                      void *user_data);

/**
 * @brief Safety violation callback function type
 *
 * Called when a safety violation is detected.
 *
 * @param violation Type of violation detected
 * @param severity Severity level (1-5, 5 being most severe)
 * @param description Human-readable description of the violation
 * @param user_data User-provided context data
 * @return 0 on success, negative error code on failure
 */
typedef int (*safety_violation_callback_t)(safety_violation_t violation,
                                          uint8_t severity,
                                          const char *description,
                                          void *user_data);

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize the safety monitor module
 *
 * Sets up safety monitoring infrastructure with default configuration.
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_init(void);

/**
 * @brief Configure safety monitor parameters
 *
 * Applies custom configuration to the safety monitor.
 *
 * @param config Pointer to configuration structure
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_configure(const safety_monitor_config_t *config);

/**
 * @brief Register a safety check callback
 *
 * Allows modules to register custom safety check implementations.
 *
 * @param check_type Type of safety check to register
 * @param callback Safety check callback function
 * @param user_data User-provided context data
 * @return Check ID on success, negative error code on failure
 */
int safety_monitor_register_check(safety_check_type_t check_type,
                                 safety_check_callback_t callback,
                                 void *user_data);

/**
 * @brief Register a safety violation callback
 *
 * Allows modules to register handlers for safety violations.
 *
 * @param callback Safety violation callback function
 * @param user_data User-provided context data
 * @return Callback ID on success, negative error code on failure
 */
int safety_monitor_register_violation_handler(safety_violation_callback_t callback,
                                             void *user_data);

/**
 * @brief Perform immediate safety check
 *
 * Executes all registered safety checks immediately.
 *
 * @param results Array to store check results (can be NULL)
 * @param max_results Maximum number of results to store
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_check_now(safety_check_result_t *results, uint32_t max_results);

/**
 * @brief Get current system safety status
 *
 * @return Current safety status
 */
safety_status_t safety_monitor_get_status(void);

/**
 * @brief Get safety monitoring statistics
 *
 * @param stats Pointer to receive statistics
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_get_statistics(safety_statistics_t *stats);

/**
 * @brief Check if a specific safety check type is available
 *
 * @param check_type Type of safety check to check
 * @return true if check type is registered, false otherwise
 */
bool safety_monitor_check_available(safety_check_type_t check_type);

/**
 * @brief Force emergency stop
 *
 * Immediately triggers emergency stop procedures regardless of
 * current safety status.
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_emergency_stop(void);

/**
 * @brief Reset safety statistics and counters
 *
 * Clears all safety monitoring statistics.
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_reset_statistics(void);

/**
 * @brief Get last safety check results
 *
 * @param results Array to receive check results
 * @param max_results Maximum number of results to retrieve
 * @param count Pointer to receive actual number of results returned
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_get_last_results(safety_check_result_t *results,
                                   uint32_t max_results, uint32_t *count);

/**
 * @brief Pause safety monitoring
 *
 * Temporarily suspends safety checks (for maintenance, testing, etc.).
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_pause(void);

/**
 * @brief Resume safety monitoring
 *
 * Resumes safety checks after being paused.
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_resume(void);

/**
 * @brief Check if safety monitoring is currently paused
 *
 * @return true if paused, false if active
 */
bool safety_monitor_is_paused(void);

/**
 * @brief Cleanup safety monitor resources
 *
 * Should be called during system shutdown.
 *
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_cleanup(void);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Convert safety status to human-readable string
 *
 * @param status Safety status to convert
 * @return String representation of safety status
 */
const char *safety_status_to_string(safety_status_t status);

/**
 * @brief Convert safety check type to human-readable string
 *
 * @param check_type Safety check type to convert
 * @return String representation of check type
 */
const char *safety_check_type_to_string(safety_check_type_t check_type);

/**
 * @brief Convert safety violation to human-readable string
 *
 * @param violation Safety violation type to convert
 * @return String representation of violation
 */
const char *safety_violation_to_string(safety_violation_t violation);

/**
 * @brief Create default safety monitor configuration
 *
 * Populates a configuration structure with safe default values.
 *
 * @param config Pointer to configuration structure to initialize
 * @return 0 on success, negative error code on failure
 */
int safety_monitor_create_default_config(safety_monitor_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
