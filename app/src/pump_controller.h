/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Pump Controller Module
 *
 * Handles pump state management and control operations:
 * - Pump relay control (ON/OFF)
 * - Safety timer management
 * - Pump state transitions and validation
 * - Runtime protection and error handling
 * - GPIO hardware abstraction
 */

/* ============================================================================
 * External Variable Declarations (for Phase 2 compatibility)
 * ============================================================================ */

// Pump control state
extern bool pump_on;
extern int64_t initial_plateau_period;
extern int64_t latest_plateau_period;

// GPIO device reference
extern const struct device *gpio_dev;

// Safety timer configuration
extern const int64_t PUMP_SAFETY_TIMEOUT_MIN;

/* ============================================================================
 * External Function Declarations (Phase 2 interfaces)
 * ============================================================================ */

/**
 * @brief Initialize the pump controller module
 *
 * Sets up GPIO pins, safety timer, and pump state.
 * Must be called before any pump control operations.
 *
 * @return 0 on success, negative error code on failure
 */
int pump_controller_init(void);

/**
 * @brief Turn pump ON
 *
 * Activates the pump relay and starts safety timer.
 * Records initial plateau period for runtime monitoring.
 *
 * @param plateau_period_us Current plateau period in microseconds
 * @return 0 on success, negative error code on failure
 */
int pump_controller_turn_on(int64_t plateau_period_us);

/**
 * @brief Turn pump OFF
 *
 * Deactivates the pump relay and stops safety timer.
 * Updates plateau tracking for next cycle.
 *
 * @return 0 on success, negative error code on failure
 */
int pump_controller_turn_off(void);

/**
 * @brief Get current pump state
 *
 * Thread-safe access to pump operating status.
 *
 * @return true if pump is ON, false if OFF
 */
bool pump_controller_is_on(void);

/**
 * @brief Update plateau period tracking
 *
 * Called when new plateau periods are detected.
 * Maintains history for timeout calculations.
 *
 * @param period_us Latest plateau period in microseconds
 */
void pump_controller_update_plateau_period(int64_t period_us);

/**
 * @brief Check pump safety status
 *
 * Validates that safety mechanisms are functioning properly.
 * Call periodically to ensure pump protection is active.
 *
 * @return true if safety systems are OK, false if compromised
 */
bool pump_controller_safety_check(void);

/**
 * @brief Reset pump controller state
 *
 * Clears plateau tracking and resets to initial state.
 * Used for error recovery or system restart.
 */
void pump_controller_reset(void);

/**
 * @brief Force emergency pump shutdown
 *
 * Immediate pump deactivation bypassing normal safety protocols.
 * Use only for critical safety situations.
 *
 * @return 0 on success, negative error code on failure
 */
int pump_controller_emergency_stop(void);

/**
 * @brief Cleanup pump controller resources
 *
 * Stops timers, releases GPIO resources, and cleans up state.
 * Call during system shutdown.
 *
 * @return 0 on success, negative error code on failure
 */
int pump_controller_cleanup(void);

/* ============================================================================
 * Configuration Constants (for Phase 2 compatibility)
 * ============================================================================ */

#define PUMP_CONTROLLER_RELAY_PIN 22
#define PUMP_CONTROLLER_TIMEOUT_MIN PUMP_SAFETY_TIMEOUT_MIN

#endif /* PUMP_CONTROLLER_H */
