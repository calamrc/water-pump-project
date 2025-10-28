/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "fixed_math.h"

/**
 * @brief Sensor Manager Module
 *
 * Handles all flow sensor data acquisition and processing:
 * - ISR registration and timestamp capture
 * - Workqueue-based deferred processing
 * - Period buffer management with thread safety
 * - Flow rate calculations and conversions
 * - Sensor validation and error handling
 */

/* ============================================================================
 * External Variable Declarations (for Phase 2 compatibility)
 * ============================================================================ */

// Period buffer for flow sensor measurements (shared between ISR and worker)
extern volatile int64_t period_buffer[5];
extern volatile int64_t last_pulse_ticks;
extern volatile int64_t period_us;
extern volatile int valid_periods;
extern volatile int buffer_index;

// ISR-to-workqueue communication variables
extern volatile int64_t isr_current_period_us;
extern volatile bool isr_valid_update;

// Sensor configuration constants
extern const uint32_t YF_S201C_PULSES_PER_LITER;
extern const uint32_t MIN_PERIOD_US;

/* ============================================================================
 * External Function Declarations (Phase 2 interfaces)
 * ============================================================================ */

/**
 * @brief Initialize the sensor manager module
 *
 * Sets up GPIO interrupts, workqueue, and sensor data structures.
 * Must be called before any sensor operations.
 *
 * @return 0 on success, negative error code on failure
 */
int sensor_manager_init(void);

/**
 * @brief Get the current flow rate in fixed-point format
 *
 * Calculates instantaneous flow rate from period measurements.
 * Thread-safe for main thread access.
 *
 * @return Current flow rate in Q16.16 fixed-point format (L/min)
 */
fixed_t sensor_manager_get_flow_rate(void);

/**
 * @brief Check if sensor data is currently valid
 *
 * Validates that recent measurements are available and within
 * acceptable bounds for reliability.
 *
 * @return true if sensor data is valid, false otherwise
 */
bool sensor_manager_is_data_valid(void);

/**
 * @brief Get the current period in microseconds
 *
 * Returns the most recent flow sensor period measurement.
 * Thread-safe for main thread access.
 *
 * @return Current period in microseconds, 0 if no valid data
 */
int64_t sensor_manager_get_current_period(void);

/**
 * @brief Reset sensor data and buffers
 *
 * Clears all measurement buffers and resets sensor state.
 * Used for error recovery or state transitions.
 */
void sensor_manager_reset(void);

/**
 * @brief Cleanup sensor manager resources
 *
 * Stops interrupts, cancels workqueue operations, and frees resources.
 * Call during system shutdown.
 *
 * @return 0 on success, negative error code on failure
 */
int sensor_manager_cleanup(void);

/* ============================================================================
 * Configuration Constants (for Phase 2 compatibility)
 * ============================================================================ */

#define SENSOR_MANAGER_BUFFER_SIZE 5
#define SENSOR_MANAGER_WORKQUEUE_STACK_SIZE 1024
#define SENSOR_MANAGER_WORKQUEUE_PRIORITY -1

/* External semaphore reference for main communication */
extern struct k_sem data_sem;

#endif /* SENSOR_MANAGER_H */
