/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_APP_DRIVERS_YF_S201C_H_
#define ZEPHYR_INCLUDE_APP_DRIVERS_YF_S201C_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Fixed-Point Math Types (from fixed_math.h)
 * ============================================================================ */

/* Fixed-point type */
typedef int32_t fixed_t;

/* ============================================================================
 * YF-S201C Flow Sensor API
 * ============================================================================ */

/**
 * @brief Get flow rate in L/min as fixed-point
 *
 * @param dev Pointer to the YF-S201C device
 * @param flow_rate Pointer to store the flow rate (fixed_t)
 * @return 0 on success, negative on error
 */
int yf_s201c_get_flow_rate(const struct device *dev, fixed_t *flow_rate);

/**
 * @brief Check if flow sensor data is valid
 *
 * @param dev Pointer to the YF-S201C device
 * @return true if data is valid, false otherwise
 */
bool yf_s201c_is_data_valid(const struct device *dev);

/**
 * @brief Get current period between pulses in microseconds
 *
 * @param dev Pointer to the YF-S201C device
 * @param period_us Pointer to store the period
 * @return 0 on success, negative on error
 */
int yf_s201c_get_current_period(const struct device *dev, int64_t *period_us);

/**
 * @brief Reset the flow sensor state
 *
 * @param dev Pointer to the YF-S201C device
 * @return 0 on success, negative on error
 */
int yf_s201c_reset(const struct device *dev);

/**
 * @brief Set the semaphore to signal when valid data becomes available
 *
 * This enables event-driven operation instead of polling.
 *
 * @param dev Device instance
 * @param sem Semaphore to signal (NULL to disable signaling)
 * @return 0 on success, negative error code on failure
 */
int yf_s201c_set_data_semaphore(const struct device *dev, struct k_sem *sem);

#endif /* ZEPHYR_INCLUDE_APP_DRIVERS_YF_S201C_H_ */