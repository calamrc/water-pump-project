/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public API for the YF-S201C flow sensor driver (thinned architecture).
 *
 * The driver is intentionally thin:
 *   - Only raw recent periods + basic electrical sanity + ERROR state on
 *     persistent invalid pulses.
 *   - All signal processing (median, outlier rejection), flow-rate math,
 *     and validity policy live in FlowSensorService + flow_processor.
 *
 * Deprecated: get_flow_rate() and is_data_valid() — always return error/false.
 * Preferred: yf_s201c_get_recent_periods_us() + flow_processor_*().
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
 * @deprecated This function performs application-level processing.
 *             New code should use yf_s201c_get_recent_periods_us() + flow_processor
 *             (or equivalent in FlowSensorService).
 */
int yf_s201c_get_flow_rate(const struct device *dev, fixed_t *flow_rate);

/**
 * @brief Check if flow sensor data is valid
 *
 * @deprecated Validity policy now lives in the application layer.
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

/**
 * @brief Read the most recent N raw period measurements (in microseconds).
 *
 * This is intended for application-layer processing (e.g. custom median
 * filtering or statistics in FlowSensorService). The driver only guarantees
 * basic electrical sanity on the periods it returns.
 *
 * @param dev Device instance
 * @param periods_us Output buffer for periods (most recent first)
 * @param len Number of periods to read (must be <= internal buffer size)
 * @return Number of periods actually copied, or negative error code
 */
int yf_s201c_get_recent_periods_us(const struct device *dev,
				   int64_t *periods_us,
				   size_t len);

#endif /* ZEPHYR_INCLUDE_APP_DRIVERS_YF_S201C_H_ */