/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flow Processor
 *
 * This module owns signal processing logic for the YF-S201C flow sensor
 * (median filtering, outlier rejection, etc.). It was extracted from the
 * driver during the architecture refactor so that the driver stays thin
 * and the application layer owns the algorithms.
 *
 * The FlowSensorService will use this module to process raw periods
 * coming from the driver.
 */

#include <string.h>
#include "flow_processor.h"

/**
 * @brief Calculate median value from an array of int64_t values.
 * Simple bubble sort version (small fixed-size buffers).
 */
int64_t flow_processor_median(int64_t *arr, int size)
{
	int64_t sorted[8]; /* Support up to 8 for future flexibility */

	if (size <= 0 || size > 8) {
		return 0;
	}

	memcpy(sorted, arr, size * sizeof(int64_t));

	for (int i = 0; i < size - 1; i++) {
		for (int j = 0; j < size - i - 1; j++) {
			if (sorted[j] > sorted[j + 1]) {
				int64_t temp = sorted[j];
				sorted[j] = sorted[j + 1];
				sorted[j + 1] = temp;
			}
		}
	}

	return sorted[size / 2];
}

/**
 * @brief Apply median filtering + simple outlier rejection on a new period.
 *
 * This is the logic that used to live inside the YF-S201C driver.
 *
 * @param ctx          Processing context (holds the circular buffer state)
 * @param new_period_us New raw period measurement from the driver
 * @return Filtered / accepted period value to use
 */
int64_t flow_processor_filter_period(struct flow_processor_ctx *ctx, int64_t new_period_us)
{
	if (!ctx || ctx->buffer_size <= 0) {
		return new_period_us;
	}

	/* Add to circular buffer */
	ctx->period_buffer[ctx->buffer_index] = new_period_us;
	ctx->buffer_index = (ctx->buffer_index + 1) % ctx->buffer_size;

	if (ctx->valid_count < ctx->buffer_size) {
		ctx->valid_count++;
	}

	/* Not enough samples yet for filtering */
	if (ctx->valid_count < ctx->buffer_size) {
		return new_period_us;
	}

	int64_t median = flow_processor_median(ctx->period_buffer, ctx->buffer_size);

	/* Outlier rejection: reject if outside [median/1.5 , median*1.5] */
	if (new_period_us < (median / 1.5) || new_period_us > (median * 1.5)) {
		return median; /* Use median instead */
	}

	return new_period_us;
}

void flow_processor_reset(struct flow_processor_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	ctx->buffer_index = 0;
	ctx->valid_count = 0;
	memset(ctx->period_buffer, 0, sizeof(ctx->period_buffer));
}

/**
 * @brief Calculate flow rate in L/min from a period in microseconds.
 *
 * @param period_us      Pulse period in microseconds
 * @param pulses_per_liter Pulses per liter (from device tree / config)
 * @return Flow rate in L/min (as fixed_t), or 0 if invalid
 */
fixed_t flow_processor_calculate_flow_rate(int64_t period_us, uint32_t pulses_per_liter)
{
	if (period_us <= 0 || pulses_per_liter == 0) {
		return 0;
	}

	/* Prevent overflow */
	if (period_us > (INT64_MAX / (int64_t)pulses_per_liter)) {
		return 0;
	}

	/* Flow rate (L/min) = (60 * 1e6) / (period_us * pulses_per_liter) */
	float flow_lpm = (60.0f * 1000000.0f) / ((float)period_us * (float)pulses_per_liter);
	return fixed_from_float(flow_lpm);
}