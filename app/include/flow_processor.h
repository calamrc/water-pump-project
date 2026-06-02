/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flow Processor - Signal processing for flow sensor data.
 * Extracted from the YF-S201C driver during zbus architecture refactor.
 */

#ifndef FLOW_PROCESSOR_H_
#define FLOW_PROCESSOR_H_

#include <stdint.h>
#include <stddef.h>

#include "fixed_math.h"

/**
 * @brief Context for flow period filtering (median + outlier rejection).
 */
struct flow_processor_ctx {
	int64_t period_buffer[8];
	int buffer_size;
	int buffer_index;
	int valid_count;
};

/**
 * @brief Initialize a flow processor context.
 */
static inline void flow_processor_init(struct flow_processor_ctx *ctx, int buffer_size)
{
	if (!ctx) return;
	ctx->buffer_size = (buffer_size > 8) ? 8 : buffer_size;
	ctx->buffer_index = 0;
	ctx->valid_count = 0;
}

/**
 * @brief Calculate median of an array (helper, exposed for tests).
 */
int64_t flow_processor_median(int64_t *arr, int size);

/**
 * @brief Apply median filtering + outlier rejection to a new period.
 */
int64_t flow_processor_filter_period(struct flow_processor_ctx *ctx, int64_t new_period_us);

/**
 * @brief Reset the processor state.
 */
void flow_processor_reset(struct flow_processor_ctx *ctx);

/**
 * @brief Calculate flow rate (L/min) from a raw period.
 */
fixed_t flow_processor_calculate_flow_rate(int64_t period_us, uint32_t pulses_per_liter);

#endif /* FLOW_PROCESSOR_H_ */
