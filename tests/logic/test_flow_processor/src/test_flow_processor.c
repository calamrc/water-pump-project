/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Comprehensive unit tests for flow_processor (pure logic, zero kernel/RTOS deps).
 * These exercise the authoritative signal processing extracted from the driver.
 */

#include <zephyr/ztest.h>
#include "flow_processor.h"

/* --------------------------------------------------------------------------
 * Median tests
 * -------------------------------------------------------------------------- */

ZTEST(flow_processor_tests, test_median_basic_odd)
{
	int64_t arr[5] = {100, 300, 200, 500, 150};
	zassert_equal(flow_processor_median(arr, 5), 200, "Median of 5 failed");
}

ZTEST(flow_processor_tests, test_median_basic_even)
{
	int64_t arr[4] = {10, 30, 20, 40};
	zassert_equal(flow_processor_median(arr, 4), 30, "Median of 4 (upper of two middles via our impl)");
}

ZTEST(flow_processor_tests, test_median_already_sorted)
{
	int64_t arr[3] = {1, 2, 3};
	zassert_equal(flow_processor_median(arr, 3), 2, NULL);
}

ZTEST(flow_processor_tests, test_median_single)
{
	int64_t arr[1] = {777};
	zassert_equal(flow_processor_median(arr, 1), 777, NULL);
}

ZTEST(flow_processor_tests, test_median_invalid_size)
{
	int64_t arr[3] = {1, 2, 3};
	zassert_equal(flow_processor_median(arr, 0), 0, "size 0 -> 0");
	zassert_equal(flow_processor_median(arr, 9), 0, "size > max -> 0");
}

/* --------------------------------------------------------------------------
 * Filter (median + outlier) tests
 * -------------------------------------------------------------------------- */

ZTEST(flow_processor_tests, test_filter_fills_buffer)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 5);

	int64_t r = flow_processor_filter_period(&ctx, 10000);
	zassert_equal(r, 10000, "First value passed through");
	zassert_equal(ctx.valid_count, 1, NULL);
}

ZTEST(flow_processor_tests, test_filter_outlier_rejected)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 5);

	for (int i = 0; i < 5; i++) {
		(void)flow_processor_filter_period(&ctx, 10000);
	}

	/* Gross outlier should return the current median instead */
	int64_t result = flow_processor_filter_period(&ctx, 25000);
	zassert_within(result, 10000, 100, "Outlier should yield median");
}

ZTEST(flow_processor_tests, test_filter_normal_variation_accepted)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 5);

	for (int i = 0; i < 5; i++) {
		(void)flow_processor_filter_period(&ctx, 10000);
	}

	/* 20% variation should still be accepted (within 1.5x rule) */
	int64_t result = flow_processor_filter_period(&ctx, 12000);
	zassert_equal(result, 12000, "Reasonable variation accepted");
}

ZTEST(flow_processor_tests, test_filter_reset)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 5);

	for (int i = 0; i < 3; i++) {
		(void)flow_processor_filter_period(&ctx, 8000);
	}
	zassert_true(ctx.valid_count > 0, NULL);

	flow_processor_reset(&ctx);
	zassert_equal(ctx.valid_count, 0, "Reset clears count");
	zassert_equal(ctx.buffer_index, 0, NULL);
}

/* --------------------------------------------------------------------------
 * Flow rate calculation edge cases (pure fixed-point math)
 * -------------------------------------------------------------------------- */

ZTEST(flow_processor_tests, test_flow_rate_typical)
{
	/* 13.333 L/min for 10000 us period @ 450 pulses/L */
	fixed_t rate = flow_processor_calculate_flow_rate(10000, 450);
	float f = fixed_to_float(rate);
	zassert_within(f, 13.333f, 0.1f, NULL);
}

ZTEST(flow_processor_tests, test_flow_rate_zero_period)
{
	fixed_t rate = flow_processor_calculate_flow_rate(0, 450);
	zassert_equal(rate, 0, "zero period -> 0 rate");
}

ZTEST(flow_processor_tests, test_flow_rate_negative_period)
{
	fixed_t rate = flow_processor_calculate_flow_rate(-100, 450);
	zassert_equal(rate, 0, "negative period -> 0 rate");
}

ZTEST(flow_processor_tests, test_flow_rate_zero_pulses)
{
	fixed_t rate = flow_processor_calculate_flow_rate(10000, 0);
	zassert_equal(rate, 0, NULL);
}

ZTEST(flow_processor_tests, test_flow_rate_very_slow)
{
	/* 1 pulse per 2 seconds -> very low flow */
	fixed_t rate = flow_processor_calculate_flow_rate(2000000, 450);
	float f = fixed_to_float(rate);
	zassert_true(f > 0.0f && f < 0.1f, "Very slow flow should be small positive");
}

ZTEST(flow_processor_tests, test_flow_rate_very_fast)
{
	/* 1ms period */
	fixed_t rate = flow_processor_calculate_flow_rate(1000, 450);
	float f = fixed_to_float(rate);
	zassert_true(f > 100.0f, "Very fast flow should be large");
}

/* --------------------------------------------------------------------------
 * Init / buffer size clamping
 * -------------------------------------------------------------------------- */

ZTEST(flow_processor_tests, test_init_clamps_buffer_size)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 99); /* larger than internal max of 8 */
	zassert_equal(ctx.buffer_size, 8, "Clamped to 8");
}

ZTEST(flow_processor_tests, test_init_small_buffer)
{
	struct flow_processor_ctx ctx;
	flow_processor_init(&ctx, 3);
	zassert_equal(ctx.buffer_size, 3, NULL);
}

ZTEST_SUITE(flow_processor_tests, NULL, NULL, NULL, NULL, NULL);