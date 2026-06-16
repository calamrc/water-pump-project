/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "flow_analyzer.h"
#include "fixed_math.h"

LOG_MODULE_REGISTER(flow_analyzer);

static fixed_t flow_buffer[CONFIG_APP_PLATEAU_WINDOW_SIZE] = {0};
static fixed_t flow_slope;
static fixed_t noise_std;
static fixed_t prev_flow = FIXED_MIN;
static int flow_buffer_index;
static int flow_diff_count;

static const int PLATEAU_WINDOW_SIZE = CONFIG_APP_PLATEAU_WINDOW_SIZE;
static const int PLATEAU_CONFIRM_COUNT = CONFIG_APP_PLATEAU_CONFIRM_COUNT;

static void flow_analyzer_calibrate_plateau(void)
{
	if (flow_buffer_index != 0) {
		return;
	}

	if (PLATEAU_WINDOW_SIZE <= 0 || PLATEAU_WINDOW_SIZE > 10) {
		LOG_ERR("Invalid PLATEAU_WINDOW_SIZE: %d", PLATEAU_WINDOW_SIZE);
		return;
	}

	fixed_t sum_diffs = 0;

	for (int i = 0; i < PLATEAU_WINDOW_SIZE - 1; i++) {
		fixed_t diff = fixed_sub(
			flow_buffer[(flow_buffer_index + i + 1) % PLATEAU_WINDOW_SIZE],
			flow_buffer[(flow_buffer_index + i) % PLATEAU_WINDOW_SIZE]);
		sum_diffs = fixed_add(sum_diffs, diff);
	}

	flow_slope = fixed_div_int(sum_diffs, PLATEAU_WINDOW_SIZE - 1);

	if (fixed_gt(fixed_abs(flow_slope), FIXED_PLATEAU_MIN_SLOPE)) {
		fixed_t sum_sq_res = 0;

		for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
			fixed_t slope_term = fixed_mul_int(flow_slope, i);
			fixed_t predicted = fixed_add(flow_buffer[0], slope_term);
			fixed_t residual = fixed_sub(flow_buffer[i], predicted);
			sum_sq_res = fixed_add(sum_sq_res, fixed_mul(residual, residual));
		}

		fixed_t mean_sq_res = fixed_div_int(sum_sq_res, PLATEAU_WINDOW_SIZE);
		noise_std = fixed_sqrt(fixed_abs(mean_sq_res));
	} else {
		noise_std = 0;
	}

	LOG_DBG("Calibration complete, noise_std: %.4f, flow_slope: %.4f",
		(double)fixed_to_float(noise_std), (double)fixed_to_float(flow_slope));
}

int flow_analyzer_init(void)
{
	flow_buffer_index = 0;
	prev_flow = FIXED_MIN;
	flow_diff_count = 0;
	noise_std = 0;
	flow_slope = 0;
	memset(flow_buffer, 0, sizeof(flow_buffer));

	LOG_INF("Flow analyzer initialized successfully");
	return 0;
}

bool flow_analyzer_detect_plateau(fixed_t flow_rate, fixed_t k_factor)
{
	LOG_DBG("detect_plateau called with flow_rate: %.3f, buffer_index: %d",
		(double)fixed_to_float(flow_rate), flow_buffer_index);

	if (flow_buffer_index < 0 || flow_buffer_index >= PLATEAU_WINDOW_SIZE) {
		LOG_ERR("Invalid buffer_index: %d", flow_buffer_index);
		return false;
	}

	flow_buffer[flow_buffer_index] = flow_rate;
	flow_buffer_index = (flow_buffer_index + 1) % PLATEAU_WINDOW_SIZE;

	bool buffer_full = (flow_buffer_index == 0);

	if (buffer_full) {
		LOG_DBG("Buffer full, triggering calibration");
		flow_analyzer_calibrate_plateau();
	}

	bool is_first_value = fixed_eq(prev_flow, FIXED_MIN);

	if (is_first_value) {
		LOG_DBG("First flow value, setting prev_flow: %.3f",
			(double)fixed_to_float(flow_rate));
		prev_flow = flow_rate;
		return false;
	}

	fixed_t current_prev_flow = prev_flow;
	fixed_t current_noise_std = noise_std;

	fixed_t delta = fixed_abs(fixed_sub(flow_rate, current_prev_flow));
	fixed_t epsilon = fixed_mul(k_factor, current_noise_std);

	if (fixed_eq(current_noise_std, 0)) {
		epsilon = FIXED_EPSILON_FALLBACK;
	}

	LOG_DBG("Calculated delta: %.4f, epsilon: %.4f, prev_flow: %.3f",
		(double)fixed_to_float(delta), (double)fixed_to_float(epsilon),
		(double)fixed_to_float(current_prev_flow));

	if (fixed_lt(delta, epsilon)) {
		flow_diff_count++;
		LOG_DBG("Delta < epsilon, flow_diff_count: %d", flow_diff_count);

		if (flow_diff_count >= PLATEAU_CONFIRM_COUNT) {
			LOG_DBG("Plateau detected (flow_diff_count >= %d)",
				PLATEAU_CONFIRM_COUNT);
			return true;
		}
	} else {
		LOG_DBG("Delta >= epsilon, resetting flow_diff_count");
		flow_diff_count = 0;
	}

	prev_flow = flow_rate;

	LOG_DBG("Updated prev_flow: %.3f", (double)fixed_to_float(flow_rate));

	return false;
}

void flow_analyzer_reset(void)
{
	flow_buffer_index = 0;
	prev_flow = FIXED_MIN;
	flow_diff_count = 0;
	noise_std = 0;
	flow_slope = 0;
	memset(flow_buffer, 0, sizeof(flow_buffer));

	LOG_INF("Flow analyzer reset complete");
}

fixed_t flow_analyzer_get_noise_std(void)
{
	return noise_std;
}

fixed_t flow_analyzer_get_flow_slope(void)
{
	return flow_slope;
}