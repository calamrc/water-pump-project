/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(flow_analyzer);

#include "flow_analyzer.h"
#include "fixed_math.h"

/* ============================================================================
 * Module Variables (Static to module)
 * ============================================================================ */

// Flow analysis buffers and state
fixed_t flow_buffer[5] = {0};
fixed_t flow_slope = 0;
fixed_t noise_std = 0;
fixed_t prev_flow = FIXED_MIN; // Use FIXED_MIN as sentinel value instead of NAN
int flow_buffer_index = 0;
int flow_diff_count = 0;

// Analysis configuration constants
const int PLATEAU_WINDOW_SIZE = 5;
const int PLATEAU_CONFIRM_COUNT = 2;

// Thread safety mutex
K_MUTEX_DEFINE(flow_buffer_mutex);

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

int flow_analyzer_init(void)
{
    // Reset all flow analysis state
    flow_buffer_index = 0;
    prev_flow = FIXED_MIN;
    flow_diff_count = 0;
    noise_std = 0;
    flow_slope = 0;
    memset(flow_buffer, 0, sizeof(flow_buffer));

    LOG_INF("Flow analyzer initialized successfully");
    return 0;
}

bool flow_analyzer_detect_plateau(fixed_t flow_rate)
{
    LOG_DBG("detect_plateau called with flow_rate: %.3f, buffer_index: %d",
            fixed_to_float(flow_rate), flow_buffer_index);

    // Add to circular buffer
    flow_buffer[flow_buffer_index] = flow_rate;
    flow_buffer_index = (flow_buffer_index + 1) % PLATEAU_WINDOW_SIZE;

    LOG_DBG("Added to buffer, new buffer_index: %d", flow_buffer_index);

    if (flow_buffer_index == 0) { // Buffer full, calibrate
        LOG_DBG("Buffer full, triggering calibration");
        flow_analyzer_calibrate_plateau();
    }

    if (fixed_eq(prev_flow, FIXED_MIN)) { // First value (using FIXED_MIN as sentinel)
        LOG_DBG("First flow value, setting prev_flow: %.3f", fixed_to_float(flow_rate));

        prev_flow = flow_rate;
        return false;
    }

    // Calculate delta and epsilon using fixed-point arithmetic
    fixed_t delta = fixed_abs(fixed_sub(flow_rate, prev_flow));
    fixed_t epsilon = fixed_mul(FIXED_PLATEAU_K_FACTOR, noise_std);

    // Use fallback epsilon if no calibration
    if (fixed_eq(noise_std, 0)) {
        epsilon = FIXED_EPSILON_FALLBACK;
    }

    LOG_DBG("Calculated delta: %.4f, epsilon: %.4f, prev_flow: %.3f",
            fixed_to_float(delta), fixed_to_float(epsilon), fixed_to_float(prev_flow));

    if (fixed_lt(delta, epsilon)) {
        flow_diff_count++;
        LOG_DBG("Delta < epsilon, flow_diff_count: %d", flow_diff_count);

        if (flow_diff_count >= PLATEAU_CONFIRM_COUNT) {
            LOG_DBG("Plateau detected (flow_diff_count >= %d)", PLATEAU_CONFIRM_COUNT);
            return true; // Plateau detected
        }
    } else {
        LOG_DBG("Delta >= epsilon, resetting flow_diff_count");
        flow_diff_count = 0;
    }

    prev_flow = flow_rate;

    LOG_DBG("Updated prev_flow: %.3f", fixed_to_float(prev_flow));

    return false;
}

void flow_analyzer_calibrate_plateau(void) {
    if (flow_buffer_index != 0) // Calibrate only when buffer full
        return;

    // Compute differences for slope using fixed-point arithmetic
    fixed_t sum_diffs = 0;

    for (int i = 0; i < PLATEAU_WINDOW_SIZE - 1; i++) {
        fixed_t diff = fixed_sub(flow_buffer[(flow_buffer_index + i + 1) % PLATEAU_WINDOW_SIZE],
                                flow_buffer[(flow_buffer_index + i) % PLATEAU_WINDOW_SIZE]);
        sum_diffs = fixed_add(sum_diffs, diff);
    }

    flow_slope = fixed_div_int(sum_diffs, PLATEAU_WINDOW_SIZE - 1);

    // Check if slope is significant using fixed-point comparison
    if (fixed_gt(fixed_abs(flow_slope), FIXED_PLATEAU_MIN_SLOPE)) {
        // Predict linear values
        fixed_t predicted[PLATEAU_WINDOW_SIZE];

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            fixed_t slope_term = fixed_mul_int(flow_slope, i);
            predicted[i] = fixed_add(flow_buffer[0], slope_term);
        }

        // Compute residuals and noise std using fixed-point
        fixed_t sum_sq_res = 0;

        for (int i = 0; i < PLATEAU_WINDOW_SIZE; i++) {
            fixed_t residual = fixed_sub(flow_buffer[i], predicted[i]);
            sum_sq_res = fixed_add(sum_sq_res, fixed_mul(residual, residual));
        }

        // Calculate standard deviation
        fixed_t mean_sq_res = fixed_div_int(sum_sq_res, PLATEAU_WINDOW_SIZE);
        noise_std = fixed_sqrt(fixed_abs(mean_sq_res));
    } else {
        // Slope negligible - assume plateau, no noise calculation needed
        noise_std = 0;
    }

    LOG_DBG("Calibration complete, noise_std: %.4f, flow_slope: %.4f",
            fixed_to_float(noise_std), fixed_to_float(flow_slope));
}

bool flow_analyzer_is_calibrated(void)
{
    // Considered calibrated if buffer is full and we have noise estimate
    return (flow_buffer_index == 0); // Buffer wraps when full
}

void flow_analyzer_reset(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);

    // Clear all flow analysis state
    flow_buffer_index = 0;
    prev_flow = FIXED_MIN;
    flow_diff_count = 0;
    noise_std = 0;
    flow_slope = 0;
    memset(flow_buffer, 0, sizeof(flow_buffer));

    k_mutex_unlock(&flow_buffer_mutex);

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
