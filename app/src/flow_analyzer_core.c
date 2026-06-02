/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure core implementation of the flow plateau detection algorithm.
 * No Zephyr or platform dependencies.
 */

#include "flow_analyzer_core.h"

#include <string.h>

/* Fallback if noise_std is still zero (matches original behavior) */
#ifndef FIXED_EPSILON_FALLBACK
#define FIXED_EPSILON_FALLBACK fixed_from_float(0.01f)
#endif

#ifndef FIXED_PLATEAU_MIN_SLOPE
#define FIXED_PLATEAU_MIN_SLOPE fixed_from_float(0.01f)
#endif

void flow_analyzer_core_init(struct flow_analyzer_core_state *state,
                             int window_size,
                             int confirm_count)
{
    if (!state) return;

    memset(state, 0, sizeof(*state));

    state->window_size   = (window_size > 0 && window_size <= FLOW_ANALYZER_CORE_MAX_WINDOW)
                           ? window_size : 5;
    state->confirm_count = (confirm_count > 0) ? confirm_count : 2;

    state->prev_flow = FIXED_MIN;
}

void flow_analyzer_core_calibrate(struct flow_analyzer_core_state *state)
{
    if (!state || state->buffer_index != 0) {
        return; /* only calibrate on full buffer */
    }

    fixed_t sum_diffs = 0;

    for (int i = 0; i < state->window_size - 1; i++) {
        fixed_t diff = fixed_sub(state->buffer[(state->buffer_index + i + 1) % state->window_size],
                                 state->buffer[(state->buffer_index + i)     % state->window_size]);
        sum_diffs = fixed_add(sum_diffs, diff);
    }

    state->slope = fixed_div_int(sum_diffs, state->window_size - 1);

    if (fixed_gt(fixed_abs(state->slope), FIXED_PLATEAU_MIN_SLOPE)) {
        fixed_t sum_sq_res = 0;

        for (int i = 0; i < state->window_size; i++) {
            fixed_t slope_term = fixed_mul_int(state->slope, i);
            fixed_t predicted  = fixed_add(state->buffer[0], slope_term);
            fixed_t residual   = fixed_sub(state->buffer[i], predicted);
            sum_sq_res = fixed_add(sum_sq_res, fixed_mul(residual, residual));
        }

        fixed_t mean_sq_res = fixed_div_int(sum_sq_res, state->window_size);
        state->noise_std = fixed_sqrt(fixed_abs(mean_sq_res));
    } else {
        state->noise_std = 0;
    }
}

bool flow_analyzer_core_detect_plateau(struct flow_analyzer_core_state *state,
                                       fixed_t flow_rate,
                                       fixed_t k_factor)
{
    if (!state) return false;

    /* Add to circular buffer */
    state->buffer[state->buffer_index] = flow_rate;
    state->buffer_index = (state->buffer_index + 1) % state->window_size;

    bool buffer_full = (state->buffer_index == 0);

    if (buffer_full) {
        flow_analyzer_core_calibrate(state);
    }

    if (fixed_eq(state->prev_flow, FIXED_MIN)) {
        state->prev_flow = flow_rate;
        return false;
    }

    fixed_t delta   = fixed_abs(fixed_sub(flow_rate, state->prev_flow));
    fixed_t epsilon = fixed_mul(k_factor, state->noise_std);

    if (fixed_eq(state->noise_std, 0)) {
        epsilon = FIXED_EPSILON_FALLBACK;
    }

    if (fixed_lt(delta, epsilon)) {
        state->diff_count++;

        if (state->diff_count >= state->confirm_count) {
            return true; /* Plateau confirmed */
        }
    } else {
        state->diff_count = 0;
    }

    state->prev_flow = flow_rate;
    return false;
}

fixed_t flow_analyzer_core_get_noise_std(const struct flow_analyzer_core_state *state)
{
    return state ? state->noise_std : 0;
}

fixed_t flow_analyzer_core_get_slope(const struct flow_analyzer_core_state *state)
{
    return state ? state->slope : 0;
}

bool flow_analyzer_core_is_calibrated(const struct flow_analyzer_core_state *state)
{
    if (!state) return false;
    return (state->buffer_index == 0);
}

int flow_analyzer_core_get_diff_count(const struct flow_analyzer_core_state *state)
{
    return state ? state->diff_count : 0;
}
