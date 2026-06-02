/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Implementation of the pure flow stabilization contract.
 * See the header for the design rationale (faithful reproduction of the
 * master branch pump_controller_thread + flow_analyzer usage).
 */

#include "pump/flow_stabilization_contract.h"

#include <string.h>

/* Reasonable defaults matching original Kconfig + observed behavior */
#define DEFAULT_WINDOW           5
#define DEFAULT_CONFIRM          2
#define DEFAULT_MIN_FLOW_DEMAND  fixed_from_float(0.28f)   /* tuned for realistic_20s scenario: first demand only on the deliberate low-flow stabilization (~0.35 L/min after t~2.5s) */
#define INITIAL_K                2.0f
#define NORMAL_K                 3.0f
#define WATCHDOG_MULTIPLIER      1.5
#define MAX_WATCHDOG_US          1000000   /* 1 second cap from original CONFIG_APP_MAX_TIMEOUT_US */

void flow_contract_init(struct flow_contract_state *state)
{
    if (!state) return;

    memset(state, 0, sizeof(*state));

    state->window_size = DEFAULT_WINDOW;
    state->confirm_count = DEFAULT_CONFIRM;
    state->min_flow_for_demand = DEFAULT_MIN_FLOW_DEMAND;

    flow_analyzer_core_init(&state->analyzer, state->window_size, state->confirm_count);
    state->pump_is_on = false;
    state->last_plateau_period_us = 0;
    state->last_pulse_time_s = 0.0;
}

struct flow_contract_result flow_contract_process_sample(
    struct flow_contract_state *state,
    fixed_t flow_rate,
    int64_t period_us,
    double current_time_s,
    bool current_pump_on)
{
    struct flow_contract_result res = {0};

    if (!state) return res;

    /* Update our internal view of pump state (caller is authoritative) */
    state->pump_is_on = current_pump_on;

    /* ------------------------------------------------------------------
     * 1.5x dynamic watchdog (Phase B) - exact original calculation
     * ------------------------------------------------------------------ */
    if (state->pump_is_on && state->last_plateau_period_us > 0) {
        double gap_s = current_time_s - state->last_pulse_time_s;
        double timeout_s = (state->last_plateau_period_us * WATCHDOG_MULTIPLIER) / 1000000.0;

        if (timeout_s > (MAX_WATCHDOG_US / 1000000.0)) {
            timeout_s = (MAX_WATCHDOG_US / 1000000.0);
        }

        res.suggested_next_timeout_us = (int64_t)(timeout_s * 1000000.0);

        if (gap_s > timeout_s) {
            /* Watchdog fired - behave exactly like the original thread on timeout */
            flow_analyzer_core_init(&state->analyzer, state->window_size, state->confirm_count);
            state->last_plateau_period_us = 0;
            state->pump_is_on = false;   /* caller should act on this */
            res.watchdog_timeout = true;
            res.analyzer_was_reset = true;
            state->last_pulse_time_s = current_time_s;
            return res;
        }
    }

    /* ------------------------------------------------------------------
     * Phase-aware K selection (exact original logic)
     * ------------------------------------------------------------------ */
    float k_float = state->pump_is_on ? NORMAL_K : INITIAL_K;
    fixed_t k_factor = fixed_from_float(k_float);
    res.k_used = k_factor;

    /* Run the real statistical detector */
    bool raw_plateau = flow_analyzer_core_detect_plateau(
        &state->analyzer,
        flow_rate,
        k_factor);

    fixed_t noise = flow_analyzer_core_get_noise_std(&state->analyzer);
    int diff = flow_analyzer_core_get_diff_count(&state->analyzer);

    res.diff_count = diff;
    res.noise_std = noise;

    bool calibrated = (noise > 0) || flow_analyzer_core_is_calibrated(&state->analyzer);
    bool confirmed = raw_plateau && calibrated;

    res.plateau_confirmed = confirmed;

    if (!confirmed) {
        state->last_pulse_time_s = current_time_s;
        return res;
    }

    /* We have a confirmed plateau from the analyzer core.
     *
     * Per the original contract:
     * - If pump is OFF: this can be a demand to turn it on (Phase A).
     * - If pump is already ON: we still want the latest good stable period
     *   so the 1.5x watchdog remains accurate. This is NOT a "re-demand" event.
     */

    /* Always capture the period when we have a good stable sample (for watchdog) */
    bool captured_good_period = false;
    if (period_us > 0 && period_us < 100000000LL) {
        state->last_plateau_period_us = period_us;
        captured_good_period = true;
    }
    res.suggested_next_timeout_us = (int64_t)((state->last_plateau_period_us * WATCHDOG_MULTIPLIER));

    /* ------------------------------------------------------------------
     * Explicit reset after acting on plateau (the critical original discipline)
     * ------------------------------------------------------------------ */
    flow_analyzer_core_init(&state->analyzer, state->window_size, state->confirm_count);
    res.analyzer_was_reset = true;

    /* Only generate a demand_event for the initial turn-on (Phase A).
     * While running, confirmed plateaus are only used to harvest fresh timing.
     */
    float rate_f = fixed_to_float(flow_rate);
    bool significant = (rate_f >= fixed_to_float(state->min_flow_for_demand)) ||
                       (period_us > 0 && period_us < 8000000LL);

    if (!state->pump_is_on && significant) {
        res.demand_event = true;
        state->pump_is_on = true;
    }

    if (captured_good_period && state->pump_is_on) {
        res.good_period_captured = true;   /* timing for watchdog refreshed */
    }

    state->last_pulse_time_s = current_time_s;

    return res;
}