/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure, zero-dependency module that encapsulates the original flow
 * stabilization contract from the master branch pump_controller_thread.
 *
 * This is the single source of truth for:
 *   - Phase-aware K-factor selection (INITIAL 2.0 when pump off, NORMAL 3.0 when on)
 *   - Requiring multiple confirmed samples after calibration before declaring plateau
 *   - Explicit reset of analyzer state after acting on a plateau
 *   - 1.5x dynamic pulse-period watchdog while the pump is conceptually running
 *   - Deciding whether a plateau constitutes a real "demand" event (Phase A off->on)
 *
 * Both host_sim and (later) the real Zephyr services call this module.
 * It owns one flow_analyzer_core_state internally and has no Zephyr or OS dependencies.
 */

#ifndef FLOW_STABILIZATION_CONTRACT_H
#define FLOW_STABILIZATION_CONTRACT_H

#include <stdbool.h>
#include <stdint.h>

#include "fixed_math.h"
#include "flow_analyzer_core.h"

struct flow_contract_state {
    struct flow_analyzer_core_state analyzer;
    bool pump_is_on;
    int64_t last_plateau_period_us;
    double last_pulse_time_s;
    int window_size;
    int confirm_count;
    /* Tunable threshold below which a plateau is not considered a pump demand */
    fixed_t min_flow_for_demand;
};

struct flow_contract_result {
    bool plateau_confirmed;        /* Analyzer returned true after its full rules (raw detection) */

    /* Phase A (pump off): this is a valid demand to turn the pump on */
    bool demand_event;

    /* Phase B (pump already running): we captured a fresh, good stable pulse period.
     * Use this to update the 1.5x watchdog timer with the latest timing.
     * This does NOT mean "generate a new demand / re-plateau event to keep the pump on".
     */
    bool good_period_captured;

    bool watchdog_timeout;         /* Missed pulses while on -> caller should turn pump off */
    int64_t suggested_next_timeout_us; /* 1.5x value the caller can use for scheduling */
    fixed_t k_used;
    int diff_count;
    fixed_t noise_std;
    bool analyzer_was_reset;       /* True if we performed an explicit reset this call */
};

/**
 * @brief Initialize the contract state (equivalent to power-on or after full reset).
 */
void flow_contract_init(struct flow_contract_state *state);

/**
 * @brief Process one flow sample according to the original stabilization rules.
 *
 * @param state            Contract instance (must have been init'ed)
 * @param flow_rate        Current filtered flow rate (L/min, fixed-point)
 * @param period_us        Raw or filtered pulse period in microseconds (0 = no pulse)
 * @param current_time_s   Monotonic time in seconds (host_sim uses trace time; firmware can pass 0 or uptime/1000.0)
 * @param current_pump_on  Caller's current belief about whether the pump is running
 * @return Result flags and diagnostics. The caller decides what to do with demand_event / watchdog_timeout.
 */
struct flow_contract_result flow_contract_process_sample(
    struct flow_contract_state *state,
    fixed_t flow_rate,
    int64_t period_us,
    double current_time_s,
    bool current_pump_on
);

#endif /* FLOW_STABILIZATION_CONTRACT_H */