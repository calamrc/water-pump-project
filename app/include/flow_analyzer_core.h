/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure, platform-independent core of the flow plateau detection algorithm.
 *
 * This module contains the actual statistical logic for detecting when
 * flow has stabilized. It has zero dependencies on Zephyr or any OS.
 *
 * It is intended to be used by:
 *   - The real firmware (via a thin Zephyr wrapper in flow_analyzer.c)
 *   - Host simulation (host_sim) directly for high-fidelity replay of plant traces
 */

#ifndef FLOW_ANALYZER_CORE_H
#define FLOW_ANALYZER_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "fixed_math.h"

/* Maximum supported window size (must be large enough for our Kconfig defaults) */
#define FLOW_ANALYZER_CORE_MAX_WINDOW 10

/**
 * @brief Opaque state for one instance of the flow analyzer.
 *
 * This replaces the previous global variables + mutex.
 * Each caller (real firmware instance or a host simulation) owns one.
 */
struct flow_analyzer_core_state {
    fixed_t buffer[FLOW_ANALYZER_CORE_MAX_WINDOW];
    fixed_t slope;
    fixed_t noise_std;
    fixed_t prev_flow;
    int     buffer_index;
    int     diff_count;
    int     window_size;            /* runtime configured */
    int     confirm_count;          /* runtime configured */
    bool    has_calibrated;         /* true after at least one real noise estimation */
};

/**
 * @brief Initialize the core analyzer state.
 *
 * Must be called before any other core functions.
 */
void flow_analyzer_core_init(struct flow_analyzer_core_state *state,
                             int window_size,
                             int confirm_count);

/**
 * @brief Core plateau detection logic.
 *
 * This is the pure version of the original flow_analyzer_detect_plateau.
 * It performs the sliding window, noise calibration, delta/epsilon check,
 * and consecutive confirmation counting.
 *
 * @param state     Analyzer instance state
 * @param flow_rate Current flow rate measurement
 * @param k_factor  Multiplier for noise threshold (e.g. 3.0 for 3-sigma)
 * @return true if a plateau has been confirmed, false otherwise
 */
bool flow_analyzer_core_detect_plateau(struct flow_analyzer_core_state *state,
                                       fixed_t flow_rate,
                                       fixed_t k_factor);

/**
 * @brief Perform calibration (slope + noise estimation).
 *
 * Exposed for advanced use / testing. Normally called internally.
 */
void flow_analyzer_core_calibrate(struct flow_analyzer_core_state *state);

/* Accessors for diagnostics / host simulation visibility */
fixed_t flow_analyzer_core_get_noise_std(const struct flow_analyzer_core_state *state);
fixed_t flow_analyzer_core_get_slope(const struct flow_analyzer_core_state *state);
bool    flow_analyzer_core_is_calibrated(const struct flow_analyzer_core_state *state);

/* Temporary debug accessor for understanding plateau decisions in simulation */
int flow_analyzer_core_get_diff_count(const struct flow_analyzer_core_state *state);

#endif /* FLOW_ANALYZER_CORE_H */
