/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLOW_ANALYZER_H
#define FLOW_ANALYZER_H

#include <stdbool.h>
#include "fixed_math.h"

/**
 * @brief Flow Analyzer Module
 *
 * Handles flow stability analysis and plateau detection:
 * - Statistical analysis of flow measurements
 * - Plateau detection algorithms using fixed-point math
 * - Noise characterization and adaptive thresholding
 * - Flow trend analysis and stability assessment
 */

/* ============================================================================
 * External Variable Declarations (for Phase 2 compatibility)
 * ============================================================================ */

// Flow analysis buffers and state
extern fixed_t flow_buffer[5];
extern fixed_t flow_slope;
extern fixed_t noise_std;
extern fixed_t prev_flow;
extern int flow_buffer_index;
extern int flow_diff_count;

// Analysis configuration constants
extern const int PLATEAU_WINDOW_SIZE;
extern const int PLATEAU_CONFIRM_COUNT;

/* ============================================================================
 * External Function Declarations (Phase 2 interfaces)
 * ============================================================================ */

/**
 * @brief Initialize the flow analyzer module
 *
 * Sets up flow analysis buffers and initializes state variables.
 * Must be called before any flow analysis operations.
 *
 * @return 0 on success, negative error code on failure
 */
int flow_analyzer_init(void);

/**
 * @brief Analyze flow measurement for plateau stability
 *
 * Core plateau detection algorithm using statistical analysis:
 * - Adds measurement to sliding window buffer
 * - Performs noise calibration when buffer fills
 * - Compares against adaptive epsilon threshold
 * - Tracks consecutive stable measurements
 *
 * @param flow_rate Fixed-point flow rate measurement (L/min)
 * @param k_factor Fixed-point multiplier for noise threshold (e.g., 2.0 for 2-sigma detection)
 * @return true if plateau detected, false otherwise
 */
bool flow_analyzer_detect_plateau(fixed_t flow_rate, fixed_t k_factor);

/**
 * @brief Perform statistical calibration of flow characteristics
 *
 * Analyzes current flow buffer to determine slope and noise:
 * - Computes trend slope from measurement differences
 * - Calculates residual noise for significant slopes
 * - Updates global calibration parameters
 *
 * Called automatically when flow buffer becomes full.
 */
void flow_analyzer_calibrate_plateau(void);

/**
 * @brief Check if flow analysis data is calibrated
 *
 * Validates that sufficient measurements have been collected
 * for reliable plateau detection.
 *
 * @return true if analysis is ready, false if still calibrating
 */
bool flow_analyzer_is_calibrated(void);

/**
 * @brief Reset flow analysis state
 *
 * Clears buffers and resets calibration state.
 * Used for error recovery or when switching measurement contexts.
 */
void flow_analyzer_reset(void);

/**
 * @brief Get current noise standard deviation
 *
 * Returns the current estimate of measurement noise.
 * Used for adaptive thresholding and diagnostics.
 *
 * @return Noise standard deviation in fixed-point format
 */
fixed_t flow_analyzer_get_noise_std(void);

/**
 * @brief Get current flow trend slope
 *
 * Returns the calculated slope of recent flow measurements.
 * Positive values indicate increasing flow, negative decreasing.
 *
 * @return Flow slope in fixed-point format (L/min per sample)
 */
fixed_t flow_analyzer_get_flow_slope(void);

/* ============================================================================
 * Configuration Constants (for Phase 2 compatibility)
 * ============================================================================ */

#define FLOW_ANALYZER_BUFFER_SIZE 5
#define FLOW_ANALYZER_EPSILON_FALLBACK FIXED_EPSILON_FALLBACK

#endif /* FLOW_ANALYZER_H */
