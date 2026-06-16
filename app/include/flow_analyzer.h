/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLOW_ANALYZER_H
#define FLOW_ANALYZER_H

#include <stdbool.h>
#include "fixed_math.h"

/*
 * All functions below must only be called from the flow analyzer thread.
 * No mutex protection — single-threaded access is the safety guarantee.
 */

int flow_analyzer_init(void);
bool flow_analyzer_detect_plateau(fixed_t flow_rate, fixed_t k_factor);
void flow_analyzer_reset(void);
fixed_t flow_analyzer_get_noise_std(void);
fixed_t flow_analyzer_get_flow_slope(void);

#define FLOW_ANALYZER_EPSILON_FALLBACK FIXED_EPSILON_FALLBACK

#endif /* FLOW_ANALYZER_H */