/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr wrapper around the pure flow analyzer core.
 *
 * This file provides the original global + mutex-protected API that the
 * rest of the firmware expects, while delegating the actual algorithm
 * to the portable core in flow_analyzer_core.c.
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>

#include "flow_analyzer.h"
#include "flow_analyzer_core.h"
#include "fixed_math.h"

LOG_MODULE_REGISTER(flow_analyzer);

/* Single global instance protected by mutex (original behavior) */
static struct flow_analyzer_core_state g_core_state;
static K_MUTEX_DEFINE(flow_buffer_mutex);

/* Configuration pulled from Kconfig at runtime */
static int g_window_size;
static int g_confirm_count;

int flow_analyzer_init(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);

    g_window_size   = CONFIG_APP_PLATEAU_WINDOW_SIZE;
    g_confirm_count = CONFIG_APP_PLATEAU_CONFIRM_COUNT;

    flow_analyzer_core_init(&g_core_state, g_window_size, g_confirm_count);

    k_mutex_unlock(&flow_buffer_mutex);

    LOG_INF("Flow analyzer initialized (using portable core)");
    return 0;
}

bool flow_analyzer_detect_plateau(fixed_t flow_rate, fixed_t k_factor)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);

    bool result = flow_analyzer_core_detect_plateau(&g_core_state, flow_rate, k_factor);

    k_mutex_unlock(&flow_buffer_mutex);

    return result;
}

void flow_analyzer_calibrate_plateau(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
    flow_analyzer_core_calibrate(&g_core_state);
    k_mutex_unlock(&flow_buffer_mutex);
}

bool flow_analyzer_is_calibrated(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
    bool result = flow_analyzer_core_is_calibrated(&g_core_state);
    k_mutex_unlock(&flow_buffer_mutex);
    return result;
}

void flow_analyzer_reset(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);

    g_window_size   = CONFIG_APP_PLATEAU_WINDOW_SIZE;
    g_confirm_count = CONFIG_APP_PLATEAU_CONFIRM_COUNT;

    flow_analyzer_core_init(&g_core_state, g_window_size, g_confirm_count);

    k_mutex_unlock(&flow_buffer_mutex);

    LOG_INF("Flow analyzer reset");
}

fixed_t flow_analyzer_get_noise_std(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
    fixed_t std = flow_analyzer_core_get_noise_std(&g_core_state);
    k_mutex_unlock(&flow_buffer_mutex);
    return std;
}

fixed_t flow_analyzer_get_flow_slope(void)
{
    k_mutex_lock(&flow_buffer_mutex, K_FOREVER);
    fixed_t slope = flow_analyzer_core_get_slope(&g_core_state);
    k_mutex_unlock(&flow_buffer_mutex);
    return slope;
}
