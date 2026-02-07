/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_YF_S201C_YF_S201C_H_
#define ZEPHYR_DRIVERS_SENSOR_YF_S201C_YF_S201C_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Driver State Machine
 * ============================================================================ */

enum yf_s201c_state {
    YF_S201C_INIT,
    YF_S201C_READY,
    YF_S201C_RUNNING,
    YF_S201C_ERROR
};

/* ============================================================================
 * Configuration Structure (from Device Tree)
 * ============================================================================ */

struct yf_s201c_config {
    struct gpio_dt_spec gpio;
    uint32_t pulses_per_liter;
    uint32_t min_period_us;
    int buffer_size;
    int stale_threshold_ms;
    int consecutive_invalid_threshold;
};

/* ============================================================================
 * Runtime Data Structure
 * ============================================================================ */

struct yf_s201c_data {
    /* Device reference */
    const struct device *dev;

    /* GPIO and kernel objects */
    struct gpio_callback gpio_cb;
    struct k_work work;
    struct k_mutex mutex;

    /* State machine */
    enum yf_s201c_state state;

    /* Configuration (copied from config for runtime) */
    uint32_t pulses_per_liter;
    uint32_t min_period_us;
    int buffer_size;
    int stale_threshold_ms;
    int consecutive_invalid_threshold;

    /* Period buffer for median filtering */
    volatile int64_t period_buffer[5];
    volatile int64_t last_pulse_ticks;
    volatile int64_t period_us;
    volatile int valid_periods;
    volatile int buffer_index;

    /* ISR-to-workqueue communication */
    volatile int64_t isr_current_period_us;
    volatile bool isr_valid_update;

    /* Validation and statistics */
    volatile int64_t last_valid_update_ms;
    volatile int consecutive_invalid;
};

/* ============================================================================
 * Forward Declarations
 * ============================================================================ */

void yf_s201c_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
void yf_s201c_work_handler(struct k_work *work);

#endif /* ZEPHYR_DRIVERS_SENSOR_YF_S201C_YF_S201C_H_ */