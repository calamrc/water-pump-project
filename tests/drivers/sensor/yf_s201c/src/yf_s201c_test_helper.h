/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Test helper for the YF-S201C flow sensor driver.
 * Provides utilities to generate controlled synthetic pulses using gpio_emul.
 *
 * This allows unit/emulation testing of the driver (and later the FlowSensorService)
 * without any physical hardware.
 */

#ifndef YF_S201C_TEST_HELPER_H_
#define YF_S201C_TEST_HELPER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/kernel.h>

/**
 * @brief Generate a single pulse (falling edge) on the emulated GPIO.
 *
 * The YF-S201C driver triggers on falling edge.
 *
 * @param gpio_dev Emulated GPIO device
 * @param pin Pin number the driver is configured on
 */
static inline void yf_s201c_test_pulse(const struct device *gpio_dev, gpio_pin_t pin)
{
	/* Go high first (in case it was low), then falling edge */
	gpio_emul_input_set(gpio_dev, pin, 1);
	k_busy_wait(1);  /* very short high time */
	gpio_emul_input_set(gpio_dev, pin, 0);
}

/**
 * @brief Generate a train of pulses with a specific period between falling edges.
 *
 * @param gpio_dev Emulated GPIO device
 * @param pin Pin number
 * @param period_us Desired period between pulses (in microseconds)
 * @param count Number of pulses to generate
 */
static inline void yf_s201c_test_pulse_train(const struct device *gpio_dev,
					     gpio_pin_t pin,
					     uint32_t period_us,
					     uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		yf_s201c_test_pulse(gpio_dev, pin);
		if (period_us > 0) {
			k_busy_wait(period_us);
		}
	}
}

/**
 * @brief Generate pulses that simulate a specific flow rate.
 *
 * @param gpio_dev Emulated GPIO device
 * @param pin Pin number
 * @param pulses_per_liter Pulses per liter (from DT)
 * @param flow_rate_lpm Desired flow rate in liters per minute
 * @param duration_ms How long to generate pulses for
 */
static inline void yf_s201c_test_simulate_flow(const struct device *gpio_dev,
					       gpio_pin_t pin,
					       uint32_t pulses_per_liter,
					       float flow_rate_lpm,
					       uint32_t duration_ms)
{
	if (flow_rate_lpm <= 0.0f || pulses_per_liter == 0) {
		return;
	}

	/* pulses per second = (flow_lpm / 60) * pulses_per_liter */
	float pps = (flow_rate_lpm / 60.0f) * pulses_per_liter;
	if (pps <= 0) {
		return;
	}

	uint32_t period_us = (uint32_t)(1000000.0f / pps);
	uint32_t total_pulses = (uint32_t)((duration_ms * pps) / 1000.0f);

	yf_s201c_test_pulse_train(gpio_dev, pin, period_us, total_pulses);
}

#endif /* YF_S201C_TEST_HELPER_H_ */
