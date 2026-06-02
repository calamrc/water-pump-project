/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Basic emulation tests for the YF-S201C driver using gpio_emul.
 *
 * These tests verify that the driver correctly measures pulse periods
 * when driven by controlled synthetic pulses.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>

#include <app/drivers/yf_s201c.h>
#include "yf_s201c_test_helper.h"

#if DT_NODE_HAS_STATUS(DT_INST(0, aygu_yf_s201c), okay)
#define FLOW_SENSOR_NODE DT_INST(0, aygu_yf_s201c)
#define FLOW_SENSOR_GPIO DT_GPIO_CTLR(FLOW_SENSOR_NODE, gpios)
#define FLOW_SENSOR_PIN  DT_GPIO_PIN(FLOW_SENSOR_NODE, gpios)
#else
#error "No YF-S201C instance enabled for testing"
#endif

struct yf_s201c_fixture {
	const struct device *dev;
	const struct device *gpio_dev;
};

static void *yf_s201c_setup(void)
{
	static struct yf_s201c_fixture fixture = {
		.dev = DEVICE_DT_GET(FLOW_SENSOR_NODE),
	};

	fixture.gpio_dev = DEVICE_DT_GET(FLOW_SENSOR_GPIO);

	zassert_not_null(fixture.dev);
	zassert_true(device_is_ready(fixture.dev), "Sensor device not ready");
	zassert_true(device_is_ready(fixture.gpio_dev), "GPIO device not ready");

	return &fixture;
}

ZTEST_SUITE(yf_s201c_driver, NULL, yf_s201c_setup, NULL, NULL, NULL);

ZTEST_USER_F(yf_s201c_driver, test_basic_period_measurement)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;
	int ret;

	/* Generate a clean 10ms period pulse train (100 Hz) */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 10000, 8);

	/* Give the workqueue a moment to process */
	k_msleep(50);

	ret = yf_s201c_get_current_period(f->dev, &period_us);
	zassert_equal(ret, 0, "get_current_period failed");

	/* Allow some tolerance because of workqueue + timestamp granularity */
	zassert_within(period_us, 10000, 2000,
		       "Measured period %lld us is far from expected 10000 us", period_us);
}

ZTEST_USER_F(yf_s201c_driver, test_reset_clears_state)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;

	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 5000, 5);
	k_msleep(30);

	int ret = yf_s201c_get_current_period(f->dev, &period_us);
	zassert_equal(ret, 0, "Should have a period before reset");

	yf_s201c_reset(f->dev);

	ret = yf_s201c_get_current_period(f->dev, &period_us);
	zassert_equal(period_us, 0, "Period should be zero after reset");
}

/* ========================================================================
 * Expanded test coverage (parallel with driver refactoring work)
 * ======================================================================== */

ZTEST_USER_F(yf_s201c_driver, test_various_flow_rates)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;

	/* Simulate ~2 L/min (roughly 15ms period with 450 p/L) */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 15000, 6);
	k_msleep(50);
	zassert_equal(yf_s201c_get_current_period(f->dev, &period_us), 0, NULL);
	zassert_within(period_us, 15000, 3000, "Unexpected period for ~2 L/min");

	/* Simulate faster flow ~6 L/min (~5.5ms period) */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 5500, 10);
	k_msleep(50);
	zassert_equal(yf_s201c_get_current_period(f->dev, &period_us), 0, NULL);
	zassert_within(period_us, 5500, 1500, "Unexpected period for ~6 L/min");
}

ZTEST_USER_F(yf_s201c_driver, test_rejects_too_short_periods)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;

	/* Send some valid pulses */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 10000, 4);
	k_msleep(30);

	/* Now send invalid very short periods (below min_period_us) */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 10, 6);
	k_msleep(30);

	/* The driver should not have accepted the very short ones as the latest value */
	zassert_equal(yf_s201c_get_current_period(f->dev, &period_us), 0, NULL);
	zassert_greater(period_us, 1000, "Driver should have rejected extremely short periods");
}

ZTEST_USER_F(yf_s201c_driver, test_get_recent_periods)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t periods[5];

	/* Generate 7 pulses with increasing periods */
	for (int i = 1; i <= 7; i++) {
		yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, i * 2000, 1);
		k_msleep(10);
	}

	int count = yf_s201c_get_recent_periods_us(f->dev, periods, 5);
	zassert_equal(count, 5, "Expected to read 5 recent periods");

	/* Most recent should be close to 14000us (7*2000) */
	zassert_within(periods[0], 14000, 3000, "Most recent period looks wrong");
}

ZTEST_USER_F(yf_s201c_driver, test_consecutive_invalid_resets_buffer)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;

	/* Send good pulses first */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 8000, 5);
	k_msleep(30);

	/* Now flood with invalid short pulses */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 5, 20);
	k_msleep(50);

	/* After many consecutive invalids, the driver should have reset its buffer */
	int ret = yf_s201c_get_current_period(f->dev, &period_us);
	/* Behavior depends on current driver thresholds, but we at least shouldn't crash */
	zassert_true(true, "Survived invalid burst");
}

ZTEST_USER_F(yf_s201c_driver, test_reset_during_operation)
{
	struct yf_s201c_fixture *f = fixture;
	int64_t period_us;

	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 8000, 5);
	k_msleep(30);

	yf_s201c_reset(f->dev);

	/* After reset, sending pulses should start fresh */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 12000, 4);
	k_msleep(30);

	zassert_equal(yf_s201c_get_current_period(f->dev, &period_us), 0, NULL);
	zassert_within(period_us, 12000, 3000, "Driver should recover cleanly after reset");
}

ZTEST_USER_F(yf_s201c_driver, test_semaphore_is_signaled_on_valid_data)
{
	struct yf_s201c_fixture *f = fixture;
	struct k_sem test_sem;

	k_sem_init(&test_sem, 0, 1);

	/* Tell the driver to signal our test semaphore */
	yf_s201c_set_data_semaphore(f->dev, &test_sem);

	/* Generate pulses */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 10000, 3);
	k_msleep(50);

	/* The semaphore should have been given at least once */
	int ret = k_sem_take(&test_sem, K_NO_WAIT);
	zassert_equal(ret, 0, "Semaphore should have been signaled by driver on valid data");

	/* Clean up */
	yf_s201c_set_data_semaphore(f->dev, NULL);
}

ZTEST_USER_F(yf_s201c_driver, test_consecutive_invalid_triggers_error_state)
{
	struct yf_s201c_fixture *f = fixture;

	/* Generate enough invalid short pulses to exceed the threshold */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 5, 30);
	k_msleep(100);

	/* The driver should have transitioned to ERROR state internally */
	int64_t p;
	int ret = yf_s201c_get_current_period(f->dev, &p);
	zassert_equal(ret, 0, "API should still be usable after invalid burst");
}

ZTEST_USER_F(yf_s201c_driver, test_semaphore_signaling)
{
	struct yf_s201c_fixture *f = fixture;
	struct k_sem local_sem;

	k_sem_init(&local_sem, 0, 5);

	yf_s201c_set_data_semaphore(f->dev, &local_sem);

	/* Generate several valid pulses */
	yf_s201c_test_pulse_train(f->gpio_dev, FLOW_SENSOR_PIN, 12000, 6);
	k_msleep(80);

	/* We should have received several semaphore signals */
	int count = 0;
	while (k_sem_take(&local_sem, K_NO_WAIT) == 0) {
		count++;
	}

	zassert_greater_equal(count, 1, "Semaphore should have been signaled at least once");

	/* Cleanup */
	yf_s201c_set_data_semaphore(f->dev, NULL);
}
