/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulation tests for the feedback relay driver using gpio-emul.
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>

#include <app/drivers/feedback_relay.h>

#if DT_NODE_HAS_STATUS(DT_INST(0, aygu_feedback_relay), okay)
#define FEEDBACK_RELAY_NODE DT_INST(0, aygu_feedback_relay)
#define FEEDBACK_RELAY_GPIO DT_GPIO_CTLR(FEEDBACK_RELAY_NODE, gpios)
#define FEEDBACK_RELAY_PIN  DT_GPIO_PIN(FEEDBACK_RELAY_NODE, gpios)
#else
#error "No feedback relay instance enabled for testing"
#endif

ZTEST_SUITE(feedback_relay_driver, NULL, NULL, NULL, NULL, NULL);

ZTEST(feedback_relay_driver, test_click_pulse)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);

	zassert_true(device_is_ready(relay), "Feedback relay not ready");

	/* A click should be a short pulse */
	int ret = feedback_relay_click(relay);
	zassert_equal(ret, 0, "click should succeed");

	/* Give the workqueue time to process the delayed OFF */
	k_msleep(100);

	bool is_on = false;
	ret = feedback_relay_get_state(relay, &is_on);
	zassert_equal(ret, 0, "get_state failed");
	zassert_false(is_on, "Relay should be OFF after click pulse completes");
}

ZTEST(feedback_relay_driver, test_pulse_duration)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);
	zassert_true(device_is_ready(relay));

	/* Request a 200ms pulse */
	int ret = feedback_relay_pulse(relay, 200);
	zassert_equal(ret, 0, "pulse should succeed");

	/* Immediately after scheduling, it should be ON */
	bool is_on;
	ret = feedback_relay_get_state(relay, &is_on);
	zassert_equal(ret, 0, NULL);
	zassert_true(is_on, "Relay should be ON right after starting pulse");

	/* Wait longer than the pulse duration */
	k_msleep(300);

	ret = feedback_relay_get_state(relay, &is_on);
	zassert_equal(ret, 0, NULL);
	zassert_false(is_on, "Relay should be OFF after pulse duration");
}

ZTEST(feedback_relay_driver, test_explicit_on_off)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);
	zassert_true(device_is_ready(relay));

	int ret = feedback_relay_on(relay);
	zassert_equal(ret, 0, NULL);

	bool is_on;
	ret = feedback_relay_get_state(relay, &is_on);
	zassert_equal(ret, 0, NULL);
	zassert_true(is_on, "Should be ON after explicit ON");

	ret = feedback_relay_off(relay);
	zassert_equal(ret, 0, NULL);

	ret = feedback_relay_get_state(relay, &is_on);
	zassert_equal(ret, 0, NULL);
	zassert_false(is_on, "Should be OFF after explicit OFF");
}

ZTEST(feedback_relay_driver, test_pulse_zero_duration_rejected)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);
	zassert_true(device_is_ready(relay));

	int ret = feedback_relay_pulse(relay, 0);
	zassert_not_equal(ret, 0, "Zero duration pulse should be rejected");
}

ZTEST(feedback_relay_driver, test_pulse_override)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);
	zassert_true(device_is_ready(relay));

	/* Start a long pulse */
	feedback_relay_pulse(relay, 5000);

	/* Immediately override with a short one */
	feedback_relay_pulse(relay, 50);

	k_msleep(100);

	bool is_on;
	feedback_relay_get_state(relay, &is_on);
	zassert_false(is_on, "Short overriding pulse should have completed");
}

ZTEST(feedback_relay_driver, test_get_state_null_pointer)
{
	const struct device *relay = DEVICE_DT_GET(FEEDBACK_RELAY_NODE);
	zassert_true(device_is_ready(relay));

	int ret = feedback_relay_get_state(relay, NULL);
	zassert_not_equal(ret, 0, "Passing NULL to get_state should fail");
}
