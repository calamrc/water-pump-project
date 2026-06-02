/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the pure timer_state_machine module.
 * These tests have zero dependency on Zephyr kernel objects
 * (beyond ztest itself) and can run on native_sim or qemu_x86.
 */

#include <zephyr/ztest.h>
#include "timer/timer_state_machine.h"

ZTEST(timer_state_machine_tests, test_init_defaults)
{
	timer_sm_init();

	zassert_equal(timer_sm_get_state(), TIMER_STATE_SETTING, "Initial state should be SETTING");
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_DEFAULT_SECONDS, "Default duration");
}

ZTEST(timer_state_machine_tests, test_adjust_time_in_setting)
{
	timer_sm_init();

	timer_sm_adjust_time(30);
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_DEFAULT_SECONDS + 30, NULL);

	timer_sm_adjust_time(-20);
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_DEFAULT_SECONDS + 10, NULL);
}

ZTEST(timer_state_machine_tests, test_adjust_clamping)
{
	timer_sm_init();

	/* Try to go below minimum */
	timer_sm_adjust_time(-1000);
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_MIN_SECONDS, "Should clamp to MIN");

	/* Try to go above maximum */
	timer_sm_adjust_time(10000);
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_MAX_SECONDS, "Should clamp to MAX");
}

ZTEST(timer_state_machine_tests, test_start_from_setting)
{
	timer_sm_init();

	int ret = timer_sm_start();
	zassert_equal(ret, 0, "Start should succeed from SETTING");
	zassert_equal(timer_sm_get_state(), TIMER_STATE_RUNNING, NULL);
}

ZTEST(timer_state_machine_tests, test_cannot_start_from_wrong_state)
{
	timer_sm_init();
	timer_sm_start(); /* now RUNNING */

	int ret = timer_sm_start();
	zassert_not_equal(ret, 0, "Start should fail from RUNNING");
}

ZTEST(timer_state_machine_tests, test_pause_and_resume)
{
	timer_sm_init();
	timer_sm_start();

	int ret = timer_sm_pause();
	zassert_equal(ret, 0, "Pause should succeed");
	zassert_equal(timer_sm_get_state(), TIMER_STATE_PAUSED, NULL);

	ret = timer_sm_resume();
	zassert_equal(ret, 0, "Resume should succeed");
	zassert_equal(timer_sm_get_state(), TIMER_STATE_RUNNING, NULL);
}

ZTEST(timer_state_machine_tests, test_update_decrements_time)
{
	timer_sm_init();
	timer_sm_adjust_time(5); /* set to 65 seconds for easy testing */
	timer_sm_start();

	bool state_changed = timer_sm_update();
	zassert_false(state_changed, "Should not complete after one tick");
	zassert_equal(timer_sm_get_remaining_seconds(), 64, "Should decrement by 1");
}

ZTEST(timer_state_machine_tests, test_update_reaches_completed)
{
	timer_sm_init();
	/* Set to very short time */
	/* We have to use adjust carefully because of clamping, but we can tick many times */
	timer_sm_adjust_time(-50); /* Should land at MIN (10) */
	timer_sm_start();

	/* Tick 10 times */
	for (int i = 0; i < 10; i++) {
		timer_sm_update();
	}

	zassert_equal(timer_sm_get_state(), TIMER_STATE_COMPLETED, "Should be COMPLETED");
	zassert_true(timer_sm_is_completed(), NULL);
	zassert_false(timer_sm_is_active(), NULL);
}

ZTEST(timer_state_machine_tests, test_reset)
{
	timer_sm_init();
	timer_sm_start();
	timer_sm_update();

	timer_sm_reset();

	zassert_equal(timer_sm_get_state(), TIMER_STATE_SETTING, NULL);
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_DEFAULT_SECONDS, NULL);
}

ZTEST(timer_state_machine_tests, test_cannot_adjust_while_running)
{
	timer_sm_init();
	timer_sm_start();

	uint32_t before = timer_sm_get_remaining_seconds();
	timer_sm_adjust_time(100);
	uint32_t after = timer_sm_get_remaining_seconds();

	zassert_equal(before, after, "Should not allow adjustment while RUNNING");
}

ZTEST(timer_state_machine_tests, test_cannot_pause_when_not_running)
{
	timer_sm_init();

	int ret = timer_sm_pause();
	zassert_not_equal(ret, 0, "Pause should fail from SETTING");

	timer_sm_start();
	timer_sm_pause();
	ret = timer_sm_pause();
	zassert_not_equal(ret, 0, "Pause should fail from PAUSED");
}

ZTEST(timer_state_machine_tests, test_cannot_resume_when_not_paused)
{
	timer_sm_init();

	int ret = timer_sm_resume();
	zassert_not_equal(ret, 0, "Resume should fail from SETTING");

	timer_sm_start();
	ret = timer_sm_resume();
	zassert_not_equal(ret, 0, "Resume should fail from RUNNING");
}

ZTEST(timer_state_machine_tests, test_remaining_time_in_all_states)
{
	timer_sm_init();
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_DEFAULT_SECONDS, "SETTING");

	timer_sm_start();
	zassert_true(timer_sm_get_remaining_seconds() > 0, "RUNNING should have time");

	timer_sm_pause();
	zassert_true(timer_sm_get_remaining_seconds() > 0, "PAUSED should preserve time");

	/* Force completion by ticking */
	while (timer_sm_get_state() != TIMER_STATE_COMPLETED) {
		timer_sm_update();
	}
	zassert_equal(timer_sm_get_remaining_seconds(), 0, "COMPLETED should report 0");
}

ZTEST(timer_state_machine_tests, test_is_active_and_is_completed)
{
	timer_sm_init();
	zassert_false(timer_sm_is_active(), "SETTING is not active");
	zassert_false(timer_sm_is_completed(), NULL);

	timer_sm_start();
	zassert_true(timer_sm_is_active(), "RUNNING is active");

	timer_sm_pause();
	zassert_true(timer_sm_is_active(), "PAUSED is still considered active");

	while (timer_sm_get_state() != TIMER_STATE_COMPLETED) {
		timer_sm_update();
	}
	zassert_false(timer_sm_is_active(), NULL);
	zassert_true(timer_sm_is_completed(), NULL);
}

ZTEST(timer_state_machine_tests, test_minimum_time_edge)
{
	timer_sm_init();

	/* Set exactly to minimum */
	while (timer_sm_get_remaining_seconds() > TIMER_MIN_SECONDS) {
		timer_sm_adjust_time(-10);
	}
	zassert_equal(timer_sm_get_remaining_seconds(), TIMER_MIN_SECONDS, NULL);

	timer_sm_start();
	/* Tick it down */
	for (int i = 0; i < TIMER_MIN_SECONDS; i++) {
		timer_sm_update();
	}
	zassert_equal(timer_sm_get_state(), TIMER_STATE_COMPLETED, NULL);
}

ZTEST_SUITE(timer_state_machine_tests, NULL, NULL, NULL, NULL, NULL);
