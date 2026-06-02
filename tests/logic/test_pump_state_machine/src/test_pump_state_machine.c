/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the pure pump_state_machine module.
 * These tests have zero dependency on hardware or Zephyr kernel objects
 * (beyond ztest itself) and can run on native_sim.
 */

#include <zephyr/ztest.h>
#include "pump_state_machine.h"
#include "pump/pump_demand.h" /* pure demand policy tests (timer complete, period, etc) */
#include "pump/pump_data_types.h" /* for timer_pure_status in tests */
#include "fixed_math.h" /* for fixed_from_float in test inputs */

ZTEST(pump_state_machine_tests, test_initial_state)
{
	zassert_equal(PUMP_SM_STATE_OFF, 0, "OFF should be 0");
	/* COUNT is the sentinel, so actual state count is COUNT (value of the last enum) */
	zassert_true(PUMP_SM_STATE_COUNT >= 6, "Too few states defined");
}

ZTEST(pump_state_machine_tests, test_off_to_starting_on_plateau)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_OFF,
							PUMP_SM_EVENT_PLATEAU_DETECTED);
	zassert_equal(next, PUMP_SM_STATE_STARTING, "OFF + PLATEAU should -> STARTING");
}

ZTEST(pump_state_machine_tests, test_starting_to_running_on_plateau)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_STARTING,
							PUMP_SM_EVENT_PLATEAU_DETECTED);
	zassert_equal(next, PUMP_SM_STATE_RUNNING, "STARTING + PLATEAU should -> RUNNING");
}

ZTEST(pump_state_machine_tests, test_running_to_timeout_on_timeout_event)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_RUNNING,
							PUMP_SM_EVENT_TIMEOUT);
	zassert_equal(next, PUMP_SM_STATE_TIMEOUT, "RUNNING + TIMEOUT should -> TIMEOUT");
}

ZTEST(pump_state_machine_tests, test_running_emergency_stop_on_safety_timeout)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_RUNNING,
							PUMP_SM_EVENT_SAFETY_TIMEOUT);
	zassert_equal(next, PUMP_SM_STATE_OFF, "RUNNING + SAFETY_TIMEOUT should -> OFF");
}

ZTEST(pump_state_machine_tests, test_error_recovery_on_reset)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_ERROR,
							PUMP_SM_EVENT_RESET);
	zassert_equal(next, PUMP_SM_STATE_OFF, "ERROR + RESET should -> OFF");
}

ZTEST(pump_state_machine_tests, test_maintenance_exit)
{
	enum pump_sm_state next = pump_sm_process_event(PUMP_SM_STATE_MAINTENANCE,
							PUMP_SM_EVENT_MAINTENANCE_EXIT);
	zassert_equal(next, PUMP_SM_STATE_OFF, "MAINTENANCE + EXIT should -> OFF");
}

ZTEST(pump_state_machine_tests, test_invalid_inputs_safely_go_to_error)
{
	enum pump_sm_state next = pump_sm_process_event(99, PUMP_SM_EVENT_RESET);
	zassert_equal(next, PUMP_SM_STATE_ERROR, "Invalid state should map to ERROR");

	next = pump_sm_process_event(PUMP_SM_STATE_OFF, 99);
	zassert_equal(next, PUMP_SM_STATE_ERROR, "Invalid event should map to ERROR");
}

ZTEST(pump_state_machine_tests, test_is_active_helper)
{
	zassert_false(pump_sm_is_active(PUMP_SM_STATE_OFF), "OFF is not active");
	zassert_false(pump_sm_is_active(PUMP_SM_STATE_ERROR), "ERROR is not active");
	zassert_true(pump_sm_is_active(PUMP_SM_STATE_RUNNING), "RUNNING is active");
	zassert_true(pump_sm_is_active(PUMP_SM_STATE_STARTING), "STARTING is active");
}

ZTEST(pump_state_machine_tests, test_string_helpers)
{
	zassert_not_null(pump_sm_state_to_str(PUMP_SM_STATE_RUNNING), "state string");
	zassert_not_null(pump_sm_event_to_str(PUMP_SM_EVENT_PLATEAU_DETECTED), "event string");
	zassert_str_equal(pump_sm_state_to_str(PUMP_SM_STATE_OFF), "OFF", "OFF string");
}

/* --------------------------------------------------------------------------
 * Additional policy-oriented tests for the pure pump state machine.
 * Note: As of the latest architecture update, the TimerService is fully
 * independent from the PumpService. These tests exercise general state
 * machine behavior rather than any specific cross-service reconciliation.
 * -------------------------------------------------------------------------- */

ZTEST(pump_state_machine_tests, test_flow_drop_stops_pump)
{
	/* Flow stops while pump is running → should go to OFF */
	enum pump_sm_state s = PUMP_SM_STATE_RUNNING;
	s = pump_sm_process_event(s, PUMP_SM_EVENT_RESET);
	zassert_equal(s, PUMP_SM_STATE_OFF, "Flow drop should stop pump");
}

/* --------------------------------------------------------------------------
 * Pure demand policy tests (pump_demand_evaluate). Exercises the timer
 * complete -> force off path (PR3), always-propagate period, watchdog-ish
 * (timer before valid flow gate), and confirms plateau sole on-gate (no
 * timer turn-on).
 * -------------------------------------------------------------------------- */

ZTEST(pump_state_machine_tests, test_demand_plateau_on_when_off)
{
	struct flow_sample f = {
		.rate = fixed_from_float(1.5f),
		.valid = true,
		.plateau_detected = true,
		.period_us = 12345,
	};
	struct pump_demand_input in = {.flow = &f, .timer = NULL};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_OFF, &res);
	zassert_equal(res.recommended_event, PUMP_SM_EVENT_PLATEAU_DETECTED,
		      "plateau should recommend on from off");
	zassert_equal(res.primary_reason, PUMP_DEMAND_REASON_SIGNIFICANT_FLOW,
		      "significant flow reason");
	zassert_false(res.should_force_off, "flow on not force-off");
	zassert_true(res.updated_plateau_period_us == 0 ||
		     res.updated_plateau_period_us == 12345,
		     "period may be set");
}

ZTEST(pump_state_machine_tests, test_demand_timer_complete_forces_off_when_active)
{
	struct flow_sample f = {
		.valid = true,
		.plateau_detected = false,
		.period_us = 10000,
	};
	struct timer_pure_status t = {.completed = true, .remaining_sec = 0};
	struct pump_demand_input in = {.flow = &f, .timer = &t};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_RUNNING, &res);
	zassert_equal(res.recommended_event, PUMP_SM_EVENT_SAFETY_TIMEOUT,
		      "timer complete + active -> SAFETY_TIMEOUT");
	zassert_true(res.should_force_off, "should force off");
	zassert_equal(res.primary_reason, PUMP_DEMAND_REASON_TIMER_COMPLETE,
		      "timer complete reason");
	zassert_str_equal(res.reason_str,
			  "timer completed while pump active -> safety off",
			  "reason str");
}

ZTEST(pump_state_machine_tests, test_demand_timer_complete_no_turn_on)
{
	struct timer_pure_status t = {.completed = true, .remaining_sec = 0};
	struct pump_demand_input in = {.flow = NULL, .timer = &t};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_OFF, &res);
	zassert_equal(res.recommended_event, PUMP_SM_EVENT_RESET,
		      "timer complete must not turn on (default reset)");
	zassert_false(res.should_force_off,
		      "not active so no force off either");
}

ZTEST(pump_state_machine_tests, test_demand_timer_complete_with_no_valid_flow_still_offs)
{
	/* exercises "watchdog gate before valid" for timer path */
	struct timer_pure_status t = {.completed = true, .remaining_sec = 5};
	struct pump_demand_input in = {.flow = NULL, .timer = &t};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_STARTING, &res);
	zassert_equal(res.recommended_event, PUMP_SM_EVENT_SAFETY_TIMEOUT,
		      "timer offs even without valid flow when active");
	zassert_true(res.should_force_off, "force off");
	zassert_equal(res.primary_reason, PUMP_DEMAND_REASON_TIMER_COMPLETE,
		      "reason timer");
}

ZTEST(pump_state_machine_tests, test_demand_always_propagates_period_while_active)
{
	/* period for watchdog even on non-plateau sample (while running) */
	struct flow_sample f = {
		.valid = true,
		.plateau_detected = false, /* no new demand */
		.period_us = 54321,
	};
	struct pump_demand_input in = {.flow = &f, .timer = NULL};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_RUNNING, &res);
	zassert_equal(res.updated_plateau_period_us, 54321,
		      "period always propagated for 1.5x watchdog while active");
	zassert_equal(res.recommended_event, PUMP_SM_EVENT_RESET,
		      "non-plateau while running emits no on event");
	zassert_equal(res.primary_reason, PUMP_DEMAND_REASON_NONE, "none");
}

ZTEST(pump_state_machine_tests, test_demand_plateau_while_running_only_for_period)
{
	struct flow_sample f = {
		.valid = true,
		.plateau_detected = true,
		.period_us = 22222,
	};
	struct pump_demand_input in = {.flow = &f, .timer = NULL};
	struct pump_demand_result res;
	pump_demand_evaluate(&in, PUMP_SM_STATE_RUNNING, &res);
	zassert_equal(res.updated_plateau_period_us, 22222, "period updated");
	zassert_not_equal(res.recommended_event, PUMP_SM_EVENT_PLATEAU_DETECTED,
			  "plateau while running must not re-trigger on");
}

ZTEST_SUITE(pump_state_machine_tests, NULL, NULL, NULL, NULL, NULL);
