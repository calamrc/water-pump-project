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

ZTEST_SUITE(pump_state_machine_tests, NULL, NULL, NULL, NULL, NULL);
