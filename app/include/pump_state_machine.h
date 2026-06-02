/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure, hardware-independent pump state machine.
 *
 * Extracted from the pump_controller driver for testability and reuse
 * by higher-level services (PumpService, SafetyService, tests, emulation).
 *
 * This module contains **zero** kernel objects, mutexes, or device references.
 * It is fully unit-testable on the host / native_sim.
 */

#ifndef APP_PUMP_STATE_MACHINE_H_
#define APP_PUMP_STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pump operational states (pure state machine version)
 */
enum pump_sm_state {
	PUMP_SM_STATE_OFF = 0,
	PUMP_SM_STATE_STARTING,
	PUMP_SM_STATE_RUNNING,
	PUMP_SM_STATE_TIMEOUT,
	PUMP_SM_STATE_ERROR,
	PUMP_SM_STATE_MAINTENANCE,
	PUMP_SM_STATE_COUNT
};

/**
 * @brief Events that can drive state transitions
 */
enum pump_sm_event {
	PUMP_SM_EVENT_PLATEAU_DETECTED = 0,
	PUMP_SM_EVENT_TIMEOUT,
	PUMP_SM_EVENT_SAFETY_TIMEOUT,
	PUMP_SM_EVENT_ERROR_DETECTED,
	PUMP_SM_EVENT_MAINTENANCE_ENTER,
	PUMP_SM_EVENT_MAINTENANCE_EXIT,
	PUMP_SM_EVENT_RESET,
	PUMP_SM_EVENT_COUNT
};

/**
 * @brief Process a state + event and return the next state.
 *
 * This is a pure function with no side effects. Perfect for unit testing.
 *
 * @param current Current state
 * @param event   Event that occurred
 * @return        Next state according to the transition table
 */
enum pump_sm_state pump_sm_process_event(enum pump_sm_state current,
					 enum pump_sm_event event);

/**
 * @brief Convert state to human-readable string (for logging/tests)
 */
const char *pump_sm_state_to_str(enum pump_sm_state state);

/**
 * @brief Convert event to human-readable string (for logging/tests)
 */
const char *pump_sm_event_to_str(enum pump_sm_event event);

/**
 * @brief Check if a state represents the pump being physically active
 */
static inline bool pump_sm_is_active(enum pump_sm_state state)
{
	return (state == PUMP_SM_STATE_RUNNING) ||
	       (state == PUMP_SM_STATE_STARTING);
}

#ifdef __cplusplus
}
#endif

#endif /* APP_PUMP_STATE_MACHINE_H_ */
