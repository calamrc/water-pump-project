/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure pump state machine implementation.
 *
 * The transition table is the single source of truth for pump behavior.
 * Services and tests use pump_sm_process_event().
 */

#include "pump_state_machine.h"

#include <stddef.h>

/* --------------------------------------------------------------------------
 * State Transition Table (authoritative)
 * Row = current state, Column = event
 * -------------------------------------------------------------------------- */
static const enum pump_sm_state transition_table[PUMP_SM_STATE_COUNT][PUMP_SM_EVENT_COUNT] = {
	/* OFF -> */
	{
		PUMP_SM_STATE_STARTING,      /* PLATEAU_DETECTED */
		PUMP_SM_STATE_OFF,           /* TIMEOUT */
		PUMP_SM_STATE_OFF,           /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_OFF,           /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	},
	/* STARTING -> */
	{
		PUMP_SM_STATE_RUNNING,       /* PLATEAU_DETECTED */
		PUMP_SM_STATE_TIMEOUT,       /* TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_OFF,           /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	},
	/* RUNNING -> */
	{
		PUMP_SM_STATE_RUNNING,       /* PLATEAU_DETECTED */
		PUMP_SM_STATE_TIMEOUT,       /* TIMEOUT */
		PUMP_SM_STATE_OFF,           /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_RUNNING,       /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	},
	/* TIMEOUT -> */
	{
		PUMP_SM_STATE_OFF,           /* PLATEAU_DETECTED */
		PUMP_SM_STATE_TIMEOUT,       /* TIMEOUT */
		PUMP_SM_STATE_OFF,           /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_TIMEOUT,       /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	},
	/* ERROR -> */
	{
		PUMP_SM_STATE_ERROR,         /* PLATEAU_DETECTED */
		PUMP_SM_STATE_ERROR,         /* TIMEOUT */
		PUMP_SM_STATE_OFF,           /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_ERROR,         /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_ERROR,         /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	},
	/* MAINTENANCE -> */
	{
		PUMP_SM_STATE_MAINTENANCE,   /* PLATEAU_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* TIMEOUT */
		PUMP_SM_STATE_MAINTENANCE,   /* SAFETY_TIMEOUT */
		PUMP_SM_STATE_MAINTENANCE,   /* ERROR_DETECTED */
		PUMP_SM_STATE_MAINTENANCE,   /* MAINTENANCE_ENTER */
		PUMP_SM_STATE_OFF,           /* MAINTENANCE_EXIT */
		PUMP_SM_STATE_OFF            /* RESET */
	}
};

enum pump_sm_state pump_sm_process_event(enum pump_sm_state current,
					 enum pump_sm_event event)
{
	if ((unsigned)current >= PUMP_SM_STATE_COUNT ||
	    (unsigned)event   >= PUMP_SM_EVENT_COUNT) {
		return PUMP_SM_STATE_ERROR;
	}

	return transition_table[current][event];
}

const char *pump_sm_state_to_str(enum pump_sm_state state)
{
	switch (state) {
	case PUMP_SM_STATE_OFF:         return "OFF";
	case PUMP_SM_STATE_STARTING:    return "STARTING";
	case PUMP_SM_STATE_RUNNING:     return "RUNNING";
	case PUMP_SM_STATE_TIMEOUT:     return "TIMEOUT";
	case PUMP_SM_STATE_ERROR:       return "ERROR";
	case PUMP_SM_STATE_MAINTENANCE: return "MAINTENANCE";
	default:                        return "UNKNOWN";
	}
}

const char *pump_sm_event_to_str(enum pump_sm_event event)
{
	switch (event) {
	case PUMP_SM_EVENT_PLATEAU_DETECTED:   return "PLATEAU_DETECTED";
	case PUMP_SM_EVENT_TIMEOUT:            return "TIMEOUT";
	case PUMP_SM_EVENT_SAFETY_TIMEOUT:     return "SAFETY_TIMEOUT";
	case PUMP_SM_EVENT_ERROR_DETECTED:     return "ERROR_DETECTED";
	case PUMP_SM_EVENT_MAINTENANCE_ENTER:  return "MAINTENANCE_ENTER";
	case PUMP_SM_EVENT_MAINTENANCE_EXIT:   return "MAINTENANCE_EXIT";
	case PUMP_SM_EVENT_RESET:              return "RESET";
	default:                               return "UNKNOWN";
	}
}
