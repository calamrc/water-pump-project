/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Central Zbus channel declarations for the water pump application.
 * All message types and channel declarations live here (or in the corresponding
 * .c that performs the DEFINE). Consumers include this header and use
 * ZBUS_CHAN_DECLARE for the channels they need.
 */

#ifndef APP_ZBUS_CHANNELS_H_
#define APP_ZBUS_CHANNELS_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/zbus/zbus.h>

#include "fixed_math.h"
#include "timer/timer_state_machine.h"

/* ============================================================================
 * Message Types (single source of truth)
 * ============================================================================
 */

/**
 * @brief Flow measurement sample published by FlowSensorService
 */
/* Authoritative definition. Guarded to avoid duplicate with pump_data_types.h
 * when both are pulled into the same .c file during builds.
 */
#ifndef FLOW_SAMPLE_STRUCT_DEFINED
#define FLOW_SAMPLE_STRUCT_DEFINED
struct flow_sample {
  fixed_t rate;          /* L/min in Q16.16 fixed point */
  uint64_t timestamp;    /* k_uptime_get() ms */
  uint32_t sequence;     /* Monotonic sequence number */
  bool valid;            /* Data passed validation / not stale */
  bool plateau_detected; /* True only when flow_analyzer confirms a stable
                            plateau */
  int64_t period_us;     /* Latest filtered pulse period (for watchdog timing
                            updates while running) */
};
#endif

/**
 * @brief Lightweight flow control events (e.g. watchdog from contract).
 * Separate from high-rate flow_sample (measurements only). Per design choice.
 * Guarded duplicate in pump_data_types.h for pure seams (host_sim, logic).
 */
#ifndef FLOW_EVENT_STRUCT_DEFINED
#define FLOW_EVENT_STRUCT_DEFINED
struct flow_event {
  enum {
    FLOW_EVENT_WATCHDOG_TIMEOUT = 0,
    /* future: FLOW_EVENT_ANOMALY, ... */
  } type;
  uint64_t timestamp;
  int64_t associated_period_us; /* snapshot of period at event for context */
};
#endif

/**
 * @brief Commands sent to the pump control service
 */
struct pump_cmd {
  enum {
    PUMP_CMD_OFF = 0,
    PUMP_CMD_ON,
    PUMP_CMD_EMERGENCY,
    PUMP_CMD_SET_SAFETY_TIMEOUT,
  } action;
  uint64_t value; /* plateau period (us) or safety timeout (min) depending on
                     action */
};

/**
 * @brief Published status of the pump (authoritative)
 */
struct pump_status {
  bool is_on;
  uint64_t runtime_ms; /* Cumulative time in RUNNING since last off */
  uint8_t state; /* Mirrors pump_state enum from driver for observability */
  uint8_t safety_flags; /* Bitfield: bit0 = safety_timer_active, bit1 =
                           emergency, etc. */
};

/**
 * @brief Timer status published by TimerService
 */
struct timer_status {
  uint32_t remaining_sec;
  enum timer_state state;
  bool flash; /* UI hint for completed state */
};

/**
 * @brief Raw input events from the EC11 encoder + button
 */
struct ui_input {
  int32_t encoder_delta; /* Net detents since last event (positive = CW) */
  enum {
    BUTTON_NONE = 0,
    BUTTON_SHORT,
    BUTTON_LONG,
  } button;
};

/**
 * @brief Requests for auditory feedback clicks
 */
struct feedback_cmd {
  uint32_t duration_ms; /* 0 = off/cancel, >0 = pulse for this long */
};

/**
 * @brief System-wide events (shutdown, errors, health broadcasts)
 */
struct system_event {
  enum {
    SYS_EVENT_SHUTDOWN = 0,
    SYS_EVENT_ERROR,
    SYS_EVENT_HEALTH_UPDATE,
  } type;
  uint32_t code;
};

/* ============================================================================
 * Channel Declarations (DEFINE lives in channels.c)
 * ============================================================================
 */

ZBUS_CHAN_DECLARE(flow_sample_chan);
ZBUS_CHAN_DECLARE(pump_cmd_chan);
ZBUS_CHAN_DECLARE(pump_status_chan);
ZBUS_CHAN_DECLARE(timer_state_chan);
ZBUS_CHAN_DECLARE(ui_input_chan);
ZBUS_CHAN_DECLARE(feedback_cmd_chan);
ZBUS_CHAN_DECLARE(system_event_chan);
ZBUS_CHAN_DECLARE(flow_event_chan);

#endif /* APP_ZBUS_CHANNELS_H_ */
