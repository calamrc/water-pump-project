/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal pure data types used by pump policy / demand modules.
 * These are duplicated here (small structs) so the policy modules
 * can remain completely independent of Zephyr and zbus headers.
 *
 * The authoritative definitions live in include/zbus/channels.h
 * for the real firmware.
 */

#ifndef APP_PUMP_PUMP_DATA_TYPES_H_
#define APP_PUMP_PUMP_DATA_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "fixed_math.h" /* fixed_t */

/* Avoid redefinition when both this header and zbus/channels.h are included
 * in the same translation unit during full firmware builds.
 */
#ifndef FLOW_SAMPLE_STRUCT_DEFINED
#define FLOW_SAMPLE_STRUCT_DEFINED
struct flow_sample {
  fixed_t rate;
  uint64_t timestamp;
  uint32_t sequence;
  bool valid;
  bool
      plateau_detected; /* from flow_analyzer - the real stabilization signal */
  int64_t period_us; /* Latest filtered pulse period (for watchdog timing while
                        running) */
};
#endif

/**
 * @brief Lightweight flow control events (guarded dup for pure modules).
 * Matches the authoritative def in zbus/channels.h .
 */
#ifndef FLOW_EVENT_STRUCT_DEFINED
#define FLOW_EVENT_STRUCT_DEFINED
struct flow_event {
  enum {
    FLOW_EVENT_WATCHDOG_TIMEOUT = 0,
  } type;
  uint64_t timestamp;
  int64_t associated_period_us;
};
#endif

/**
 * @brief Minimal pure timer status for demand policy (no Zephyr deps).
 * Used in pump_demand_input; mapped from timer_status at service boundary.
 */
#ifndef TIMER_PURE_STATUS_STRUCT_DEFINED
#define TIMER_PURE_STATUS_STRUCT_DEFINED
struct timer_pure_status {
  bool completed;         /* true if TIMER_STATE_COMPLETED */
  uint32_t remaining_sec; /* for completeness; primarily completed used */
};
#endif

#endif /* APP_PUMP_PUMP_DATA_TYPES_H_ */
