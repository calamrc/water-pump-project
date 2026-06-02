/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pump demand policy module.
 *
 * This is a pure, zero-dependency module that encapsulates the logic
 * for deciding what the pump should do next given current inputs
 * (flow + pure timer; flow_event_chan and cmds handled at service layer).
 *
 * It is the single source of truth for pump demand decisions and is
 * intended to be used by both the real PumpService and by host-side
 * simulation (host_sim).
 */

#ifndef APP_PUMP_PUMP_DEMAND_H_
#define APP_PUMP_PUMP_DEMAND_H_

#include <stdbool.h>
#include <stdint.h>

#include "pump/pump_data_types.h" /* pure struct flow_sample (no Zephyr deps) */
#include "pump_state_machine.h"   /* enum pump_sm_event, enum pump_sm_state */

/**
 * @brief Reasons that drove a demand decision.
 * These are machine-readable so both firmware and host simulation
 * can log, assert on, or display them.
 */
enum pump_demand_reason {
  PUMP_DEMAND_REASON_NONE = 0,
  PUMP_DEMAND_REASON_SIGNIFICANT_FLOW,
  PUMP_DEMAND_REASON_FLOW_DROPPED_WHILE_RUNNING,
  PUMP_DEMAND_REASON_TIMER_COMPLETE,
  PUMP_DEMAND_REASON_EXPLICIT_ON,
  PUMP_DEMAND_REASON_EXPLICIT_OFF,
  PUMP_DEMAND_REASON_EMERGENCY,
  PUMP_DEMAND_REASON_SAFETY_TIMEOUT,
};

/**
 * @brief Input snapshot for the demand policy evaluator.
 * Timer uses pure type (NULLable). Watchdog events come via flow_event_chan
 * listener (not in flow).
 */
struct pump_demand_input {
  const struct flow_sample *flow; /* may be NULL */
  const struct timer_pure_status
      *timer; /* NULL ok; pure type for timer complete checks (seam) */
              /* Future increments:
               * bool safety_timeout_active;
               * enum pump_cmd_action external_cmd;
               */
};

/**
 * @brief Rich output from the pump demand policy.
 * Contains the recommended action for the state machine plus
 * diagnostics that are useful for logging, testing, and simulation.
 */
struct pump_demand_result {
  enum pump_sm_event recommended_event;

  bool should_force_on;  /* explicit ON command received */
  bool should_force_off; /* COMPLETE, EMERGENCY, explicit OFF, etc. */

  /* Rich diagnostics */
  enum pump_demand_reason primary_reason;
  float flow_rate_used; /* flow rate that influenced the decision (if any) */
  bool significant_flow_detected;
  const char *reason_str; /* human-readable explanation */

  /* When the pump is already running and we captured a good stable period
   * (good_period_captured case from the contract), this carries the value
   * that should be passed to pump_controller_update_plateau_period so the
   * 1.5x watchdog stays accurate.
   */
  int64_t updated_plateau_period_us;
};

/**
 * @brief Evaluate the current pump demand policy.
 *
 * This is a pure function with no side effects and no kernel dependencies.
 * It decides what event the pump state machine should see next based on
 * the provided inputs.
 *
 * @param input          Current world state (flow data, later
 * timer/safety/etc.)
 * @param current_state  Current state of the pump state machine (for
 * hysteresis)
 * @param[out] result    Recommended action + rich diagnostics
 */
void pump_demand_evaluate(const struct pump_demand_input *input,
                          enum pump_sm_state current_state,
                          struct pump_demand_result *result);

#endif /* APP_PUMP_PUMP_DEMAND_H_ */
