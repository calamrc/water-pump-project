/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pump demand policy implementation (pure logic).
 */

#include "pump/pump_demand.h"
#include "fixed_math.h"
#include "pump/pump_data_types.h" /* for the pure struct flow_sample + timer_pure_status */

#include <string.h>

void pump_demand_evaluate(const struct pump_demand_input *input,
                          enum pump_sm_state current_state,
                          struct pump_demand_result *result) {
  memset(result, 0, sizeof(*result));

  /* Safe defaults — do not assume we want to turn the pump on */
  result->recommended_event = PUMP_SM_EVENT_RESET; /* conservative default */
  result->primary_reason = PUMP_DEMAND_REASON_NONE;
  result->reason_str = "no decision";

  bool pump_already_active = (current_state == PUMP_SM_STATE_RUNNING ||
                              current_state == PUMP_SM_STATE_STARTING);

  if (!input || !input->flow || !input->flow->valid) {
    /* timer (if non-NULL) not yet used for decisions in PR1 (timer complete ->
     * off via other path) */
    result->reason_str = "no valid flow data";
    /* Protect active pump from spurious off on invalid data (e.g. direct calls
     * from host_sim/tests with !valid flow while running); 1.5x wd via event is
     * the off path. Per review. */
    if (pump_already_active) {
      result->recommended_event = PUMP_SM_EVENT_PLATEAU_DETECTED;
      result->reason_str = "pump running - invalid flow data (keep; 1.5x wd governs)";
    }
    return;
  }

  float rate = fixed_to_float(input->flow->rate);
  result->flow_rate_used = rate;

  /* The authoritative signal for "flow demand exists" is the real
   * statistical plateau detector (flow_analyzer), not a local threshold.
   * This restores the original behavior: we only drive the pump on
   * after the flow has been confirmed stable.
   */
  bool confirmed_plateau = input->flow->plateau_detected;

  result->significant_flow_detected = confirmed_plateau;

  /* Determine whether the pump is already actively running or starting.
   * Per the original contract:
   * - Plateau detection is primarily the gate to *turn the pump on*.
   * - Once the pump is running, we still want fresh stable pulse periods
   *   to keep the 1.5x dynamic watchdog accurate, but new plateaus no
   *   longer act as "demand" events to keep the pump alive.
   * (pump_already_active computed early, before valid check, per review for
   * keep logic on !valid + active.)
   */

  if (confirmed_plateau) {
    /* Always propagate updated_plateau on confirmed (for Phase A handoff to
     * turn_on(>0) in pump_handle + Phase B refresh). Per PR2 requirement. */
    if (input->flow->period_us > 0) {
      result->updated_plateau_period_us = input->flow->period_us;
    }

    if (!pump_already_active) {
      /* Phase A (pump off or not yet stably running): a confirmed
       * plateau is a valid demand to turn the pump on.
       */
      result->recommended_event = PUMP_SM_EVENT_PLATEAU_DETECTED;
      result->primary_reason = PUMP_DEMAND_REASON_SIGNIFICANT_FLOW;
      result->reason_str =
          "confirmed plateau from flow_analyzer (initial demand)";
    } else {
      /* Phase B (pump already running): this plateau (if any) is only
       * useful for harvesting the latest good pulse period for the
       * 1.5x watchdog timer. It is deliberately *not* treated as a
       * demand / re-plateau event.
       */
      result->primary_reason = PUMP_DEMAND_REASON_NONE;
      result->reason_str =
          "pump running - plateau only for watchdog period refresh (no demand)";

      /* Emit PLATEAU_DETECTED (no-op transition while RUNNING per SM table)
       * so that pump_handle reaches the period update path even with no
       * state change. (The 1.5x mechanism keeps pump alive; plateau is
       * only for period harvest here.)
       */
      result->recommended_event = PUMP_SM_EVENT_PLATEAU_DETECTED;
    }
  } else {
    result->primary_reason = PUMP_DEMAND_REASON_NONE;
    result->reason_str = "no confirmed plateau from analyzer";
  }

  /* While pump active, do not let default RESET (from !plateau samples between
   * confirms) force off via SM. Only flow_event WATCHDOG or timer/safety/cmds off.
   * PLATEAU_DETECTED is no-op keep for RUNNING (per SM table). */
  if (pump_already_active && result->recommended_event == PUMP_SM_EVENT_RESET) {
    result->recommended_event = PUMP_SM_EVENT_PLATEAU_DETECTED;
    result->reason_str = "pump running - no action this sample (1.5x wd governs)";
  }
}
