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

  if (!input || !input->flow || !input->flow->valid) {
    /* timer (if non-NULL) not yet used for decisions in PR1 (timer complete ->
     * off via other path) */
    result->reason_str = "no valid flow data";
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
   */
  bool pump_already_active = (current_state == PUMP_SM_STATE_RUNNING ||
                              current_state == PUMP_SM_STATE_STARTING);

  if (confirmed_plateau) {
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

      /* Propagate the latest good period so the service can update the driver
       */
      if (input->flow->period_us > 0) {
        result->updated_plateau_period_us = input->flow->period_us;
      }

      /* Do not emit PLATEAU_DETECTED here. The 1.5x pulse timing
       * mechanism (not continuous plateau confirmation) is what keeps
       * the pump running safely in the original design.
       */
    }
  } else {
    result->primary_reason = PUMP_DEMAND_REASON_NONE;
    result->reason_str = "no confirmed plateau from analyzer";
  }
}
