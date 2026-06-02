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

  /* Timer complete (from pure timer_pure_status) forces safe off via
   * SAFETY_TIMEOUT only when pump is active. Timer never produces a turn-on
   * (flow plateau remains sole auto-on gate). This check is before the
   * valid-flow gate so timer completion can safely stop even on stale flow
   * (watchdog-like gate for timer path).
   */
  if (input && input->timer && input->timer->completed && pump_already_active) {
    result->recommended_event = PUMP_SM_EVENT_SAFETY_TIMEOUT;
    result->should_force_off = true;
    result->primary_reason = PUMP_DEMAND_REASON_TIMER_COMPLETE;
    result->reason_str = "timer completed while pump active -> safety off";
    /* continue so we can still propagate period if flow present */
  }

  if (!input || !input->flow || !input->flow->valid) {
    if (result->primary_reason != PUMP_DEMAND_REASON_TIMER_COMPLETE) {
      result->reason_str = "no valid flow data";
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

  /* Always propagate latest period (for 1.5x watchdog) when pump active,
   * regardless of whether this sample carries a new plateau. (Previously
   * only inside the plateau+active branch.)
   */
  if (pump_already_active && input->flow->period_us > 0) {
    result->updated_plateau_period_us = input->flow->period_us;
  }

  if (confirmed_plateau) {
    if (!pump_already_active) {
      /* Phase A (pump off or not yet stably running): a confirmed
       * plateau is a valid demand to turn the pump on.
       * (Do not override if timer already forced off, though !active case.)
       */
      if (result->recommended_event != PUMP_SM_EVENT_SAFETY_TIMEOUT) {
        result->recommended_event = PUMP_SM_EVENT_PLATEAU_DETECTED;
        result->primary_reason = PUMP_DEMAND_REASON_SIGNIFICANT_FLOW;
        result->reason_str =
            "confirmed plateau from flow_analyzer (initial demand)";
      }
    } else {
      /* Phase B (pump already running): this plateau (if any) is only
       * useful for harvesting the latest good pulse period for the
       * 1.5x watchdog timer. It is deliberately *not* treated as a
       * demand / re-plateau event.
       */
      if (result->primary_reason != PUMP_DEMAND_REASON_TIMER_COMPLETE) {
        result->primary_reason = PUMP_DEMAND_REASON_NONE;
        result->reason_str =
            "pump running - plateau only for watchdog period refresh (no demand)";
      }
      /* period already propagated above for all active cases */
    }
  } else {
    if (result->primary_reason != PUMP_DEMAND_REASON_TIMER_COMPLETE) {
      result->primary_reason = PUMP_DEMAND_REASON_NONE;
      result->reason_str = "no confirmed plateau from analyzer";
    }
  }
}
