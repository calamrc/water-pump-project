/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 1 zbus contract / smoke tests.
 * These will grow into full pub/sub, observer, validator, and backpressure
 * tests. They compile and link against the central channels even on targets
 * without full emulation.
 */

#include "pump/pump_data_types.h" /* for pure flow_event + timer_pure_status coverage */
#include "zbus/channels.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/ztest.h>

ZTEST(zbus_contract, test_channels_exist) {
  /* If the channels were not defined correctly this will fail at link time */
  zassert_not_null(&flow_sample_chan, "flow_sample_chan missing");
  zassert_not_null(&pump_cmd_chan, "pump_cmd_chan missing");
  zassert_not_null(&pump_status_chan, "pump_status_chan missing");
  zassert_not_null(&timer_state_chan, "timer_state_chan missing");
  zassert_not_null(&ui_input_chan, "ui_input_chan missing");
  zassert_not_null(&system_event_chan, "system_event_chan missing");
  zassert_not_null(&flow_event_chan, "flow_event_chan missing");
}

ZTEST(zbus_contract, test_flow_event_and_pure_timer_types) {
  /* Basic coverage for new lightweight chan payload + pure seam types (no Z
   * deps) */
  struct flow_event ev = {
      .type = FLOW_EVENT_WATCHDOG_TIMEOUT,
      .timestamp = 123456789ULL,
      .associated_period_us = 12345,
  };
  zassert_equal(ev.type, FLOW_EVENT_WATCHDOG_TIMEOUT, "flow_event type enum");
  zassert_equal(ev.associated_period_us, 12345, "associated period");

  /* Pure timer type (used in pump_demand_input) */
  struct timer_pure_status tps = {
      .completed = true,
      .remaining_sec = 42,
  };
  zassert_true(tps.completed, "pure timer completed");
  zassert_equal(tps.remaining_sec, 42, "pure timer remaining");

  /* Also verify flow_sample (guarded) still constructs with designated */
  struct flow_sample fs = {
      .rate = 0,
      .timestamp = 0,
      .sequence = 1,
      .valid = true,
      .plateau_detected = false,
      .period_us = 0,
  };
  zassert_true(fs.valid, "flow_sample designated init");
}

ZTEST_SUITE(zbus_contract, NULL, NULL, NULL, NULL, NULL);
