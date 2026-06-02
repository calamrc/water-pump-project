/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * Central zbus channel definitions. This is the ONLY place DEFINE macros
 * appear. All other files use ZBUS_CHAN_DECLARE via channels.h.
 */

#include "zbus/channels.h"
#include <zephyr/zbus/zbus.h>

/* Flow samples (high rate, best-effort) */
ZBUS_CHAN_DEFINE_WITH_ID(flow_sample_chan, 0x10000001, struct flow_sample,
                         NULL, /* No validator in Phase 1; can add later */
                         NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Pump commands (from UI, safety, timer, tests) */
ZBUS_CHAN_DEFINE_WITH_ID(pump_cmd_chan, 0x10000002, struct pump_cmd, NULL, NULL,
                         ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Pump authoritative status */
ZBUS_CHAN_DEFINE_WITH_ID(pump_status_chan, 0x10000003, struct pump_status, NULL,
                         NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Timer state (1 Hz or on transition) */
ZBUS_CHAN_DEFINE_WITH_ID(timer_state_chan, 0x10000005, struct timer_status,
                         NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Raw UI input (encoder + buttons) */
ZBUS_CHAN_DEFINE_WITH_ID(ui_input_chan, 0x10000008, struct ui_input, NULL, NULL,
                         ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Feedback (clicks) commands */
ZBUS_CHAN_DEFINE_WITH_ID(feedback_cmd_chan, 0x10000009, struct feedback_cmd,
                         NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* System / error / health broadcasts */
ZBUS_CHAN_DEFINE_WITH_ID(system_event_chan, 0x10000004, struct system_event,
                         NULL, NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));

/* Lightweight flow events (watchdog etc) - low rate control plane, separate
 * from samples */
ZBUS_CHAN_DEFINE_WITH_ID(flow_event_chan, 0x1000000a, struct flow_event, NULL,
                         NULL, ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
