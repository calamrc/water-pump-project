/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PUMP_SERVICE_H
#define PUMP_SERVICE_H

#include <zephyr/zbus/zbus.h>

/**
 * @brief Pump command message
 */
struct pump_cmd {
    enum {
        PUMP_CMD_OFF = 0,
        PUMP_CMD_ON,
        PUMP_CMD_EMERGENCY
    } action;
    uint64_t plateau_period_us;
};

/**
 * @brief Pump status message
 */
struct pump_status {
    bool is_on;
    uint64_t runtime_ms;
    uint8_t safety_state;  /* 0=OK, 1=WARNING, 2=EMERGENCY */
};

/* Pump command channel (with validator in implementation) */
ZBUS_CHAN_DEFINE_WITH_ID(pump_command_chan,
    0x10000002,
    struct pump_cmd,
    NULL,                          /* TODO: Add SMF transition validator */
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

/* Pump state channel */
ZBUS_CHAN_DEFINE_WITH_ID(pump_state_chan,
    0x10000003,
    struct pump_status,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

#endif /* PUMP_SERVICE_H */
