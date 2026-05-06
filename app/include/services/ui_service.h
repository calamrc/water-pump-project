/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef UI_SERVICE_H
#define UI_SERVICE_H

#include <zephyr/zbus/zbus.h>

/**
 * @brief Timer event message
 */
struct timer_evt {
    uint8_t state;
    uint8_t minutes;
    uint8_t seconds;
};

/* Timer event channel */
ZBUS_CHAN_DEFINE_WITH_ID(timer_event_chan,
    0x10000005,
    struct timer_evt,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

#endif /* UI_SERVICE_H */
