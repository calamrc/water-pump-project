/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYSTEM_SERVICE_H
#define SYSTEM_SERVICE_H

#include <zephyr/zbus/zbus.h>

/**
 * @brief System event message
 */
struct sys_event {
    enum {
        SYS_EVENT_SHUTDOWN = 0,
        SYS_EVENT_ERROR,
        SYS_EVENT_HEALTH
    } type;
    uint32_t code;
};

/**
 * @brief Thread health report
 */
struct thread_health {
    void *tid;
    uint32_t stack_peak;
    uint32_t errors;
    uint32_t messages;
};

/**
 * @brief Simplified error event (3 levels)
 */
struct error_event {
    uint32_t code;
    enum {
        ERROR_SEVERITY_WARNING = 0,
        ERROR_SEVERITY_ERROR,
        ERROR_SEVERITY_CRITICAL
    } severity;
};

/* System event channel */
ZBUS_CHAN_DEFINE_WITH_ID(system_event_chan,
    0x10000004,
    struct sys_event,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

/* Health report channel */
ZBUS_CHAN_DEFINE_WITH_ID(health_report_chan,
    0x10000006,
    struct thread_health,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

/* Error event channel (simplified) */
ZBUS_CHAN_DEFINE_WITH_ID(error_event_chan,
    0x10000007,
    struct error_event,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

#endif /* SYSTEM_SERVICE_H */
