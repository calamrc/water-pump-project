/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THREAD_COMM_H_
#define THREAD_COMM_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include "fixed_math.h"

/**
 * @brief Sensor data message structure for inter-thread communication
 */
struct sensor_data_msg {
    /** Flow rate in fixed-point format */
    fixed_t flow_rate;
    /** Timestamp when data was acquired (uptime ticks) */
    int64_t timestamp;
    /** Data validity flag */
    bool data_valid;
    /** Sequence number for message ordering */
    uint32_t sequence_number;
};

/**
 * @brief Pump control event types for thread signaling
 */
enum pump_event_type {
    /** Pump has been started */
    PUMP_EVENT_STARTED,
    /** Pump has been stopped */
    PUMP_EVENT_STOPPED,
    /** Safety timeout warning issued */
    PUMP_EVENT_TIMEOUT_WARNING,
    /** Plateau period has been updated */
    PUMP_EVENT_PLATEAU_UPDATED,
    /** Pump control error occurred */
    PUMP_EVENT_ERROR
};

/**
 * @brief Thread health status enumeration
 */
enum thread_health_status {
    /** Thread is healthy and responding */
    THREAD_HEALTH_OK,
    /** Thread health check timed out */
    THREAD_HEALTH_TIMEOUT,
    /** Thread encountered an error */
    THREAD_HEALTH_ERROR,
    /** Thread stack usage is critical */
    THREAD_HEALTH_STACK_CRITICAL
};

/**
 * @brief Thread health monitoring structure
 */
struct thread_health_info {
    /** Thread identifier */
    k_tid_t thread_id;
    /** Last health check timestamp */
    int64_t last_check_time;
    /** Current health status */
    enum thread_health_status status;
    /** Total stack size in bytes */
    size_t stack_size;
    /** Peak stack usage in bytes */
    size_t stack_peak_usage;
    /** Messages processed by thread */
    uint32_t messages_processed;
    /** Errors encountered by thread */
    uint32_t errors_encountered;
};

/* Thread Health Monitoring Constants */
#define THREAD_HEALTH_CHECK_INTERVAL_MS 1000
#define THREAD_HEALTH_TIMEOUT_MS 5000

/* External declarations for thread communication primitives */
/* These are defined in thread_manager.c */
extern struct k_msgq sensor_data_msgq;
extern struct k_sem pump_event_sem;

/* Thread health update function - defined in thread_manager.c */
void thread_health_update(k_tid_t thread_id, enum thread_health_status status,
                         uint32_t messages_processed, uint32_t errors_encountered);

#endif /* THREAD_COMM_H_ */