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
	fixed_t flow_rate;
	int64_t timestamp;
	bool data_valid;
	uint32_t sequence_number;
};

/**
 * @brief Thread health status enumeration
 */
enum thread_health_status {
	THREAD_HEALTH_OK,
	THREAD_HEALTH_TIMEOUT,
	THREAD_HEALTH_ERROR,
	THREAD_HEALTH_STACK_CRITICAL
};

/**
 * @brief Thread health monitoring structure
 */
struct thread_health_info {
	k_tid_t thread_id;
	int64_t last_check_time;
	enum thread_health_status status;
	size_t stack_size;
	size_t stack_peak_usage;
	uint32_t messages_processed;
	uint32_t errors_encountered;
};

#define THREAD_HEALTH_CHECK_INTERVAL_MS 1000
#define THREAD_HEALTH_TIMEOUT_MS 5000

extern struct k_msgq sensor_data_msgq;

void thread_health_update(k_tid_t thread_id, enum thread_health_status status,
			  uint32_t messages_processed, uint32_t errors_encountered);

#endif /* THREAD_COMM_H_ */