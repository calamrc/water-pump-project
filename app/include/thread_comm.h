/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THREAD_COMM_H_
#define THREAD_COMM_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

enum thread_health_status {
	THREAD_HEALTH_OK,
	THREAD_HEALTH_TIMEOUT,
	THREAD_HEALTH_ERROR,
	THREAD_HEALTH_STACK_CRITICAL
};

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

void thread_health_update(k_tid_t thread_id, enum thread_health_status status,
			  uint32_t messages_processed, uint32_t errors_encountered);

#endif /* THREAD_COMM_H_ */