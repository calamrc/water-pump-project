/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef THREAD_MANAGER_H_
#define THREAD_MANAGER_H_

#include <stdint.h>

/**
 * @brief Create and start all application threads
 *
 * @return 0 on success, negative errno on failure
 */
int thread_manager_create_all_threads(void);

/**
 * @brief Monitor thread health and handle system coordination
 *
 * This function runs in the main thread and monitors the health of all
 * application threads. It should be called from main() after thread creation.
 */
void thread_manager_monitor_health(void);

/**
 * @brief Gracefully shutdown all threads
 *
 * @return 0 on success, negative errno on failure
 */
int thread_manager_shutdown_all_threads(void);

#endif /* THREAD_MANAGER_H_ */