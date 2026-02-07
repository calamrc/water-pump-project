/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "thread_comm.h"

LOG_MODULE_REGISTER(thread_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread stack definitions */
K_THREAD_STACK_DEFINE(sensor_monitor_stack, CONFIG_SENSOR_MONITOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(pump_controller_stack, CONFIG_PUMP_CONTROLLER_STACK_SIZE);
K_THREAD_STACK_DEFINE(safety_monitor_stack, CONFIG_SAFETY_MONITOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(supervisor_stack, CONFIG_SUPERVISOR_STACK_SIZE);

/* Thread control blocks */
static struct k_thread sensor_monitor_thread_cb;
static struct k_thread pump_controller_thread_cb;
static struct k_thread safety_monitor_thread_cb;
static struct k_thread supervisor_thread_cb;

/* Thread health monitoring */
static struct thread_health_info thread_health[4];
static int thread_health_count = 0;

/* Message Queue Definition */
K_MSGQ_DEFINE(sensor_data_msgq, sizeof(struct sensor_data_msg), CONFIG_APP_SENSOR_MSGQ_SIZE, 4);

/* Semaphore Definition */
K_SEM_DEFINE(pump_event_sem, 0, 10);

/* Poll Signal Definition */
static struct k_poll_signal thread_health_signal;

/* Forward declarations for thread entry functions */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3);
void pump_controller_thread(void *arg1, void *arg2, void *arg3);
void safety_monitor_thread(void *arg1, void *arg2, void *arg3);
void supervisor_thread(void *arg1, void *arg2, void *arg3);

/**
 * @brief Initialize thread health monitoring for a thread
 *
 * @param thread_id Thread identifier
 * @param thread_name Human-readable thread name for logging
 * @return 0 on success, negative errno on failure
 */
static int thread_health_init(k_tid_t thread_id, const char *thread_name, size_t stack_size)
{
    if (thread_health_count >= ARRAY_SIZE(thread_health)) {
        LOG_ERR("Thread health array full, cannot monitor %s", thread_name);
        return -ENOSPC;
    }

    thread_health[thread_health_count].thread_id = thread_id;
    thread_health[thread_health_count].last_check_time = k_uptime_get();
    thread_health[thread_health_count].status = THREAD_HEALTH_OK;
    thread_health[thread_health_count].stack_size = stack_size;
    thread_health[thread_health_count].stack_peak_usage = 0;
    thread_health[thread_health_count].messages_processed = 0;
    thread_health[thread_health_count].errors_encountered = 0;

    thread_health_count++;
    LOG_INF("Initialized health monitoring for %s thread", thread_name);

    return 0;
}

/**
 * @brief Update thread health status
 *
 * @param thread_id Thread identifier
 * @param status New health status
 * @param messages_processed Messages processed since last update
 * @param errors_encountered Errors encountered since last update
 */
void thread_health_update(k_tid_t thread_id, enum thread_health_status status,
                         uint32_t messages_processed, uint32_t errors_encountered)
{
    for (int i = 0; i < thread_health_count; i++) {
        if (thread_health[i].thread_id == thread_id) {
            thread_health[i].last_check_time = k_uptime_get();
            thread_health[i].status = status;
            thread_health[i].messages_processed += messages_processed;
            thread_health[i].errors_encountered += errors_encountered;

            /* Check stack usage */
            size_t stack_unused;
            if (k_thread_stack_space_get(thread_id, &stack_unused) == 0) {
                size_t stack_size = thread_health[i].stack_size;
                size_t stack_used = stack_size - stack_unused;

                if (stack_used > thread_health[i].stack_peak_usage) {
                    thread_health[i].stack_peak_usage = stack_used;
                }

                /* Log critical stack usage */
                if (stack_used > (stack_size * 90 / 100)) {  // >90% usage
                    thread_health[i].status = THREAD_HEALTH_STACK_CRITICAL;
                    LOG_WRN("Thread stack usage critical: %zu/%zu bytes",
                           stack_used, stack_size);
                }
            }

            break;
        }
    }
}

/**
 * @brief Check health of all monitored threads
 *
 * @return 0 if all threads healthy, negative errno if issues found
 */
static int thread_health_check_all(void)
{
    int64_t current_time = k_uptime_get();
    int unhealthy_count = 0;

    for (int i = 0; i < thread_health_count; i++) {
        int64_t time_since_check = current_time - thread_health[i].last_check_time;

        if (time_since_check > THREAD_HEALTH_TIMEOUT_MS) {
            thread_health[i].status = THREAD_HEALTH_TIMEOUT;
            unhealthy_count++;
            LOG_ERR("Thread health timeout: %lld ms since last check",
                   time_since_check);
        }
    }

    return unhealthy_count == 0 ? 0 : -ETIMEDOUT;
}

/**
 * @brief Create and start all application threads
 *
 * @return 0 on success, negative errno on failure
 */
int thread_manager_create_all_threads(void)
{
    LOG_INF("Creating application threads...");

    /* Initialize thread communication primitives */
    k_poll_signal_init(&thread_health_signal);

    /* Create Sensor Monitor Thread */
    k_tid_t sensor_tid = k_thread_create(&sensor_monitor_thread_cb, sensor_monitor_stack,
                                        K_THREAD_STACK_SIZEOF(sensor_monitor_stack),
                                        sensor_monitor_thread, NULL, NULL, NULL,
                                        CONFIG_SENSOR_MONITOR_PRIORITY, 0, K_NO_WAIT);
    if (sensor_tid == NULL) {
        LOG_ERR("Failed to create sensor monitor thread");
        return -EAGAIN;
    }
    k_thread_name_set(&sensor_monitor_thread_cb, "sensor_mon");
    thread_health_init(sensor_tid, "sensor_monitor", CONFIG_SENSOR_MONITOR_STACK_SIZE);

    /* Create Pump Controller Thread */
    k_tid_t pump_tid = k_thread_create(&pump_controller_thread_cb, pump_controller_stack,
                                      K_THREAD_STACK_SIZEOF(pump_controller_stack),
                                      pump_controller_thread, NULL, NULL, NULL,
                                      CONFIG_PUMP_CONTROLLER_PRIORITY, 0, K_NO_WAIT);
    if (pump_tid == NULL) {
        LOG_ERR("Failed to create pump controller thread");
        return -EAGAIN;
    }
    k_thread_name_set(&pump_controller_thread_cb, "pump_ctrl");
    thread_health_init(pump_tid, "pump_controller", CONFIG_PUMP_CONTROLLER_STACK_SIZE);

    /* Create Safety Monitor Thread */
    k_tid_t safety_tid = k_thread_create(&safety_monitor_thread_cb, safety_monitor_stack,
                                        K_THREAD_STACK_SIZEOF(safety_monitor_stack),
                                        safety_monitor_thread, NULL, NULL, NULL,
                                        CONFIG_SAFETY_MONITOR_PRIORITY, 0, K_NO_WAIT);
    if (safety_tid == NULL) {
        LOG_ERR("Failed to create safety monitor thread");
        return -EAGAIN;
    }
    k_thread_name_set(&safety_monitor_thread_cb, "safety_mon");
    thread_health_init(safety_tid, "safety_monitor", CONFIG_SAFETY_MONITOR_STACK_SIZE);

    /* Create Supervisor Thread */
    k_tid_t supervisor_tid = k_thread_create(&supervisor_thread_cb, supervisor_stack,
                                            K_THREAD_STACK_SIZEOF(supervisor_stack),
                                            supervisor_thread, NULL, NULL, NULL,
                                            CONFIG_SUPERVISOR_PRIORITY, 0, K_NO_WAIT);
    if (supervisor_tid == NULL) {
        LOG_ERR("Failed to create supervisor thread");
        return -EAGAIN;
    }
    k_thread_name_set(&supervisor_thread_cb, "supervisor");
    thread_health_init(supervisor_tid, "supervisor", CONFIG_SUPERVISOR_STACK_SIZE);

    LOG_INF("All threads created successfully");
    return 0;
}

/**
 * @brief Monitor thread health and handle system coordination
 *
 * This function runs in the main thread and monitors the health of all
 * application threads. It should be called from main() after thread creation.
 */
void thread_manager_monitor_health(void)
{
    LOG_INF("Starting thread health monitoring...");

    while (true) {
        /* Check thread health */
        int health_status = thread_health_check_all();
        if (health_status != 0) {
            LOG_ERR("Thread health issues detected: %d", health_status);
            /* TODO: Implement recovery strategies */
        }

        /* Log health summary periodically */
        static int64_t last_summary = 0;
        int64_t current_time = k_uptime_get();
        if (current_time - last_summary > 30000) {  // Every 30 seconds
            LOG_INF("Thread health summary:");
            for (int i = 0; i < thread_health_count; i++) {
                LOG_INF("  %s: status=%d, stack_peak=%zu, msgs=%u, errs=%u",
                       k_thread_name_get(thread_health[i].thread_id),
                       thread_health[i].status,
                       thread_health[i].stack_peak_usage,
                       thread_health[i].messages_processed,
                       thread_health[i].errors_encountered);
            }
            last_summary = current_time;
        }

        /* Health check interval */
        k_sleep(K_MSEC(THREAD_HEALTH_CHECK_INTERVAL_MS));
    }
}

/**
 * @brief Gracefully shutdown all threads
 *
 * @return 0 on success, negative errno on failure
 */
int thread_manager_shutdown_all_threads(void)
{
    LOG_INF("Shutting down all threads...");

    /* Signal threads to exit (implementation depends on thread design) */
    /* TODO: Implement proper thread shutdown signaling */

    /* Wait for threads to exit */
    k_thread_join(&sensor_monitor_thread_cb, K_SECONDS(5));
    k_thread_join(&pump_controller_thread_cb, K_SECONDS(5));
    k_thread_join(&safety_monitor_thread_cb, K_SECONDS(5));
    k_thread_join(&supervisor_thread_cb, K_SECONDS(5));

    LOG_INF("All threads shut down");
    return 0;
}

/* Placeholder thread implementations - will be replaced in Phase 2-4 */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Sensor monitor thread started (placeholder)");
    while (true) {
        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
        k_sleep(K_SECONDS(1));
    }
}

void pump_controller_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Pump controller thread started (placeholder)");
    while (true) {
        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
        k_sleep(K_SECONDS(1));
    }
}

void safety_monitor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Safety monitor thread started (placeholder)");
    while (true) {
        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
        k_sleep(K_SECONDS(1));
    }
}

void supervisor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Supervisor thread started (placeholder)");
    while (true) {
        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
        k_sleep(K_SECONDS(1));
    }
}