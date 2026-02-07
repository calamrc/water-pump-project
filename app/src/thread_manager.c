/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app/drivers/yf_s201c.h>
#include <app/drivers/pump_controller.h>

#include "thread_comm.h"
#include "flow_analyzer.h"

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

/* Sensor data semaphore for ISR-to-thread signaling */
K_SEM_DEFINE(data_sem, 0, 1);

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

            /* TODO: Implement stack usage monitoring */
            thread_health[i].stack_peak_usage = 0;

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
/* Sensor monitor thread implementation */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Sensor monitor thread started");

    /* Get flow sensor device reference */
    const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
    if (!device_is_ready(flow_sensor)) {
        LOG_ERR("Flow sensor device not ready in sensor monitor thread");
        return;
    }

    /* Configure sensor for event-driven operation */
    int ret = yf_s201c_set_data_semaphore(flow_sensor, &data_sem);
    if (ret < 0) {
        LOG_ERR("Could not configure sensor semaphore in sensor monitor thread (%d)", ret);
        return;
    }

    static uint32_t sequence_counter = 0;

    LOG_INF("Sensor monitor thread ready for data acquisition");

    while (true) {
        /* Wait for sensor data (preserve existing logic) */
        if (k_sem_take(&data_sem, K_FOREVER) == 0) {
            /* Acquire and validate sensor data */
            fixed_t flow_rate;
            ret = yf_s201c_get_flow_rate(flow_sensor, &flow_rate);

            if (ret == 0 && yf_s201c_is_data_valid(flow_sensor)) {
                /* Forward to pump controller via message queue */
                struct sensor_data_msg msg = {
                    .flow_rate = flow_rate,
                    .timestamp = k_uptime_get(),
                    .data_valid = true,
                    .sequence_number = sequence_counter++
                };

                ret = k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
                if (ret == 0) {
                    LOG_DBG("Sensor data forwarded to pump controller (seq: %u, flow: %.2f L/min)",
                           msg.sequence_number, fixed_to_float(flow_rate));
                    thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                } else {
                    LOG_WRN("Failed to queue sensor data (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
                }
            } else {
                LOG_DBG("Invalid sensor data received");
                thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
            }
        }
    }
}

/* Pump controller thread implementation */
void pump_controller_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Pump controller thread started");

    /* Get pump controller device reference */
    const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
    if (!device_is_ready(pump)) {
        LOG_ERR("Pump controller device not ready in pump controller thread");
        return;
    }

    /* Get flow sensor device reference for period calculations */
    const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
    if (!device_is_ready(flow_sensor)) {
        LOG_ERR("Flow sensor device not ready in pump controller thread");
        return;
    }

    /* State variables (preserve from original main.c) */
    int64_t initial_plateau_period = 0;
    int64_t latest_plateau_period = 0;
    int64_t pump_start_time = 0;

    LOG_INF("Pump controller thread ready for sensor data processing");

    while (true) {
        struct sensor_data_msg msg;

        /* Wait for sensor data from message queue with timeout when pump is on */
        k_timeout_t timeout = pump_controller_is_on(pump) ? K_MSEC(100) : K_FOREVER;

        if (k_msgq_get(&sensor_data_msgq, &msg, timeout) == 0) {
            LOG_DBG("Pump controller received sensor data (seq: %u, flow: %.2f L/min)",
                   msg.sequence_number, fixed_to_float(msg.flow_rate));

            /* Perform plateau detection (preserve exact logic from main.c) */
            bool current_pump_on = pump_controller_is_on(pump);
            bool plateau_detected = flow_analyzer_detect_plateau(
                msg.flow_rate,
                !current_pump_on ? FIXED_PLATEAU_INITIAL_K_FACTOR : FIXED_PLATEAU_K_FACTOR
            );

            if (plateau_detected) {
                LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)",
                        fixed_to_float(msg.flow_rate),
                        fixed_to_float(flow_analyzer_get_noise_std()),
                        fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, flow_analyzer_get_noise_std())));

                /* Get current period for plateau tracking */
                int64_t current_period;
                int ret = yf_s201c_get_current_period(flow_sensor, &current_period);
                if (ret < 0) {
                    LOG_ERR("Failed to get current period (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 1, 1);
                    continue;
                }

                /* Update plateau period (preserve logic) */
                if (!(current_pump_on && current_period < initial_plateau_period)) {
                    latest_plateau_period = current_period;
                }

                /* Update pump plateau period */
                pump_controller_update_plateau_period(pump, latest_plateau_period);
                flow_analyzer_reset();

                /* Turn on pump if conditions met */
                if (!pump_controller_is_on(pump) && current_period > 0) {
                    ret = pump_controller_turn_on(pump, latest_plateau_period);
                    if (ret < 0) {
                        LOG_ERR("Failed to turn on pump (%d)", ret);
                        thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 1, 1);
                    } else {
                        initial_plateau_period = current_period;
                        pump_start_time = k_uptime_get();
                        LOG_INF("Pump turned on with plateau period: %lld ms", latest_plateau_period);
                        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                    }
                }
            } else {
                /* No plateau detected - pump controller handles timeout internally */
                thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
            }
        } else {
            /* Timeout occurred - check if pump should be turned off */
            if (pump_controller_is_on(pump) && latest_plateau_period > 0) {
                int64_t elapsed = k_uptime_get() - pump_start_time;
                int64_t timeout_limit = latest_plateau_period * 15 / 10; // 1.5x plateau period

                if (elapsed > timeout_limit) {
                    LOG_INF("Plateau period expired, turning off pump (elapsed: %lld ms, limit: %lld ms)",
                           elapsed, timeout_limit);
                    int ret = pump_controller_turn_off(pump);
                    if (ret < 0) {
                        LOG_ERR("Failed to turn off pump on timeout (%d)", ret);
                        thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
                    } else {
                        initial_plateau_period = 0;
                        latest_plateau_period = 0;
                        yf_s201c_reset(flow_sensor);
                        flow_analyzer_reset();
                        LOG_INF("Pump turned off due to timeout");
                        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
                    }
                }
            }
        }
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