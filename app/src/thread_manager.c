/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/pump_controller.h>
#include <app/drivers/yf_s201c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "thread_comm.h"
#include "flow_analyzer.h"

LOG_MODULE_REGISTER(thread_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread stack definitions */
K_THREAD_STACK_DEFINE(sensor_monitor_stack, CONFIG_SENSOR_MONITOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(pump_controller_stack, CONFIG_PUMP_CONTROLLER_STACK_SIZE);
K_THREAD_STACK_DEFINE(safety_monitor_stack, CONFIG_SAFETY_MONITOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(supervisor_stack, CONFIG_SUPERVISOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(ui_manager_stack, CONFIG_UI_MANAGER_STACK_SIZE);

/* Thread control blocks */
static struct k_thread sensor_monitor_thread_cb;
static struct k_thread pump_controller_thread_cb;
static struct k_thread safety_monitor_thread_cb;
static struct k_thread supervisor_thread_cb;
static struct k_thread ui_manager_thread_cb;

/* Thread health monitoring */
static struct thread_health_info thread_health[5];
static int thread_health_count = 0;

/* Message Queue Definition */
K_MSGQ_DEFINE(sensor_data_msgq, sizeof(struct sensor_data_msg), CONFIG_APP_SENSOR_MSGQ_SIZE, 4);

/* Semaphore Definition */
K_SEM_DEFINE(pump_event_sem, 0, 10);

/* Poll Signal Definition */
static struct k_poll_signal thread_health_signal;

/* Shutdown coordination */
static bool system_shutdown_requested = false;
static struct k_sem shutdown_sem;

/* Sensor data semaphore for ISR-to-thread signaling */
K_SEM_DEFINE(data_sem, 0, 1);

/* Forward declarations for thread entry functions */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3);
void pump_controller_thread(void *arg1, void *arg2, void *arg3);
void safety_monitor_thread(void *arg1, void *arg2, void *arg3);
void supervisor_thread(void *arg1, void *arg2, void *arg3);
void ui_manager_thread(void *arg1, void *arg2, void *arg3);

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
 * @brief Check health of all monitored threads and attempt recovery
 *
 * @return 0 if all threads healthy, negative errno on failure
 */
static int thread_health_check_all(void)
{
    int64_t current_time = k_uptime_get();
    int unhealthy_count = 0;
    int critical_count = 0;

    for (int i = 0; i < thread_health_count; i++) {
        /* Validate thread ID before using it */
        if (thread_health[i].thread_id == NULL) {
            LOG_ERR("Thread health entry %d has NULL thread_id", i);
            unhealthy_count++;
            continue;
        }
        
        int64_t time_since_check = current_time - thread_health[i].last_check_time;
        const char *thread_name = k_thread_name_get(thread_health[i].thread_id);
        
        /* Handle case where thread name is NULL */
        if (thread_name == NULL) {
            thread_name = "unknown";
        }

        /* Check for timeout */
        if (time_since_check > THREAD_HEALTH_TIMEOUT_MS) {
            thread_health[i].status = THREAD_HEALTH_TIMEOUT;
            unhealthy_count++;

            /* Critical threads get immediate attention - check by name safely */
            bool is_critical = (thread_name != NULL) && 
                              (strcmp(thread_name, "safety_mon") == 0 || 
                               strcmp(thread_name, "pump_ctrl") == 0);
            if (is_critical) {
                critical_count++;
                LOG_ERR("CRITICAL: Thread %s health timeout: %lld ms since last check",
                        thread_name, time_since_check);
            } else {
                LOG_WRN("Thread %s health timeout: %lld ms since last check",
                        thread_name, time_since_check);
            }
        }

        /* Check for stack usage issues */
        if (thread_health[i].stack_peak_usage > thread_health[i].stack_size * 90 / 100) {
            thread_health[i].status = THREAD_HEALTH_STACK_CRITICAL;
            unhealthy_count++;
            LOG_WRN("Thread %s stack usage critical: %zu/%zu bytes (%.1f%%)",
                   thread_name,
                   thread_health[i].stack_peak_usage,
                   thread_health[i].stack_size,
                   (double)((float)thread_health[i].stack_peak_usage * 100 / thread_health[i].stack_size));
        }

        /* Check for excessive error rates */
        if (thread_health[i].errors_encountered > 10) {
            LOG_WRN("Thread %s has high error count: %u errors",
                   thread_name, thread_health[i].errors_encountered);
        }
    }

    /* Recovery strategies for critical issues */
    if (critical_count > 0) {
        LOG_ERR("CRITICAL: %d critical threads unhealthy - initiating emergency procedures", critical_count);

        /* Emergency: Force pump off if pump controller is unresponsive */
        const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
        if (device_is_ready(pump) && pump_controller_is_on(pump)) {
            LOG_ERR("Emergency: Forcing pump off due to critical thread failure");
            int ret = pump_controller_emergency_stop(pump);
            if (ret < 0) {
                LOG_ERR("Emergency stop failed: %d", ret);
            }
        }
    }

    return (unhealthy_count == 0) ? 0 : (critical_count > 0 ? -ECANCELED : -ETIMEDOUT);
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

    /* Initialize shutdown semaphore */
    k_sem_init(&shutdown_sem, 0, 1);

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

    /* Create UI Manager Thread */
    k_tid_t ui_tid = k_thread_create(&ui_manager_thread_cb, ui_manager_stack,
                                    K_THREAD_STACK_SIZEOF(ui_manager_stack),
                                    ui_manager_thread, NULL, NULL, NULL,
                                    CONFIG_UI_MANAGER_PRIORITY, 0, K_NO_WAIT);
    if (ui_tid == NULL) {
        LOG_ERR("Failed to create UI manager thread");
        return -EAGAIN;
    }
    k_thread_name_set(&ui_manager_thread_cb, "ui_manager");
    thread_health_init(ui_tid, "ui_manager", CONFIG_UI_MANAGER_STACK_SIZE);

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
                const char *name = k_thread_name_get(thread_health[i].thread_id);
                if (name == NULL) name = "unknown";
                LOG_INF("  %s: status=%d, stack_peak=%zu, msgs=%u, errs=%u",
                       name,
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

int thread_manager_request_shutdown(void)
{
    if (system_shutdown_requested) {
        return -EALREADY;
    }

    system_shutdown_requested = true;
    k_sem_give(&shutdown_sem);

    LOG_INF("System shutdown requested");
    return 0;
}

/**
 * @brief Check if shutdown has been requested
 *
 * @return true if shutdown requested, false otherwise
 */
bool thread_manager_is_shutdown_requested(void)
{
    return system_shutdown_requested;
}

/**
 * @brief Gracefully shutdown all threads
 *
 * @return 0 on success, negative errno on failure
 */
int thread_manager_shutdown_all_threads(void)
{
    LOG_INF("Initiating graceful system shutdown...");

    /* Signal shutdown to all threads */
    system_shutdown_requested = true;
    k_sem_give(&shutdown_sem);

    /* Give threads time to acknowledge shutdown */
    k_sleep(K_MSEC(500));

    /* Ensure pump is safely off before shutdown */
    const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
    if (device_is_ready(pump) && pump_controller_is_on(pump)) {
        LOG_INF("Shutdown: Ensuring pump is safely off");
        int ret = pump_controller_turn_off(pump);
        if (ret < 0) {
            LOG_WRN("Shutdown: Failed to turn off pump gracefully (%d), using emergency stop", ret);
            ret = pump_controller_emergency_stop(pump);
            if (ret < 0) {
                LOG_ERR("Shutdown: Emergency stop also failed (%d)", ret);
            }
        }
    }

    /* Wait for threads to exit with timeout */
    LOG_INF("Waiting for threads to exit...");

    int timeout_count = 0;
    const int max_timeouts = 10;  // 5 seconds total

    while (timeout_count < max_timeouts) {
        bool all_exited = true;

        /* Check if threads are still running */
        if (k_thread_join(&sensor_monitor_thread_cb, K_MSEC(100)) != 0) {
            all_exited = false;
        }
        if (k_thread_join(&pump_controller_thread_cb, K_MSEC(100)) != 0) {
            all_exited = false;
        }
        if (k_thread_join(&safety_monitor_thread_cb, K_MSEC(100)) != 0) {
            all_exited = false;
        }
        if (k_thread_join(&supervisor_thread_cb, K_MSEC(100)) != 0) {
            all_exited = false;
        }
        if (k_thread_join(&ui_manager_thread_cb, K_MSEC(100)) != 0) {
            all_exited = false;
        }

        if (all_exited) {
            break;
        }

        timeout_count++;
        LOG_INF("Shutdown: Waiting for threads to exit (%d/%d)...", timeout_count, max_timeouts);
    }

    if (timeout_count >= max_timeouts) {
        LOG_WRN("Shutdown: Some threads did not exit gracefully within timeout");
    } else {
        LOG_INF("Shutdown: All threads exited gracefully");
    }

    /* Final cleanup */
    system_shutdown_requested = false;

    LOG_INF("System shutdown complete");
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

    int64_t last_health_update = k_uptime_get();
    
    while (!thread_manager_is_shutdown_requested()) {
        /* Wait for sensor data with timeout to allow periodic health updates */
        if (k_sem_take(&data_sem, K_MSEC(1000)) == 0) {
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
                           msg.sequence_number, (double)fixed_to_float(flow_rate));
                    thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                } else {
                    LOG_WRN("Failed to queue sensor data (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
                }
            } else {
                LOG_DBG("Invalid sensor data received");
                thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
            }
            last_health_update = k_uptime_get();
        } else {
            /* Timeout - update health even when no data */
            int64_t now = k_uptime_get();
            if ((now - last_health_update) >= 1000) {
                last_health_update = now;
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

    LOG_INF("Pump controller thread ready for sensor data processing");

    int64_t last_health_update = k_uptime_get();

    while (!thread_manager_is_shutdown_requested()) {
        struct sensor_data_msg msg;

        /* Wait for sensor data from message queue with timeout when pump is on */
        k_timeout_t timeout;
        if (pump_controller_is_on(pump) && latest_plateau_period > 0) {
            /* Timeout after 1.5x plateau period, same as original logic */
            int64_t timeout_us = latest_plateau_period * 15 / 10; // 1.5x plateau period
            timeout_us = MIN(timeout_us, (int64_t)CONFIG_APP_MAX_TIMEOUT_US);
            timeout = K_USEC(timeout_us);
        } else {
            /* When pump is off, use 1 second timeout to allow periodic health updates */
            timeout = K_MSEC(1000);
        }

        if (k_msgq_get(&sensor_data_msgq, &msg, timeout) == 0) {
            LOG_DBG("Pump controller received sensor data (seq: %u, flow: %.2f L/min)",
                   msg.sequence_number, (double)fixed_to_float(msg.flow_rate));

            /* Perform plateau detection (preserve exact logic from main.c) */
            bool current_pump_on = pump_controller_is_on(pump);
            bool plateau_detected = flow_analyzer_detect_plateau(
                msg.flow_rate,
                !current_pump_on ? FIXED_PLATEAU_INITIAL_K_FACTOR : FIXED_PLATEAU_K_FACTOR
            );

            if (plateau_detected) {
                LOG_INF("Plateau detected at flow rate: %.2f L/min (noise std: %.4f, epsilon: %.4f)",
                        (double)fixed_to_float(msg.flow_rate),
                        (double)fixed_to_float(flow_analyzer_get_noise_std()),
                        (double)fixed_to_float(fixed_mul(FIXED_PLATEAU_K_FACTOR, flow_analyzer_get_noise_std())));

                /* Get current period for plateau tracking */
                int64_t current_period;
                int ret = yf_s201c_get_current_period(flow_sensor, &current_period);
                if (ret < 0) {
                    LOG_ERR("Failed to get current period (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 1, 1);
                    last_health_update = k_uptime_get();
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
                        LOG_INF("Pump turned on with plateau period: %lld ms", latest_plateau_period);
                        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                    }
                } else {
                    thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                }
                last_health_update = k_uptime_get();
            } else {
                /* No plateau detected - pump controller handles timeout internally */
                thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
                last_health_update = k_uptime_get();
            }
        } else {
            /* Timeout occurred - could be sensor timeout or just periodic wakeup */
            if (pump_controller_is_on(pump)) {
                /* Sensor timeout while pump was on - turn off pump */
                LOG_INF("Sensor data timeout, turning off pump");
                int ret = pump_controller_turn_off(pump);
                if (ret < 0) {
                    LOG_ERR("Failed to turn off pump on timeout (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
                } else {
                    initial_plateau_period = 0;
                    latest_plateau_period = 0;
                    yf_s201c_reset(flow_sensor);
                    flow_analyzer_reset();
                LOG_INF("Pump turned off due to sensor timeout");
                thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
                last_health_update = k_uptime_get();
                }
            } else {
                /* Periodic wakeup when pump is off - update health */
                int64_t now = k_uptime_get();
                if ((now - last_health_update) >= 1000) {
                    last_health_update = now;
                    thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
                }
            }
        }
    }
}

void safety_monitor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Safety monitor thread started");

    /* Get pump controller device reference */
    const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
    if (!device_is_ready(pump)) {
        LOG_ERR("Pump controller device not ready in safety monitor thread");
        return;
    }

    /* Safety monitoring state */
    int64_t pump_start_time = 0;
    bool pump_was_on = false;
    bool emergency_stop_active = false;
    uint32_t safety_check_count = 0;
    uint32_t safety_warnings = 0;
    uint32_t emergency_stops = 0;

    LOG_INF("Safety monitor thread ready - max runtime: %d min, check interval: %d ms",
           CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES,
           CONFIG_SAFETY_MONITOR_CHECK_INTERVAL_MS);

    while (!thread_manager_is_shutdown_requested()) {
        int64_t current_time = k_uptime_get();
        bool pump_currently_on = pump_controller_is_on(pump);

        /* Track pump state changes for independent runtime monitoring */
        if (pump_currently_on && !pump_was_on) {
            /* Pump just turned on - start independent timing */
            pump_start_time = current_time;
            emergency_stop_active = false;
            LOG_INF("Safety monitor: Pump start detected, beginning runtime tracking");
        } else if (!pump_currently_on && pump_was_on) {
            /* Pump just turned off - reset timing */
            pump_start_time = 0;
            LOG_INF("Safety monitor: Pump stop detected, resetting runtime tracking");
        }
        pump_was_on = pump_currently_on;

        /* Perform safety checks when pump is running */
        if (pump_currently_on && !emergency_stop_active) {
            safety_check_count++;

            /* Independent runtime timeout check */
            int64_t runtime_ms = current_time - pump_start_time;
            int64_t max_runtime_ms = (int64_t)CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES * 60 * 1000;
            int64_t warning_threshold_ms = (int64_t)CONFIG_SAFETY_MONITOR_WARNING_THRESHOLD_MINUTES * 60 * 1000;

            /* Issue warning when approaching timeout */
            if (runtime_ms >= warning_threshold_ms && runtime_ms < max_runtime_ms) {
                safety_warnings++;
                LOG_WRN("Safety monitor: Pump runtime warning - %lld/%lld ms (check #%u)",
                       runtime_ms, max_runtime_ms, safety_check_count);
            }

            /* Emergency stop if maximum runtime exceeded */
            if (runtime_ms >= max_runtime_ms) {
                emergency_stops++;
                emergency_stop_active = true;
                LOG_ERR("Safety monitor: EMERGENCY STOP - Maximum runtime exceeded (%lld ms > %lld ms)",
                       runtime_ms, max_runtime_ms);

                int ret = pump_controller_emergency_stop(pump);
                if (ret < 0) {
                    LOG_ERR("Safety monitor: Failed to execute emergency stop (%d)", ret);
                    thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
                } else {
                    LOG_INF("Safety monitor: Emergency stop executed successfully");
                    thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
                }
            }

            /* Periodic safety system check */
            bool safety_ok = pump_controller_safety_check(pump);
            if (!safety_ok) {
                safety_warnings++;
                LOG_WRN("Safety monitor: Safety system check failed (check #%u)", safety_check_count);

                /* Get detailed pump state for diagnostics */
                struct pump_state_info state_info;
                int ret = pump_controller_get_state(pump, &state_info);
                if (ret == 0) {
                    LOG_WRN("Safety monitor: Pump state - current: %d, previous: %d, safety_active: %d",
                           state_info.current_state, state_info.previous_state,
                           state_info.safety_systems_active);
                }
            }

            /* Log periodic safety status */
            static int64_t last_status_log = 0;
            if (current_time - last_status_log > 30000) {  // Every 30 seconds
                LOG_INF("Safety monitor: Pump running - runtime: %lld ms, checks: %u, warnings: %u, stops: %u",
                       runtime_ms, safety_check_count, safety_warnings, emergency_stops);
                last_status_log = current_time;
            }
        } else if (!pump_currently_on) {
            /* Pump is off - log recovery if we had an emergency stop */
            if (emergency_stop_active) {
                emergency_stop_active = false;
                LOG_INF("Safety monitor: Emergency stop recovery detected - pump is now off");
            }

            /* Periodic status when pump is off */
            static int64_t last_off_status = 0;
            if (current_time - last_off_status > 60000) {  // Every minute
                LOG_INF("Safety monitor: Pump off - total checks: %u, warnings: %u, emergency stops: %u",
                       safety_check_count, safety_warnings, emergency_stops);
                last_off_status = current_time;
            }
        }

        /* Update thread health */
        thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);

        /* Safety check interval */
        k_sleep(K_MSEC(CONFIG_SAFETY_MONITOR_CHECK_INTERVAL_MS));
    }
}

void supervisor_thread(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Supervisor thread started - system coordination and monitoring");

    /* Supervisor state tracking */
    int64_t system_start_time = k_uptime_get();
    uint32_t system_health_checks = 0;
    uint32_t system_warnings = 0;
    uint32_t system_errors = 0;
    bool system_stable = true;

    /* System coordination state */
    struct {
        bool sensor_monitor_active;
        bool pump_controller_active;
        bool safety_monitor_active;
        int64_t last_coordination_check;
    } coordination_state = {false, false, false, 0};

    LOG_INF("Supervisor thread ready for system coordination");

    while (!thread_manager_is_shutdown_requested()) {
        int64_t current_time = k_uptime_get();
        system_health_checks++;

        /* System health assessment */
        int health_status = thread_health_check_all();
        if (health_status != 0) {
            system_warnings++;
            system_stable = false;
            LOG_WRN("Supervisor: System health issues detected (check #%u, warnings: %u)",
                   system_health_checks, system_warnings);

            /* Log detailed thread health status */
            for (int i = 0; i < thread_health_count; i++) {
                if (thread_health[i].status != THREAD_HEALTH_OK) {
                    const char *name = k_thread_name_get(thread_health[i].thread_id);
                    if (name == NULL) name = "unknown";
                    LOG_WRN("Supervisor: Thread %s health: %d, last check: %lld ms ago",
                           name,
                           thread_health[i].status,
                           current_time - thread_health[i].last_check_time);
                }
            }
        } else {
            /* System is healthy */
            if (!system_stable) {
                system_stable = true;
                LOG_INF("Supervisor: System health restored (check #%u)", system_health_checks);
            }
        }

        /* Thread coordination and status monitoring */
        if (current_time - coordination_state.last_coordination_check > 10000) {  // Every 10 seconds
            coordination_state.last_coordination_check = current_time;

            /* Check thread activity indicators */
            bool sensor_active = (thread_health[0].messages_processed > 0);  // Sensor monitor
            bool pump_active = (thread_health[1].messages_processed > 0);    // Pump controller
            bool safety_active = (thread_health[2].errors_encountered >= 0); // Safety monitor (always active)

            /* Detect thread state changes */
            if (sensor_active != coordination_state.sensor_monitor_active) {
                coordination_state.sensor_monitor_active = sensor_active;
                LOG_INF("Supervisor: Sensor monitor activity: %s", sensor_active ? "ACTIVE" : "QUIET");
            }

            if (pump_active != coordination_state.pump_controller_active) {
                coordination_state.pump_controller_active = pump_active;
                LOG_INF("Supervisor: Pump controller activity: %s", pump_active ? "ACTIVE" : "QUIET");
            }

            if (safety_active != coordination_state.safety_monitor_active) {
                coordination_state.safety_monitor_active = safety_active;
                LOG_INF("Supervisor: Safety monitor activity: %s", safety_active ? "ACTIVE" : "QUIET");
            }
        }

        /* System status reporting */
        static int64_t last_status_report = 0;
        if (current_time - last_status_report > 60000) {  // Every minute
            int64_t uptime_minutes = (current_time - system_start_time) / 60000;

            LOG_INF("Supervisor: System status - Uptime: %lld min, Checks: %u, Warnings: %u, Errors: %u, Stable: %s",
                   uptime_minutes, system_health_checks, system_warnings, system_errors,
                   system_stable ? "YES" : "NO");

            /* Detailed thread statistics */
            LOG_INF("Supervisor: Thread statistics:");
            for (int i = 0; i < thread_health_count; i++) {
                const char *name = k_thread_name_get(thread_health[i].thread_id);
                if (name == NULL) name = "unknown";
                LOG_INF("  %s: msgs=%u, errs=%u, stack_peak=%zu B",
                       name,
                       thread_health[i].messages_processed,
                       thread_health[i].errors_encountered,
                       thread_health[i].stack_peak_usage);
            }

            last_status_report = current_time;
        }

        /* System event handling and coordination */
        /* TODO: Add system event handling (shutdown requests, configuration updates, etc.) */

        /* Update supervisor health */
        thread_health_update(k_current_get(), system_stable ? THREAD_HEALTH_OK : THREAD_HEALTH_ERROR, 0, 0);

        /* Supervisor check interval - less frequent than safety monitor */
        k_sleep(K_MSEC(2000));  // 2 second intervals
    }
}