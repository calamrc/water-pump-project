/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <app/drivers/pump_controller.h>
#include <app/drivers/yf_s201c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include "thread_comm.h"
#include "zbus/messages.h"

LOG_MODULE_REGISTER(thread_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread stack definitions */
K_THREAD_STACK_DEFINE(sensor_monitor_stack, CONFIG_SENSOR_MONITOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(pump_controller_stack, CONFIG_PUMP_CONTROLLER_STACK_SIZE);
K_THREAD_STACK_DEFINE(safety_monitor_stack, CONFIG_SAFETY_MONITOR_STACK_SIZE);

K_THREAD_STACK_DEFINE(ui_manager_stack, CONFIG_UI_MANAGER_STACK_SIZE);
K_THREAD_STACK_DEFINE(flow_analyzer_stack, CONFIG_FLOW_ANALYZER_STACK_SIZE);

/* Thread control blocks */
static struct k_thread sensor_monitor_thread_cb;
static struct k_thread pump_controller_thread_cb;
static struct k_thread safety_monitor_thread_cb;

static struct k_thread ui_manager_thread_cb;
static struct k_thread flow_analyzer_thread_cb;

/* Thread health monitoring */
static struct thread_health_info thread_health[5];
static int thread_health_count = 0;

/* Shutdown coordination */
static bool system_shutdown_requested = false;
static struct k_sem shutdown_sem;

/* Sensor data semaphore for ISR-to-thread signaling */
K_SEM_DEFINE(data_sem, 0, 1);

/* Forward declarations for thread entry functions */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3);
void pump_controller_thread(void *arg1, void *arg2, void *arg3);
void safety_monitor_thread(void *arg1, void *arg2, void *arg3);

void ui_manager_thread(void *arg1, void *arg2, void *arg3);
void flow_analyzer_thread(void *arg1, void *arg2, void *arg3);

ZBUS_CHAN_DECLARE(sensor_data_ch);
ZBUS_CHAN_DECLARE(pump_state_ch);
ZBUS_CHAN_DECLARE(plateau_detected_ch);
ZBUS_CHAN_DECLARE(sensor_cmd_ch);

extern const struct zbus_observer pump_event_sub;
extern const struct zbus_observer safety_event_sub;

/**
 * @brief Initialize thread health monitoring for a thread
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

void thread_health_update(k_tid_t thread_id, enum thread_health_status status,
			 uint32_t messages_processed, uint32_t errors_encountered)
{
	for (int i = 0; i < thread_health_count; i++) {
		if (thread_health[i].thread_id == thread_id) {
			thread_health[i].last_check_time = k_uptime_get();
			thread_health[i].status = status;
			thread_health[i].messages_processed += messages_processed;
			thread_health[i].errors_encountered += errors_encountered;
			thread_health[i].stack_peak_usage = 0;
			break;
		}
	}
}

static int thread_health_check_all(void)
{
	int64_t current_time = k_uptime_get();
	int unhealthy_count = 0;
	bool safety_monitor_timeout = false;
	bool pump_controller_timeout = false;

	for (int i = 0; i < thread_health_count; i++) {
		if (thread_health[i].thread_id == NULL) {
			LOG_ERR("Thread health entry %d has NULL thread_id", i);
			unhealthy_count++;
			continue;
		}

		int64_t time_since_check = current_time - thread_health[i].last_check_time;
		const char *thread_name = k_thread_name_get(thread_health[i].thread_id);

		if (thread_name == NULL) {
			thread_name = "unknown";
		}

		if (time_since_check > THREAD_HEALTH_TIMEOUT_MS) {
			thread_health[i].status = THREAD_HEALTH_TIMEOUT;
			unhealthy_count++;

			bool is_critical = (thread_name != NULL) &&
					   (strcmp(thread_name, "safety_mon") == 0 ||
					    strcmp(thread_name, "pump_ctrl") == 0);
			if (is_critical) {
				LOG_ERR("CRITICAL: Thread %s health timeout: %lld ms since last check",
					thread_name, time_since_check);

				if (strcmp(thread_name, "safety_mon") == 0) {
					safety_monitor_timeout = true;
				} else if (strcmp(thread_name, "pump_ctrl") == 0) {
					pump_controller_timeout = true;
				}
			} else {
				LOG_WRN("Thread %s health timeout: %lld ms since last check",
					thread_name, time_since_check);
			}
		}

		if (thread_health[i].stack_peak_usage > thread_health[i].stack_size * 90 / 100) {
			thread_health[i].status = THREAD_HEALTH_STACK_CRITICAL;
			unhealthy_count++;
			LOG_WRN("Thread %s stack usage critical: %zu/%zu bytes (%.1f%%)",
				thread_name,
				thread_health[i].stack_peak_usage,
				thread_health[i].stack_size,
				(double)((float)thread_health[i].stack_peak_usage * 100 /
					 thread_health[i].stack_size));
		}

		if (thread_health[i].errors_encountered > 10) {
			LOG_WRN("Thread %s has high error count: %u errors",
				thread_name, thread_health[i].errors_encountered);
		}
	}

	if (safety_monitor_timeout || pump_controller_timeout) {
		LOG_ERR("CRITICAL: Safety-critical thread timeout - forcing pump off");

		const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
		if (device_is_ready(pump) && pump_controller_is_on(pump)) {
			int ret = pump_controller_emergency_stop(pump);
			if (ret < 0) {
				LOG_ERR("Emergency stop from health monitor failed: %d", ret);
			}
		}
	}

	return (unhealthy_count == 0) ? 0 : -ETIMEDOUT;
}

int thread_manager_create_all_threads(void)
{
	LOG_INF("Creating application threads...");

	k_sem_init(&shutdown_sem, 0, 1);

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

	k_tid_t flow_tid = k_thread_create(&flow_analyzer_thread_cb, flow_analyzer_stack,
					    K_THREAD_STACK_SIZEOF(flow_analyzer_stack),
					    flow_analyzer_thread, NULL, NULL, NULL,
					    CONFIG_FLOW_ANALYZER_PRIORITY, 0, K_NO_WAIT);
	if (flow_tid == NULL) {
		LOG_ERR("Failed to create flow analyzer thread");
		return -EAGAIN;
	}
	k_thread_name_set(&flow_analyzer_thread_cb, "flow_analyzer");
	thread_health_init(flow_tid, "flow_analyzer", CONFIG_FLOW_ANALYZER_STACK_SIZE);

	LOG_INF("All threads created successfully");
	return 0;
}

void thread_manager_monitor_health(void)
{
	LOG_INF("Starting thread health monitoring...");

	while (true) {
		int health_status = thread_health_check_all();
		if (health_status != 0) {
			LOG_ERR("Thread health issues detected: %d", health_status);
		}

		static int64_t last_summary = 0;
		int64_t current_time = k_uptime_get();
		if (current_time - last_summary > 30000) {
			LOG_INF("Thread health summary:");
			for (int i = 0; i < thread_health_count; i++) {
				const char *name = k_thread_name_get(thread_health[i].thread_id);
				if (name == NULL) {
					name = "unknown";
				}
				LOG_INF("  %s: status=%d, stack_peak=%zu, msgs=%u, errs=%u",
					name,
					thread_health[i].status,
					thread_health[i].stack_peak_usage,
					thread_health[i].messages_processed,
					thread_health[i].errors_encountered);
			}
			last_summary = current_time;
		}

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

bool thread_manager_is_shutdown_requested(void)
{
	return system_shutdown_requested;
}

int thread_manager_shutdown_all_threads(void)
{
	LOG_INF("Initiating graceful system shutdown...");

	system_shutdown_requested = true;
	k_sem_give(&shutdown_sem);

	k_sleep(K_MSEC(500));

	const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
	if (device_is_ready(pump) && pump_controller_is_on(pump)) {
		LOG_INF("Shutdown: Ensuring pump is safely off");
		int ret = pump_controller_turn_off(pump);
		if (ret < 0) {
			LOG_WRN("Shutdown: Failed to turn off pump gracefully (%d), using emergency stop",
				ret);
			ret = pump_controller_emergency_stop(pump);
			if (ret < 0) {
				LOG_ERR("Shutdown: Emergency stop also failed (%d)", ret);
			}
		}
	}

	LOG_INF("Waiting for threads to exit...");

	int timeout_count = 0;
	const int max_timeouts = 10;

	while (timeout_count < max_timeouts) {
		bool all_exited = true;

		if (k_thread_join(&sensor_monitor_thread_cb, K_MSEC(100)) != 0) {
			all_exited = false;
		}
		if (k_thread_join(&pump_controller_thread_cb, K_MSEC(100)) != 0) {
			all_exited = false;
		}
		if (k_thread_join(&safety_monitor_thread_cb, K_MSEC(100)) != 0) {
			all_exited = false;
		}
		if (k_thread_join(&ui_manager_thread_cb, K_MSEC(100)) != 0) {
			all_exited = false;
		}
		if (k_thread_join(&flow_analyzer_thread_cb, K_MSEC(100)) != 0) {
			all_exited = false;
		}

		if (all_exited) {
			break;
		}

		timeout_count++;
		LOG_INF("Shutdown: Waiting for threads to exit (%d/%d)...", timeout_count,
			max_timeouts);
	}

	if (timeout_count >= max_timeouts) {
		LOG_WRN("Shutdown: Some threads did not exit gracefully within timeout");
	} else {
		LOG_INF("Shutdown: All threads exited gracefully");
	}

	system_shutdown_requested = false;

	LOG_INF("System shutdown complete");
	return 0;
}

/* Sensor monitor thread - publishes sensor_data_ch via Zbus */
void sensor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	LOG_INF("Sensor monitor thread started");

	const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));
	if (!device_is_ready(flow_sensor)) {
		LOG_ERR("Flow sensor device not ready in sensor monitor thread");
		return;
	}

	int ret = yf_s201c_set_data_semaphore(flow_sensor, &data_sem);
	if (ret < 0) {
		LOG_ERR("Could not configure sensor semaphore (%d)", ret);
		return;
	}

	static uint32_t sequence_counter = 0;

	LOG_INF("Sensor monitor thread ready for data acquisition");

	int64_t last_health_update = k_uptime_get();

	while (!thread_manager_is_shutdown_requested()) {
		if (k_sem_take(&data_sem, K_MSEC(1000)) == 0) {
			fixed_t flow_rate;
			ret = yf_s201c_get_flow_rate(flow_sensor, &flow_rate);

			if (ret == 0 && yf_s201c_is_data_valid(flow_sensor)) {
				int64_t period_us;
				ret = yf_s201c_get_current_period(flow_sensor, &period_us);
				if (ret < 0) {
					period_us = 0;
				}

				struct sensor_data_msg msg = {
					.flow_rate = flow_rate,
					.period_us = period_us,
					.timestamp = k_uptime_get(),
					.data_valid = true,
					.sequence_number = sequence_counter++,
				};

				ret = zbus_chan_pub(&sensor_data_ch, &msg, K_MSEC(100));
				if (ret == 0) {
					LOG_DBG("Sensor data published (seq: %u, flow: %.2f L/min)",
						msg.sequence_number,
						(double)fixed_to_float(flow_rate));
					thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
				} else {
					LOG_WRN("Failed to publish sensor data (%d)", ret);
					thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0,
							     1);
				}
			} else {
				LOG_DBG("Invalid sensor data received");
				thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
			}
			last_health_update = k_uptime_get();
		} else {
			int64_t now = k_uptime_get();
			if ((now - last_health_update) >= 1000) {
				last_health_update = now;
				thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
			}
		}
	}
}

/* Pump controller thread - subscribes to sensor_data_ch (liveness tracking)
 * and plateau_detected_ch (pump control), publishes pump_state_ch and
 * sensor_cmd_ch.
 *
 * The pump is turned off when zbus_sub_wait_msg times out while the pump is
 * on — meaning no sensor data or plateau messages arrived within 1.5x the
 * plateau period. This mirrors the original k_msgq_get timeout logic.
 * Lack of plateau detection alone does NOT turn off the pump, because
 * the flow analyzer needs several samples to re-detect a plateau after
 * each reset. Sensor timeout = water stopped flowing = turn off pump.
 */
void pump_controller_thread(void *arg1, void *arg2, void *arg3)
{
	LOG_INF("Pump controller thread started");

	const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
	if (!device_is_ready(pump)) {
		LOG_ERR("Pump controller device not ready");
		return;
	}

	int64_t initial_plateau_period = 0;
	int64_t latest_plateau_period = 0;
	int64_t pump_off_time = 0;

	LOG_INF("Pump controller thread ready");

	int64_t last_health_update = k_uptime_get();

	while (!thread_manager_is_shutdown_requested()) {
		const struct zbus_channel *chan;
		uint8_t msg_buf[MAX(sizeof(struct sensor_data_msg),
				    sizeof(struct plateau_detected_msg))];
		k_timeout_t timeout;

		if (pump_controller_is_on(pump) && latest_plateau_period > 0) {
			int64_t timeout_us = latest_plateau_period * 15 / 10;
			timeout_us = MIN(timeout_us, (int64_t)CONFIG_APP_MAX_TIMEOUT_US);
			timeout = K_USEC(timeout_us);
		} else {
			timeout = K_MSEC(1000);
		}

		int ret = zbus_sub_wait_msg(&pump_event_sub, &chan, msg_buf, timeout);

		if (ret == 0 && chan == &sensor_data_ch) {
			thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
			last_health_update = k_uptime_get();
			continue;
		}

		if (ret == 0 && chan == &plateau_detected_ch) {
			struct plateau_detected_msg plateau_msg;

			memcpy(&plateau_msg, msg_buf, sizeof(plateau_msg));

			if (pump_off_time > 0) {
				int64_t elapsed_ms = k_uptime_get() - pump_off_time;
				if (elapsed_ms < 2000) {
					LOG_WRN("Ignoring stale plateau detected %lld ms after pump off",
						elapsed_ms);
					continue;
				}
				pump_off_time = 0;
			}

			LOG_INF("Pump controller: plateau detected (flow=%.2f, period=%lld us)",
				(double)fixed_to_float(plateau_msg.flow_rate),
				plateau_msg.period_us);

			bool current_pump_on = pump_controller_is_on(pump);
			int64_t current_period = plateau_msg.period_us;

			if (!(current_pump_on && current_period < initial_plateau_period)) {
				latest_plateau_period = current_period;
			}

			pump_controller_update_plateau_period(pump, latest_plateau_period);

			if (!current_pump_on && current_period > 0) {
				ret = pump_controller_turn_on(pump, latest_plateau_period);
				if (ret < 0) {
					LOG_ERR("Failed to turn on pump (%d)", ret);
					thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 1,
							     1);
				} else {
					initial_plateau_period = current_period;
					LOG_INF("Pump turned on with plateau period: %lld us",
						latest_plateau_period);

					struct pump_state_msg state_msg = {
						.change = PUMP_TURNED_ON,
						.plateau_period_us = latest_plateau_period,
					};

					ret = zbus_chan_pub(&pump_state_ch, &state_msg, K_MSEC(100));
					if (ret < 0) {
						LOG_WRN("Failed to publish pump on state: %d", ret);
					}

					thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
				}
			} else {
				thread_health_update(k_current_get(), THREAD_HEALTH_OK, 1, 0);
			}
			last_health_update = k_uptime_get();
			continue;
		}

		/* Timeout — no sensor data or plateau detected within the timeout */
		if (pump_controller_is_on(pump)) {
			LOG_INF("Sensor data timeout, turning off pump");

			int res = pump_controller_turn_off(pump);
			if (res < 0) {
				LOG_ERR("Failed to turn off pump on timeout (%d)", res);
				thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0, 1);
			} else {
				initial_plateau_period = 0;
				latest_plateau_period = 0;
				pump_off_time = k_uptime_get();

				struct pump_state_msg state_msg = {
					.change = PUMP_TURNED_OFF,
					.plateau_period_us = 0,
				};

				ret = zbus_chan_pub(&pump_state_ch, &state_msg, K_MSEC(100));
				if (ret < 0) {
					LOG_WRN("Failed to publish pump off state: %d", ret);
				}

				struct sensor_cmd_msg cmd_msg = {
					.cmd = SENSOR_CMD_RESET,
				};

				ret = zbus_chan_pub(&sensor_cmd_ch, &cmd_msg, K_MSEC(100));
				if (ret < 0) {
					LOG_WRN("Failed to publish sensor reset cmd: %d", ret);
				}

				/* Drain stale messages from the subscriber queue to prevent
			 * a queued plateau_detected from immediately re-enabling
			 * the pump after a timeout turn-off.
			 */
			const struct zbus_channel *drain_chan;
			uint8_t drain_buf[MAX(sizeof(struct sensor_data_msg),
					     sizeof(struct plateau_detected_msg))];

			while (zbus_sub_wait_msg(&pump_event_sub, &drain_chan,
						 drain_buf, K_NO_WAIT) == 0) {
				LOG_DBG("Drained stale message after pump timeout");
			}

			LOG_INF("Pump turned off due to sensor timeout");
			thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
			}
			last_health_update = k_uptime_get();
		} else {
			int64_t now = k_uptime_get();
			if ((now - last_health_update) >= 1000) {
				last_health_update = now;
				thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
			}
		}
	}
}

static int execute_emergency_stop(const struct device *pump)
{
	int ret = pump_controller_emergency_stop(pump);
	if (ret < 0) {
		LOG_ERR("Safety monitor: Failed to execute emergency stop (%d)", ret);
		return ret;
	}

	LOG_INF("Safety monitor: Emergency stop executed successfully");

	struct pump_state_msg state_msg = {
		.change = PUMP_EMERGENCY_STOP,
		.plateau_period_us = 0,
	};

	ret = zbus_chan_pub(&pump_state_ch, &state_msg, K_MSEC(100));
	if (ret < 0) {
		LOG_WRN("Failed to publish emergency stop state: %d", ret);
	}

	return 0;
}

/* Safety monitor thread - subscribes to pump_state_ch and sensor_data_ch.
 * Watches for max runtime exceeded and sensor data liveness (stall detection).
 * Triggers emergency stop via direct call + Zbus publication.
 */
void safety_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	LOG_INF("Safety monitor thread started");

	const struct device *pump = PUMP_CONTROLLER_DT_GET(DT_NODELABEL(pump_controller));
	if (!device_is_ready(pump)) {
		LOG_ERR("Pump controller device not ready in safety monitor thread");
		return;
	}

	int64_t pump_on_time = 0;
	bool pump_on = false;
	bool emergency_stop_active = false;
	int64_t last_sensor_data_time = 0;
	uint32_t safety_check_count = 0;
	uint32_t safety_warnings = 0;
	uint32_t emergency_stops = 0;

	LOG_INF("Safety monitor thread ready - max runtime: %d min, sensor timeout: %d ms",
		CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES,
		CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS);

	while (!thread_manager_is_shutdown_requested()) {
		const struct zbus_channel *chan;
		uint8_t msg_buf[MAX(sizeof(struct pump_state_msg),
				    sizeof(struct sensor_data_msg))];

		int ret = zbus_sub_wait_msg(&safety_event_sub, &chan, msg_buf,
					    K_MSEC(CONFIG_SAFETY_MONITOR_CHECK_INTERVAL_MS));

		if (ret == 0 && chan == &pump_state_ch) {
			struct pump_state_msg pump_msg;

			memcpy(&pump_msg, msg_buf, sizeof(pump_msg));

			if (pump_msg.change == PUMP_TURNED_ON) {
				if (!pump_on) {
					pump_on_time = k_uptime_get();
					last_sensor_data_time = k_uptime_get();
					emergency_stop_active = false;
					pump_on = true;
					LOG_INF("Safety monitor: Pump start detected via Zbus");
				}
			} else if (pump_msg.change == PUMP_TURNED_OFF) {
				if (pump_on) {
					pump_on = false;
					pump_on_time = 0;
					last_sensor_data_time = 0;
					LOG_INF("Safety monitor: Pump stop detected via Zbus");
				}
				if (emergency_stop_active) {
					emergency_stop_active = false;
					LOG_INF("Safety monitor: Emergency stop recovery detected");
				}
			} else if (pump_msg.change == PUMP_EMERGENCY_STOP) {
				pump_on = false;
				pump_on_time = 0;
				last_sensor_data_time = 0;
				emergency_stop_active = false;
				LOG_INF("Safety monitor: Emergency stop detected via Zbus");
			}
		} else if (ret == 0 && chan == &sensor_data_ch) {
			if (pump_on) {
				last_sensor_data_time = k_uptime_get();
			}
		}

		if (pump_on && !emergency_stop_active) {
			int64_t current_time = k_uptime_get();
			safety_check_count++;

			int64_t runtime_ms = current_time - pump_on_time;
			int64_t max_runtime_ms =
				(int64_t)CONFIG_SAFETY_MONITOR_MAX_RUNTIME_MINUTES * 60 * 1000;
			int64_t warning_threshold_ms =
				(int64_t)CONFIG_SAFETY_MONITOR_WARNING_THRESHOLD_MINUTES * 60 *
				1000;

			if (runtime_ms >= warning_threshold_ms && runtime_ms < max_runtime_ms) {
				safety_warnings++;
				LOG_WRN("Safety monitor: Pump runtime warning - %lld/%lld ms (check #%u)",
					runtime_ms, max_runtime_ms, safety_check_count);
			}

			if (runtime_ms >= max_runtime_ms) {
				emergency_stops++;
				emergency_stop_active = true;
				LOG_ERR("Safety monitor: EMERGENCY STOP - Maximum runtime exceeded "
					"(%lld ms > %lld ms)",
					runtime_ms, max_runtime_ms);

				if (execute_emergency_stop(pump) < 0) {
					thread_health_update(k_current_get(), THREAD_HEALTH_ERROR, 0,
							     1);
				} else {
					thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
				}
				continue;
			}

			if (last_sensor_data_time <= 0) {
				int64_t sensor_age_ms = current_time - pump_on_time;

				if (sensor_age_ms > CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS) {
					emergency_stops++;
					emergency_stop_active = true;
					LOG_ERR("Safety monitor: EMERGENCY STOP - No sensor data "
						"since pump start (%lld ms)",
						sensor_age_ms);

					if (execute_emergency_stop(pump) < 0) {
						thread_health_update(k_current_get(), THREAD_HEALTH_ERROR,
								     0, 1);
					} else {
						thread_health_update(k_current_get(), THREAD_HEALTH_OK,
								     0, 0);
					}
					continue;
				}
			} else {
				int64_t sensor_age_ms = current_time - last_sensor_data_time;
				if (sensor_age_ms > CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS) {
					emergency_stops++;
					emergency_stop_active = true;
					LOG_ERR("Safety monitor: EMERGENCY STOP - Sensor data stall "
						"(%lld ms > %d ms)",
						sensor_age_ms,
						CONFIG_APP_SAFETY_MONITOR_MAX_SENSOR_TIMEOUT_MS);

					if (execute_emergency_stop(pump) < 0) {
						thread_health_update(k_current_get(), THREAD_HEALTH_ERROR,
								     0, 1);
					} else {
						thread_health_update(k_current_get(), THREAD_HEALTH_OK,
								     0, 0);
					}
					continue;
				}
			}

			bool safety_ok = pump_controller_safety_check(pump);
			if (!safety_ok) {
				safety_warnings++;
				LOG_WRN("Safety monitor: Safety system check failed (check #%u)",
					safety_check_count);

				struct pump_state_info state_info;
				ret = pump_controller_get_state(pump, &state_info);
				if (ret == 0) {
					LOG_WRN("Safety monitor: Pump state - current: %d, previous: %d, "
						"safety_active: %d",
						state_info.current_state, state_info.previous_state,
						state_info.safety_systems_active);
				}
			}

			static int64_t last_status_log = 0;
			int64_t now = k_uptime_get();
			if (now - last_status_log > 30000) {
				LOG_INF("Safety monitor: Pump running - runtime: %lld ms, checks: %u, "
					"warnings: %u, stops: %u",
					runtime_ms, safety_check_count, safety_warnings,
					emergency_stops);
				last_status_log = now;
			}
		} else if (!pump_on) {
			static int64_t last_off_status = 0;
			int64_t now = k_uptime_get();
			if (now - last_off_status > 60000) {
				LOG_INF("Safety monitor: Pump off - total checks: %u, warnings: %u, "
					"emergency stops: %u",
					safety_check_count, safety_warnings, emergency_stops);
				last_off_status = now;
			}
		}

		thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
	}
}

