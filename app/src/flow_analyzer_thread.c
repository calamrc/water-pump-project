/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "flow_analyzer.h"
#include "fixed_math.h"
#include "zbus/messages.h"
#include "thread_comm.h"
#include "thread_manager.h"

LOG_MODULE_REGISTER(flow_analyzer_thread, CONFIG_APP_LOG_LEVEL);

ZBUS_CHAN_DECLARE(sensor_data_ch);
ZBUS_CHAN_DECLARE(pump_state_ch);
ZBUS_CHAN_DECLARE(plateau_detected_ch);

extern const struct zbus_observer flow_event_sub;

static bool pump_on;

static int flow_analyzer_thread_init(void)
{
	flow_analyzer_init();
	pump_on = false;
	LOG_INF("Flow analyzer thread initialized");
	return 0;
}

void flow_analyzer_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("Flow analyzer thread started");

	flow_analyzer_thread_init();

	int64_t last_health_update = k_uptime_get();

	while (!thread_manager_is_shutdown_requested()) {
		const struct zbus_channel *chan;
		uint8_t msg_buf[MAX(sizeof(struct sensor_data_msg),
				    sizeof(struct pump_state_msg))];

		int ret = zbus_sub_wait_msg(&flow_event_sub, &chan, msg_buf,
					    K_MSEC(2000));

		if (ret == 0) {
			if (chan == &sensor_data_ch) {
				struct sensor_data_msg sensor_msg;

				memcpy(&sensor_msg, msg_buf, sizeof(sensor_msg));

				if (!sensor_msg.data_valid) {
					continue;
				}

				fixed_t k_factor = pump_on ? FIXED_PLATEAU_K_FACTOR
							   : FIXED_PLATEAU_INITIAL_K_FACTOR;

				bool plateau = flow_analyzer_detect_plateau(
					sensor_msg.flow_rate, k_factor);

				if (plateau) {
					struct plateau_detected_msg plateau_msg = {
						.flow_rate = sensor_msg.flow_rate,
						.noise_std = flow_analyzer_get_noise_std(),
						.flow_slope = flow_analyzer_get_flow_slope(),
						.period_us = sensor_msg.period_us,
					};

					LOG_INF("Plateau detected: flow=%.2f L/min, "
						"noise_std=%.4f, period=%lld us",
						(double)fixed_to_float(sensor_msg.flow_rate),
						(double)fixed_to_float(plateau_msg.noise_std),
						plateau_msg.period_us);

					ret = zbus_chan_pub(&plateau_detected_ch,
							   &plateau_msg, K_MSEC(100));
					if (ret < 0) {
						LOG_WRN("Failed to publish plateau: %d",
							ret);
					}

					flow_analyzer_reset();
				}
			} else if (chan == &pump_state_ch) {
				struct pump_state_msg pump_msg;

				memcpy(&pump_msg, msg_buf, sizeof(pump_msg));

				if (pump_msg.change == PUMP_TURNED_ON) {
					pump_on = true;
					LOG_INF("Flow analyzer: pump turned on, "
						"switching to k_factor=%.1f",
						(double)fixed_to_float(FIXED_PLATEAU_K_FACTOR));
				} else if (pump_msg.change == PUMP_TURNED_OFF ||
					   pump_msg.change == PUMP_EMERGENCY_STOP) {
					pump_on = false;
					flow_analyzer_reset();
					LOG_INF("Flow analyzer: pump off, reset");
				}
			}

			int64_t now = k_uptime_get();
			if ((now - last_health_update) >= 1000) {
				last_health_update = now;
				thread_health_update(k_current_get(), THREAD_HEALTH_OK,
						     1, 0);
			}
		} else {
			int64_t now = k_uptime_get();
			if ((now - last_health_update) >= 5000) {
				last_health_update = now;
				thread_health_update(k_current_get(), THREAD_HEALTH_OK,
						     0, 0);
			}
		}
	}
}