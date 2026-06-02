/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "services/ui_service.h"
#include "display/display_manager.h"
#include "input/input_manager.h"
#include "timer/timer_state_machine.h"
#include "zbus/channels.h"
#include <app/drivers/feedback_relay.h>

LOG_MODULE_REGISTER(ui_service, CONFIG_APP_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(ui_stack, 8192);
static struct k_thread ui_thread_cb;

static void ui_thread(void *a1, void *a2, void *a3)
{
	ARG_UNUSED(a1); ARG_UNUSED(a2); ARG_UNUSED(a3);

	LOG_INF("UIService started");

	while (1) {
		k_sleep(K_SECONDS(30));
		/* Future: subscribe to timer_state_chan + pump_status_chan and drive display */
	}
}

/* Listener for timer updates - handles last-10s feedback clicks */
static void on_timer_state(const struct zbus_channel *chan)
{
	ARG_UNUSED(chan);

	struct timer_status timer;
	if (zbus_chan_read(&timer_state_chan, &timer, K_NO_WAIT) != 0) {
		return;
	}

	/* Only give clicks during the last 10 seconds while running */
	if (timer.state == TIMER_STATE_RUNNING && timer.remaining_sec <= 10) {
		const struct device *feedback = FEEDBACK_RELAY_DT_GET(DT_NODELABEL(feedback_relay));
		if (device_is_ready(feedback)) {
			(void)feedback_relay_click(feedback);
		}
	}
}

ZBUS_LISTENER_DEFINE(ui_timer_listener, on_timer_state);

int ui_service_start(void)
{
	/* Initialize the thin UI managers (display + input + timer SM) */
	int ret = display_manager_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize display manager (%d)", ret);
		return ret;
	}

	ret = input_manager_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize input manager (%d)", ret);
		return ret;
	}

	timer_sm_init();

	/* Subscribe to timer updates so we can drive feedback clicks */
	ret = zbus_chan_add_obs(&timer_state_chan, &ui_timer_listener, K_NO_WAIT);
	if (ret < 0 && ret != -EALREADY) {
		LOG_WRN("Failed to attach UI timer listener (%d)", ret);
	}

	k_tid_t tid = k_thread_create(&ui_thread_cb, ui_stack,
				      K_THREAD_STACK_SIZEOF(ui_stack),
				      ui_thread, NULL, NULL, NULL,
				      7, 0, K_NO_WAIT);
	if (!tid) return -EAGAIN;

	k_thread_name_set(&ui_thread_cb, "ui");
	LOG_INF("UIService thread created");
	return 0;
}
