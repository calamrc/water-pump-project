/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 *
 * TimerService — owns the countdown timer state machine and publishes
 * authoritative state on timer_state_chan via zbus.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#include "services/timer_service.h"
#include "zbus/channels.h"
#include "timer/timer_state_machine.h"

LOG_MODULE_REGISTER(timer_service, CONFIG_APP_LOG_LEVEL);

#define TIMER_SERVICE_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(timer_stack, TIMER_SERVICE_STACK_SIZE);
static struct k_thread timer_thread_cb;

static void publish_timer_state(void)
{
	uint8_t minutes, seconds;
	timer_sm_get_time(&minutes, &seconds);

	enum timer_state raw_state = timer_sm_get_state();

	enum timer_state zbus_state;
	switch (raw_state) {
	case TIMER_STATE_SETTING:  zbus_state = TIMER_STATE_SETTING; break;
	case TIMER_STATE_RUNNING:  zbus_state = TIMER_STATE_RUNNING; break;
	case TIMER_STATE_PAUSED:   zbus_state = TIMER_STATE_PAUSED; break;
	case TIMER_STATE_COMPLETED: zbus_state = TIMER_STATE_COMPLETED; break;
	default:                   zbus_state = TIMER_STATE_SETTING; break;
	}

	struct timer_status msg = {
		.remaining_sec = timer_sm_get_remaining_seconds(),
		.state = zbus_state,
		.flash = (zbus_state == TIMER_STATE_COMPLETED),
	};

	(void)zbus_chan_pub(&timer_state_chan, &msg, K_NO_WAIT);
}

static void timer_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("TimerService started — publishing on timer_state_chan");

	/* Initial state */
	publish_timer_state();

	int64_t last_tick = k_uptime_get();

	while (1) {
		int64_t now = k_uptime_get();

		bool state_changed = false;

		if (timer_sm_get_state() == TIMER_STATE_RUNNING) {
			if ((now - last_tick) >= 1000) {
				last_tick = now;
				state_changed = timer_sm_update();
				publish_timer_state();
			}
		}

		/* Periodic heartbeat publish even when not running */
		static int64_t last_pub = 0;
		if ((now - last_pub) > 1000) {
			last_pub = now;
			if (!state_changed) {
				publish_timer_state();
			}
		}

		k_sleep(K_MSEC(100));
	}
}

int timer_service_start(void)
{
	k_tid_t tid = k_thread_create(&timer_thread_cb,
				      timer_stack,
				      K_THREAD_STACK_SIZEOF(timer_stack),
				      timer_thread,
				      NULL, NULL, NULL,
				      6, 0, K_NO_WAIT);
	if (!tid) {
		LOG_ERR("Failed to create TimerService thread");
		return -EAGAIN;
	}

	k_thread_name_set(&timer_thread_cb, "timer");
	LOG_INF("TimerService thread created");
	return 0;
}
