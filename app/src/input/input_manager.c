/**
 * @file input_manager.c
 * @brief Input manager implementation for rotary encoder and button using input subsystem
 *
 * Accumulates events from ISR context and publishes them via Zbus from workqueue.
 */

#include "input/input_manager.h"
#include "zbus/messages.h"

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(input_manager, CONFIG_APP_LOG_LEVEL);

#define LONG_PRESS_MS 1000
#define ENCODER_STEPS_PER_DETENT 2

static atomic_t encoder_accumulator = ATOMIC_INIT(0);
static atomic_t encoder_delta = ATOMIC_INIT(0);

static atomic_t button_pressed = ATOMIC_INIT(0);
static atomic_t long_press_reported = ATOMIC_INIT(0);

static atomic_t input_enabled = ATOMIC_INIT(1);

static struct k_timer long_press_timer;

ZBUS_CHAN_DECLARE(input_event_ch);

static void input_publish_work_handler(struct k_work *work);
static struct k_work input_publish_work;

static void long_press_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (!atomic_get(&input_enabled)) {
		return;
	}

	if (atomic_get(&button_pressed) && !atomic_get(&long_press_reported)) {
		atomic_set(&long_press_reported, 1);
		k_work_submit(&input_publish_work);
	}
}

static void input_callback(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!atomic_get(&input_enabled)) {
		return;
	}

	if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_WHEEL) {
		atomic_add(&encoder_accumulator, evt->value);

		int32_t accum = atomic_get(&encoder_accumulator);

		if (accum >= ENCODER_STEPS_PER_DETENT) {
			atomic_sub(&encoder_accumulator, ENCODER_STEPS_PER_DETENT);
			atomic_inc(&encoder_delta);
			k_work_submit(&input_publish_work);
		} else if (accum <= -ENCODER_STEPS_PER_DETENT) {
			atomic_add(&encoder_accumulator, ENCODER_STEPS_PER_DETENT);
			atomic_dec(&encoder_delta);
			k_work_submit(&input_publish_work);
		}
	}

	if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_ENTER) {
		if (evt->value == 1) {
			atomic_set(&button_pressed, 1);
			atomic_set(&long_press_reported, 0);
			k_timer_start(&long_press_timer, K_MSEC(LONG_PRESS_MS), K_NO_WAIT);
		} else if (evt->value == 0) {
			k_timer_stop(&long_press_timer);

			if (atomic_get(&button_pressed) && !atomic_get(&long_press_reported)) {
				k_work_submit(&input_publish_work);
			}

			atomic_set(&button_pressed, 0);
		}
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_callback, NULL);

static void input_publish_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	struct ui_input_event event = {
		.encoder_moved = false,
		.encoder_delta = 0,
		.button_press = BUTTON_PRESS_NONE,
	};

	if (atomic_get(&encoder_delta) != 0) {
		event.encoder_moved = true;
		event.encoder_delta = atomic_get(&encoder_delta);
		atomic_set(&encoder_delta, 0);
	}

	if (atomic_get(&button_pressed) && atomic_get(&long_press_reported)) {
		event.button_press = BUTTON_PRESS_LONG;
		atomic_set(&long_press_reported, 0);
	} else if (!atomic_get(&button_pressed) && !atomic_get(&long_press_reported)) {
		event.button_press = BUTTON_PRESS_SHORT;
	}

	if (event.encoder_moved || event.button_press != BUTTON_PRESS_NONE) {
		if (event.encoder_moved) {
			LOG_DBG("Encoder event: delta=%d", event.encoder_delta);
		}
		if (event.button_press != BUTTON_PRESS_NONE) {
			LOG_INF("Button event: %s",
				event.button_press == BUTTON_PRESS_SHORT ? "SHORT" : "LONG");
		}

		int ret = zbus_chan_pub(&input_event_ch, &event, K_MSEC(100));
		if (ret < 0) {
			LOG_WRN("Failed to publish input event: %d", ret);
		}
	}
}

int input_manager_init(void)
{
	LOG_INF("Initializing input manager...");

	atomic_set(&encoder_accumulator, 0);
	atomic_set(&encoder_delta, 0);
	atomic_set(&button_pressed, 0);
	atomic_set(&long_press_reported, 0);
	atomic_set(&input_enabled, 1);

	k_timer_init(&long_press_timer, long_press_timer_handler, NULL);
	k_work_init(&input_publish_work, input_publish_work_handler);

	LOG_INF("Input manager initialized");
	return 0;
}

void input_manager_enable(bool enable)
{
	atomic_set(&input_enabled, enable ? 1 : 0);

	if (!enable) {
		k_timer_stop(&long_press_timer);
	}
}