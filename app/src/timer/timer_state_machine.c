/**
 * @file timer_state_machine.c
 * @brief Countdown timer state machine implementation
 *
 * Publishes state changes via Zbus and uses k_timer for 1-second ticks.
 */

#include "timer/timer_state_machine.h"
#include "zbus/messages.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(timer_sm, CONFIG_APP_LOG_LEVEL);

static enum timer_state current_state = TIMER_STATE_SETTING;

static uint32_t total_seconds = TIMER_DEFAULT_SECONDS;
static uint32_t remaining_seconds = TIMER_DEFAULT_SECONDS;

static struct k_timer timer_tick_timer;
static struct k_work timer_tick_work;

ZBUS_CHAN_DECLARE(timer_state_ch);

static void seconds_to_ms(uint32_t seconds, uint8_t *minutes, uint8_t *secs)
{
	*minutes = seconds / 60;
	*secs = seconds % 60;
}

static void timer_sm_publish_state(void)
{
	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = current_state,
		.remaining_seconds = timer_sm_get_remaining_seconds(),
		.total_seconds = total_seconds,
	};

	int ret = zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));
	if (ret < 0) {
		LOG_WRN("Failed to publish timer state: %d", ret);
	}
}

static void timer_tick_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	bool state_changed = timer_sm_update();

	if (state_changed || current_state == TIMER_STATE_RUNNING) {
		timer_sm_publish_state();
	}
}

static void timer_tick_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&timer_tick_work);
}

void timer_sm_init(void)
{
	LOG_INF("Initializing timer state machine");

	current_state = TIMER_STATE_SETTING;
	total_seconds = TIMER_DEFAULT_SECONDS;
	remaining_seconds = TIMER_DEFAULT_SECONDS;

	k_timer_init(&timer_tick_timer, timer_tick_timer_handler, NULL);
	k_work_init(&timer_tick_work, timer_tick_work_handler);

	LOG_INF("Timer initialized: %u seconds (%u:%02u)",
		total_seconds, total_seconds / 60, total_seconds % 60);

	timer_sm_publish_state();
}

void timer_sm_start_tick(void)
{
	k_timer_start(&timer_tick_timer, K_SECONDS(1), K_SECONDS(1));
	LOG_INF("Timer tick started");
}

void timer_sm_stop_tick(void)
{
	k_timer_stop(&timer_tick_timer);
	LOG_INF("Timer tick stopped");
}

enum timer_state timer_sm_get_state(void)
{
	return current_state;
}

void timer_sm_get_time(uint8_t *minutes, uint8_t *seconds)
{
	if (minutes == NULL || seconds == NULL) {
		return;
	}

	uint32_t secs_to_convert;

	switch (current_state) {
	case TIMER_STATE_SETTING:
		secs_to_convert = total_seconds;
		break;
	case TIMER_STATE_RUNNING:
	case TIMER_STATE_PAUSED:
		secs_to_convert = remaining_seconds;
		break;
	case TIMER_STATE_COMPLETED:
		secs_to_convert = 0;
		break;
	default:
		secs_to_convert = 0;
		break;
	}

	seconds_to_ms(secs_to_convert, minutes, seconds);
}

uint32_t timer_sm_get_remaining_seconds(void)
{
	switch (current_state) {
	case TIMER_STATE_SETTING:
		return total_seconds;
	case TIMER_STATE_RUNNING:
	case TIMER_STATE_PAUSED:
		return remaining_seconds;
	case TIMER_STATE_COMPLETED:
		return 0;
	default:
		return 0;
	}
}

void timer_sm_adjust_time(int32_t delta_seconds)
{
	uint32_t *target_seconds;

	if (current_state == TIMER_STATE_SETTING) {
		target_seconds = &total_seconds;
	} else if (current_state == TIMER_STATE_PAUSED) {
		target_seconds = &remaining_seconds;
	} else {
		return;
	}

	int64_t new_value = (int64_t)*target_seconds + delta_seconds;

	if (new_value < TIMER_MIN_SECONDS) {
		*target_seconds = TIMER_MIN_SECONDS;
	} else if (new_value > TIMER_MAX_SECONDS) {
		*target_seconds = TIMER_MAX_SECONDS;
	} else {
		*target_seconds = (uint32_t)new_value;
	}

	LOG_DBG("Timer adjusted: %u seconds", *target_seconds);

	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = current_state,
		.remaining_seconds = timer_sm_get_remaining_seconds(),
		.total_seconds = total_seconds,
	};

	zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));
}

int timer_sm_start(void)
{
	if (current_state != TIMER_STATE_SETTING) {
		LOG_WRN("Cannot start timer from state: %s",
			timer_sm_state_to_string(current_state));
		return -EINVAL;
	}

	enum timer_state prev_state = current_state;

	remaining_seconds = total_seconds;
	current_state = TIMER_STATE_RUNNING;

	LOG_INF("Timer started: %u seconds remaining", remaining_seconds);

	timer_sm_start_tick();

	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = prev_state,
		.remaining_seconds = remaining_seconds,
		.total_seconds = total_seconds,
	};

	zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));

	return 0;
}

int timer_sm_pause(void)
{
	if (current_state != TIMER_STATE_RUNNING) {
		LOG_WRN("Cannot pause timer from state: %s",
			timer_sm_state_to_string(current_state));
		return -EINVAL;
	}

	enum timer_state prev_state = current_state;

	current_state = TIMER_STATE_PAUSED;

	timer_sm_stop_tick();

	LOG_INF("Timer paused: %u seconds remaining", remaining_seconds);

	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = prev_state,
		.remaining_seconds = remaining_seconds,
		.total_seconds = total_seconds,
	};

	zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));

	return 0;
}

int timer_sm_resume(void)
{
	if (current_state != TIMER_STATE_PAUSED) {
		LOG_WRN("Cannot resume timer from state: %s",
			timer_sm_state_to_string(current_state));
		return -EINVAL;
	}

	enum timer_state prev_state = current_state;

	current_state = TIMER_STATE_RUNNING;

	timer_sm_start_tick();

	LOG_INF("Timer resumed: %u seconds remaining", remaining_seconds);

	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = prev_state,
		.remaining_seconds = remaining_seconds,
		.total_seconds = total_seconds,
	};

	zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));

	return 0;
}

void timer_sm_reset(void)
{
	enum timer_state prev_state = current_state;

	timer_sm_stop_tick();

	current_state = TIMER_STATE_SETTING;
	total_seconds = TIMER_DEFAULT_SECONDS;
	remaining_seconds = TIMER_DEFAULT_SECONDS;

	LOG_INF("Timer reset to default: %u seconds", total_seconds);

	struct timer_state_msg msg = {
		.new_state = current_state,
		.prev_state = prev_state,
		.remaining_seconds = remaining_seconds,
		.total_seconds = total_seconds,
	};

	zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));
}

bool timer_sm_update(void)
{
	if (current_state != TIMER_STATE_RUNNING) {
		return false;
	}

	if (remaining_seconds > 0) {
		remaining_seconds--;

		LOG_DBG("Timer tick: %u seconds remaining", remaining_seconds);

		if (remaining_seconds == 0) {
			enum timer_state prev_state = current_state;

			current_state = TIMER_STATE_COMPLETED;

			timer_sm_stop_tick();

			LOG_INF("Timer completed!");

			struct timer_state_msg msg = {
				.new_state = current_state,
				.prev_state = prev_state,
				.remaining_seconds = 0,
				.total_seconds = total_seconds,
			};

			zbus_chan_pub(&timer_state_ch, &msg, K_MSEC(100));

			return true;
		}
	}

	return false;
}

bool timer_sm_is_completed(void)
{
	return current_state == TIMER_STATE_COMPLETED;
}

bool timer_sm_is_active(void)
{
	return (current_state == TIMER_STATE_RUNNING) ||
	       (current_state == TIMER_STATE_PAUSED);
}

const char *timer_sm_state_to_string(enum timer_state state)
{
	switch (state) {
	case TIMER_STATE_SETTING:
		return "SETTING";
	case TIMER_STATE_RUNNING:
		return "RUNNING";
	case TIMER_STATE_PAUSED:
		return "PAUSED";
	case TIMER_STATE_COMPLETED:
		return "COMPLETED";
	default:
		return "UNKNOWN";
	}
}