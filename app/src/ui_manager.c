/**
 * @file ui_manager.c
 * @brief UI Manager implementation
 *
 * Event-driven UI thread that subscribes to input and timer state channels
 * via Zbus msg_subscriber, and publishes feedback commands.
 */

#include "ui_manager.h"
#include "display/display_manager.h"
#include "input/input_manager.h"
#include "timer/timer_state_machine.h"
#include "thread_comm.h"
#include "zbus/messages.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(ui_manager, CONFIG_APP_LOG_LEVEL);

#define FLASH_INTERVAL_MS 500
#define HEALTH_UPDATE_INTERVAL_MS 1000
#define SPLASH_TYPEWRITER_CHAR_MS 50
#define SPLASH_LINE_GAP_MS 80
#define SPLASH_FINAL_HOLD_MS 200

ZBUS_CHAN_DECLARE(input_event_ch);
ZBUS_CHAN_DECLARE(timer_state_ch);
ZBUS_CHAN_DECLARE(feedback_cmd_ch);

extern const struct zbus_observer ui_event_sub;

static bool in_restart_confirm;
static bool restart_confirm_yes = true;

static void publish_feedback(enum feedback_action action, uint32_t duration_ms)
{
	struct feedback_cmd_msg msg = {
		.action = action,
		.duration_ms = duration_ms,
	};

	int ret = zbus_chan_pub(&feedback_cmd_ch, &msg, K_MSEC(100));
	if (ret < 0) {
		LOG_WRN("Failed to publish feedback cmd: %d", ret);
	}
}

static void handle_input_event(const struct ui_input_event *event)
{
	if (in_restart_confirm) {
		if (event->encoder_moved && event->encoder_delta != 0) {
			if (event->encoder_delta > 0) {
				restart_confirm_yes = false;
			} else {
				restart_confirm_yes = true;
			}
			display_manager_show_dialog("Restart?", "Yes", "No", restart_confirm_yes);
			display_manager_update();
			publish_feedback(FEEDBACK_ACTION_CLICK, 0);
		}

		if (event->button_press == BUTTON_PRESS_SHORT) {
			if (restart_confirm_yes) {
				LOG_INF("Reboot confirmed by user");
				sys_reboot(SYS_REBOOT_COLD);
			} else {
				LOG_INF("Reboot cancelled by user");
				in_restart_confirm = false;
				uint8_t minutes, seconds;
				timer_sm_get_time(&minutes, &seconds);
				display_manager_show_time(minutes, seconds, false);
				display_manager_update();
			}
		}
		return;
	}

	if (event->button_press == BUTTON_PRESS_HOLD) {
		LOG_INF("Hold detected - entering restart confirmation");
		in_restart_confirm = true;
		restart_confirm_yes = true;
		input_manager_enable(false);
		display_manager_show_dialog("Restart?", "Yes", "No", true);
		display_manager_update();
		publish_feedback(FEEDBACK_ACTION_CLICK, 0);
		input_manager_enable(true);
		return;
	}

	enum timer_state current_state = timer_sm_get_state();

	if (event->encoder_moved && event->encoder_delta != 0) {
		int32_t delta = event->encoder_delta * TIMER_STEP_SECONDS;
		LOG_INF("Encoder: %d steps -> %d sec", event->encoder_delta, delta);
		timer_sm_adjust_time(delta);
	}

	if (event->button_press != BUTTON_PRESS_NONE) {
		publish_feedback(FEEDBACK_ACTION_CLICK, 0);

		current_state = timer_sm_get_state();

		LOG_INF("Button: %s (state=%s)",
			event->button_press == BUTTON_PRESS_SHORT ? "SHORT" :
			event->button_press == BUTTON_PRESS_LONG ? "LONG" : "HOLD",
			timer_sm_state_to_string(current_state));

		switch (current_state) {
		case TIMER_STATE_SETTING:
			if (event->button_press == BUTTON_PRESS_SHORT) {
				LOG_INF("Action: START");
				timer_sm_start();
			} else if (event->button_press == BUTTON_PRESS_LONG) {
				LOG_INF("Action: RESET");
				timer_sm_reset();
			}
			break;

		case TIMER_STATE_RUNNING:
			if (event->button_press == BUTTON_PRESS_SHORT) {
				LOG_INF("Action: PAUSE");
				timer_sm_pause();
			} else if (event->button_press == BUTTON_PRESS_LONG) {
				LOG_INF("Action: STOP");
				timer_sm_reset();
			}
			break;

		case TIMER_STATE_PAUSED:
			if (event->button_press == BUTTON_PRESS_SHORT) {
				LOG_INF("Action: RESUME");
				timer_sm_resume();
			} else if (event->button_press == BUTTON_PRESS_LONG) {
				LOG_INF("Action: STOP");
				timer_sm_reset();
			}
			break;

		case TIMER_STATE_COMPLETED:
			LOG_INF("Action: RESET from complete");
			timer_sm_reset();
			break;

		default:
			break;
		}
	}
}

static void handle_timer_state(const struct timer_state_msg *msg)
{
	if (in_restart_confirm) {
		return;
	}

	uint8_t minutes, seconds;
	timer_sm_get_time(&minutes, &seconds);

	LOG_INF("Timer state: %s -> %s, remaining=%u",
		timer_sm_state_to_string(msg->prev_state),
		timer_sm_state_to_string(msg->new_state),
		msg->remaining_seconds);

	switch (msg->new_state) {
	case TIMER_STATE_SETTING:
		display_manager_show_time(minutes, seconds, false);
		display_manager_update();
		break;

	case TIMER_STATE_RUNNING:
		display_manager_show_time(minutes, seconds, false);
		display_manager_update();
		if (msg->remaining_seconds <= 10 && msg->remaining_seconds > 0) {
			publish_feedback(FEEDBACK_ACTION_PULSE, 500);
		}
		break;

	case TIMER_STATE_PAUSED:
		display_manager_show_time(minutes, seconds, false);
		display_manager_update();
		break;

	case TIMER_STATE_COMPLETED:
		display_manager_show_time(0, 0, true);
		display_manager_update();
		publish_feedback(FEEDBACK_ACTION_PULSE, 300);
		break;
	}
}

int ui_manager_init(void)
{
	LOG_INF("Initializing UI manager...");

	int ret = display_manager_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize display manager: %d", ret);
		return ret;
	}

	ret = input_manager_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize input manager: %d", ret);
		return ret;
	}

	timer_sm_init();

	LOG_INF("UI manager initialized successfully");
	return 0;
}

void ui_manager_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("========================================");
	LOG_INF("UI THREAD STARTED - EVENT DRIVEN ZBUS");
	LOG_INF("========================================");

	display_manager_show_splash(0, 0, true);
	display_manager_update();
	k_sleep(K_MSEC(SPLASH_TYPEWRITER_CHAR_MS));

	publish_feedback(FEEDBACK_ACTION_CLICK, 0);

	for (uint8_t i = 1; i <= 9; i++) {
		uint8_t line1 = i <= 5 ? i : 5;
		uint8_t line2 = i > 5 ? i - 5 : 0;
		display_manager_show_splash(line1, line2, true);
		display_manager_update();

		if (i == 6) {
			publish_feedback(FEEDBACK_ACTION_CLICK, 0);
		}

		if (i == 5) {
			k_sleep(K_MSEC(SPLASH_LINE_GAP_MS));
		} else {
			k_sleep(K_MSEC(SPLASH_TYPEWRITER_CHAR_MS));
		}
	}

	k_sleep(K_MSEC(SPLASH_FINAL_HOLD_MS));

	display_manager_clear();
	display_manager_update();

	display_manager_show_splash(5, 4, false);
	display_manager_update();
	k_sleep(K_MSEC(SPLASH_FINAL_HOLD_MS));

	int64_t last_health_update = 0;
	int64_t last_flash_toggle = 0;
	bool flash_state = false;
	enum timer_state current_state = TIMER_STATE_SETTING;

	uint8_t minutes = 1, seconds = 0;
	timer_sm_get_time(&minutes, &seconds);
	display_manager_show_time(minutes, seconds, false);
	display_manager_update();

	LOG_INF("UI ready - Timer at %02u:%02u", minutes, seconds);

	while (true) {
		const struct zbus_channel *chan;
		uint8_t msg_buf[sizeof(struct timer_state_msg)];
		int ret;

		ret = zbus_sub_wait_msg(&ui_event_sub, &chan, msg_buf,
					K_MSEC(100));

		if (ret == 0) {
			if (chan == &input_event_ch) {
				struct ui_input_event input_evt;
				memcpy(&input_evt, msg_buf, sizeof(input_evt));
				handle_input_event(&input_evt);
			} else if (chan == &timer_state_ch) {
				struct timer_state_msg timer_evt;
				memcpy(&timer_evt, msg_buf, sizeof(timer_evt));
				current_state = timer_evt.new_state;
				handle_timer_state(&timer_evt);
			}
		}

		int64_t now = k_uptime_get();

		if (current_state == TIMER_STATE_COMPLETED && !in_restart_confirm) {
			if ((now - last_flash_toggle) >= FLASH_INTERVAL_MS) {
				last_flash_toggle = now;
				flash_state = !flash_state;
				timer_sm_get_time(&minutes, &seconds);
				display_manager_show_time(minutes, seconds,
							  flash_state);
				display_manager_update();
			}
		} else {
			flash_state = false;
		}

		if ((now - last_health_update) >= HEALTH_UPDATE_INTERVAL_MS) {
			last_health_update = now;
			LOG_DBG("Updating thread health...");
			thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
		}
	}
}