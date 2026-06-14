#include "zbus/messages.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include <app/drivers/feedback_relay.h>
#include <app/drivers/yf_s201c.h>

LOG_MODULE_REGISTER(zbus_channels, CONFIG_APP_LOG_LEVEL);

ZBUS_CHAN_DEFINE(input_event_ch,
		 struct ui_input_event,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(ui_event_sub),
		 ZBUS_MSG_INIT(.encoder_moved = false,
			       .encoder_delta = 0,
			       .button_press = BUTTON_PRESS_NONE));

ZBUS_CHAN_DEFINE(timer_state_ch,
		 struct timer_state_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(ui_event_sub),
		 ZBUS_MSG_INIT(.new_state = TIMER_STATE_SETTING,
			       .prev_state = TIMER_STATE_SETTING,
			       .remaining_seconds = TIMER_DEFAULT_SECONDS,
			       .total_seconds = TIMER_DEFAULT_SECONDS));

ZBUS_CHAN_DEFINE(feedback_cmd_ch,
		 struct feedback_cmd_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(feedback_relay_listener),
		 ZBUS_MSG_INIT(.action = FEEDBACK_ACTION_OFF, .duration_ms = 0));

ZBUS_CHAN_DEFINE(sensor_data_ch,
		 struct sensor_data_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(flow_event_sub, pump_event_sub),
		 ZBUS_MSG_INIT(.flow_rate = 0, .period_us = 0, .timestamp = 0,
			       .data_valid = false, .sequence_number = 0));

ZBUS_CHAN_DEFINE(plateau_detected_ch,
		 struct plateau_detected_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(pump_event_sub),
		 ZBUS_MSG_INIT(.flow_rate = 0, .noise_std = 0, .flow_slope = 0,
			       .period_us = 0));

ZBUS_CHAN_DEFINE(pump_state_ch,
		 struct pump_state_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(flow_event_sub, safety_event_sub),
		 ZBUS_MSG_INIT(.change = PUMP_TURNED_OFF, .plateau_period_us = 0));

ZBUS_CHAN_DEFINE(sensor_cmd_ch,
		 struct sensor_cmd_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(sensor_cmd_listener),
		 ZBUS_MSG_INIT(.cmd = SENSOR_CMD_RESET));

ZBUS_MSG_SUBSCRIBER_DEFINE(ui_event_sub);
ZBUS_MSG_SUBSCRIBER_DEFINE(flow_event_sub);
ZBUS_MSG_SUBSCRIBER_DEFINE(pump_event_sub);
ZBUS_MSG_SUBSCRIBER_DEFINE(safety_event_sub);

static void feedback_relay_msg_handler(const struct zbus_channel *chan)
{
	const struct feedback_cmd_msg *msg = zbus_chan_const_msg(chan);
	const struct device *feedback =
		FEEDBACK_RELAY_DT_GET(DT_NODELABEL(feedback_relay));

	if (!device_is_ready(feedback)) {
		LOG_WRN("Feedback relay device not ready");
		return;
	}

	switch (msg->action) {
	case FEEDBACK_ACTION_PULSE:
		feedback_relay_pulse(feedback, msg->duration_ms);
		break;
	case FEEDBACK_ACTION_CLICK:
		feedback_relay_click(feedback);
		break;
	case FEEDBACK_ACTION_OFF:
		feedback_relay_off(feedback);
		break;
	}
}

ZBUS_LISTENER_DEFINE(feedback_relay_listener, feedback_relay_msg_handler);

static void sensor_cmd_handler(const struct zbus_channel *chan)
{
	const struct sensor_cmd_msg *msg = zbus_chan_const_msg(chan);
	const struct device *flow_sensor = DEVICE_DT_GET(DT_NODELABEL(flow_sensor));

	if (!device_is_ready(flow_sensor)) {
		LOG_WRN("Flow sensor device not ready for command");
		return;
	}

	switch (msg->cmd) {
	case SENSOR_CMD_RESET:
		LOG_INF("Resetting flow sensor via Zbus command");
		yf_s201c_reset(flow_sensor);
		break;
	}
}

ZBUS_LISTENER_DEFINE(sensor_cmd_listener, sensor_cmd_handler);