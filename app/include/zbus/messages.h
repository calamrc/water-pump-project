#ifndef ZBUS_MESSAGES_H_
#define ZBUS_MESSAGES_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include "timer/timer_state_machine.h"
#include "input/input_manager.h"
#include "fixed_math.h"

struct timer_state_msg {
	enum timer_state new_state;
	enum timer_state prev_state;
	uint32_t remaining_seconds;
	uint32_t total_seconds;
};

enum feedback_action {
	FEEDBACK_ACTION_PULSE,
	FEEDBACK_ACTION_CLICK,
	FEEDBACK_ACTION_OFF,
};

struct feedback_cmd_msg {
	enum feedback_action action;
	uint32_t duration_ms;
};

struct sensor_data_msg {
	fixed_t flow_rate;
	int64_t period_us;
	int64_t timestamp;
	bool data_valid;
	uint32_t sequence_number;
};

struct plateau_detected_msg {
	fixed_t flow_rate;
	fixed_t noise_std;
	fixed_t flow_slope;
	int64_t period_us;
};

enum pump_state_change {
	PUMP_TURNED_ON,
	PUMP_TURNED_OFF,
	PUMP_EMERGENCY_STOP,
};

struct pump_state_msg {
	enum pump_state_change change;
	int64_t plateau_period_us;
};

enum sensor_cmd {
	SENSOR_CMD_RESET,
};

struct sensor_cmd_msg {
	enum sensor_cmd cmd;
};

#endif /* ZBUS_MESSAGES_H_ */