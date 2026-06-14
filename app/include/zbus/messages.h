#ifndef ZBUS_MESSAGES_H_
#define ZBUS_MESSAGES_H_

#include <stdint.h>
#include <stdbool.h>
#include "timer/timer_state_machine.h"
#include "input/input_manager.h"

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

#endif /* ZBUS_MESSAGES_H_ */