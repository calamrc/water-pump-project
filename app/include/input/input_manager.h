/**
 * @file input_manager.h
 * @brief Input manager for rotary encoder and button
 *
 * Handles input events from the EC11 rotary encoder and EC11 button.
 * Publishes events via Zbus input_event_ch.
 */

#ifndef INPUT_MANAGER_H_
#define INPUT_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Button press types */
enum button_press_type {
	BUTTON_PRESS_NONE = 0,
	BUTTON_PRESS_SHORT,
	BUTTON_PRESS_LONG,
	BUTTON_PRESS_HOLD,
};

/* Application input event structure */
struct ui_input_event {
	bool encoder_moved;
	int32_t encoder_delta;
	enum button_press_type button_press;
};

/**
 * @brief Initialize the input manager
 *
 * Sets up rotary encoder and button input handlers,
 * and initializes Zbus publishing.
 *
 * @return 0 on success, negative errno on failure
 */
int input_manager_init(void);

/**
 * @brief Enable or disable input processing
 *
 * @param enable true to enable, false to disable
 */
void input_manager_enable(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* INPUT_MANAGER_H_ */