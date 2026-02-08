/**
 * @file input_manager.h
 * @brief Input manager for rotary encoder and button
 *
 * Handles input events from the EC11 rotary encoder and EC11 button.
 * Provides debouncing and long-press detection for the button.
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
    BUTTON_PRESS_SHORT,    /* Short press (< 1 second) */
    BUTTON_PRESS_LONG,     /* Long press (>= 1 second) */
};

/* Application input event structure (renamed to avoid conflict with Zephyr) */
struct ui_input_event {
    bool encoder_moved;           /* True if encoder was rotated */
    int32_t encoder_delta;        /* +1 for CW, -1 for CCW */
    enum button_press_type button_press;
};

/**
 * @brief Initialize the input manager
 *
 * Sets up rotary encoder and button input handlers
 *
 * @return 0 on success, negative errno on failure
 */
int input_manager_init(void);

/**
 * @brief Get pending input events
 *
 * Non-blocking check for input events. Should be called periodically
 * from the UI thread.
 *
 * @param event Pointer to event structure to fill
 * @return true if an event was pending, false otherwise
 */
bool input_manager_get_event(struct ui_input_event *event);

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
