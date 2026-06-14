/**
 * @file display_manager.h
 * @brief Display manager for SH1106 OLED using Character Framebuffer
 *
 * Manages the SH1106 display initialization and rendering using CFB.
 * Provides a simple interface for displaying the countdown timer.
 */

#ifndef DISPLAY_MANAGER_H_
#define DISPLAY_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Display dimensions */
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

/**
 * @brief Initialize the display manager and SH1106 display
 *
 * @return 0 on success, negative errno on failure
 */
int display_manager_init(void);

/**
 * @brief Check if display is ready
 *
 * @return true if display is initialized and ready
 */
bool display_manager_is_ready(void);

/**
 * @brief Clear the entire display
 */
void display_manager_clear(void);

/**
 * @brief Display partial splash text for animation
 *
 * Renders up to @p line1_chars characters of "WATER" and
 * @p line2_chars characters of "PUMP", centered on the display.
 * Use to animate the splash by incrementing char counts over time.
 *
 * @param line1_chars Number of characters to show from "WATER" (0-5)
 * @param line2_chars Number of characters to show from "PUMP" (0-4)
 * @param inverted If true, inverts the entire display
 */
void display_manager_show_splash(uint8_t line1_chars, uint8_t line2_chars, bool inverted);

/**
 * @brief Display time in MM:SS format centered on screen
 *
 * @param minutes Minutes value (0-99)
 * @param seconds Seconds value (0-59)
 * @param flash If true, display will be inverted/flashing
 */
void display_manager_show_time(uint8_t minutes, uint8_t seconds, bool flash);

/**
 * @brief Update the display (flush framebuffer to display)
 */
void display_manager_update(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_MANAGER_H_ */
