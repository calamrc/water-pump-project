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
#include <fixed_math.h>

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

#define FLOW_HISTORY_SIZE 128

/**
 * @brief Display time in MM:SS format in the header row
 *
 * @param minutes Minutes value (0-99)
 * @param seconds Seconds value (0-59)
 * @param flash If true, header area will be inverted for flash effect
 */
void display_manager_show_time_header(uint8_t minutes, uint8_t seconds, bool flash);

/**
 * @brief Display flow rate line graph in the plot area
 *
 * Draws a scrolling line chart of flow rate history between
 * the header and footer rows. Auto-scales Y axis to fit data.
 *
 * @param history Ring buffer of fixed_t flow rate samples (FLOW_HISTORY_SIZE elements)
 * @param count Number of valid samples in the buffer (0 to FLOW_HISTORY_SIZE)
 * @param start_idx Index of the oldest sample in the ring buffer
 */
void display_manager_show_flow_plot(const fixed_t *history, int count, int start_idx);

/**
 * @brief Display status bar at bottom of screen
 *
 * Shows flow rate and motor uptime in two columns.
 * Left column (centered): flow rate ("X.XX")
 * Right column (centered): uptime ("MM:SS") when pump on, "OFF" when pump off
 *
 * @param flow_rate Current flow rate in L/min (Q16.16 fixed-point)
 * @param pump_on true if pump is running
 * @param uptime_s Pump session uptime in seconds (0 if pump off)
 */
void display_manager_show_status_bar(fixed_t flow_rate, bool pump_on,
				      int64_t uptime_s);

/**
 * @brief Show a two-option confirmation dialog
 *
 * Displays a title on the first row and two options on the second row.
 * The selected option is inverted (highlighted).
 *
 * @param title Title text for the first row
 * @param opt_left Left option text (e.g. "YES")
 * @param opt_right Right option text (e.g. "NO")
 * @param left_selected true to highlight left option, false for right
 */
void display_manager_show_dialog(const char *title, const char *opt_left,
				 const char *opt_right, bool left_selected);

/**
 * @brief Update the display (flush framebuffer to display)
 */
void display_manager_update(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_MANAGER_H_ */
