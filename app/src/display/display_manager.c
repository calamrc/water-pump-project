/**
 * @file display_manager.c
 * @brief Display manager implementation for SH1106 OLED using Character Framebuffer
 */

#include "display/display_manager.h"

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(display_manager, CONFIG_APP_LOG_LEVEL);

/* Display device reference */
static const struct device *display_dev;

/* Display ready flag */
static bool display_ready = false;

/* Currently selected font index (large, for timer) */
static uint8_t selected_font_idx = 0;

/* Status bar font index (small, for status bar) */
static uint8_t status_bar_font_idx = 0;
static uint8_t status_bar_font_h = 0;
static uint8_t status_bar_font_w = 0;

int display_manager_init(void)
{
    LOG_INF("Initializing display manager with CFB...");

    /* Get display device reference */
    display_dev = DEVICE_DT_GET(DT_NODELABEL(sh1106));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("SH1106 display device not ready");
        return -ENODEV;
    }

    /* Initialize CFB framebuffer */
    int ret = cfb_framebuffer_init(display_dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize CFB framebuffer: %d", ret);
        return ret;
    }

    /* Get display parameters */
    uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
    uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);
    uint8_t ppt = cfb_get_display_parameter(display_dev, CFB_DISPLAY_PPT);
    
    LOG_INF("Display: %dx%d, Pixels per tile: %d", display_width, display_height, ppt);

    /* Find and select the largest available font */
    uint8_t num_fonts = cfb_get_numof_fonts(display_dev);
    LOG_INF("Available fonts: %d", num_fonts);
    
    uint8_t largest_font_idx = 0;
    uint8_t largest_font_size = 0;
    
    for (uint8_t i = 0; i < num_fonts; i++) {
        uint8_t font_width, font_height;
        cfb_get_font_size(display_dev, i, &font_width, &font_height);
        LOG_INF("Font %d: %dx%d", i, font_width, font_height);
        
        /* Select font with largest height */
        if (font_height > largest_font_size) {
            largest_font_size = font_height;
            largest_font_idx = i;
        }
    }
    
    selected_font_idx = largest_font_idx;
    LOG_INF("Using font %d (largest)", selected_font_idx);
    cfb_framebuffer_set_font(display_dev, selected_font_idx);

    /* Find and select the smallest available font for status bar */
    uint8_t smallest_font_idx = 0;
    uint8_t smallest_font_h = UINT8_MAX;
    uint8_t smallest_font_w = 0;
    
    for (uint8_t i = 0; i < num_fonts; i++) {
        uint8_t w, h;
        cfb_get_font_size(display_dev, i, &w, &h);
        if (h < smallest_font_h) {
            smallest_font_idx = i;
            smallest_font_w = w;
            smallest_font_h = h;
        }
    }
    
    status_bar_font_idx = smallest_font_idx;
    status_bar_font_w = smallest_font_w;
    status_bar_font_h = smallest_font_h;
    LOG_INF("Status bar font %d (%dx%d)", status_bar_font_idx,
            status_bar_font_w, status_bar_font_h);

    /* Clear display */
    cfb_framebuffer_clear(display_dev, true);

    display_ready = true;
    LOG_INF("Display manager initialized successfully");
    
    return 0;
}

bool display_manager_is_ready(void)
{
    return display_ready;
}

void display_manager_clear(void)
{
    if (!display_ready) {
        return;
    }

    cfb_framebuffer_clear(display_dev, false);
}

void display_manager_show_splash(uint8_t line1_chars, uint8_t line2_chars, bool inverted)
{
	if (!display_ready) {
		return;
	}

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
	uint8_t font_width, font_height;
	cfb_get_font_size(display_dev, selected_font_idx, &font_width, &font_height);

	static const char line1[] = "WATER";
	static const char line2[] = "PUMP";

	cfb_framebuffer_clear(display_dev, false);

	if (line1_chars > 0) {
		uint8_t n = line1_chars > 5 ? 5 : line1_chars;
		char buf[6] = {0};
		memcpy(buf, line1, n);
		uint16_t x = (display_width - n * font_width) / 2;
		cfb_print(display_dev, buf, x, 0);
	}

	if (line2_chars > 0) {
		uint8_t n = line2_chars > 4 ? 4 : line2_chars;
		char buf[5] = {0};
		memcpy(buf, line2, n);
		uint16_t x = (display_width - n * font_width) / 2;
		cfb_print(display_dev, buf, x, font_height);
	}

	if (inverted) {
		cfb_invert_area(display_dev, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
	}
}

void display_manager_show_time(uint8_t minutes, uint8_t seconds, bool flash)
{
    if (!display_ready) {
        return;
    }

    /* Format time string "MM:SS" */
    char time_str[8];
    snprintf(time_str, sizeof(time_str), "%02u:%02u", minutes, seconds);

    /* Get display dimensions */
    uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
    uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);
    
    /* Get current font size using the SELECTED font index */
    uint8_t font_width, font_height;
    cfb_get_font_size(display_dev, selected_font_idx, &font_width, &font_height);
    
    /* Calculate text dimensions (5 characters: MM:SS) */
    uint16_t text_width = 5 * font_width;
    uint16_t text_height = font_height;
    
    /* Calculate centered position (above status bar area) */
    uint16_t available_height = display_height - status_bar_font_h;
    uint16_t x = (display_width - text_width) / 2;
    uint16_t y = (available_height - text_height) / 2;

    /* Clear framebuffer to appropriate background */
    cfb_framebuffer_clear(display_dev, flash);

    /* Print the time string at calculated position (text will be contrasting) */
    cfb_print(display_dev, time_str, x, y);

    LOG_DBG("Display: %s at (%d, %d) font=%d flash=%d bg=%s", 
            time_str, x, y, selected_font_idx, flash, flash ? "WHITE" : "BLACK");
}

void display_manager_show_dialog(const char *title, const char *opt_left,
				 const char *opt_right, bool left_selected)
{
	if (!display_ready) {
		return;
	}

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
	uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);
	uint8_t num_fonts = cfb_get_numof_fonts(display_dev);

	uint8_t dialog_font_idx = 0;
	uint8_t dialog_font_h = UINT8_MAX;
	uint8_t dialog_font_w = 0;

	for (uint8_t i = 0; i < num_fonts; i++) {
		uint8_t w, h;
		cfb_get_font_size(display_dev, i, &w, &h);
		if (h < dialog_font_h) {
			dialog_font_idx = i;
			dialog_font_w = w;
			dialog_font_h = h;
		}
	}

	cfb_framebuffer_set_font(display_dev, dialog_font_idx);
	cfb_framebuffer_clear(display_dev, false);

	uint8_t title_len = strlen(title);
	uint16_t title_x = (display_width - title_len * dialog_font_w) / 2;
	uint16_t title_y = (display_height / 2 - dialog_font_h) / 2;
	cfb_print(display_dev, title, title_x, title_y);

	uint8_t left_len = strlen(opt_left);
	uint8_t right_len = strlen(opt_right);
	uint8_t gap = 2;
	uint16_t opts_width = (left_len + right_len + gap) * dialog_font_w;
	uint16_t opts_x = (display_width - opts_width) / 2;
	uint16_t opts_y = display_height / 2 + (display_height / 2 - dialog_font_h) / 2;

	cfb_print(display_dev, opt_left, opts_x, opts_y);
	cfb_print(display_dev, opt_right,
		  opts_x + (left_len + gap) * dialog_font_w, opts_y);

	if (left_selected) {
		cfb_invert_area(display_dev, opts_x, opts_y,
				left_len * dialog_font_w, dialog_font_h);
	} else {
		uint16_t right_x = opts_x + (left_len + gap) * dialog_font_w;
		cfb_invert_area(display_dev, right_x, opts_y,
				right_len * dialog_font_w, dialog_font_h);
	}

	cfb_framebuffer_set_font(display_dev, selected_font_idx);
}

void display_manager_show_status_bar(fixed_t flow_rate, bool pump_on,
				      int64_t uptime_s, bool data_valid)
{
	if (!display_ready) {
		return;
	}

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
	uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);

	cfb_framebuffer_set_font(display_dev, status_bar_font_idx);

	char status_buf[32];

	if (pump_on) {
		int32_t uptime_mins = (int32_t)(uptime_s / 60);
		int32_t uptime_secs = (int32_t)(uptime_s % 60);

		if (data_valid && flow_rate >= 0) {
			int32_t int_part = flow_rate >> 16;
			uint32_t frac_raw = (uint32_t)(flow_rate & 0xFFFF);
			uint32_t frac_100 = (frac_raw * 100 + 32768) >> 16;
			if (frac_100 >= 100) {
				int_part += 1;
				frac_100 -= 100;
			}
			snprintf(status_buf, sizeof(status_buf), ">%ld.%02ld %02ld:%02ld",
				 (long)int_part, (long)frac_100,
				 (long)uptime_mins, (long)uptime_secs);
		} else {
			snprintf(status_buf, sizeof(status_buf), ">--.- %02ld:%02ld",
				 (long)uptime_mins, (long)uptime_secs);
		}
	} else {
		strcpy(status_buf, "OFF");
	}

	uint16_t text_width = strlen(status_buf) * status_bar_font_w;
	uint16_t x = (display_width - text_width) / 2;
	uint16_t y = display_height - status_bar_font_h;

	cfb_print(display_dev, status_buf, x, y);

	cfb_framebuffer_set_font(display_dev, selected_font_idx);
}

void display_manager_update(void)
{
    if (!display_ready) {
        return;
    }

    cfb_framebuffer_finalize(display_dev);
}
