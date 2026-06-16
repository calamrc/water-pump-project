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

void display_manager_show_time_header(uint8_t minutes, uint8_t seconds, bool flash)
{
	if (!display_ready) {
		return;
	}

	char time_str[8];
	snprintf(time_str, sizeof(time_str), "%02u:%02u", minutes, seconds);

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);

	cfb_framebuffer_set_font(display_dev, status_bar_font_idx);

	uint8_t font_width, font_height;
	cfb_get_font_size(display_dev, status_bar_font_idx, &font_width, &font_height);

	uint16_t text_width = 5 * font_width;
	uint16_t x = (display_width - text_width) / 2;

	cfb_print(display_dev, time_str, x, 0);

	if (flash) {
		cfb_invert_area(display_dev, 0, 0, display_width, font_height);
	}
}

void display_manager_show_flow_plot(const fixed_t *history, int count, int start_idx)
{
	if (!display_ready || count < 1) {
		return;
	}

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
	uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);

	int16_t header_h = status_bar_font_h + 1;
	int16_t footer_h = status_bar_font_h;
	int16_t plot_y = header_h;
	int16_t plot_h = display_height - header_h - footer_h;
	int16_t plot_x = 0;
	int16_t plot_w = display_width;

	fixed_t flow_min = FIXED_MAX;
	fixed_t flow_max = 0;
	int valid_count = 0;

	for (int i = 0; i < count; i++) {
		int idx = (start_idx + i) % FLOW_HISTORY_SIZE;
		fixed_t val = history[idx];
		if (val == FIXED_MIN) {
			continue;
		}
		valid_count++;
		if (val < flow_min) {
			flow_min = val;
		}
		if (val > flow_max) {
			flow_max = val;
		}
	}

	if (valid_count == 0) {
		return;
	}

	fixed_t range = fixed_sub(flow_max, flow_min);
	fixed_t min_range = fixed_from_int(1);
	if (range < min_range) {
		range = min_range;
	}
	fixed_t padding = fixed_div_int(range, 10);
	flow_min = fixed_sub(flow_min, padding);
	flow_max = fixed_add(flow_max, padding);

	if (flow_min < 0) {
		flow_max = fixed_sub(flow_max, flow_min);
		flow_min = 0;
	}
	range = fixed_sub(flow_max, flow_min);
	if (range == 0) {
		range = FIXED_ONE;
	}

	struct cfb_position prev;
	bool prev_valid = false;

	for (int i = 0; i < count; i++) {
		int idx = (start_idx + i) % FLOW_HISTORY_SIZE;
		fixed_t val = history[idx];

		if (val == FIXED_MIN) {
			prev_valid = false;
			continue;
		}

		int16_t x;
		if (valid_count == 1) {
			x = plot_x + plot_w / 2;
		} else {
			x = plot_x + plot_w - count + i;
		}

		fixed_t normalized = fixed_div(fixed_sub(val, flow_min), range);
		int32_t y_offset = fixed_to_int(fixed_mul_int(normalized, plot_h - 1));
		int16_t y = plot_y + plot_h - 1 - y_offset;

		if (y < plot_y) {
			y = plot_y;
		}
		if (y >= plot_y + plot_h) {
			y = plot_y + plot_h - 1;
		}

		struct cfb_position pos = {x, y};

		if (prev_valid) {
			cfb_draw_line(display_dev, &prev, &pos);
		} else if (count == 1) {
			cfb_draw_point(display_dev, &pos);
		}

		prev = pos;
		prev_valid = true;
	}
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
				      int64_t uptime_s)
{
	if (!display_ready) {
		return;
	}

	uint16_t display_width = cfb_get_display_parameter(display_dev, CFB_DISPLAY_WIDTH);
	uint16_t display_height = cfb_get_display_parameter(display_dev, CFB_DISPLAY_HEIGHT);

	cfb_framebuffer_set_font(display_dev, status_bar_font_idx);

	char flow_buf[16];
	char uptime_buf[16];

	int32_t int_part = flow_rate >> 16;
	uint32_t frac_raw = (uint32_t)(flow_rate & 0xFFFF);
	uint32_t frac_100 = (frac_raw * 100 + 32768) >> 16;
	if (frac_100 >= 100) {
		int_part += 1;
		frac_100 -= 100;
	}
	snprintf(flow_buf, sizeof(flow_buf), "%ld.%02ld",
		 (long)int_part, (long)frac_100);

	if (pump_on) {
		int32_t uptime_mins = (int32_t)(uptime_s / 60);
		int32_t uptime_secs = (int32_t)(uptime_s % 60);
		snprintf(uptime_buf, sizeof(uptime_buf), "%02ld:%02ld",
			 (long)uptime_mins, (long)uptime_secs);
	} else {
		strcpy(uptime_buf, "OFF");
	}

	uint16_t col_width = display_width / 2;
	uint16_t y = display_height - status_bar_font_h;

	uint16_t flow_width = strlen(flow_buf) * status_bar_font_w;
	uint16_t flow_x = (col_width - flow_width) / 2;
	cfb_print(display_dev, flow_buf, flow_x, y);

	uint16_t uptime_width = strlen(uptime_buf) * status_bar_font_w;
	uint16_t uptime_x = col_width + (col_width - uptime_width) / 2;
	cfb_print(display_dev, uptime_buf, uptime_x, y);

	cfb_framebuffer_set_font(display_dev, selected_font_idx);
}

void display_manager_update(void)
{
    if (!display_ready) {
        return;
    }

    cfb_framebuffer_finalize(display_dev);
}
