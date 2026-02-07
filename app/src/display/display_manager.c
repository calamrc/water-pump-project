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

LOG_MODULE_REGISTER(display_manager, CONFIG_APP_LOG_LEVEL);

/* Display device reference */
static const struct device *display_dev;

/* Display ready flag */
static bool display_ready = false;

/* Currently selected font index */
static uint8_t selected_font_idx = 0;

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

void display_manager_show_time(uint8_t minutes, uint8_t seconds, bool flash)
{
    if (!display_ready) {
        return;
    }

    /* Format time string "MM:SS" */
    char time_str[6];
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
    
    /* Calculate centered position */
    uint16_t x = (display_width - text_width) / 2;
    uint16_t y = (display_height - text_height) / 2;

    /* Clear framebuffer to appropriate background */
    cfb_framebuffer_clear(display_dev, flash);

    /* Print the time string at calculated position (text will be contrasting) */
    cfb_print(display_dev, time_str, x, y);

    LOG_DBG("Display: %s at (%d, %d) font=%d flash=%d bg=%s", 
            time_str, x, y, selected_font_idx, flash, flash ? "WHITE" : "BLACK");
}

void display_manager_update(void)
{
    if (!display_ready) {
        return;
    }

    cfb_framebuffer_finalize(display_dev);
}
