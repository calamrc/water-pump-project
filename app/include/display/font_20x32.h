/**
 * @file font_20x32.h
 * @brief 20x32 pixel font for large timer display
 *
 * Only contains digits 0-9 and colon for MM:SS format
 * Each character is 20 pixels wide, 32 pixels tall
 * Total text width for "MM:SS" = 4*20 + 12 = 92 pixels
 */

#ifndef FONT_20X32_H_
#define FONT_20X32_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FONT_20X32_WIDTH   20
#define FONT_20X32_HEIGHT  32
#define FONT_20X32_COLON_WIDTH 12

/* Character indices */
#define FONT_CHAR_0     0
#define FONT_CHAR_1     1
#define FONT_CHAR_2     2
#define FONT_CHAR_3     3
#define FONT_CHAR_4     4
#define FONT_CHAR_5     5
#define FONT_CHAR_6     6
#define FONT_CHAR_7     7
#define FONT_CHAR_8     8
#define FONT_CHAR_9     9
#define FONT_CHAR_COLON 10

/* Font glyph structure */
struct font_glyph {
    uint8_t width;
    uint8_t height;
    const uint8_t *bitmap;
};

/* Get glyph for a character ('0'-'9' or ':') */
const struct font_glyph *font_20x32_get_glyph(char c);

/* Get text width for string */
uint16_t font_20x32_get_text_width(const char *text);

/* Get font height */
static inline uint8_t font_20x32_get_height(void) {
    return FONT_20X32_HEIGHT;
}

#ifdef __cplusplus
}
#endif

#endif /* FONT_20X32_H_ */
