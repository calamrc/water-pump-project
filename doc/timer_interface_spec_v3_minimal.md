# Minimal Countdown Timer Specification v3.0

## Zephyr Water Pump Project - Ultra-Simple Timer Interface
**Date:** 2026-02-08  
**Target:** ESP32-DevKitC / Zephyr 4.2.x / SH1106 128x64  
**Status:** Implementation-Ready Specification

---

## 1. Design Philosophy: Maximum Simplicity

**Only ONE thing on screen: the timer.**

No status bars. No progress bars. No hints. No icons. No auxiliary text.
Just large, centered, readable time.

---

## 2. Font Size Calculation & Recommendation

### Display Constraints
- **Resolution:** 128 × 64 pixels
- **Content:** "MM:SS" (5 characters)
- **Requirement:** Maximum readable size, perfectly centered

### Font Size Analysis

| Font Size | Char Width | 5-Char Width | Height | Screen Usage | Margins (H/V) | Verdict |
|-----------|-----------|--------------|--------|--------------|---------------|---------|
| 16×24 | 16px | 80px | 24px | 63% width | 24px / 20px | Too small |
| **20×32** | **20px** | **100px** | **32px** | **78% width** | **14px / 16px** | **RECOMMENDED** |
| 24×32 | 24px | 120px | 32px | 94% width | 4px / 16px | Too tight |

### Recommended: 20×32 Font with Variable-Width Colon

**Calculation:**
- Digits (0-9): 20px wide × 4 = 80px
- Colon (:): 12px wide × 1 = 12px (colon is narrower in most fonts)
- **Total: 92px width**
- **Height: 32px**

**Centering:**
```
Horizontal: (128 - 92) / 2 = 18 pixels from left edge
Vertical:   (64 - 32) / 2 = 16 pixels from top edge
```

**Alternative if using monospace:** 20×32 uniform (100px width, 14px margin)

### Minimum Viable Font Data

**Required Glyphs (11 total):**
- Digits: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 (10 glyphs)
- Punctuation: : (colon) (1 glyph)

**Estimated Flash Usage:**
- 20×32 monochrome bitmap: 20×32 = 640 bits = 80 bytes per glyph
- 11 glyphs × 80 bytes = **~880 bytes** font data
- Plus CFB overhead: ~200 bytes
- **Total: ~1KB font storage**

---

## 3. State Machine (Updated)

```
                    ┌─────────────────────────────────────┐
                    │         LONG PRESS (1s+)            │
                    ▼                                     │
     ┌──────────┐         SHORT PRESS         ┌──────────┐│
     │ SETTING  │────────────────────────────▶│ RUNNING  ││
     │  (init)  │◄────────────────────────────│          │┘
     └────┬─────┘         LONG PRESS (1s+)    └────┬─────┘
          │                              │          │
          │                              │   TIMEOUT (00:00)
          │                              │          ▼
          │                              │   ┌──────────┐
          │                              │   │ COMPLETE │
          │                              │   │  00:00   │
          │                              │   └────┬─────┘
          │                              │        │
          │                              │   ANY PRESS
          │                    SHORT PRESS        │
          │                              ▼        │
          │                         ┌──────────┐  │
          │                         │  PAUSED  │◄─┘
          │                         │  MM:SS   │
          │                         └────┬─────┘
          │                              │
          │                    SHORT PRESS (resume)
          │                    LONG PRESS (1s+ → setting)
          │                              │
          └──────────────────────────────┘
```

### State Definitions

#### STATE: SETTING (Initial State)
- **Display:** "MM:SS" (default 00:30)
- **Encoder:** +/- 30 seconds per detent
- **Range:** 30 seconds to 60:00 (60 minutes)
- **Short Press:** Start countdown → RUNNING
- **Long Press (1s+):** Reset to 00:30 (30 seconds)

#### STATE: RUNNING
- **Display:** Counting down "MM:SS"
- **Encoder:** Disabled (locked during countdown)
- **Short Press:** Pause → PAUSED
- **Long Press (1s+):** Stop and return to SETTING
- **Timeout (00:00):** → COMPLETE

#### STATE: PAUSED
- **Display:** Static "MM:SS" (frozen remaining time)
- **Encoder:** +/- 30 seconds to adjust remaining time
- **Range:** Minimum 30 seconds, maximum 60:00
- **Short Press:** Resume → RUNNING
- **Long Press (1s+):** Stop and return to SETTING

#### STATE: COMPLETE
- **Display:** "00:00" (static)
- **Any EC11 Press:** Return to SETTING mode
- **Action:** Signal pump controller to force shutdown

---

## 4. Time Range Specification

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **Minimum** | 30 seconds | Safety minimum (encoder starts here) |
| **Maximum** | 60:00 (60 min) | Practical upper limit for water pump |
| **Default** | 00:30 (30 sec) | Quick-start default |
| **Increment** | 30 seconds | Balanced granularity (2 clicks = 1 min) |
| **Format** | MM:SS | Always 2-digit minutes, 2-digit seconds |

**Time Display Examples:**
- 30 seconds: `00:30`
- 1 minute 30 seconds: `01:30`
- 15 minutes: `15:00`
- 60 minutes: `60:00`

---

## 5. Display Layout (128×64 SH1106)

### Single-Element Layout

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│                    ┌──────────────────┐                     │
│                    │                  │                     │
│    (16px margin)   │     MM:SS        │   (16px margin)     │
│                    │    20×32 font    │                     │
│                    │    92px width    │                     │
│                    │    32px height   │                     │
│                    │                  │                     │
│                    └──────────────────┘                     │
│                                                             │
│  18px margin (left)      92px text        18px margin (right)│
│                                                             │
└─────────────────────────────────────────────────────────────┘
128 pixels wide × 64 pixels high
```

### Exact Coordinates

```c
/* Display Constants */
#define DISPLAY_WIDTH       128
#define DISPLAY_HEIGHT      64
#define FONT_WIDTH_DIGIT    20
#define FONT_WIDTH_COLON    12
#define FONT_HEIGHT         32

/* Text dimensions for "MM:SS" (variable-width colon) */
#define TEXT_WIDTH          ((4 * FONT_WIDTH_DIGIT) + FONT_WIDTH_COLON)  /* 92px */
#define TEXT_HEIGHT         FONT_HEIGHT                                  /* 32px */

/* Centered coordinates */
#define TEXT_X              ((DISPLAY_WIDTH - TEXT_WIDTH) / 2)   /* 18 */
#define TEXT_Y              ((DISPLAY_HEIGHT - TEXT_HEIGHT) / 2) /* 16 */

/* For monospace 20×32 (if variable not available) */
#define TEXT_WIDTH_MONO     (5 * 20)  /* 100px */
#define TEXT_X_MONO         ((128 - 100) / 2)  /* 14 */
```

### Rendering Pseudocode

```c
void display_render_time(uint32_t total_seconds)
{
    uint8_t minutes = total_seconds / 60;
    uint8_t seconds = total_seconds % 60;
    
    /* Format as MM:SS */
    char time_str[6];
    snprintf(time_str, sizeof(time_str), "%02u:%02u", minutes, seconds);
    
    /* Clear entire display */
    cfb_framebuffer_clear(dev, true);
    
    /* Set font */
    cfb_set_font(dev, FONT_INDEX_20X32);
    
    /* Draw at centered position */
    cfb_draw_text(dev, time_str, TEXT_X, TEXT_Y);
    
    /* Blit to display */
    cfb_framebuffer_finalize(dev);
}
```

---

## 6. Simplified Architecture

### Thread Model

```
┌─────────────────────────────────────────────────────────┐
│                      UI Thread                          │
│                    (Priority 4)                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Input Handler│  │ State Machine│  │   Display    │  │
│  │  (10ms poll) │  │   (4 states) │  │  (10Hz upd)  │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘  │
│         │                 │                             │
│         ▼                 ▼                             │
│  ┌──────────────────────────────────────┐              │
│  │      Timer Context (single struct)   │              │
│  │  - state (enum)                      │              │
│  │  - total_seconds (uint32_t)          │              │
│  │  - remaining_seconds (uint32_t)      │              │
│  │  - last_update_tick (int64_t)        │              │
│  └──────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────┘
                           │
                           │ timer_status_msgq
                           ▼
                  ┌─────────────────┐
                  │ Pump Controller │
                  │  (Priority 2)   │
                  └─────────────────┘
```

### Single Display Element

```c
/* Only ONE display function needed */
void ui_display_timer(uint32_t seconds_remaining);

/* Called from state machine on:
 * - State entry
 * - Every 1-second tick (RUNNING state)
 * - Encoder change (SETTING/PAUSED states)
 */
```

---

## 7. Kconfig Configuration

```kconfig
# UI Thread
config UI_THREAD_STACK_SIZE
    int "UI thread stack size"
    default 2048

config UI_THREAD_PRIORITY
    int "UI thread priority"
    default 4

config UI_UPDATE_INTERVAL_MS
    int "Display refresh interval (ms)"
    default 100

# Timer Defaults
config TIMER_DEFAULT_SECONDS
    int "Default timer duration (seconds)"
    default 30
    range 30 3600

config TIMER_MIN_SECONDS
    int "Minimum timer duration (seconds)"
    default 30

config TIMER_MAX_SECONDS
    int "Maximum timer duration (seconds)"
    default 3600  # 60 minutes

config TIMER_ENCODER_STEP_SECONDS
    int "Encoder increment (seconds)"
    default 30

# Input Timing
config TIMER_BUTTON_SHORT_MAX_MS
    int "Short press max duration (ms)"
    default 800

config TIMER_BUTTON_LONG_MIN_MS
    int "Long press min duration (ms)"
    default 1000

config TIMER_BUTTON_DEBOUNCE_MS
    int "Button debounce time (ms)"
    default 50

# Font Selection
choice TIMER_FONT_SIZE
    prompt "Timer display font size"
    default TIMER_FONT_20X32
    
    config TIMER_FONT_16X24
        bool "16×24 (smaller, more margin)"
        
    config TIMER_FONT_20X32
        bool "20×32 (recommended)"
        
    config TIMER_FONT_24X32
        bool "24×32 (largest, minimal margin)"
endchoice
```

---

## 8. Implementation File Structure

```
app/src/ui/
├── ui_thread.c           # Thread entry, initialization
├── ui_input.c            # Encoder + button handling
├── ui_display.c          # Single-function display renderer
└── ui_timer.c            # 4-state FSM

app/include/ui/
├── ui_thread.h           # Public UI API
├── ui_input.h            # Input event types
├── ui_display.h          # Display constants (coords, fonts)
└── ui_timer.h            # Timer state machine API
```

---

## 9. Questions for User Confirmation

| Question | Current Proposal | Options |
|----------|------------------|---------|
| **Q1: Maximum time limit?** | 60 minutes (60:00) | A) 60 min B) 99 min C) Other: ___ |
| **Q2: Minimum time?** | 30 seconds (00:30) | A) 30 sec B) 0 (allow 00:00) C) Other: ___ |
| **Q3: Complete mode display?** | Show "00:00" | A) 00:00 B) DONE C) Flash 00:00 D) Other: ___ |
| **Q4: Long press in SETTING?** | Reset to 00:30 | A) Reset to 00:30 B) Reset to 00:00 C) No action D) ___ |

---

## 10. Quick Reference Card

```
┌─────────────────────────────────────────────┐
│         COUNTDOWN TIMER QUICK REF           │
├─────────────────────────────────────────────┤
│ DISPLAY: Single large "MM:SS" centered      │
│ FONT: 20×32 pixels (or 16×24 if needed)     │
│ POSITION: x=18, y=16 (centered)             │
├─────────────────────────────────────────────┤
│ SETTING Mode:                               │
│   • Rotate: +/- 30 seconds                  │
│   • Short press: Start                      │
│   • Long press: Reset to 00:30              │
├─────────────────────────────────────────────┤
│ RUNNING Mode:                               │
│   • Short press: Pause                      │
│   • Long press: Stop → Setting              │
│   • Reaches 00:00 → COMPLETE                │
├─────────────────────────────────────────────┤
│ PAUSED Mode:                                │
│   • Rotate: +/- 30 seconds (adjust time)    │
│   • Short press: Resume                     │
│   • Long press: Stop → Setting              │
├─────────────────────────────────────────────┤
│ COMPLETE Mode:                              │
│   • Any press: Return to Setting            │
│   • Pump: FORCED OFF                        │
├─────────────────────────────────────────────┤
│ RANGE: 00:30 to 60:00                       │
│ STEP: 30 seconds per encoder click          │
└─────────────────────────────────────────────┘
```

---

## 11. Deliverables Summary

1. ✅ **Font Size:** 20×32 recommended (92px total width, 78% screen usage)
2. ✅ **Minimum Font Data:** 11 glyphs (0-9, :) = ~880 bytes + overhead
3. ✅ **State Machine:** 4 states with 30-second encoder increments
4. ✅ **Time Range:** 30 seconds to 60 minutes (configurable)
5. ✅ **Display Position:** x=18, y=16 (exact center for 92px width)
6. ✅ **Simplified Architecture:** Single display function, one element

---

**Status:** READY FOR IMPLEMENTATION  
**Next Step:** Confirm 4 questions above, then proceed to code
