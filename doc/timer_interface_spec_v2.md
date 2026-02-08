# Revised Countdown Timer Interface Specification

## Zephyr Water Pump Project - Timer Interface v2.0
**Date:** 2025-02-08  
**Target:** ESP32-DevKitC / Zephyr 4.2.x  
**Status:** Specification Ready for Implementation

---

## 1. Executive Summary

This specification defines a countdown timer interface for the water pump control system using **only** three hardware components: SH1106 OLED display, EC11 rotary encoder, and EC11 push button. The timer operates as a **safety override** (Option C) - it sets a maximum runtime limit while preserving the existing flow-sensor-based pump control.

**Key Design Decision:** The timer acts as a safety maximum runtime limiter. The pump continues to operate based on flow sensor detection, but the timer will force pump shutdown when the countdown expires.

---

## 2. Hardware Configuration (Revised)

### 2.1 Device Tree Changes

**File:** `app/boards/esp32_devkitc_procpu.overlay`

Add the EC11 button to the existing overlay. The encoder A/B are already on GPIO0 pin 25 and GPIO1 pin 33.

```dts
/* EC11 Rotary Encoder Button (GPIO0 pin 26) - ADD THIS */
ec11_button: ec11-button {
    compatible = "gpio-keys";
    ec11_btn: ec11_btn {
        gpios = <&gpio0 26 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
        zephyr,code = <INPUT_BTN_0>;
        label = "EC11 Button";
    };
};
```

**Reserved for Future (DO NOT ADD YET):**
- Confirm button: GPIO0 pin 27
- Back button: GPIO1 pin 22

### 2.2 Hardware Summary

| Component | GPIO | Function | Status |
|-----------|------|----------|--------|
| SH1106 SDA | GPIO0 21 | I2C data | Existing |
| SH1106 SCL | GPIO0 22 | I2C clock | Existing |
| Flow Sensor | GPIO0 18 | Pulse input | Existing |
| Pump Relay | GPIO0 19 | Active LOW | Existing |
| EC11 A | GPIO0 25 | Encoder channel A | Existing |
| EC11 B | GPIO1 33 | Encoder channel B | Existing |
| **EC11 Button** | **GPIO0 26** | **Push switch** | **TO ADD** |
| Confirm Button | GPIO0 27 | Reserved | Future |
| Back Button | GPIO1 22 | Reserved | Future |

---

## 3. Architecture Overview

### 3.1 New Thread: UI Thread

Add a fifth application thread dedicated to UI management:

```c
/* Thread configuration (add to prj.conf) */
CONFIG_UI_THREAD_STACK_SIZE=2048
CONFIG_UI_THREAD_PRIORITY=4  /* Lower than pump controller (2), higher than supervisor (7) */

/* UI thread update rate */
CONFIG_UI_UPDATE_INTERVAL_MS=100  /* 10Hz display refresh */
CONFIG_ENCODER_POLL_INTERVAL_MS=10 /* 100Hz encoder polling */
```

**Thread Responsibilities:**
1. Display management (SH1106 via CFB/Character Framebuffer)
2. Input handling (encoder rotation + button events)
3. Timer state machine execution
4. Timer-pump coordination (signal pump controller on timeout)

### 3.2 Inter-Thread Communication

Add timer-related messages to existing communication infrastructure:

```c
/* Add to thread_comm.h */

/* Timer event types for UI-to-pump signaling */
enum timer_event_type {
    TIMER_EVENT_NONE,
    TIMER_EVENT_STARTED,      /* User started countdown */
    TIMER_EVENT_PAUSED,       /* User paused countdown */
    TIMER_EVENT_RESUMED,      /* User resumed countdown */
    TIMER_EVENT_EXPIRED,      /* Countdown reached zero */
    TIMER_EVENT_RESET,        /* User reset timer */
};

/* Timer state for display synchronization */
enum timer_display_state {
    TIMER_STATE_SETTING,      /* Setting initial time */
    TIMER_STATE_RUNNING,      /* Countdown active */
    TIMER_STATE_PAUSED,       /* Countdown paused */
    TIMER_STATE_COMPLETE,     /* Countdown finished */
};

/* Timer status message (UI thread → pump controller) */
struct timer_status_msg {
    enum timer_event_type event;
    uint32_t remaining_seconds;   /* Current countdown value */
    uint32_t total_seconds;       /* Initial countdown value */
    enum timer_display_state state;
    int64_t timestamp;
};

/* External declaration */
extern struct k_msgq timer_status_msgq;
```

### 3.3 Integration with Pump Controller

The timer integrates with pump control via **cooperative signaling**:

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│   UI Thread     │─────▶│  timer_status_msgq │─────▶│ Pump Controller │
│  (Timer logic)  │      │   (Kconfig: 4)    │      │   (reads msg)   │
└─────────────────┘      └──────────────────┘      └─────────────────┘
                                                          │
                                                          ▼
                                                   ┌─────────────────┐
                                                   │  Timer Expired? │
                                                   │  → Force pump   │
                                                   │    shutdown     │
                                                   └─────────────────┘
```

**Timer-Pump Relationship (Option C - Safety Override):**

| Timer State | Pump Behavior |
|-------------|---------------|
| SETTING | Pump operates on flow sensor only (no timer limit) |
| RUNNING | Pump operates on flow sensor, but timer enforces max runtime |
| PAUSED | Pump continues on flow sensor, timer frozen |
| COMPLETE | Timer forces pump OFF, flow sensor ignored |

**Rationale:** This preserves existing flow-based control while adding safety. Users can still pause/resume timer without affecting pump operation during active flow.

---

## 4. State Machine Specification

### 4.1 States and Transitions

```
                    ┌─────────────────────────────────────┐
                    │         LONG PRESS (1s+)            │
                    ▼                                     │
    ┌──────────┐         SHORT PRESS         ┌──────────┐│
    │ SETTING  │────────────────────────────▶│ RUNNING  ││
    │  (init)  │◄────────────────────────────│          │┘
    └────┬─────┘         LONG PRESS (1s+)    └────┬─────┘
         │                                         │
         │                               SHORT PRESS
         │                                         ▼
         │                               ┌──────────┐
         │                               │  PAUSED  │
         │                               │          │
         │                               └────┬─────┘
         │                                    │
         │                          SHORT PRESS (resume)
         │                          LONG PRESS (1s+ → reset)
         │                                    │
         │                                    ▼
         │                          ┌──────────┐
         │                          │ RUNNING  │
         │                          └──────────┘
         │                                 │
         │                                 │ TIMEOUT
         ▼                                 ▼
    ┌──────────┐                      ┌──────────┐
    │ SETTING  │◄─────────────────────│ COMPLETE │
    │  (reset) │    ANY BUTTON PRESS  │ (blink)  │
    └──────────┘                      └──────────┘
```

### 4.2 State Definitions

#### STATE: SETTING (Initial State)
- **Entry Action:** Display "SET TIMER" with default value (15 min)
- **Encoder:** Adjust minutes (1-120, wraps around)
- **Short Press:** Start timer → transition to RUNNING
- **Long Press (1+ sec):** Reset to default (15 min)

#### STATE: RUNNING
- **Entry Action:** Start countdown, enable timer-pump safety check
- **Display:** Show countdown + progress bar
- **Encoder:** No function (or fine-tune if we want)
- **Short Press:** Pause timer → transition to PAUSED
- **Long Press (1+ sec):** Reset → transition to SETTING
- **Timeout (0:00):** → transition to COMPLETE

#### STATE: PAUSED
- **Entry Action:** Freeze countdown display
- **Display:** "PAUSED" + remaining time
- **Encoder:** Adjust remaining time (+/- 1 min per click)
- **Short Press:** Resume → transition to RUNNING
- **Long Press (1+ sec):** Reset → transition to SETTING

#### STATE: COMPLETE
- **Entry Action:** Signal pump controller to force shutdown
- **Display:** Flashing "TIME'S UP!" or "00:00"
- **Any Button Press:** Reset → transition to SETTING

---

## 5. UI Layout Design (128x64 SH1106)

### 5.1 Display Layout Grid

```
┌─────────────────────────────────────────────────────────────┐
│  Row 0-7 (8px)    │  Status Bar: Mode Icon + Flow Indicator │
├─────────────────────────────────────────────────────────────┤
│                    │                                         │
│  Row 8-39 (32px)   │     LARGE TIMER DISPLAY (MM:SS)         │
│   Main Display     │        or "SET TIMER"                   │
│                    │                                         │
├─────────────────────────────────────────────────────────────┤
│  Row 40-47 (8px)   │  Progress Bar (if running/paused)       │
├─────────────────────────────────────────────────────────────┤
│  Row 48-63 (16px)  │  Auxiliary: "of XX min" / Status text   │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Screen Mockups

#### SETTING Mode
```
┌────────────────────────────────┐
│[⏱️] SET TIMER      [≈]        │  ← Status: Icon + Flow icon
│                                │
│         15                     │  ← Large number (24px font)
│        min                     │  ← Unit label (8px font)
│                                │
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│  ← Empty progress bar outline
│                                │
│   Press to start  [<<] [>>]   │  ← Hint: Short press = start
└────────────────────────────────┘
```

#### RUNNING Mode
```
┌────────────────────────────────┐
│[▶] RUNNING         [≈≈]       │  ← Status: Play icon + Flow active
│                                │
│        12:34                   │  ← Large countdown (32px font)
│                                │
│████████████████████░░░░░░░░░░░░│  ← Progress bar (filled %)
│                                │
│    of 15 min      [II]        │  ← Context + Pause hint
└────────────────────────────────┘
```

#### PAUSED Mode
```
┌────────────────────────────────┐
│[II] PAUSED         [≈≈]       │  ← Status: Pause icon + Flow still active
│                                │
│        12:34                   │  ← Frozen countdown (32px)
│                                │
│████████████████████░░░░░░░░░░░░│  ← Frozen progress bar
│                                │
│    of 15 min    Turn = adjust │  ← Encoder hint
└────────────────────────────────┘
```

#### COMPLETE Mode (Flashing)
```
┌────────────────────────────────┐
│[!] TIME'S UP!      [  ]       │  ← Alert icon + No flow
│                                │
│        00:00                   │  ← Blinking zeros
│                                │
│░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│  ← Empty progress bar
│                                │
│   Pump OFF by timer            │
└────────────────────────────────┘
```

### 5.3 Font Specifications

| Element | Font Size | CFB Font | Notes |
|---------|-----------|----------|-------|
| Large digits | 32px | `cfb_fonts/32x48` or custom | Main timer display |
| Status text | 8px | `cfb_fonts/8x16` | Mode labels |
| Units/Hints | 8px | `cfb_fonts/8x16` | "min", hints |
| Icons | 8x8 bitmap | Custom | ⏱️, ▶, II, !, ≈ |

**Progress Bar:**
- Width: 128 pixels
- Height: 8 pixels
- Fill: Solid blocks (█)
- Empty: Outline or light blocks (░)
- Update: Every second or on encoder change

---

## 6. Implementation Phases

### Phase 1: Hardware & Input (Week 1)
**Goal:** Verify all inputs work independently

1. **Device Tree:** Add EC11 button overlay
2. **Kconfig:** Add UI thread configuration options
3. **Input Module:** Create `ui_input.c/h`
   - Initialize encoder (already in DT, just use input subsystem)
   - Initialize button with GPIO callback
   - Implement debouncing (50ms for button)
   - Generate events: ENCODER_CW, ENCODER_CCW, BUTTON_SHORT, BUTTON_LONG
4. **Test:** Print events to console/log

**Deliverable:** `west flash` and rotate encoder/press button → see events in log

### Phase 2: Display Foundation (Week 1-2)
**Goal:** Get SH1106 working with CFB

1. **Display Module:** Create `ui_display.c/h`
   - Initialize CFB with SH1106
   - Implement display buffer management
   - Create font rendering utilities
   - Implement screen clear/blit
2. **Test Screens:** Create test mode to cycle through:
   - Full screen fill patterns
   - Text rendering at different sizes
   - Progress bar rendering
3. **Layout Engine:** Create screen layout functions

**Deliverable:** Display cycles through test patterns automatically

### Phase 3: Timer State Machine (Week 2)
**Goal:** Functional timer without pump integration

1. **Timer Module:** Create `ui_timer.c/h`
   - Implement 4-state FSM
   - Encoder value mapping (1-120 minutes)
   - Countdown logic (1-second timer)
   - Button event handlers per state
2. **UI Integration:** Wire display to timer state
   - Render appropriate screen for each state
   - Update display on state changes
   - Handle encoder in SETTING and PAUSED modes
3. **Inter-thread:** Add timer_status_msgq
   - Pump controller reads but doesn't act on messages yet

**Deliverable:** Timer runs standalone - can set, start, pause, resume, reset

### Phase 4: Pump Integration (Week 3)
**Goal:** Timer controls pump safety shutdown

1. **Pump Controller Modification:**
   - Read timer_status_msgq in pump_controller_thread
   - Add `timer_expired` flag to pump state
   - Modify timeout handling:
     - If timer_expired: force pump off regardless of flow
     - Else: existing flow-based logic
2. **Safety Coordination:**
   - Timer COMPLETE state triggers emergency_stop
   - Safety monitor respects timer expiration
3. **Display Integration:**
   - Show pump state on display (flow indicator)

**Deliverable:** Timer expires → pump shuts down

### Phase 5: Polish & Testing (Week 3-4)
**Goal:** Production-ready interface

1. **Visual Polish:**
   - Smooth animations (fade effects)
   - Better icons/bitmap graphics
   - Progress bar animations
2. **User Experience:**
   - Encoder acceleration (faster turning = bigger jumps)
   - Visual feedback on button press
   - Auto-brightness or contrast adjustment
3. **Testing:**
   - Unit tests for state machine
   - Integration tests for timer-pump interaction
   - Edge cases: timer expires during flow, pause during flow, etc.

**Deliverable:** Complete, tested countdown timer system

---

## 7. Integration Questions - Answers

### Q1: Should timer control pump independently or alongside flow sensor?
**Answer:** Alongside (Option C - Timer as Safety). The timer sets a maximum runtime limit, but the pump still responds to flow sensor for normal operation. When timer expires, pump is forced off.

### Q2: When timer runs out, should it force pump off regardless of flow?
**Answer:** Yes. Timer expiration triggers emergency shutdown. This is the safety feature - prevents pump from running indefinitely even if flow is detected.

### Q3: Should display show flow rate information alongside timer?
**Answer:** Yes, but minimal. Show a flow indicator icon (≈ when flowing, empty when not) in the status bar. Full flow rate values can be added later via supervisor thread stats display.

### Q4: What happens if timer expires while pump is running?
**Answer:** 
1. Timer enters COMPLETE state
2. UI sends TIMER_EVENT_EXPIRED to pump controller
3. Pump controller calls emergency_stop()
4. Display shows "TIME'S UP!" + "Pump OFF"
5. User must press button to reset and restart timer

### Q5: Can user adjust timer while pump is running?
**Answer:** Only when PAUSED. In RUNNING state, encoder is disabled to prevent accidental changes. User must pause first, then adjust.

### Q6: What is default timer value?
**Answer:** 15 minutes. Configurable via Kconfig: `CONFIG_TIMER_DEFAULT_MINUTES=15`

---

## 8. File Structure

```
app/
├── src/
│   ├── main.c                    # Existing
│   ├── thread_manager.c          # Existing - add UI thread creation
│   ├── flow_analyzer.c           # Existing
│   ├── error_handler.c           # Existing
│   ├── ui/
│   │   ├── ui_thread.c           # NEW: UI thread entry point
│   │   ├── ui_thread.h           # NEW: UI thread interface
│   │   ├── ui_input.c            # NEW: Encoder + button input handling
│   │   ├── ui_input.h            # NEW: Input events and API
│   │   ├── ui_display.c          # NEW: SH1106 display rendering
│   │   ├── ui_display.h          # NEW: Display API
│   │   ├── ui_timer.c            # NEW: Timer state machine
│   │   └── ui_timer.h            # NEW: Timer API and states
│   └── ...
├── include/
│   ├── thread_comm.h             # MODIFY: Add timer messages
│   └── ...
├── boards/
│   └── esp32_devkitc_procpu.overlay  # MODIFY: Add EC11 button
└── prj.conf                      # MODIFY: Add UI thread config
```

---

## 9. Configuration Summary (Kconfig additions)

```kconfig
# UI Thread Configuration
config UI_THREAD_STACK_SIZE
    int "UI thread stack size"
    default 2048
    help
        Stack size for UI management thread.

config UI_THREAD_PRIORITY
    int "UI thread priority"
    default 4
    help
        Priority for UI thread. Lower number = higher priority.
        Should be below pump controller (2) but above supervisor (7).

config UI_UPDATE_INTERVAL_MS
    int "UI update interval (ms)"
    default 100
    help
        Display refresh rate in milliseconds (10Hz).

config UI_ENCODER_POLL_INTERVAL_MS
    int "Encoder poll interval (ms)"
    default 10
    help
        Encoder polling rate in milliseconds (100Hz).

# Timer Configuration
config TIMER_DEFAULT_MINUTES
    int "Default timer duration (minutes)"
    default 15
    range 1 120
    help
        Default countdown timer value in minutes.

config TIMER_BUTTON_LONG_PRESS_MS
    int "Long press duration (ms)"
    default 1000
    help
        Duration for long press detection in milliseconds.

config TIMER_BUTTON_DEBOUNCE_MS
    int "Button debounce time (ms)"
    default 50
    help
        Debounce time for button press detection.

config TIMER_ENCODER_ACCELERATION
    bool "Enable encoder acceleration"
    default y
    help
        Enable faster value changes with rapid encoder rotation.

# Message Queue Configuration
config TIMER_STATUS_MSGQ_SIZE
    int "Timer status message queue size"
    default 4
    help
        Size of message queue for timer-to-pump communication.

# Display Configuration
config UI_DISPLAY_CONTRAST
    int "Display contrast (0-255)"
    default 128
    help
        OLED display contrast level.

config UI_PROGRESS_BAR_HEIGHT
    int "Progress bar height (pixels)"
    default 8
    help
        Height of countdown progress bar.
```

---

## 10. Risk Assessment & Mitigation

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Encoder rotation too sensitive | Medium | Medium | Add deadband + acceleration curve |
| Display refresh causes pump jitter | Medium | Low | UI thread at lower priority than pump |
| Timer-pump race condition | High | Low | Mutex protection on shared pump state |
| Button debounce misses presses | Low | Low | 50ms debounce + state machine robustness |
| Display burn-in over time | Low | High | Screensaver mode after 5 min idle |
| Memory exhaustion with CFB | Medium | Low | Monitor heap usage, static allocation |

---

## 11. Success Criteria

1. **Functional:** All 4 timer states work correctly with encoder + button
2. **Safety:** Timer expiration always forces pump shutdown
3. **Usability:** Setting timer takes < 10 seconds for typical use
4. **Reliability:** 48-hour continuous operation without crashes
5. **Integration:** No regression in existing pump flow-control behavior

---

## 12. Appendix: Code Snippets

### A. Input Event Handling (ui_input.c)

```c
/* Input event types */
enum ui_input_event {
    UI_INPUT_ENCODER_CW,      /* Clockwise rotation */
    UI_INPUT_ENCODER_CCW,     /* Counter-clockwise rotation */
    UI_INPUT_BUTTON_SHORT,    /* Short press (< 1s) */
    UI_INPUT_BUTTON_LONG,     /* Long press (>= 1s) */
};

/* Input callback type */
typedef void (*ui_input_callback_t)(enum ui_input_event event, void *user_data);

/* API */
int ui_input_init(ui_input_callback_t callback, void *user_data);
int ui_input_start_polling(void);
```

### B. Timer State Machine (ui_timer.c)

```c
/* Timer state */
enum timer_state {
    TIMER_STATE_SETTING,
    TIMER_STATE_RUNNING,
    TIMER_STATE_PAUSED,
    TIMER_STATE_COMPLETE,
};

/* Timer context */
struct timer_context {
    enum timer_state state;
    uint32_t total_seconds;      /* Initial setting */
    uint32_t remaining_seconds;  /* Current countdown */
    int64_t last_tick;          /* For 1-second countdown */
    bool flash_state;           /* For COMPLETE blinking */
};

/* State handlers */
typedef void (*state_handler_t)(struct timer_context *ctx, enum ui_input_event event);

static void handle_setting(struct timer_context *ctx, enum ui_input_event event);
static void handle_running(struct timer_context *ctx, enum ui_input_event event);
static void handle_paused(struct timer_context *ctx, enum ui_input_event event);
static void handle_complete(struct timer_context *ctx, enum ui_input_event event);

static const state_handler_t state_handlers[] = {
    [TIMER_STATE_SETTING] = handle_setting,
    [TIMER_STATE_RUNNING] = handle_running,
    [TIMER_STATE_PAUSED] = handle_paused,
    [TIMER_STATE_COMPLETE] = handle_complete,
};
```

### C. Pump Controller Integration (pump_controller.c additions)

```c
/* In pump_controller_thread() message processing */

struct timer_status_msg timer_msg;
static bool timer_expired = false;

/* Check for timer messages (non-blocking) */
if (k_msgq_get(&timer_status_msgq, &timer_msg, K_NO_WAIT) == 0) {
    switch (timer_msg.event) {
    case TIMER_EVENT_STARTED:
        timer_expired = false;
        LOG_INF("Timer started: %u min", timer_msg.total_seconds / 60);
        break;
    case TIMER_EVENT_EXPIRED:
        timer_expired = true;
        LOG_ERR("Timer expired - forcing pump shutdown");
        pump_controller_emergency_stop(pump);
        break;
    case TIMER_EVENT_RESET:
        timer_expired = false;
        break;
    default:
        break;
    }
}

/* In timeout handling - check timer_expired */
if (timer_expired) {
    LOG_INF("Pump shutdown prevented by timer expiration");
    /* Do not auto-restart pump even if flow detected */
}
```

---

**Document Status:** READY FOR REVIEW  
**Next Step:** Begin Phase 1 implementation upon approval
