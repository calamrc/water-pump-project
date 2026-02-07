/**
 * @file input_manager.c
 * @brief Input manager implementation for rotary encoder and button using input subsystem
 * 
 * Thread-safe input event handling with minimal ISR processing
 */

#include "input/input_manager.h"

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(input_manager, CONFIG_APP_LOG_LEVEL);

/* Long press threshold (milliseconds) */
#define LONG_PRESS_MS 1000

/* Encoder step threshold - qdec generates 4 events per detent */
#define ENCODER_STEPS_PER_DETENT 4

/* Maximum accumulated events to prevent overflow */
#define MAX_ENCODER_DELTA 100

/* Encoder state - accessed from ISR */
static atomic_t encoder_accumulator = ATOMIC_INIT(0);
static atomic_t encoder_delta = ATOMIC_INIT(0);

/* Button state - accessed from ISR and thread */
static atomic_t button_pressed = ATOMIC_INIT(0);
static atomic_t button_press_time = ATOMIC_INIT(0);
static atomic_t long_press_reported = ATOMIC_INIT(0);

/* Event flags - atomic for thread safety */
static atomic_t encoder_event_pending = ATOMIC_INIT(0);
static atomic_t button_event_pending = ATOMIC_INIT(0);
static atomic_t button_event_type = ATOMIC_INIT(BUTTON_PRESS_NONE);

/* Input enabled flag */
static volatile bool input_enabled = true;

/* Timer for long press detection - runs in thread context */
static struct k_timer long_press_timer;
static bool timer_initialized = false;

/**
 * @brief Long press timer handler - runs in system workqueue context
 */
static void long_press_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
    if (!input_enabled) {
        return;
    }
    
    /* Check if button is still pressed and long press not yet reported */
    if (atomic_get(&button_pressed) && !atomic_get(&long_press_reported)) {
        atomic_set(&long_press_reported, 1);
        atomic_set(&button_event_type, BUTTON_PRESS_LONG);
        atomic_set(&button_event_pending, 1);
        LOG_INF("LONG press detected");
    }
}

/**
 * @brief Input callback - runs in ISR context, must be fast and safe
 */
static void input_callback(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);
    
    if (!input_enabled) {
        return;
    }

    /* Handle encoder - just accumulate, process in thread */
    if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_WHEEL) {
        /* Atomically add to accumulator */
        atomic_add(&encoder_accumulator, evt->value);
        
        /* Simple detent detection in ISR - just flag that we have events */
        int32_t accum = atomic_get(&encoder_accumulator);
        
        /* Quick check if we have a complete detent */
        if (accum >= ENCODER_STEPS_PER_DETENT) {
            atomic_sub(&encoder_accumulator, ENCODER_STEPS_PER_DETENT);
            atomic_inc(&encoder_delta);
            atomic_set(&encoder_event_pending, 1);
        } else if (accum <= -ENCODER_STEPS_PER_DETENT) {
            atomic_add(&encoder_accumulator, ENCODER_STEPS_PER_DETENT);
            atomic_dec(&encoder_delta);
            atomic_set(&encoder_event_pending, 1);
        }
    }
    
    /* Handle button - minimal processing in ISR */
    if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_ENTER) {
        if (evt->value == 1) {
            /* Button pressed */
            atomic_set(&button_pressed, 1);
            atomic_set(&long_press_reported, 0);
            atomic_set(&button_press_time, k_uptime_get());
            
            /* Start long press timer from thread context */
            if (timer_initialized) {
                k_timer_start(&long_press_timer, K_MSEC(LONG_PRESS_MS), K_NO_WAIT);
            }
        } else if (evt->value == 0) {
            /* Button released */
            if (timer_initialized) {
                k_timer_stop(&long_press_timer);
            }
            
            /* Only report if not already reported as long press */
            if (atomic_get(&button_pressed) && !atomic_get(&long_press_reported)) {
                atomic_set(&button_event_type, BUTTON_PRESS_SHORT);
                atomic_set(&button_event_pending, 1);
            }
            
            atomic_set(&button_pressed, 0);
        }
    }
}

/* Register callback for all input devices */
INPUT_CALLBACK_DEFINE(NULL, input_callback, NULL);

int input_manager_init(void)
{
    LOG_INF("Initializing input manager...");

    /* Initialize atomic variables */
    atomic_set(&encoder_accumulator, 0);
    atomic_set(&encoder_delta, 0);
    atomic_set(&encoder_event_pending, 0);
    
    atomic_set(&button_pressed, 0);
    atomic_set(&button_press_time, 0);
    atomic_set(&long_press_reported, 0);
    atomic_set(&button_event_pending, 0);
    atomic_set(&button_event_type, BUTTON_PRESS_NONE);

    /* Initialize long press timer */
    k_timer_init(&long_press_timer, long_press_timer_handler, NULL);
    timer_initialized = true;

    LOG_INF("Input manager initialized");
    return 0;
}

bool input_manager_get_event(struct ui_input_event *event)
{
    if (event == NULL) {
        return false;
    }

    bool has_event = false;
    
    /* Check for encoder events */
    if (atomic_get(&encoder_event_pending)) {
        event->encoder_moved = true;
        event->encoder_delta = atomic_get(&encoder_delta);
        
        /* Reset encoder state */
        atomic_set(&encoder_delta, 0);
        atomic_set(&encoder_event_pending, 0);
        
        has_event = true;
        
        if (event->encoder_delta != 0) {
            LOG_DBG("Encoder event: delta=%d", event->encoder_delta);
        }
    } else {
        event->encoder_moved = false;
        event->encoder_delta = 0;
    }
    
    /* Check for button events */
    if (atomic_get(&button_event_pending)) {
        event->button_press = atomic_get(&button_event_type);
        
        /* Reset button state */
        atomic_set(&button_event_type, BUTTON_PRESS_NONE);
        atomic_set(&button_event_pending, 0);
        
        has_event = true;
        
        LOG_INF("Button event: %s", 
                event->button_press == BUTTON_PRESS_SHORT ? "SHORT" : "LONG");
    } else {
        event->button_press = BUTTON_PRESS_NONE;
    }

    return has_event;
}

void input_manager_enable(bool enable)
{
    input_enabled = enable;
    
    if (!enable && timer_initialized) {
        k_timer_stop(&long_press_timer);
    }
}
