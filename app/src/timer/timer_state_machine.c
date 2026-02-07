/**
 * @file timer_state_machine.c
 * @brief Countdown timer state machine implementation
 */

#include "timer/timer_state_machine.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(timer_sm, CONFIG_APP_LOG_LEVEL);

/* Timer state */
static enum timer_state current_state = TIMER_STATE_SETTING;

/* Timer values */
static uint32_t total_seconds = TIMER_DEFAULT_SECONDS;
static uint32_t remaining_seconds = TIMER_DEFAULT_SECONDS;

/* State change callback (optional, for notifications) */
static void (*state_change_callback)(enum timer_state new_state) = NULL;

/**
 * @brief Clamp value between min and max
 */
static uint32_t clamp_time(uint32_t seconds)
{
    if (seconds < TIMER_MIN_SECONDS) {
        return TIMER_MIN_SECONDS;
    }
    if (seconds > TIMER_MAX_SECONDS) {
        return TIMER_MAX_SECONDS;
    }
    return seconds;
}

/**
 * @brief Update time from seconds to minutes/seconds
 */
static void seconds_to_ms(uint32_t seconds, uint8_t *minutes, uint8_t *secs)
{
    *minutes = seconds / 60;
    *secs = seconds % 60;
}

void timer_sm_init(void)
{
    LOG_INF("Initializing timer state machine");
    
    current_state = TIMER_STATE_SETTING;
    total_seconds = TIMER_DEFAULT_SECONDS;
    remaining_seconds = TIMER_DEFAULT_SECONDS;
    
    LOG_INF("Timer initialized: %u seconds (%u:%02u)", 
            total_seconds, total_seconds / 60, total_seconds % 60);
}

enum timer_state timer_sm_get_state(void)
{
    return current_state;
}

void timer_sm_get_time(uint8_t *minutes, uint8_t *seconds)
{
    if (minutes == NULL || seconds == NULL) {
        return;
    }
    
    uint32_t secs_to_convert;
    
    switch (current_state) {
        case TIMER_STATE_SETTING:
            secs_to_convert = total_seconds;
            break;
        case TIMER_STATE_RUNNING:
        case TIMER_STATE_PAUSED:
            secs_to_convert = remaining_seconds;
            break;
        case TIMER_STATE_COMPLETED:
            secs_to_convert = 0;
            break;
        default:
            secs_to_convert = 0;
            break;
    }
    
    seconds_to_ms(secs_to_convert, minutes, seconds);
}

uint32_t timer_sm_get_remaining_seconds(void)
{
    switch (current_state) {
        case TIMER_STATE_SETTING:
            return total_seconds;
        case TIMER_STATE_RUNNING:
        case TIMER_STATE_PAUSED:
            return remaining_seconds;
        case TIMER_STATE_COMPLETED:
            return 0;
        default:
            return 0;
    }
}

void timer_sm_adjust_time(int32_t delta_seconds)
{
    uint32_t *target_seconds;
    
    /* Determine which value to adjust based on state */
    if (current_state == TIMER_STATE_SETTING) {
        target_seconds = &total_seconds;
    } else if (current_state == TIMER_STATE_PAUSED) {
        target_seconds = &remaining_seconds;
    } else {
        /* Cannot adjust in RUNNING or COMPLETED states */
        return;
    }
    
    /* Apply adjustment */
    int64_t new_value = (int64_t)*target_seconds + delta_seconds;
    
    /* Clamp to valid range */
    if (new_value < TIMER_MIN_SECONDS) {
        *target_seconds = TIMER_MIN_SECONDS;
    } else if (new_value > TIMER_MAX_SECONDS) {
        *target_seconds = TIMER_MAX_SECONDS;
    } else {
        *target_seconds = (uint32_t)new_value;
    }
    
    LOG_DBG("Timer adjusted: %u seconds", *target_seconds);
}

int timer_sm_start(void)
{
    if (current_state != TIMER_STATE_SETTING) {
        LOG_WRN("Cannot start timer from state: %s", 
                timer_sm_state_to_string(current_state));
        return -EINVAL;
    }
    
    /* Initialize remaining time from total */
    remaining_seconds = total_seconds;
    current_state = TIMER_STATE_RUNNING;
    
    LOG_INF("Timer started: %u seconds remaining", remaining_seconds);
    
    if (state_change_callback != NULL) {
        state_change_callback(current_state);
    }
    
    return 0;
}

int timer_sm_pause(void)
{
    if (current_state != TIMER_STATE_RUNNING) {
        LOG_WRN("Cannot pause timer from state: %s",
                timer_sm_state_to_string(current_state));
        return -EINVAL;
    }
    
    current_state = TIMER_STATE_PAUSED;
    
    LOG_INF("Timer paused: %u seconds remaining", remaining_seconds);
    
    if (state_change_callback != NULL) {
        state_change_callback(current_state);
    }
    
    return 0;
}

int timer_sm_resume(void)
{
    if (current_state != TIMER_STATE_PAUSED) {
        LOG_WRN("Cannot resume timer from state: %s",
                timer_sm_state_to_string(current_state));
        return -EINVAL;
    }
    
    current_state = TIMER_STATE_RUNNING;
    
    LOG_INF("Timer resumed: %u seconds remaining", remaining_seconds);
    
    if (state_change_callback != NULL) {
        state_change_callback(current_state);
    }
    
    return 0;
}

void timer_sm_reset(void)
{
    current_state = TIMER_STATE_SETTING;
    total_seconds = TIMER_DEFAULT_SECONDS;
    remaining_seconds = TIMER_DEFAULT_SECONDS;
    
    LOG_INF("Timer reset to default: %u seconds", total_seconds);
    
    if (state_change_callback != NULL) {
        state_change_callback(current_state);
    }
}

bool timer_sm_update(void)
{
    if (current_state != TIMER_STATE_RUNNING) {
        return false;
    }
    
    /* Decrement remaining time */
    if (remaining_seconds > 0) {
        remaining_seconds--;
        
        LOG_DBG("Timer tick: %u seconds remaining", remaining_seconds);
        
        /* Check if timer completed */
        if (remaining_seconds == 0) {
            current_state = TIMER_STATE_COMPLETED;
            
            LOG_INF("Timer completed!");
            
            if (state_change_callback != NULL) {
                state_change_callback(current_state);
            }
            
            return true;
        }
    }
    
    return false;
}

bool timer_sm_is_completed(void)
{
    return current_state == TIMER_STATE_COMPLETED;
}

bool timer_sm_is_active(void)
{
    return (current_state == TIMER_STATE_RUNNING) || 
           (current_state == TIMER_STATE_PAUSED);
}

const char *timer_sm_state_to_string(enum timer_state state)
{
    switch (state) {
        case TIMER_STATE_SETTING:
            return "SETTING";
        case TIMER_STATE_RUNNING:
            return "RUNNING";
        case TIMER_STATE_PAUSED:
            return "PAUSED";
        case TIMER_STATE_COMPLETED:
            return "COMPLETED";
        default:
            return "UNKNOWN";
    }
}
