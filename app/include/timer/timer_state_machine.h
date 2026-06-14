/**
 * @file timer_state_machine.h
 * @brief Countdown timer state machine
 *
 * Manages the countdown timer state and transitions.
 * Handles time setting, countdown, pause, and completion states.
 * Publishes state changes via Zbus.
 */

#ifndef TIMER_STATE_MACHINE_H_
#define TIMER_STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Timer state machine states */
enum timer_state {
	TIMER_STATE_SETTING = 0,
	TIMER_STATE_RUNNING,
	TIMER_STATE_PAUSED,
	TIMER_STATE_COMPLETED,
};

/* Timer configuration */
#define TIMER_MIN_SECONDS   10
#define TIMER_MAX_SECONDS   3600
#define TIMER_DEFAULT_SECONDS 60
#define TIMER_STEP_SECONDS  10

/**
 * @brief Initialize the timer state machine
 *
 * Sets initial state to SETTING with default duration.
 * Also initializes the Zbus publishing channel and 1-second tick timer.
 */
void timer_sm_init(void);

/**
 * @brief Start the 1-second tick timer
 *
 * Starts the periodic k_timer that drives countdown.
 * Called automatically by timer_sm_start(), but can be called
 * independently for testing.
 */
void timer_sm_start_tick(void);

/**
 * @brief Stop the 1-second tick timer
 *
 * Stops the periodic k_timer.
 * Called automatically by timer_sm_pause()/timer_sm_reset(),
 * but can be called independently.
 */
void timer_sm_stop_tick(void);

/**
 * @brief Get the current timer state
 *
 * @return Current state enum value
 */
enum timer_state timer_sm_get_state(void);

/**
 * @brief Get current timer time as minutes and seconds
 *
 * @param minutes Pointer to store minutes
 * @param seconds Pointer to store seconds (0-59)
 */
void timer_sm_get_time(uint8_t *minutes, uint8_t *seconds);

/**
 * @brief Get remaining time in seconds
 *
 * @return Remaining time in seconds
 */
uint32_t timer_sm_get_remaining_seconds(void);

/**
 * @brief Adjust timer duration (used in SETTING or PAUSED states)
 *
 * @param delta_seconds Seconds to add (positive) or subtract (negative)
 */
void timer_sm_adjust_time(int32_t delta_seconds);

/**
 * @brief Start the timer
 *
 * Transitions from SETTING to RUNNING and starts the 1-second tick.
 *
 * @return 0 on success, negative errno if cannot start
 */
int timer_sm_start(void);

/**
 * @brief Pause the timer
 *
 * Transitions from RUNNING to PAUSED and stops the tick.
 *
 * @return 0 on success, negative errno if cannot pause
 */
int timer_sm_pause(void);

/**
 * @brief Resume the timer
 *
 * Transitions from PAUSED to RUNNING and restarts the tick.
 *
 * @return 0 on success, negative errno if cannot resume
 */
int timer_sm_resume(void);

/**
 * @brief Stop and reset the timer
 *
 * Transitions to SETTING state with default duration and stops the tick.
 */
void timer_sm_reset(void);

/**
 * @brief Update the timer (called internally by k_timer tick)
 *
 * Decrements remaining time when in RUNNING state.
 * Automatically transitions to COMPLETED when time reaches 0.
 *
 * @return true if state changed (including to COMPLETED)
 */
bool timer_sm_update(void);

/**
 * @brief Check if timer is completed
 *
 * @return true if in COMPLETED state
 */
bool timer_sm_is_completed(void);

/**
 * @brief Check if timer is running or active
 *
 * @return true if in RUNNING or PAUSED state
 */
bool timer_sm_is_active(void);

/**
 * @brief Get string representation of state (for debugging)
 *
 * @return State name string
 */
const char *timer_sm_state_to_string(enum timer_state state);

#ifdef __cplusplus
}
#endif

#endif /* TIMER_STATE_MACHINE_H_ */