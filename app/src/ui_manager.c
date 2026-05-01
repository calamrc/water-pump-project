/**
 * @file ui_manager.c
 * @brief UI Manager implementation
 *
 * Main UI thread that coordinates display, input, and timer state machine.
 */

#include "ui_manager.h"
#include "display/display_manager.h"
#include "input/input_manager.h"
#include "timer/timer_state_machine.h"
#include "thread_comm.h"
#include <app/drivers/feedback_relay.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ui_manager, CONFIG_APP_LOG_LEVEL);

/* UI refresh rate (Hz) - 10Hz for responsive input */
#define UI_REFRESH_RATE_HZ 10
#define UI_REFRESH_INTERVAL_MS (1000 / UI_REFRESH_RATE_HZ)

/* Timer update interval - 1 second */
#define TIMER_UPDATE_INTERVAL_MS 1000

/* Flash interval for completed state */
#define FLASH_INTERVAL_MS 500

int ui_manager_init(void)
{
    LOG_INF("Initializing UI manager...");

    /* Initialize display manager */
    int ret = display_manager_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize display manager: %d", ret);
        return ret;
    }

    /* Initialize input manager */
    ret = input_manager_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize input manager: %d", ret);
        return ret;
    }

    /* Initialize timer state machine */
    timer_sm_init();

    LOG_INF("UI manager initialized successfully");
    return 0;
}

void ui_manager_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("========================================");
    LOG_INF("UI THREAD STARTED - HEARTBEAT EVERY 1s");
    LOG_INF("========================================");

    /* Timing variables */
    int64_t last_timer_update = k_uptime_get();
    int64_t last_flash_toggle = 0;
    int64_t last_health_update = 0;
    int64_t last_heartbeat = 0;
    int64_t loop_count = 0;
    bool flash_state = false;

    /* Display state */
    uint8_t minutes = 1, seconds = 0;
    
    /* Get initial time and display */
    timer_sm_get_time(&minutes, &seconds);
    display_manager_show_time(minutes, seconds, false);
    display_manager_update();

    LOG_INF("UI ready - Timer at %02u:%02u", minutes, seconds);

    while (true) {
        int64_t now = k_uptime_get();
        bool display_needs_update = false;
        
        loop_count++;
        
        /* HEARTBEAT - Print every 1 second to confirm thread is alive */
        if ((now - last_heartbeat) >= 1000) {
            last_heartbeat = now;
            enum timer_state state = timer_sm_get_state();
            LOG_INF("[HEARTBEAT] Loop=%lld, State=%s, Time=%02u:%02u", 
                    loop_count,
                    timer_sm_state_to_string(state),
                    minutes, seconds);
        }
        
        /* Get current timer state */
        enum timer_state current_state = timer_sm_get_state();
        
        /* Process input events - read all pending events */
        struct ui_input_event event;
        int events_processed = 0;
        
        while (input_manager_get_event(&event) && events_processed < 10) {
            events_processed++;
            
            if (event.encoder_moved && event.encoder_delta != 0) {
                /* Handle encoder rotation */
                int32_t delta = event.encoder_delta * TIMER_STEP_SECONDS;
                LOG_INF("Encoder: %d steps -> %d sec", 
                        event.encoder_delta, delta);
                timer_sm_adjust_time(delta);
                display_needs_update = true;
            }

            if (event.button_press != BUTTON_PRESS_NONE) {
                /* Get fresh state */
                current_state = timer_sm_get_state();
                
                LOG_INF("Button: %s (state=%s)", 
                        event.button_press == BUTTON_PRESS_SHORT ? "SHORT" : "LONG",
                        timer_sm_state_to_string(current_state));

                switch (current_state) {
                    case TIMER_STATE_SETTING:
                        if (event.button_press == BUTTON_PRESS_SHORT) {
                            LOG_INF("Action: START");
                            timer_sm_start();
                            last_timer_update = now;
                            display_needs_update = true;
                        } else if (event.button_press == BUTTON_PRESS_LONG) {
                            /* Manual test: fire 1-second pulse on feedback relay */
                            const struct device *feedback = 
                                FEEDBACK_RELAY_DT_GET(DT_NODELABEL(feedback_relay));
                            if (device_is_ready(feedback)) {
                                LOG_INF("Manual test pulse (1s) on feedback relay");
                                int ret = feedback_relay_pulse(feedback, 1000);
                                if (ret < 0) {
                                    LOG_ERR("Manual feedback pulse failed (%d)", ret);
                                }
                            } else {
                                LOG_ERR("Feedback relay device not ready for manual test");
                            }

                            LOG_INF("Action: RESET");
                            timer_sm_reset();
                            display_needs_update = true;
                        }
                        break;

                    case TIMER_STATE_RUNNING:
                        if (event.button_press == BUTTON_PRESS_SHORT) {
                            LOG_INF("Action: PAUSE");
                            timer_sm_pause();
                            display_needs_update = true;
                        } else if (event.button_press == BUTTON_PRESS_LONG) {
                            LOG_INF("Action: STOP");
                            timer_sm_reset();
                            display_needs_update = true;
                        }
                        break;

                    case TIMER_STATE_PAUSED:
                        if (event.button_press == BUTTON_PRESS_SHORT) {
                            LOG_INF("Action: RESUME");
                            timer_sm_resume();
                            last_timer_update = now;
                            display_needs_update = true;
                        } else if (event.button_press == BUTTON_PRESS_LONG) {
                            LOG_INF("Action: STOP");
                            timer_sm_reset();
                            display_needs_update = true;
                        }
                        break;

                    case TIMER_STATE_COMPLETED:
                        LOG_INF("Action: RESET from complete");
                        timer_sm_reset();
                        display_needs_update = true;
                        break;

                    default:
                        break;
                }
                
                /* Re-get state after action */
                current_state = timer_sm_get_state();
            }
        }
        
        /* Safety limit for event processing */
        if (events_processed >= 10) {
            LOG_WRN("Too many events processed in one iteration");
        }

        /* Update timer every second */
        if ((now - last_timer_update) >= TIMER_UPDATE_INTERVAL_MS) {
            /* Calculate how many seconds passed (could be more than 1 if we were blocked) */
            last_timer_update = now;
            
            /* Update timer state machine */
            bool state_changed = timer_sm_update();
            
            if (state_changed || current_state == TIMER_STATE_RUNNING) {
                display_needs_update = true;
                timer_sm_get_time(&minutes, &seconds);
                LOG_DBG("Timer: %02u:%02u (state=%s)", 
                        minutes, seconds,
                        timer_sm_state_to_string(timer_sm_get_state()));
            }

            /* Auditory feedback: pulse relay for last 10 seconds (async) */
            if (timer_sm_get_state() == TIMER_STATE_RUNNING) {
                uint32_t remaining = timer_sm_get_remaining_seconds();
                if (remaining <= 10 && remaining > 0) {
                    const struct device *feedback = 
                        FEEDBACK_RELAY_DT_GET(DT_NODELABEL(feedback_relay));
                    if (device_is_ready(feedback)) {
                        int ret = feedback_relay_pulse(feedback, 500);
                        if (ret < 0) {
                            LOG_WRN("Feedback relay pulse failed (%d)", ret);
                        }
                    }
                }
            }
        }

        /* Handle flashing for completed state */
        if (current_state == TIMER_STATE_COMPLETED) {
            if ((now - last_flash_toggle) >= FLASH_INTERVAL_MS) {
                last_flash_toggle = now;
                flash_state = !flash_state;
                display_needs_update = true;
            }
        } else {
            flash_state = false;
        }

        /* Update display if needed */
        if (display_needs_update) {
            timer_sm_get_time(&minutes, &seconds);
            display_manager_show_time(minutes, seconds, 
                                      (current_state == TIMER_STATE_COMPLETED) && flash_state);
            display_manager_update();
        }
        
        /* Update thread health every 1 second to prevent supervisor timeout */
        if ((now - last_health_update) >= 1000) {
            last_health_update = now;
            LOG_DBG("Updating thread health...");
            thread_health_update(k_current_get(), THREAD_HEALTH_OK, 0, 0);
        }

        /* Yield to other threads */
        k_sleep(K_MSEC(UI_REFRESH_INTERVAL_MS));
    }
}
