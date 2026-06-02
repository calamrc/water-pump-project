/*
 * Host simulation shims for drivers and Zephyr primitives.
 * These allow us to compile and run selected firmware logic on macOS/Linux
 * without the full Zephyr kernel.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* --- Pump Controller Driver Shims (host version) --- */

typedef struct {
  bool is_on;
  int64_t last_on_time;
} host_pump_state_t;

extern host_pump_state_t g_host_pump;

/* Host versions don't need the real device pointer */
int host_pump_controller_turn_on(void *dev, int64_t plateau_period_us);
int host_pump_controller_turn_off(void *dev);
bool host_pump_controller_is_on(void *dev);
int host_pump_controller_emergency_stop(void *dev);

/* --- Feedback Relay Shims (host version) --- */

typedef struct {
  bool is_on;
  int click_count;
  int total_pulse_ms;
} host_feedback_state_t;

extern host_feedback_state_t g_host_feedback;

int host_feedback_relay_click(void *dev);
int host_feedback_relay_pulse(void *dev, uint32_t duration_ms);

/* --- Minimal logging helpers --- */

#define HOST_LOG(fmt, ...) printf("[HOST] " fmt "\n", ##__VA_ARGS__)

/* Note: flow_event_chan simulation (lightweight for wd asserts in 20s host_sim)
 * is done inline via contract_res in host_sim.c, not via these driver shims.
 * Shims cover pump_controller/feedback for other hosts/tests. PR4. */
