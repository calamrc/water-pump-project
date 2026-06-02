#include "host_shims.h"
#include <string.h>

/* Global simulated hardware state */
host_pump_state_t g_host_pump = {0};
host_feedback_state_t g_host_feedback = {0};

/* --- Pump Controller Shims --- */

int host_pump_controller_turn_on(void *dev, int64_t plateau_period_us) {
  (void)dev;
  if (!g_host_pump.is_on) {
    g_host_pump.is_on = true;
    g_host_pump.last_on_time = 0;
    HOST_LOG("Pump Driver: TURNED ON (plateau=%lld us)",
             (long long)plateau_period_us);
  }
  return 0;
}
/* NOTE (PR4 + retained for compat): host_sim (20s contract-driven path) uses
 * direct pump SM + demand (not these shims). Shims + g_host_pump etc. are kept
 * for other host harnesses/tests that drive via pump_controller shims. */

int host_pump_controller_turn_off(void *dev) {
  (void)dev;
  if (g_host_pump.is_on) {
    g_host_pump.is_on = false;
    HOST_LOG("Pump Driver: TURNED OFF");
  }
  return 0;
}

bool host_pump_controller_is_on(void *dev) {
  (void)dev;
  return g_host_pump.is_on;
}

int host_pump_controller_emergency_stop(void *dev) {
  (void)dev;
  if (g_host_pump.is_on) {
    g_host_pump.is_on = false;
    HOST_LOG("Pump Driver: EMERGENCY STOP");
  }
  return 0;
}

/* --- Feedback Relay Shims --- */

int host_feedback_relay_click(void *dev) {
  (void)dev;
  g_host_feedback.click_count++;
  g_host_feedback.total_pulse_ms += 50;
  HOST_LOG("Feedback Relay: CLICK (total clicks=%d)",
           g_host_feedback.click_count);
  return 0;
}

int host_feedback_relay_pulse(void *dev, uint32_t duration_ms) {
  (void)dev;
  g_host_feedback.total_pulse_ms += duration_ms;
  HOST_LOG("Feedback Relay: PULSE %u ms", duration_ms);
  return 0;
}
