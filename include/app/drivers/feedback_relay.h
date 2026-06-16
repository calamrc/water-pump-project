/**
 * @file feedback_relay.h
 * @brief Public API for feedback relay driver (auditory click feedback)
 *
 * Provides simple ON/OFF/PULSE/CLICK control for a relay whose coil
 * click is used as auditory countdown feedback during the last 10
 * seconds of the timer.
 */

#ifndef APP_DRIVERS_FEEDBACK_RELAY_H_
#define APP_DRIVERS_FEEDBACK_RELAY_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Feedback Relay Driver API
 */
struct feedback_relay_driver_api {
    int (*on)(const struct device *dev);
    int (*off)(const struct device *dev);
    int (*pulse)(const struct device *dev, uint32_t duration_ms);
    int (*click)(const struct device *dev);
    int (*get_state)(const struct device *dev, bool *is_on);
};

/* Convenience macros for API access (matches pump_controller style) */
#define feedback_relay_on(dev) \
    ((const struct feedback_relay_driver_api *)((dev)->api))->on(dev)

#define feedback_relay_off(dev) \
    ((const struct feedback_relay_driver_api *)((dev)->api))->off(dev)

#define feedback_relay_pulse(dev, duration_ms) \
    ((const struct feedback_relay_driver_api *)((dev)->api))->pulse(dev, duration_ms)

#define feedback_relay_click(dev) \
    ((const struct feedback_relay_driver_api *)((dev)->api))->click(dev)

#define feedback_relay_get_state(dev, is_on) \
    ((const struct feedback_relay_driver_api *)((dev)->api))->get_state(dev, is_on)

/**
 * @brief Get feedback relay device by node label
 */
#define FEEDBACK_RELAY_DT_GET(node_id) DEVICE_DT_GET(node_id)

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVERS_FEEDBACK_RELAY_H_ */
