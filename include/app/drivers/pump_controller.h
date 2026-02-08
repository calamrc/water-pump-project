#ifndef APP_DRIVERS_PUMP_CONTROLLER_H_
#define APP_DRIVERS_PUMP_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pump operational states
 */
enum pump_state {
    PUMP_STATE_OFF = 0,
    PUMP_STATE_STARTING,
    PUMP_STATE_RUNNING,
    PUMP_STATE_TIMEOUT,
    PUMP_STATE_ERROR,
    PUMP_STATE_MAINTENANCE,
    PUMP_STATE_COUNT
};

/**
 * @brief Pump controller configuration
 */
struct pump_config {
    int64_t safety_timeout_min;
};

/**
 * @brief Pump controller state information
 */
struct pump_state_info {
    enum pump_state current_state;
    enum pump_state previous_state;
    int64_t initial_plateau_period;
    int64_t latest_plateau_period;
    bool safety_systems_active;
    k_timepoint_t state_entry_time;
};

/**
 * @brief Pump Controller Driver API
 */
struct pump_controller_driver_api {
    int (*turn_on)(const struct device *dev, int64_t plateau_period_us);
    int (*turn_off)(const struct device *dev);
    bool (*is_on)(const struct device *dev);
    void (*update_plateau_period)(const struct device *dev, int64_t period_us);
    bool (*safety_check)(const struct device *dev);
    void (*reset)(const struct device *dev);
    int (*emergency_stop)(const struct device *dev);
    int (*get_state)(const struct device *dev, struct pump_state_info *state);
    int (*set_config)(const struct device *dev, const struct pump_config *config);
};

/**
 * @brief Convenience macros for API access
 */
#define pump_controller_turn_on(dev, plateau_period) \
    ((const struct pump_controller_driver_api *)((dev)->api))->turn_on(dev, plateau_period)

#define pump_controller_turn_off(dev) \
    ((const struct pump_controller_driver_api *)((dev)->api))->turn_off(dev)

#define pump_controller_is_on(dev) \
    ((const struct pump_controller_driver_api *)((dev)->api))->is_on(dev)

#define pump_controller_update_plateau_period(dev, period) \
    ((const struct pump_controller_driver_api *)((dev)->api))->update_plateau_period(dev, period)

#define pump_controller_safety_check(dev) \
    ((const struct pump_controller_driver_api *)((dev)->api))->safety_check(dev)

#define pump_controller_reset(dev) \
    ((const struct pump_controller_driver_api *)((dev)->api))->reset(dev)

#define pump_controller_emergency_stop(dev) \
    ((const struct pump_controller_driver_api *)((dev)->api))->emergency_stop(dev)

#define pump_controller_get_state(dev, state) \
    ((const struct pump_controller_driver_api *)((dev)->api))->get_state(dev, state)

#define pump_controller_set_config(dev, config) \
    ((const struct pump_controller_driver_api *)((dev)->api))->set_config(dev, config)

/**
 * @brief Get pump controller device by node label
 */
#define PUMP_CONTROLLER_DT_GET(node_id) DEVICE_DT_GET(node_id)

/**
 * @brief Check if pump controller device is ready
 */
#define PUMP_CONTROLLER_DT_CHECK(node_id) DEVICE_DT_GET(node_id) != NULL && device_is_ready(DEVICE_DT_GET(node_id))

#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVERS_PUMP_CONTROLLER_H_ */