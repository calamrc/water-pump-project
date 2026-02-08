/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_PUMP_PUMP_CONTROLLER_H_
#define DRIVERS_PUMP_PUMP_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/**
 * @brief Pump operational states (internal driver definition)
 */
typedef enum {
    PUMP_STATE_OFF = 0,
    PUMP_STATE_STARTING,
    PUMP_STATE_RUNNING,
    PUMP_STATE_TIMEOUT,
    PUMP_STATE_ERROR,
    PUMP_STATE_MAINTENANCE,
    PUMP_STATE_COUNT
} pump_state_t;

/**
 * @brief State transition events
 */
typedef enum {
    PUMP_EVENT_PLATEAU_DETECTED = 0,
    PUMP_EVENT_TIMEOUT,
    PUMP_EVENT_SAFETY_TIMEOUT,
    PUMP_EVENT_ERROR_DETECTED,
    PUMP_EVENT_MAINTENANCE_ENTER,
    PUMP_EVENT_MAINTENANCE_EXIT,
    PUMP_EVENT_RESET,
    PUMP_EVENT_COUNT
} pump_event_t;

/**
 * @brief Pump controller device configuration
 */
struct pump_config {
    struct gpio_dt_spec relay;
    int64_t safety_timeout_min;
};

/**
 * @brief Pump controller device data
 */
struct pump_data {
    pump_state_t current_state;
    pump_state_t previous_state;
    int64_t initial_plateau_period;
    int64_t latest_plateau_period;
    bool safety_systems_active;
    k_timepoint_t state_entry_time;
    struct k_timer safety_timer;
    struct k_mutex mutex;
};

/**
 * @brief Pump controller state information (internal version)
 */
struct pump_state_info {
    pump_state_t current_state;
    pump_state_t previous_state;
    int64_t initial_plateau_period;
    int64_t latest_plateau_period;
    bool safety_systems_active;
    k_timepoint_t state_entry_time;
};

/**
 * @brief Pump Controller Driver API (internal declaration)
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
 * @brief Forward declarations for driver functions (implemented in pump_controller.c)
 */

#endif /* DRIVERS_PUMP_PUMP_CONTROLLER_H_ */