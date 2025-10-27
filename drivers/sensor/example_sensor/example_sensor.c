/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_example_sensor

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(example_sensor, CONFIG_SENSOR_LOG_LEVEL);

struct example_sensor_data {
	uint32_t pulse_count;
	uint32_t last_pulse_count;
	uint32_t flow_rate;  // pulses per second * 100 for precision
	int64_t last_measurement_time;
	struct gpio_callback callback;
};

struct example_sensor_config {
	struct gpio_dt_spec input;
};



// GPIO interrupt callback for pulse counting
static void pulse_isr_handler(const struct device *port, struct gpio_callback *cb,
			      gpio_port_pins_t pins)
{
	struct example_sensor_data *data = CONTAINER_OF(cb, struct example_sensor_data, callback);

	data->pulse_count++;
}

static int example_sensor_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	struct example_sensor_data *data = dev->data;

	if (chan != SENSOR_CHAN_FLOW) {
		return -ENOTSUP;
	}

	int64_t current_time = k_uptime_get();
	int64_t time_diff_ms = current_time - data->last_measurement_time;

	if (time_diff_ms >= 1000) { // At least 1 second measurement window
		uint32_t pulse_diff = data->pulse_count - data->last_pulse_count;

		// Calculate flow rate in pulses per second (multiplied by 100 for precision)
		data->flow_rate = (pulse_diff * 100 * 1000) / time_diff_ms;

		// Update last measurement data
		data->last_pulse_count = data->pulse_count;
		data->last_measurement_time = current_time;

		LOG_INF("Flow rate: %d.%02d pulses/sec (%d pulses in %lld ms)",
			data->flow_rate / 100, data->flow_rate % 100,
			pulse_diff, time_diff_ms);
	}

	return 0;
}

static int example_sensor_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct example_sensor_data *data = dev->data;

	if (chan != SENSOR_CHAN_FLOW) {
		return -ENOTSUP;
	}

	// Return flow rate in pulses per second (val1.val = integer part, val1.val2 = decimal part)
	val->val1 = data->flow_rate / 100;
	val->val2 = (data->flow_rate % 100) * 10000;

	return 0;
}

static DEVICE_API(sensor, example_sensor_api) = {
	.sample_fetch = &example_sensor_sample_fetch,
	.channel_get = &example_sensor_channel_get,
};

static int example_sensor_init(const struct device *dev)
{
	const struct example_sensor_config *config = dev->config;
	struct example_sensor_data *data = dev->data;

	int ret;

	if (!device_is_ready(config->input.port)) {
		LOG_ERR("Input GPIO not ready");
		return -ENODEV;
	}

	// Configure GPIO for interrupt on falling edge (pulse detection)
	ret = gpio_pin_configure_dt(&config->input, GPIO_INPUT | GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		LOG_ERR("Could not configure input GPIO for interrupts (%d)", ret);
		return ret;
	}

	// Initialize GPIO callback
	gpio_init_callback(&data->callback, pulse_isr_handler, BIT(config->input.pin));

	ret = gpio_add_callback(config->input.port, &data->callback);
	if (ret < 0) {
		LOG_ERR("Could not add GPIO callback (%d)", ret);
		return ret;
	}

	// Initialize measurement data
	data->pulse_count = 0;
	data->last_pulse_count = 0;
	data->flow_rate = 0;
	data->last_measurement_time = k_uptime_get();

	LOG_INF("Flow sensor initialized on GPIO %d", config->input.pin);

	return 0;
}

#define EXAMPLE_SENSOR_INIT(i)						       \
	static struct example_sensor_data example_sensor_data_##i;	       \
									       \
	static const struct example_sensor_config example_sensor_config_##i = {\
		.input = GPIO_DT_SPEC_INST_GET(i, input_gpios),		       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(i, example_sensor_init, NULL,		       \
			      &example_sensor_data_##i,			       \
			      &example_sensor_config_##i, POST_KERNEL,	       \
			      CONFIG_SENSOR_INIT_PRIORITY, &example_sensor_api);

DT_INST_FOREACH_STATUS_OKAY(EXAMPLE_SENSOR_INIT)
