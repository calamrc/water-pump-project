/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVICES_FLOW_SENSOR_SERVICE_H_
#define SERVICES_FLOW_SENSOR_SERVICE_H_

#include <zephyr/kernel.h>

/**
 * @brief Start the Flow Sensor Service (owns YF-S201C driver + publishes flow_sample_chan)
 */
int flow_sensor_service_start(void);

#endif /* SERVICES_FLOW_SENSOR_SERVICE_H_ */
