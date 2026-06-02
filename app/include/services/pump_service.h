/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVICES_PUMP_SERVICE_H_
#define SERVICES_PUMP_SERVICE_H_

#include <zephyr/kernel.h>

/** Start the Pump Control Service (owns pump_controller driver, subscribes to flow + cmds) */
int pump_service_start(void);

#endif /* SERVICES_PUMP_SERVICE_H_ */
