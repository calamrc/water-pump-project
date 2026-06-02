/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVICES_UI_SERVICE_H_
#define SERVICES_UI_SERVICE_H_

#include <zephyr/kernel.h>

/** Start the UI Service (subscribes to timer/pump, drives display + feedback) */
int ui_service_start(void);

#endif /* SERVICES_UI_SERVICE_H_ */
