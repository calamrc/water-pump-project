/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SERVICES_TIMER_SERVICE_H_
#define SERVICES_TIMER_SERVICE_H_

#include <zephyr/kernel.h>

/** Start the Timer Service (owns countdown SM, publishes timer_state_chan) */
int timer_service_start(void);

#endif /* SERVICES_TIMER_SERVICE_H_ */
