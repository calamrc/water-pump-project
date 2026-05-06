/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_version.h>

#include "services/flow_service.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("ZEPHYR WATER PUMP - SERVICE ARCHITECTURE");
    LOG_INF("Version: %s", APP_VERSION_STRING);
    LOG_INF("========================================");

    /* Start Flow Service */
    int ret = flow_service_start();
    if (ret < 0) {
        LOG_ERR("Failed to start Flow Service (%d)", ret);
        return ret;
    }

    LOG_INF("Flow Service started - listening for flow pulses...");

    /* Future services will start here in later phases */

    while (1) {
        k_sleep(K_SECONDS(60));
    }

    return 0;
}
