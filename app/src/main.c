/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_version.h>

/* Flow handling is exclusively done by the new FlowSensorService. */
#include "services/flow_sensor_service.h"
#include "services/pump_service.h"
#include "services/timer_service.h"
#include "services/ui_service.h"
#include "services/safety_service.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("ZEPHYR WATER PUMP - SERVICE ARCHITECTURE");
    LOG_INF("Version: %s", APP_VERSION_STRING);
    LOG_INF("========================================");

    /* ========================================================================
     * NEW ZBUS ARCHITECTURE (Phase 2+)
     * ======================================================================== */
    int ret = flow_sensor_service_start();
    if (ret < 0) {
        LOG_ERR("Failed to start FlowSensorService (%d)", ret);
        return ret;
    }
    LOG_INF("FlowSensorService started — publishing on flow_sample_chan");

    /* All services are now zbus-based. */

    /* Other new zbus services (stubs being filled in subsequent phases) */
    (void)pump_service_start();
    (void)timer_service_start();
    (void)ui_service_start();
    (void)safety_service_start();

    /* Future services will start here in later phases */

    while (1) {
        k_sleep(K_SECONDS(60));
    }

    return 0;
}
