/*
 * Copyright (c) 2025 Ramon Cristopher Calam
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLOW_SERVICE_H
#define FLOW_SERVICE_H

#include <zephyr/zbus/zbus.h>
#include "fixed_math.h"

int flow_service_start(void);

/**
 * @brief Flow sample message published by Flow Service
 */
struct flow_sample {
    fixed_t rate;          /* Flow rate in fixed-point (L/min) */
    uint64_t timestamp;    /* Timestamp in milliseconds */
    uint32_t sequence;     /* Sequence number */
};

/* Flow data channel owned by Flow Service */
ZBUS_CHAN_DECLARE(flow_data_chan);

#endif /* FLOW_SERVICE_H */
