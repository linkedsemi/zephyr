/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "plat_ipmi.h"

#include <stdio.h>
#include <stdlib.h>
// #include <logging/log.h>
// #include "libutil.h"
// #include "ipmi.h"
// #include "fru.h"
// #include "eeprom.h"
// #include "power_status.h"
// #include "sensor.h"
// #include "plat_fru.h"
// #include "plat_class.h"
// #include "plat_hook.h"
// #include "plat_ipmb.h"
// #include "plat_dimm.h"
// #include "plat_sensor_table.h"
// #include "pmbus.h"
// #include "oem_1s_handler.h"

// #include "pldm.h"
// #include "plat_mctp.h"

// #include "pldm.h"
// #include "plat_mctp.h"

// #define LOG_LEVEL LOG_LEVEL_DBG
// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(ipmi);

bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
{
    if (netfn == NETFN_APP_REQ) {
        if (cmd == CMD_APP_GET_DEVICE_ID) {
            return true;
        }
    }

    return false;
}
