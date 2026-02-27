/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/bluetooth/uuid.h>
#include <zmk/status_advertisement.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Prospector Status GATT Service UUIDs (128-bit)
 *
 * Service:        e5a10001-1523-abcd-0001-0123456789ab
 * Characteristic: e5a10002-1523-abcd-0001-0123456789ab
 */

#define BT_UUID_PROSPECTOR_STATUS_SVC_VAL \
    BT_UUID_128_ENCODE(0xe5a10001, 0x1523, 0xabcd, 0x0001, 0x0123456789ab)

#define BT_UUID_PROSPECTOR_STATUS_CHR_VAL \
    BT_UUID_128_ENCODE(0xe5a10002, 0x1523, 0xabcd, 0x0001, 0x0123456789ab)

#define BT_UUID_PROSPECTOR_STATUS_SVC BT_UUID_DECLARE_128(BT_UUID_PROSPECTOR_STATUS_SVC_VAL)
#define BT_UUID_PROSPECTOR_STATUS_CHR BT_UUID_DECLARE_128(BT_UUID_PROSPECTOR_STATUS_CHR_VAL)

#ifdef __cplusplus
}
#endif
