/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/net_buf.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Called by status_scanner scan_callback when a connectable
 *        advertisement is received.
 *
 * @param addr  BLE address of the advertising device
 * @param rssi  Signal strength
 * @param type  Advertisement type
 * @param buf   Raw advertisement data buffer
 */
void hid_central_on_scan_result(const bt_addr_le_t *addr, int8_t rssi,
                                uint8_t type, struct net_buf_simple *buf);

/**
 * @brief Check if the HID central is currently connected to a keyboard.
 *
 * @return true if connected
 */
bool hid_central_is_connected(void);

#ifdef __cplusplus
}
#endif
