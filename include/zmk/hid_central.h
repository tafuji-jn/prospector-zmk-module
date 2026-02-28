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
 * @param name  Cached device name from scan response (NULL if unknown)
 */
void hid_central_on_scan_result(const bt_addr_le_t *addr, int8_t rssi,
                                uint8_t type, struct net_buf_simple *buf,
                                const char *name);

/**
 * @brief Check if the HID central is currently connected to a keyboard.
 *
 * @return true if connected
 */
bool hid_central_is_connected(void);

/* ------------------------------------------------------------------ */
/* Multi-keyboard bonding support                                     */
/* ------------------------------------------------------------------ */

/** Bonded keyboard information */
struct bonded_keyboard_info {
    bt_addr_le_t addr;
    char name[32];
    bool valid;
};

/**
 * @brief Get list of bonded keyboards.
 *
 * @param out       Array to fill with bonded keyboard info
 * @param max_count Size of the output array
 * @return Number of bonded keyboards found
 */
int hid_central_get_bonded_keyboards(struct bonded_keyboard_info *out, int max_count);

/**
 * @brief Get the index of the currently active (connected/connecting) keyboard.
 *
 * @return Active index (0-based), or -1 if none selected
 */
int hid_central_get_active_index(void);

/**
 * @brief Switch connection to a different bonded keyboard.
 *
 * Disconnects current keyboard (if any), saves selection, and initiates
 * connection to the keyboard at the given index.
 *
 * @param index  Index into bonded keyboard array (from get_bonded_keyboards)
 * @return 0 on success, negative errno on failure
 */
int hid_central_select_keyboard(int index);

/**
 * @brief Enter pairing mode: scan for new HID keyboards.
 *
 * Disconnects any current connection and starts scanning for undiscovered
 * HID keyboards. Use hid_central_get_discovered_keyboards() to get results.
 *
 * @return 0 on success, negative errno on failure
 */
int hid_central_enter_pairing_mode(void);

/**
 * @brief Exit pairing mode and return to normal operation.
 *
 * If an active keyboard was selected before pairing mode, reconnect to it.
 */
void hid_central_exit_pairing_mode(void);

/**
 * @brief Check if currently in pairing mode.
 *
 * @return true if in pairing mode
 */
bool hid_central_is_pairing_mode(void);

/** Discovered (not yet paired) keyboard information */
struct discovered_keyboard_info {
    bt_addr_le_t addr;
    char name[32];
    int8_t rssi;
};

/**
 * @brief Get list of keyboards discovered during pairing mode scan.
 *
 * @param out       Array to fill with discovered keyboard info
 * @param max_count Size of the output array
 * @return Number of discovered keyboards
 */
int hid_central_get_discovered_keyboards(struct discovered_keyboard_info *out, int max_count);

/**
 * @brief Initiate pairing with a discovered keyboard.
 *
 * @param discovered_index  Index from get_discovered_keyboards result
 * @return 0 on success, negative errno on failure
 */
int hid_central_pair_with(int discovered_index);

/**
 * @brief Remove bond for a bonded keyboard.
 *
 * @param bonded_index  Index from get_bonded_keyboards result
 * @return 0 on success, negative errno on failure
 */
int hid_central_unpair(int bonded_index);

#ifdef __cplusplus
}
#endif
