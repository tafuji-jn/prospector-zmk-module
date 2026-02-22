/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USB HID forwarder device
 *
 * Registers the USB HID device with a fixed 6KRO keyboard + consumer control
 * report descriptor and enables the USB subsystem.
 *
 * @return 0 on success, negative error code on failure
 */
int usb_hid_forwarder_init(void);

/**
 * @brief Send an HID report to the host PC via USB
 *
 * @param report Pointer to the HID report data
 * @param len    Length of the report in bytes
 * @return 0 on success, negative error code on failure
 */
int usb_hid_forwarder_send(const uint8_t *report, uint16_t len);

/**
 * @brief Check if USB HID forwarder is ready to send reports
 *
 * @return true if the USB endpoint is configured and ready
 */
bool usb_hid_forwarder_is_ready(void);

#ifdef __cplusplus
}
#endif
