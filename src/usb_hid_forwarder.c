/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>

#include <zmk/usb_hid_forwarder.h>

LOG_MODULE_REGISTER(usb_hid_fwd, CONFIG_ZMK_LOG_LEVEL);

/*
 * Fixed HID Report Descriptor: 6KRO Keyboard + Consumer Control
 *
 * This matches the standard ZMK HID descriptor and covers 95%+ of use cases.
 * We must register this at boot time before knowing the actual keyboard's descriptor.
 *
 * Report ID 1: Keyboard (8-byte boot protocol compatible)
 *   Byte 0: Modifier keys (bitmap)
 *   Byte 1: Reserved
 *   Bytes 2-7: Key codes (up to 6 simultaneous keys)
 *
 * Report ID 2: Consumer Control (2-byte usage code)
 */
static const uint8_t hid_report_desc[] = {
    /* Keyboard */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x01,       /*   Report ID (1) */
    /* Modifier keys */
    0x05, 0x07,       /*   Usage Page (Keyboard/Keypad) */
    0x19, 0xE0,       /*   Usage Minimum (Left Control) */
    0x29, 0xE7,       /*   Usage Maximum (Right GUI) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x01,       /*   Logical Maximum (1) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x08,       /*   Report Count (8) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
    /* Reserved byte */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x08,       /*   Report Size (8) */
    0x81, 0x01,       /*   Input (Constant) */
    /* LEDs (output report for Caps Lock etc.) */
    0x05, 0x08,       /*   Usage Page (LEDs) */
    0x19, 0x01,       /*   Usage Minimum (Num Lock) */
    0x29, 0x05,       /*   Usage Maximum (Kana) */
    0x95, 0x05,       /*   Report Count (5) */
    0x75, 0x01,       /*   Report Size (1) */
    0x91, 0x02,       /*   Output (Data, Variable, Absolute) */
    /* LED padding */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x03,       /*   Report Size (3) */
    0x91, 0x01,       /*   Output (Constant) */
    /* Key array (6KRO) */
    0x05, 0x07,       /*   Usage Page (Keyboard/Keypad) */
    0x19, 0x00,       /*   Usage Minimum (0) */
    0x29, 0xFF,       /*   Usage Maximum (255) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
    0x95, 0x06,       /*   Report Count (6) */
    0x75, 0x08,       /*   Report Size (8) */
    0x81, 0x00,       /*   Input (Data, Array) */
    0xC0,             /* End Collection */

    /* Consumer Control */
    0x05, 0x0C,       /* Usage Page (Consumer) */
    0x09, 0x01,       /* Usage (Consumer Control) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x02,       /*   Report ID (2) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x03, /*   Logical Maximum (1023) */
    0x19, 0x00,       /*   Usage Minimum (0) */
    0x2A, 0xFF, 0x03, /*   Usage Maximum (1023) */
    0x75, 0x10,       /*   Report Size (16) */
    0x95, 0x01,       /*   Report Count (1) */
    0x81, 0x00,       /*   Input (Data, Array) */
    0xC0,             /* End Collection */
};

static const struct device *hid_dev;
static struct k_sem usb_sem;
static bool usb_ready;

/* Callback when USB HID interrupt endpoint write completes */
static void int_in_ready_cb(const struct device *dev)
{
    k_sem_give(&usb_sem);
}

/* Callback when host sends LED output report (Caps Lock etc.) */
static int set_report_cb(const struct device *dev,
                          struct usb_setup_packet *setup,
                          int32_t *len, uint8_t **data)
{
    /* Report ID 1, Output report = LED indicators */
    if (*len > 0) {
        LOG_DBG("LED output report: 0x%02x", (*data)[0]);
        /* Could forward LED state back to keyboard via BLE if desired */
    }
    return 0;
}

static const struct hid_ops usb_hid_ops = {
    .int_in_ready = int_in_ready_cb,
    .set_report = set_report_cb,
};

/* USB device status callback */
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB HID configured");
        usb_ready = true;
        break;
    case USB_DC_DISCONNECTED:
        LOG_INF("USB HID disconnected");
        usb_ready = false;
        break;
    case USB_DC_SUSPEND:
        LOG_DBG("USB HID suspended");
        break;
    case USB_DC_RESUME:
        LOG_DBG("USB HID resumed");
        break;
    default:
        break;
    }
}

int usb_hid_forwarder_init(void)
{
    int ret;

    k_sem_init(&usb_sem, 1, 1);

    hid_dev = device_get_binding("HID_0");
    if (hid_dev == NULL) {
        LOG_ERR("Cannot find USB HID device");
        return -ENODEV;
    }

    usb_hid_register_device(hid_dev, hid_report_desc,
                            sizeof(hid_report_desc), &usb_hid_ops);

    ret = usb_hid_init(hid_dev);
    if (ret) {
        LOG_ERR("USB HID init failed: %d", ret);
        return ret;
    }

    ret = usb_enable(usb_status_cb);
    if (ret == -EALREADY) {
        /* USB was initialized at boot (CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT).
         * Our status callback was NOT registered, so assume USB is ready.
         * The host has had time to enumerate during the boot delay. */
        usb_ready = true;
        printk("*** USB_HID: USB already enabled, assuming ready ***\n");
    } else if (ret) {
        LOG_ERR("USB enable failed: %d", ret);
        return ret;
    }

    printk("*** USB_HID: forwarder initialized (ready=%d) ***\n", usb_ready);
    return 0;
}

int usb_hid_forwarder_send(const uint8_t *report, uint16_t len)
{
    if (!usb_ready || hid_dev == NULL) {
        return -ENODEV;
    }

    /* Wait for previous write to complete (100ms timeout) */
    if (k_sem_take(&usb_sem, K_MSEC(100)) != 0) {
        LOG_WRN("USB HID write timeout");
        return -ETIMEDOUT;
    }

    int ret = hid_int_ep_write(hid_dev, report, len, NULL);
    if (ret) {
        LOG_ERR("USB HID write failed: %d", ret);
        k_sem_give(&usb_sem);
        return ret;
    }

    return 0;
}

bool usb_hid_forwarder_is_ready(void)
{
    return usb_ready && hid_dev != NULL;
}

static int usb_hid_forwarder_sys_init(void)
{
    return usb_hid_forwarder_init();
}

SYS_INIT(usb_hid_forwarder_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
