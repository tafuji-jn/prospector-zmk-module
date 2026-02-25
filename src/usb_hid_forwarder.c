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
 * USB HID Report Descriptors — split into two interfaces.
 *
 * Windows requires separate USB HID interfaces for keyboard and mouse to
 * create distinct TLC (Top-Level Collection) device nodes.  A single
 * interface with multiple Application Collections works on macOS/Linux
 * but Windows silently ignores the mouse collection.
 *
 * HID_0: Keyboard (Report ID 1) + Consumer Control (Report ID 2)
 * HID_1: Mouse / Pointing Device (Report ID 3)
 */

/* HID_0: Keyboard + Consumer Control */
static const uint8_t hid_report_desc_kb[] = {
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

/* HID_1: Mouse / Pointing Device */
static const uint8_t hid_report_desc_mouse[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x02,       /* Usage (Mouse) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x03,       /*   Report ID (3) */
    0x09, 0x01,       /*   Usage (Pointer) */
    0xA1, 0x00,       /*   Collection (Physical) */
    /* Buttons (5 buttons) */
    0x05, 0x09,       /*     Usage Page (Button) */
    0x19, 0x01,       /*     Usage Minimum (Button 1) */
    0x29, 0x05,       /*     Usage Maximum (Button 5) */
    0x15, 0x00,       /*     Logical Minimum (0) */
    0x25, 0x01,       /*     Logical Maximum (1) */
    0x75, 0x01,       /*     Report Size (1) */
    0x95, 0x05,       /*     Report Count (5) */
    0x81, 0x02,       /*     Input (Data, Variable, Absolute) */
    /* Padding (3 bits) */
    0x75, 0x03,       /*     Report Size (3) */
    0x95, 0x01,       /*     Report Count (1) */
    0x81, 0x01,       /*     Input (Constant) */
    /* X, Y (16-bit signed relative) */
    0x05, 0x01,       /*     Usage Page (Generic Desktop) */
    0x09, 0x30,       /*     Usage (X) */
    0x09, 0x31,       /*     Usage (Y) */
    0x16, 0x01, 0x80, /*     Logical Minimum (-32767) */
    0x26, 0xFF, 0x7F, /*     Logical Maximum (32767) */
    0x75, 0x10,       /*     Report Size (16) */
    0x95, 0x02,       /*     Report Count (2) */
    0x81, 0x06,       /*     Input (Data, Variable, Relative) */
    /* Vertical scroll (16-bit signed) */
    0x09, 0x38,       /*     Usage (Wheel) */
    0x16, 0x01, 0x80, /*     Logical Minimum (-32767) */
    0x26, 0xFF, 0x7F, /*     Logical Maximum (32767) */
    0x75, 0x10,       /*     Report Size (16) */
    0x95, 0x01,       /*     Report Count (1) */
    0x81, 0x06,       /*     Input (Data, Variable, Relative) */
    /* Horizontal scroll (16-bit signed) */
    0x05, 0x0C,       /*     Usage Page (Consumer) */
    0x0A, 0x38, 0x02, /*     Usage (AC Pan) */
    0x16, 0x01, 0x80, /*     Logical Minimum (-32767) */
    0x26, 0xFF, 0x7F, /*     Logical Maximum (32767) */
    0x75, 0x10,       /*     Report Size (16) */
    0x95, 0x01,       /*     Report Count (1) */
    0x81, 0x06,       /*     Input (Data, Variable, Relative) */
    0xC0,             /*   End Collection (Physical) */
    0xC0,             /* End Collection (Application) */
};

static const struct device *hid_dev_kb;    /* HID_0: keyboard + consumer */
static const struct device *hid_dev_mouse; /* HID_1: mouse */
static bool usb_ready;

/* Callback when USB HID interrupt endpoint write completes */
static void int_in_ready_cb(const struct device *dev)
{
    LOG_DBG("USB HID EP write complete");
}

/* Callback when host sends LED output report (Caps Lock etc.) */
static int set_report_cb(const struct device *dev,
                          struct usb_setup_packet *setup,
                          int32_t *len, uint8_t **data)
{
    if (*len > 0) {
        LOG_DBG("LED output report: 0x%02x", (*data)[0]);
    }
    return 0;
}

static const struct hid_ops usb_hid_kb_ops = {
    .int_in_ready = int_in_ready_cb,
    .set_report = set_report_cb,
};

static const struct hid_ops usb_hid_mouse_ops = {
    .int_in_ready = int_in_ready_cb,
};

/* USB device status callback */
static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status) {
    case USB_DC_CONFIGURED:
        LOG_INF("USB_HID: Host configured");
        usb_ready = true;
        break;
    case USB_DC_DISCONNECTED:
        LOG_INF("USB_HID: Disconnected");
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

    /* HID_0: Keyboard + Consumer Control */
    hid_dev_kb = device_get_binding("HID_0");
    if (hid_dev_kb == NULL) {
        LOG_ERR("Cannot find USB HID_0 device");
        return -ENODEV;
    }

    usb_hid_register_device(hid_dev_kb, hid_report_desc_kb,
                            sizeof(hid_report_desc_kb), &usb_hid_kb_ops);

    ret = usb_hid_init(hid_dev_kb);
    if (ret) {
        LOG_ERR("USB HID_0 init failed: %d", ret);
        return ret;
    }

    /* HID_1: Mouse */
    hid_dev_mouse = device_get_binding("HID_1");
    if (hid_dev_mouse == NULL) {
        LOG_ERR("Cannot find USB HID_1 device");
        return -ENODEV;
    }

    usb_hid_register_device(hid_dev_mouse, hid_report_desc_mouse,
                            sizeof(hid_report_desc_mouse), &usb_hid_mouse_ops);

    ret = usb_hid_init(hid_dev_mouse);
    if (ret) {
        LOG_ERR("USB HID_1 init failed: %d", ret);
        return ret;
    }

    ret = usb_enable(usb_status_cb);
    if (ret == -EALREADY) {
        usb_ready = true;
        LOG_INF("USB_HID: USB already enabled, assuming ready");
    } else if (ret) {
        LOG_ERR("USB enable failed: %d", ret);
        return ret;
    } else {
        LOG_INF("USB_HID: USB enabled, waiting for host");
    }

    LOG_INF("USB_HID: forwarder initialized (kb=%p mouse=%p ready=%d)",
            hid_dev_kb, hid_dev_mouse, usb_ready);
    return 0;
}

int usb_hid_forwarder_send(const uint8_t *report, uint16_t len)
{
    if (!usb_ready || len == 0) {
        return -ENODEV;
    }

    /* Route to correct HID interface based on Report ID (first byte) */
    const struct device *dev;
    if (report[0] == 0x03) {
        dev = hid_dev_mouse;
    } else {
        dev = hid_dev_kb;
    }

    if (dev == NULL) {
        return -ENODEV;
    }

    /* Called from system work queue (not BT RX thread), so short
     * sleeps on EAGAIN are safe and won't stall BLE processing. */
    int ret = hid_int_ep_write(dev, report, len, NULL);
    if (ret == -EAGAIN) {
        k_msleep(1);
        ret = hid_int_ep_write(dev, report, len, NULL);
        if (ret == -EAGAIN) {
            k_msleep(1);
            ret = hid_int_ep_write(dev, report, len, NULL);
        }
    }
    if (ret) {
        LOG_WRN("USB HID write: %d (id=0x%02x)", ret, report[0]);
    }

    return ret;
}

bool usb_hid_forwarder_is_ready(void)
{
    return usb_ready && hid_dev_kb != NULL;
}

static int usb_hid_forwarder_sys_init(void)
{
    return usb_hid_forwarder_init();
}

/* Priority 20: must run BEFORE dongle_bt_enable (50) which has a 5s delay,
 * and before CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT would enable USB.
 * HID interface must be registered before usb_enable() is called. */
SYS_INIT(usb_hid_forwarder_sys_init, APPLICATION, 20);
