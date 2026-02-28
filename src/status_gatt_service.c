/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Self-contained GATT service that exposes keyboard status (layer, battery,
 * WPM, modifiers, etc.) as a BLE GATT notification.  Any Prospector Dongle
 * can subscribe to receive 26-byte zmk_status_adv_data updates without
 * relying on BLE advertisement scanning.
 *
 * To enable on any ZMK keyboard:
 *   west.yml: add prospector-zmk-module
 *   .conf:    CONFIG_PROSPECTOR_STATUS_GATT_SERVICE=y
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <string.h>

#include <zmk/status_advertisement.h>
#include <zmk/status_gatt_service.h>
#include <zmk/battery.h>
#include <zmk/hid.h>

/* keymap / BLE / USB APIs — only on Central or Standalone */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/keymap.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_BLE) && \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT))
#include <zmk/ble.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/usb.h>
#endif

/* ZMK event system */
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/modifiers_state_changed.h>
#include <zmk/events/battery_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_BLE) && \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT))
#include <zmk/events/ble_active_profile_changed.h>
#endif

LOG_MODULE_REGISTER(status_gatt, CONFIG_ZMK_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* WPM — rolling-window implementation (identical to status_adv)      */
/* ------------------------------------------------------------------ */

#ifndef CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS
#define CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS 30
#endif

#define WPM_HISTORY_SIZE 60
static uint8_t wpm_key_history[WPM_HISTORY_SIZE];
static uint32_t wpm_history_index;
static uint32_t wpm_last_second;
static uint8_t wpm_current_second_keys;
static uint8_t current_wpm;
static uint32_t last_activity_time;

#define WPM_WINDOW_MS (CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS * 1000)
#define WPM_WINDOW_MULTIPLIER \
    ((CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS > 0) ? \
     (60 / CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS) : 2)

#ifndef CONFIG_ZMK_STATUS_ADV_WPM_DECAY_TIMEOUT_SECONDS
#define CONFIG_ZMK_STATUS_ADV_WPM_DECAY_TIMEOUT_SECONDS 0
#endif

#define WPM_DECAY_TIMEOUT_MS \
    ((CONFIG_ZMK_STATUS_ADV_WPM_DECAY_TIMEOUT_SECONDS == 0) ? \
     (WPM_WINDOW_MS * 2) : \
     ((CONFIG_ZMK_STATUS_ADV_WPM_DECAY_TIMEOUT_SECONDS >= 10) ? \
      (CONFIG_ZMK_STATUS_ADV_WPM_DECAY_TIMEOUT_SECONDS * 1000) : \
      (WPM_WINDOW_MS * 2)))

/* ------------------------------------------------------------------ */
/* Peripheral battery tracking (split keyboards)                      */
/* ------------------------------------------------------------------ */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static uint8_t peripheral_batteries[3];

static int gatt_peripheral_battery_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev && ev->source < 3) {
        peripheral_batteries[ev->source] = ev->state_of_charge;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_peripheral_battery, gatt_peripheral_battery_listener);
ZMK_SUBSCRIPTION(gatt_peripheral_battery, zmk_peripheral_battery_state_changed);
#endif

/* ------------------------------------------------------------------ */
/* CCC (subscriber) tracking                                          */
/* ------------------------------------------------------------------ */

static bool has_subscriber;

static void status_ccc_changed_cb(const struct bt_gatt_attr *attr,
                                   uint16_t value)
{
    has_subscriber = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status GATT CCC: %s", has_subscriber ? "subscribed" : "unsubscribed");
}

/* ------------------------------------------------------------------ */
/* Build 26-byte status payload (self-contained, no status_adv dep)   */
/* ------------------------------------------------------------------ */

static void update_wpm(uint32_t now)
{
    uint32_t current_second = now / 1000;

    if (current_second != wpm_last_second && wpm_last_second > 0) {
        uint32_t seconds_elapsed = current_second - wpm_last_second;
        for (uint32_t i = 0; i < seconds_elapsed && i < WPM_HISTORY_SIZE; i++) {
            wpm_history_index = (wpm_history_index + 1) % WPM_HISTORY_SIZE;
            wpm_key_history[wpm_history_index] = (i == 0) ? wpm_current_second_keys : 0;
        }
        wpm_last_second = current_second;
        wpm_current_second_keys = 0;
    }

    if (wpm_last_second == 0) {
        wpm_last_second = current_second;
    }

    uint32_t window_keys = 0;
    uint32_t window_seconds = CONFIG_ZMK_STATUS_ADV_WPM_WINDOW_SECONDS;

    for (uint32_t i = 0; i < window_seconds && i < WPM_HISTORY_SIZE; i++) {
        uint32_t idx = (wpm_history_index + WPM_HISTORY_SIZE - i) % WPM_HISTORY_SIZE;
        window_keys += wpm_key_history[idx];
    }
    window_keys += wpm_current_second_keys;

    if (window_seconds > 0 && window_keys > 0) {
        uint32_t new_wpm = (window_keys * 12 * WPM_WINDOW_MULTIPLIER) / window_seconds;
        if (current_wpm == 0 || abs((int)new_wpm - (int)current_wpm) > 50) {
            current_wpm = new_wpm;
        } else {
            current_wpm = (new_wpm * 7 + current_wpm * 3) / 10;
        }
        if (current_wpm > 255) {
            current_wpm = 255;
        }
    }

    uint32_t time_since_activity = now - last_activity_time;
    if (time_since_activity > WPM_DECAY_TIMEOUT_MS) {
        current_wpm = 0;
        memset(wpm_key_history, 0, sizeof(wpm_key_history));
        wpm_history_index = 0;
        wpm_current_second_keys = 0;
        wpm_last_second = 0;
    } else if (time_since_activity > 5000 && current_wpm > 0) {
        float idle_seconds = (time_since_activity - 5000) / 1000.0f;
        float decay_factor = 1.0f - (idle_seconds / (WPM_WINDOW_MS / 1000.0f));
        if (decay_factor < 0.0f) {
            decay_factor = 0.0f;
        }
        current_wpm = (uint8_t)(current_wpm * decay_factor);
    }
}

static void build_status_data(struct zmk_status_adv_data *out)
{
    uint32_t now = k_uptime_get_32();
    memset(out, 0, sizeof(*out));

    /* Header */
    out->manufacturer_id[0] = 0xFF;
    out->manufacturer_id[1] = 0xFF;
    out->service_uuid[0] = 0xAB;
    out->service_uuid[1] = 0xCD;
    out->version = ZMK_STATUS_ADV_VERSION;

    /* Battery */
    uint8_t battery_level = zmk_battery_state_of_charge();
    if (battery_level > 100) {
        battery_level = 100;
    }
    out->battery_level = battery_level;

    /* Layer */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)
    out->active_layer = zmk_keymap_highest_layer_active();
#endif

    /* Profile */
#if IS_ENABLED(CONFIG_ZMK_BLE) && \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT))
    out->profile_slot = zmk_ble_active_profile_index();
#endif

    /* Connection count */
    uint8_t connection_count = 1;
#if IS_ENABLED(CONFIG_ZMK_USB)
    if (zmk_usb_is_hid_ready()) {
        connection_count++;
    }
#endif
    out->connection_count = connection_count;

    /* Status flags */
    uint8_t flags = 0;
#if IS_ENABLED(CONFIG_ZMK_USB)
    if (zmk_usb_is_powered()) {
        flags |= ZMK_STATUS_FLAG_USB_CONNECTED;
    }
    if (zmk_usb_is_hid_ready()) {
        flags |= ZMK_STATUS_FLAG_USB_HID_READY;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE) && \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT))
    if (zmk_ble_active_profile_is_connected()) {
        flags |= ZMK_STATUS_FLAG_BLE_CONNECTED;
    }
    if (!zmk_ble_active_profile_is_open()) {
        flags |= ZMK_STATUS_FLAG_BLE_BONDED;
    }
#endif
    out->status_flags = flags;

    /* Device role & peripheral batteries */
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    out->device_role = ZMK_DEVICE_ROLE_CENTRAL;
    out->device_index = 0;

    const char *central_side = "RIGHT";
#ifdef CONFIG_ZMK_STATUS_ADV_CENTRAL_SIDE
    central_side = CONFIG_ZMK_STATUS_ADV_CENTRAL_SIDE;
#endif
#ifndef CONFIG_ZMK_STATUS_ADV_HALF_PERIPHERAL
#define CONFIG_ZMK_STATUS_ADV_HALF_PERIPHERAL 0
#endif
#ifndef CONFIG_ZMK_STATUS_ADV_AUX1_PERIPHERAL
#define CONFIG_ZMK_STATUS_ADV_AUX1_PERIPHERAL 1
#endif
#ifndef CONFIG_ZMK_STATUS_ADV_AUX2_PERIPHERAL
#define CONFIG_ZMK_STATUS_ADV_AUX2_PERIPHERAL 2
#endif

    uint8_t half_battery = peripheral_batteries[CONFIG_ZMK_STATUS_ADV_HALF_PERIPHERAL];
    uint8_t aux1_battery = peripheral_batteries[CONFIG_ZMK_STATUS_ADV_AUX1_PERIPHERAL];
    uint8_t aux2_battery = peripheral_batteries[CONFIG_ZMK_STATUS_ADV_AUX2_PERIPHERAL];

    if (strcmp(central_side, "LEFT") == 0) {
        uint8_t central_battery = battery_level;
        out->battery_level = half_battery;
        out->peripheral_battery[0] = central_battery;
        out->peripheral_battery[1] = aux1_battery;
        out->peripheral_battery[2] = aux2_battery;
    } else {
        out->peripheral_battery[0] = half_battery;
        out->peripheral_battery[1] = aux1_battery;
        out->peripheral_battery[2] = aux2_battery;
    }
#elif IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    out->device_role = ZMK_DEVICE_ROLE_PERIPHERAL;
    out->device_index = 0;
#else
    out->device_role = ZMK_DEVICE_ROLE_STANDALONE;
    out->device_index = 0;
#endif

    /* Layer name */
    snprintf(out->layer_name, sizeof(out->layer_name), "L%d", out->active_layer);

    /* Keyboard ID */
#ifdef CONFIG_ZMK_STATUS_ADV_KEYBOARD_NAME
    const char *keyboard_name = CONFIG_ZMK_STATUS_ADV_KEYBOARD_NAME;
#else
    const char *keyboard_name = "ZMK";
#endif
    uint32_t id_hash = 0;
    for (int i = 0; keyboard_name[i] && i < 8; i++) {
        id_hash = id_hash * 31 + keyboard_name[i];
    }
    memcpy(out->keyboard_id, &id_hash, 4);

    /* Modifiers */
    struct zmk_hid_keyboard_report *report = zmk_hid_get_keyboard_report();
    if (report) {
        uint8_t mods = report->body.modifiers;
        uint8_t modifier_flags = 0;
        if (mods & (0x01 | 0x10)) modifier_flags |= ZMK_MOD_FLAG_LCTL | ZMK_MOD_FLAG_RCTL;
        if (mods & (0x02 | 0x20)) modifier_flags |= ZMK_MOD_FLAG_LSFT | ZMK_MOD_FLAG_RSFT;
        if (mods & (0x04 | 0x40)) modifier_flags |= ZMK_MOD_FLAG_LALT | ZMK_MOD_FLAG_RALT;
        if (mods & (0x08 | 0x80)) modifier_flags |= ZMK_MOD_FLAG_LGUI | ZMK_MOD_FLAG_RGUI;
        out->modifier_flags = modifier_flags;
    }

    /* WPM */
    update_wpm(now);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)
    out->wpm_value = current_wpm;
#endif

    /* Channel */
#ifdef CONFIG_PROSPECTOR_CHANNEL
    out->channel = CONFIG_PROSPECTOR_CHANNEL;
#endif
}

/* ------------------------------------------------------------------ */
/* Layer name resolution                                              */
/* ------------------------------------------------------------------ */

#define GATT_STATUS_MAX_LAYER_NAME 12

static const char *get_layer_name(uint8_t layer_idx) {
#if defined(CONFIG_PROSPECTOR_LAYER_NAMES)
    static char name_buf[GATT_STATUS_MAX_LAYER_NAME + 1];
    const char *names = CONFIG_PROSPECTOR_LAYER_NAMES;
    if (names[0] == '\0') {
        goto fallback;
    }

    /* Parse comma-separated name at layer_idx position */
    int idx = 0;
    const char *start = names;
    for (const char *p = names; ; p++) {
        if (*p == ',' || *p == '\0') {
            if (idx == layer_idx) {
                int l = p - start;
                if (l > GATT_STATUS_MAX_LAYER_NAME) {
                    l = GATT_STATUS_MAX_LAYER_NAME;
                }
                memcpy(name_buf, start, l);
                name_buf[l] = '\0';
                return name_buf;
            }
            if (*p == '\0') {
                break;
            }
            idx++;
            start = p + 1;
        }
    }
fallback:
#endif
    {
        static char fb[4];
        snprintf(fb, sizeof(fb), "L%d", layer_idx);
        return fb;
    }
}

/* ------------------------------------------------------------------ */
/* Read callback (for on-demand reads)                                */
/* ------------------------------------------------------------------ */

/* Extended buffer: 26-byte status + layer name (null-terminated) */
static uint8_t gatt_buf[sizeof(struct zmk_status_adv_data) + GATT_STATUS_MAX_LAYER_NAME + 1];

static ssize_t read_status_cb(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    struct zmk_status_adv_data *data = (struct zmk_status_adv_data *)gatt_buf;
    build_status_data(data);

    const char *name = get_layer_name(data->active_layer);
    size_t name_len = strlen(name) + 1; /* include null */
    memcpy(gatt_buf + sizeof(struct zmk_status_adv_data), name, name_len);

    size_t total = sizeof(struct zmk_status_adv_data) + name_len;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, gatt_buf, total);
}

/* ------------------------------------------------------------------ */
/* GATT service definition                                            */
/* ------------------------------------------------------------------ */

BT_GATT_SERVICE_DEFINE(prospector_status_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_PROSPECTOR_STATUS_SVC),
    BT_GATT_CHARACTERISTIC(BT_UUID_PROSPECTOR_STATUS_CHR,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        read_status_cb, NULL, NULL),
    BT_GATT_CCC(status_ccc_changed_cb,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),
);

/* ------------------------------------------------------------------ */
/* Notify helper                                                      */
/* ------------------------------------------------------------------ */

static void send_status_notify(void)
{
    if (!has_subscriber) {
        return;
    }

    /* Build 26-byte base + layer name extension */
    static uint8_t notify_buf[sizeof(struct zmk_status_adv_data) + GATT_STATUS_MAX_LAYER_NAME + 1];
    struct zmk_status_adv_data *data = (struct zmk_status_adv_data *)notify_buf;
    build_status_data(data);

    const char *name = get_layer_name(data->active_layer);
    size_t name_len = strlen(name) + 1; /* include null */
    memcpy(notify_buf + sizeof(struct zmk_status_adv_data), name, name_len);

    size_t total = sizeof(struct zmk_status_adv_data) + name_len;
    int err = bt_gatt_notify(NULL, &prospector_status_svc.attrs[1],
                              notify_buf, total);
    if (err && err != -ENOTCONN) {
        LOG_WRN("GATT notify failed: %d", err);
    }
}

/* ------------------------------------------------------------------ */
/* ZMK event listeners — trigger notify on state changes              */
/* ------------------------------------------------------------------ */

static int gatt_position_listener(const zmk_event_t *eh)
{
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev && ev->state) {
        uint32_t now = k_uptime_get_32();
        last_activity_time = now;

        /* Update circular buffer */
        uint32_t current_second = now / 1000;
        if (current_second != wpm_last_second) {
            uint32_t seconds_elapsed = current_second - wpm_last_second;
            for (uint32_t i = 0; i < seconds_elapsed && i < WPM_HISTORY_SIZE; i++) {
                wpm_history_index = (wpm_history_index + 1) % WPM_HISTORY_SIZE;
                wpm_key_history[wpm_history_index] = (i == 0) ? wpm_current_second_keys : 0;
            }
            wpm_last_second = current_second;
            wpm_current_second_keys = 0;
        }
        wpm_current_second_keys++;

        send_status_notify();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_position, gatt_position_listener);
ZMK_SUBSCRIPTION(gatt_position, zmk_position_state_changed);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT)
static int gatt_layer_listener(const zmk_event_t *eh)
{
    send_status_notify();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_layer, gatt_layer_listener);
ZMK_SUBSCRIPTION(gatt_layer, zmk_layer_state_changed);
#endif

static int gatt_modifiers_listener(const zmk_event_t *eh)
{
    send_status_notify();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_modifiers, gatt_modifiers_listener);
ZMK_SUBSCRIPTION(gatt_modifiers, zmk_modifiers_state_changed);

#if IS_ENABLED(CONFIG_ZMK_BLE) && \
    (IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) || !IS_ENABLED(CONFIG_ZMK_SPLIT))
static int gatt_profile_listener(const zmk_event_t *eh)
{
    send_status_notify();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_profile, gatt_profile_listener);
ZMK_SUBSCRIPTION(gatt_profile, zmk_ble_active_profile_changed);
#endif

static int gatt_battery_listener(const zmk_event_t *eh)
{
    send_status_notify();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(gatt_battery, gatt_battery_listener);
ZMK_SUBSCRIPTION(gatt_battery, zmk_battery_state_changed);

/* ------------------------------------------------------------------ */
/* Init                                                               */
/* ------------------------------------------------------------------ */

static int status_gatt_service_init(void)
{
    LOG_INF("Prospector Status GATT Service initialized");
    return 0;
}

SYS_INIT(status_gatt_service_init, APPLICATION, 95);
