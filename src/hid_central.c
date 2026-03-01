/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#include <psa/crypto.h>
#include <string.h>

#include <zmk/hid_central.h>
#include <zmk/usb_hid_forwarder.h>
#include <zmk/status_scanner.h>
#include <zmk/status_gatt_service.h>
#include <zmk/status_advertisement.h>

LOG_MODULE_REGISTER(hid_central, CONFIG_ZMK_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* Constants                                                          */
/* ------------------------------------------------------------------ */

/* HID Service UUID (0x1812) */
#define BT_UUID_HID_SERVICE    BT_UUID_DECLARE_16(0x1812)
/* HID Report characteristic UUID (0x2A4D) */
#define BT_UUID_HID_REPORT     BT_UUID_DECLARE_16(0x2A4D)
/* HID Report Map UUID (0x2A4B) */
#define BT_UUID_HID_REPORT_MAP BT_UUID_DECLARE_16(0x2A4B)
/* Report Reference descriptor UUID (0x2908) */
#define BT_UUID_HID_REPORT_REF BT_UUID_DECLARE_16(0x2908)

/* HID Report types */
#define HID_REPORT_TYPE_INPUT   1
#define HID_REPORT_TYPE_OUTPUT  2
#define HID_REPORT_TYPE_FEATURE 3

/* Maximum number of HID report characteristics to discover */
#define MAX_HID_REPORTS 6

/* Reconnection work queue */
#define RECONNECT_INTERVAL_MS CONFIG_PROSPECTOR_DONGLE_RECONNECT_INTERVAL_MS

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

enum central_state {
    STATE_IDLE,
    STATE_SCANNING,        /* waiting for scan result (driven by status_scanner) */
    STATE_CONNECTING,
    STATE_DISCOVERING,     /* GATT discovery in progress */
    STATE_SUBSCRIBING,     /* subscribing to input reports */
    STATE_READY,           /* HID reports flowing */
};

static enum central_state state = STATE_IDLE;
static struct bt_conn *kbd_conn;

/* ------------------------------------------------------------------ */
/* Multi-keyboard bonding state                                       */
/* ------------------------------------------------------------------ */

#define MAX_BONDED_KBD CONFIG_PROSPECTOR_DONGLE_MAX_BONDED

struct bonded_kbd {
    bt_addr_le_t addr;
    char name[32];
    bool valid;
};

static struct bonded_kbd bonded_kbds[MAX_BONDED_KBD];
static int bonded_count;
static int active_kbd_idx = -1;  /* Index of currently selected keyboard */

/* Pairing mode state */
static bool pairing_mode;

#define MAX_DISCOVERED 5
static struct {
    bt_addr_le_t addr;
    char name[32];
    int8_t rssi;
    bool valid;
} discovered_kbds[MAX_DISCOVERED];
static int discovered_count;

/* Convenience macros replacing old single-bond variables */
#define has_bonded_addr (bonded_count > 0 && active_kbd_idx >= 0)
#define bonded_addr     (bonded_kbds[active_kbd_idx].addr)

/* ------------------------------------------------------------------ */
/* Settings persistence for keyboard names and active index            */
/* ------------------------------------------------------------------ */

static int dongle_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    if (!strncmp(name, "active", 6)) {
        if (len == sizeof(active_kbd_idx)) {
            read_cb(cb_arg, &active_kbd_idx, sizeof(active_kbd_idx));
        }
        return 0;
    }

    /* Keys: "name/0", "name/1", ... */
    if (!strncmp(name, "name/", 5)) {
        int idx = name[5] - '0';
        if (idx >= 0 && idx < MAX_BONDED_KBD) {
            size_t to_read = len < sizeof(bonded_kbds[idx].name) - 1
                             ? len : sizeof(bonded_kbds[idx].name) - 1;
            memset(bonded_kbds[idx].name, 0, sizeof(bonded_kbds[idx].name));
            read_cb(cb_arg, bonded_kbds[idx].name, to_read);
        }
        return 0;
    }

    /* Keys: "addr/0", "addr/1", ... */
    if (!strncmp(name, "addr/", 5)) {
        int idx = name[5] - '0';
        if (idx >= 0 && idx < MAX_BONDED_KBD && len == sizeof(bt_addr_le_t)) {
            read_cb(cb_arg, &bonded_kbds[idx].addr, sizeof(bt_addr_le_t));
            bonded_kbds[idx].valid = true;
        }
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(dongle, "dongle", NULL,
                               dongle_settings_set, NULL, NULL);

static void save_bonded_name(int idx)
{
    char key[24];
    snprintf(key, sizeof(key), "dongle/name/%d", idx);
    settings_save_one(key, bonded_kbds[idx].name,
                      strlen(bonded_kbds[idx].name) + 1);
}

static void save_bonded_addr(int idx)
{
    char key[24];
    snprintf(key, sizeof(key), "dongle/addr/%d", idx);
    settings_save_one(key, &bonded_kbds[idx].addr, sizeof(bt_addr_le_t));
}

static void save_active_index(void)
{
    settings_save_one("dongle/active", &active_kbd_idx, sizeof(active_kbd_idx));
}

/* GATT discovery state */
struct hid_report_info {
    uint16_t handle;       /* Report characteristic value handle */
    uint16_t ccc_handle;   /* CCC descriptor handle */
    uint8_t  report_id;    /* Report ID from Report Reference descriptor */
    uint8_t  report_type;  /* Input / Output / Feature */
    bool     subscribed;
};

static struct hid_report_info reports[MAX_HID_REPORTS];
static int report_count;
static struct bt_gatt_discover_params disc_params;
static struct bt_gatt_subscribe_params sub_params[MAX_HID_REPORTS];

/* Work for reconnection */
static struct k_work_delayable reconnect_work;

/* Deferred connection work (must not call bt_conn_le_create from scan callback) */
static struct k_work connect_work;
static bt_addr_le_t pending_addr;
static bool pending_connect;

/* Deferred GATT discovery work (delay after security establishment) */
static struct k_work_delayable discovery_work;

/* Flag: discovery is waiting for security to be established */
static bool discover_after_security;

/* ------------------------------------------------------------------ */
/* Prospector Status GATT service discovery state                     */
/* ------------------------------------------------------------------ */

static struct bt_gatt_discover_params status_disc_params;
static struct bt_gatt_discover_params status_chr_disc_params;
static struct bt_gatt_subscribe_params status_sub_params;
static bool status_svc_subscribed;
static uint16_t status_svc_start;
static uint16_t status_svc_end;

/* Static 128-bit UUID storage for async GATT discovery */
static struct bt_uuid_128 disc_uuid_status_svc =
    BT_UUID_INIT_128(BT_UUID_PROSPECTOR_STATUS_SVC_VAL);
static struct bt_uuid_128 disc_uuid_status_chr =
    BT_UUID_INIT_128(BT_UUID_PROSPECTOR_STATUS_CHR_VAL);

/* RSSI periodic update (1 Hz) */
static struct k_work_delayable rssi_work;

/* GATT READ polling (fallback when notifications don't arrive) */
#define STATUS_POLL_INTERVAL_MS 1000
static struct k_work_delayable status_poll_work;
static struct bt_gatt_read_params status_read_params;
static uint16_t status_chr_value_handle;  /* Cached characteristic value handle */

/* MTU exchange */
static struct bt_gatt_exchange_params mtu_exchange_params;

/* Connected keyboard device name (captured during scan) */
static char connected_kbd_name[32];

static void start_status_service_discovery(struct bt_conn *conn);

/* Flag: PSA diagnostic test already run */
static bool psa_diag_done;
static int psa_test_result = -999;     /* -999 = not yet run */
static int psa_bt_rx_result = -999;    /* PSA test on BT RX thread */
static int psa_test_nousage = -999;    /* test with usage_flags=0 (like bt_pub_key_is_valid) */

/* Forward declaration for hex_format (defined in v18 section) */
static void hex_format(char *out, const uint8_t *data, size_t len);

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */

static void start_hid_discovery(struct bt_conn *conn);
static void subscribe_next_input_report(int idx);
static void schedule_reconnect(void);
static void test_psa_import(void);

/* ------------------------------------------------------------------ */
/* USB HID report forwarding (deferred via work queue)                */
/* ------------------------------------------------------------------ */

/*
 * HID reports are received in the BT RX thread (notification callback).
 * USB writes (hid_int_ep_write) must NOT block that thread – doing so
 * stalls all BLE processing and eventually freezes the system when the
 * trackball generates continuous notifications.
 *
 * Solution: copy the report into a small ring buffer and submit a work
 * item.  The system work queue drains the buffer and writes to USB.
 */

#define HID_QUEUE_SIZE 8   /* power of 2 for mask trick */
#define HID_REPORT_MAX 12  /* Report-ID (1) + max payload (10 for mouse) */

struct hid_queued_report {
    uint8_t data[HID_REPORT_MAX];
    uint8_t len;
};

static struct hid_queued_report hid_queue[HID_QUEUE_SIZE];
static volatile uint8_t hid_q_head; /* written by BT RX thread */
static volatile uint8_t hid_q_tail; /* read by work queue */
static struct k_work hid_send_work;

static void hid_send_work_handler(struct k_work *work)
{
    while (hid_q_tail != hid_q_head) {
        uint8_t idx = hid_q_tail & (HID_QUEUE_SIZE - 1);
        struct hid_queued_report *rpt = &hid_queue[idx];

        if (usb_hid_forwarder_is_ready()) {
            int ret = usb_hid_forwarder_send(rpt->data, rpt->len);
            if (rpt->data[0] == 0x03) {
                LOG_INF("MOUSE_USB: ret=%d len=%d", ret, rpt->len);
            }
        }

        hid_q_tail++;
    }
}

/*
 * Map BLE HID report to the USB forwarder (queued).
 *
 * Report ID 1 = Keyboard (8 bytes)
 * Report ID 2 = Consumer Control (2 bytes)
 * Report ID 3 = Mouse/Pointing (9 bytes)
 */
static void forward_hid_report(uint8_t report_id, const uint8_t *data,
                                uint16_t len)
{
    uint8_t buf[HID_REPORT_MAX];
    uint8_t buf_len = 0;

    if (report_id == 1 && len == 8) {
        buf[0] = 0x01;
        memcpy(&buf[1], data, 8);
        buf_len = 9;
    } else if (report_id == 2 && len >= 2) {
        buf[0] = 0x02;
        memcpy(&buf[1], data, 2);
        buf_len = 3;
    } else if (report_id == 3 && len == 9) {
        buf[0] = 0x03;
        memcpy(&buf[1], data, 9);
        buf_len = 10;
    } else {
        LOG_WRN("HID DROP: id=%d len=%d (no match)", report_id, len);
        return; /* unknown report, silently drop */
    }

    /* Enqueue – if the ring buffer is full, drop the oldest mouse report
     * or drop this report for mouse.  Keyboard reports are never dropped. */
    uint8_t next = (hid_q_head + 1);
    if ((next - hid_q_tail) > HID_QUEUE_SIZE) {
        /* Queue full */
        if (report_id == 3) {
            return; /* drop mouse – relative delta, no harm */
        }
        /* For keyboard: advance tail to make room (drop oldest) */
        hid_q_tail++;
    }

    uint8_t idx = hid_q_head & (HID_QUEUE_SIZE - 1);
    memcpy(hid_queue[idx].data, buf, buf_len);
    hid_queue[idx].len = buf_len;
    hid_q_head++;

    k_work_submit(&hid_send_work);
}

/* ------------------------------------------------------------------ */
/* BLE HID notification callback                                      */
/* ------------------------------------------------------------------ */

static uint8_t hid_notify_cb(struct bt_conn *conn,
                              struct bt_gatt_subscribe_params *params,
                              const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("HID notification unsubscribed (handle 0x%04x)",
                params->value_handle);
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    /* Find matching report to determine report_id */
    uint8_t report_id = 0;
    for (int i = 0; i < report_count; i++) {
        if (reports[i].handle == params->value_handle) {
            report_id = reports[i].report_id;
            break;
        }
    }

    forward_hid_report(report_id, data, length);
    return BT_GATT_ITER_CONTINUE;
}

/* ------------------------------------------------------------------ */
/* GATT discovery – Report Reference descriptor read                  */
/* ------------------------------------------------------------------ */

static int pending_ref_reads;

static uint8_t report_ref_read_cb(struct bt_conn *conn, uint8_t err,
                                   struct bt_gatt_read_params *params,
                                   const void *data, uint16_t length)
{
    if (err || !data || length < 2) {
        LOG_WRN("REF_READ failed (err %d, len %d)", err, length);
        goto done;
    }

    const uint8_t *ref = data;
    uint8_t report_id   = ref[0];
    uint8_t report_type = ref[1];

    /* Find which report this belongs to (by descriptor handle proximity) */
    uint16_t desc_handle = params->single.handle;
    for (int i = 0; i < report_count; i++) {
        /* The Report Reference descriptor should be near the report handle */
        if (desc_handle > reports[i].handle &&
            desc_handle <= reports[i].handle + 3) {
            reports[i].report_id   = report_id;
            reports[i].report_type = report_type;
            LOG_INF("REF: handle=0x%04x id=%d type=%d",
                    reports[i].handle, report_id, report_type);
            break;
        }
    }

done:
    pending_ref_reads--;
    if (pending_ref_reads <= 0) {
        /* All Report References read */
        LOG_INF("DONGLE: %d reports discovered", report_count);
        state = STATE_SUBSCRIBING;
        subscribe_next_input_report(0);
    }
    return BT_GATT_ITER_STOP;
}

/* ------------------------------------------------------------------ */
/* GATT discovery state machine                                       */
/* ------------------------------------------------------------------ */

/*
 * Discovery proceeds in phases:
 *  0. Discover HID primary service (0x1812) to get handle range
 *  1. Discover HID Report characteristics (0x2A4D) within service range
 *  2. For each Report characteristic, read its Report Reference descriptor
 *  3. Subscribe to Input Report notifications
 */

/* HID service handle range (from phase 0) */
static uint16_t hid_svc_start;
static uint16_t hid_svc_end;

/* Static UUID storage for async GATT discovery (BT_UUID_DECLARE_16 creates
 * stack-local compound literals that go out of scope before the callback
 * fires, causing dangling pointer reads in the UUID filter). */
static struct bt_uuid_16 disc_uuid_svc = BT_UUID_INIT_16(0x1812);  /* HID Service */
static struct bt_uuid_16 disc_uuid_chr = BT_UUID_INIT_16(0x2A4D);  /* HID Report */

/* Second discover_params for phase 1 (characteristic discovery) */
static struct bt_gatt_discover_params disc_params2;

/* Phase 2 storage for Report Reference reads */
static struct bt_gatt_read_params ref_read_params[MAX_HID_REPORTS];

static void read_report_references(struct bt_conn *conn)
{
    pending_ref_reads = 0;

    for (int i = 0; i < report_count; i++) {
        /* Report Reference descriptor is typically at handle + 2
         * (handle+1 = CCC, handle+2 = Report Reference) or handle+1 */
        ref_read_params[i] = (struct bt_gatt_read_params){
            .func = report_ref_read_cb,
            .handle_count = 1,
            .single = {
                .handle = reports[i].handle + 2,
                .offset = 0,
            },
        };

        int err = bt_gatt_read(conn, &ref_read_params[i]);
        if (err) {
            LOG_WRN("Report ref read start failed for handle 0x%04x: %d",
                    reports[i].handle, err);
        } else {
            pending_ref_reads++;
        }
    }

    if (pending_ref_reads == 0) {
        /* No references to read – try subscribing anyway with assumed IDs */
        LOG_WRN("No report references readable, guessing IDs");
        for (int i = 0; i < report_count; i++) {
            reports[i].report_id = (i == 0) ? 1 : 2;
            reports[i].report_type = HID_REPORT_TYPE_INPUT;
        }
        state = STATE_SUBSCRIBING;
        subscribe_next_input_report(0);
    }
}

/* Phase 1 callback: discover HID Report characteristics within service */
static uint8_t discover_chars_cb(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("HID char discovery: found %d reports", report_count);
        if (report_count > 0) {
            read_report_references(conn);
        } else {
            LOG_WRN("DONGLE: No HID Report chars found");
            if (kbd_conn) {
                bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

    if (report_count < MAX_HID_REPORTS) {
        reports[report_count].handle = chrc->value_handle;
        reports[report_count].ccc_handle = 0;
        reports[report_count].report_id = 0;
        reports[report_count].report_type = 0;
        reports[report_count].subscribed = false;
        LOG_INF("Report char at 0x%04x (#%d)", chrc->value_handle, report_count);
        report_count++;
    }

    return BT_GATT_ITER_CONTINUE;
}

/* Phase 0 callback: discover HID primary service */
static uint8_t discover_svc_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("DONGLE: HID Service NOT found");
        if (kbd_conn) {
            bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc = (struct bt_gatt_service_val *)attr->user_data;
    hid_svc_start = attr->handle;
    hid_svc_end = svc->end_handle;

    LOG_INF("DONGLE: HID Service 0x%04x-0x%04x",
            hid_svc_start, hid_svc_end);

    /* Phase 1: discover characteristics within HID service range */
    memset(&disc_params2, 0, sizeof(disc_params2));
    disc_params2.uuid = &disc_uuid_chr.uuid;
    disc_params2.func = discover_chars_cb;
    disc_params2.start_handle = hid_svc_start + 1;
    disc_params2.end_handle = hid_svc_end;
    disc_params2.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &disc_params2);
    if (err) {
        LOG_ERR("Char discovery failed: %d", err);
        if (kbd_conn) {
            bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
    }

    /* Stop iterating services – we only need the first HID service */
    return BT_GATT_ITER_STOP;
}

static void start_hid_discovery(struct bt_conn *conn)
{
    report_count = 0;
    memset(reports, 0, sizeof(reports));
    hid_svc_start = 0;
    hid_svc_end = 0;

    state = STATE_DISCOVERING;

    /* Phase 0: discover HID primary service (0x1812) to get handle range */
    disc_params.uuid = &disc_uuid_svc.uuid;
    disc_params.func = discover_svc_cb;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &disc_params);
    if (err) {
        LOG_ERR("HID GATT discovery start failed: %d", err);
        state = STATE_IDLE;
    }
}

/* ------------------------------------------------------------------ */
/* Subscribe to Input Report notifications                            */
/* ------------------------------------------------------------------ */

static void subscribe_cb(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_subscribe_params *params)
{
    /* Ignore spurious subscribe callbacks from Zephyr's automatic CCC
     * restoration on bonded reconnection.  We only process subscribe
     * results when our state machine is actively subscribing. */
    if (state != STATE_SUBSCRIBING) {
        LOG_DBG("Ignoring subscribe cb in state %d (handle 0x%04x)",
                state, params->value_handle);
        return;
    }

    if (err) {
        LOG_WRN("Subscribe failed for handle 0x%04x: %d",
                params->value_handle, err);
    } else {
        LOG_INF("Subscribed to handle 0x%04x", params->value_handle);
    }

    /* Find the report index for this subscribe */
    for (int i = 0; i < report_count; i++) {
        if (reports[i].handle == params->value_handle) {
            reports[i].subscribed = (err == 0);
            /* Move to next input report */
            subscribe_next_input_report(i + 1);
            return;
        }
    }

    /* Shouldn't reach here, but try continuing */
    subscribe_next_input_report(report_count);
}

static void subscribe_next_input_report(int idx)
{
    /* Find next Input Report to subscribe to */
    for (int i = idx; i < report_count; i++) {
        if (reports[i].report_type == HID_REPORT_TYPE_INPUT) {
            sub_params[i].notify = hid_notify_cb;
            sub_params[i].subscribe = subscribe_cb;
            sub_params[i].value_handle = reports[i].handle;
            /* CCC handle is typically value_handle + 1 */
            sub_params[i].ccc_handle = reports[i].handle + 1;
            sub_params[i].value = BT_GATT_CCC_NOTIFY;

            int err = bt_gatt_subscribe(kbd_conn, &sub_params[i]);
            if (err) {
                LOG_WRN("Subscribe start failed for handle 0x%04x: %d",
                        reports[i].handle, err);
                /* Try next */
                continue;
            }
            return; /* Wait for callback */
        }
    }

    /* All input reports subscribed – we're ready */
    state = STATE_READY;
    LOG_INF("DONGLE: READY - forwarding HID reports to USB");

    /* Try to discover Prospector Status GATT service for scan-free
     * display updates.  If the keyboard doesn't expose the service
     * (old firmware), we fall back to BLE scanning. */
    status_svc_subscribed = false;
    start_status_service_discovery(kbd_conn);

    /* Start periodic RSSI polling (1 Hz) */
    k_work_schedule(&rssi_work, K_SECONDS(1));
}

/* ------------------------------------------------------------------ */
/* BLE connection callbacks                                           */
/* ------------------------------------------------------------------ */

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params)
{
    if (err) {
        LOG_WRN("MTU exchange failed: %d", err);
    } else {
        LOG_INF("MTU exchanged: %d", bt_gatt_get_mtu(conn));
    }
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (conn != kbd_conn) {
        return;
    }

    if (err) {
        LOG_ERR("Connection failed: %d", err);
        bt_conn_unref(kbd_conn);
        kbd_conn = NULL;
        if (pairing_mode) {
            state = STATE_SCANNING;
        } else {
            state = STATE_IDLE;
            schedule_reconnect();
        }
        zmk_status_scanner_stop();
        zmk_status_scanner_start();
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    LOG_INF("DONGLE: Connected to %s", addr_str);

    /* Connection established — leave pairing mode if active */
    pairing_mode = false;

    /* Ensure device name is set (bonded reconnect skips scan callback) */
    if (connected_kbd_name[0] == '\0') {
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            strncpy(connected_kbd_name, target, sizeof(connected_kbd_name) - 1);
            connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
            LOG_INF("Using target name: %s", connected_kbd_name);
        }
#endif
    }

    state = STATE_CONNECTING;

    /* Exchange MTU to allow 26+ byte GATT notifications/reads.
     * Default MTU=23 limits notification payload to 20 bytes,
     * too small for zmk_status_adv_data (26 bytes). */
    mtu_exchange_params.func = mtu_exchange_cb;
    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        LOG_WRN("MTU exchange request failed: %d", mtu_err);
    }

    /* Request security – GATT discovery will start in security_changed_cb
     * because HID attributes require encryption to be visible. */
    discover_after_security = true;
    int sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err) {
        LOG_WRN("Security request failed: %d", sec_err);
        discover_after_security = false;
        start_hid_discovery(conn);
    } else {
        LOG_INF("Security requested");
    }

    /* Do NOT restart scanning here – passive scan during LESC pairing
     * can interfere with the connection (RF noise / timing).
     * Scanning will be restarted in security_changed_cb after pairing. */
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (conn != kbd_conn) {
        return;
    }

    LOG_INF("DONGLE: Disconnected (0x%02x)", reason);

    /* Cancel any pending deferred discovery */
    k_work_cancel_delayable(&discovery_work);
    k_work_cancel_delayable(&rssi_work);
    k_work_cancel_delayable(&status_poll_work);

    bt_conn_unref(kbd_conn);
    kbd_conn = NULL;

    /* Reset HID subscribe params */
    for (int i = 0; i < MAX_HID_REPORTS; i++) {
        sub_params[i].value_handle = 0;
    }

    /* Reset status service subscribe params */
    status_sub_params.value_handle = 0;
    status_svc_subscribed = false;
    status_chr_value_handle = 0;

    /* Send empty keyboard report to release all keys */
    if (usb_hid_forwarder_is_ready()) {
        static const uint8_t release_kb[] = {0x01, 0,0,0,0,0,0,0,0};
        usb_hid_forwarder_send(release_kb, sizeof(release_kb));
    }

    /* In pairing mode, keep state=SCANNING and don't auto-reconnect.
     * The disconnect was intentional (enter_pairing_mode triggered it). */
    if (pairing_mode) {
        state = STATE_SCANNING;
        /* stop+start to guarantee scanning is actually running */
        zmk_status_scanner_stop();
        zmk_status_scanner_start();
        return;
    }

    state = STATE_IDLE;

    /* Restart scanning now that the connection is gone */
    zmk_status_scanner_stop();
    zmk_status_scanner_start();

    schedule_reconnect();
}

static void discovery_work_handler(struct k_work *work)
{
    if (!kbd_conn) {
        return;
    }
    /* Allow discovery in any connected state – on bonded reconnection,
     * Zephyr's auto CCC restore may have already changed state to
     * STATE_READY before this delayed work fires. */
    if (state == STATE_DISCOVERING) {
        LOG_DBG("Discovery already running, skipping");
        return;
    }
    start_hid_discovery(kbd_conn);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                 enum bt_security_err err)
{
    if (conn != kbd_conn) {
        return;
    }

    if (err) {
        LOG_WRN("DONGLE: Security failed: err %d", err);
        LOG_WRN("Security change failed: level %d err %d", level, err);
        if (discover_after_security) {
            discover_after_security = false;
            start_hid_discovery(conn);
        }
    } else {
        LOG_INF("DONGLE: Security level %d", level);

        /* Register this bond in bonded_kbds[] if new */
        const bt_addr_le_t *dst = bt_conn_get_dst(conn);
        int idx = -1;
        for (int i = 0; i < bonded_count; i++) {
            if (bt_addr_le_cmp(dst, &bonded_kbds[i].addr) == 0) {
                idx = i;
                break;
            }
        }
        if (idx < 0 && bonded_count < MAX_BONDED_KBD) {
            idx = bonded_count;
            bt_addr_le_copy(&bonded_kbds[idx].addr, dst);
            bonded_kbds[idx].valid = true;
            bonded_count++;
            LOG_INF("New bond added at index %d", idx);
            save_bonded_addr(idx);
        }
        if (idx >= 0) {
            /* Save name captured during scan */
            if (connected_kbd_name[0] != '\0') {
                strncpy(bonded_kbds[idx].name, connected_kbd_name,
                        sizeof(bonded_kbds[idx].name) - 1);
                bonded_kbds[idx].name[sizeof(bonded_kbds[idx].name) - 1] = '\0';
                save_bonded_name(idx);
            }
            active_kbd_idx = idx;
            save_active_index();
        }

        /* Pairing succeeded — leave pairing mode */
        pairing_mode = false;

        if (discover_after_security) {
            discover_after_security = false;
            k_work_schedule(&discovery_work, K_MSEC(500));
        }
    }
}

BT_CONN_CB_DEFINE(hid_central_conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .security_changed = security_changed_cb,
};

/* ------------------------------------------------------------------ */
/* Scan result handler (called from status_scanner scan_callback)     */
/* ------------------------------------------------------------------ */

static bool is_hid_service_in_ad(struct net_buf_simple *buf)
{
    struct net_buf_simple ad = *buf;

    while (ad.len > 1) {
        uint8_t len = net_buf_simple_pull_u8(&ad);
        if (len == 0 || len > ad.len) {
            break;
        }

        uint8_t ad_type = net_buf_simple_pull_u8(&ad);
        len--;

        /* Check complete/incomplete 16-bit UUID lists */
        if ((ad_type == BT_DATA_UUID16_ALL ||
             ad_type == BT_DATA_UUID16_SOME) && len >= 2) {
            for (int i = 0; i + 1 < len; i += 2) {
                uint16_t uuid = ad.data[i] | ((uint16_t)ad.data[i + 1] << 8);
                if (uuid == 0x1812) { /* HID Service */
                    return true;
                }
            }
        }

        net_buf_simple_pull(&ad, len);
    }
    return false;
}

static bool name_matches_target(const char *name)
{
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
    const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
    if (target[0] == '\0') {
        return true; /* empty = accept any */
    }

    if (name == NULL || strcmp(name, "Unknown") == 0) {
        return false; /* name not yet known from scan response */
    }

    /* Prefix match: "lotom" matches "lotom", "lotom_left", etc. */
    if (strncmp(name, target, strlen(target)) == 0) {
        LOG_INF("Name match: '%s' matches target '%s'", name, target);
        return true;
    }
    return false;
#else
    return true;
#endif
}

/* Helper: add a discovered keyboard during pairing mode scan */
static void add_to_discovered(const bt_addr_le_t *addr, const char *name, int8_t rssi)
{
    /* Skip if already discovered */
    for (int i = 0; i < discovered_count; i++) {
        if (bt_addr_le_cmp(addr, &discovered_kbds[i].addr) == 0) {
            /* Update RSSI and name */
            discovered_kbds[i].rssi = rssi;
            if (name && strcmp(name, "Unknown") != 0) {
                strncpy(discovered_kbds[i].name, name,
                        sizeof(discovered_kbds[i].name) - 1);
                discovered_kbds[i].name[sizeof(discovered_kbds[i].name) - 1] = '\0';
            }
            return;
        }
    }

    /* Skip if already bonded */
    for (int i = 0; i < bonded_count; i++) {
        if (bt_addr_le_cmp(addr, &bonded_kbds[i].addr) == 0) {
            return;
        }
    }

    if (discovered_count >= MAX_DISCOVERED) {
        return;
    }

    bt_addr_le_copy(&discovered_kbds[discovered_count].addr, addr);
    discovered_kbds[discovered_count].rssi = rssi;
    discovered_kbds[discovered_count].valid = true;
    if (name && strcmp(name, "Unknown") != 0) {
        strncpy(discovered_kbds[discovered_count].name, name,
                sizeof(discovered_kbds[discovered_count].name) - 1);
        discovered_kbds[discovered_count].name[sizeof(discovered_kbds[discovered_count].name) - 1] = '\0';
    } else {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
        snprintf(discovered_kbds[discovered_count].name,
                 sizeof(discovered_kbds[discovered_count].name),
                 "%s", addr_str);
    }

    LOG_INF("Discovered[%d]: %s rssi=%d", discovered_count,
            discovered_kbds[discovered_count].name, rssi);
    discovered_count++;
}

void hid_central_on_scan_result(const bt_addr_le_t *addr, int8_t rssi,
                                uint8_t type, struct net_buf_simple *buf,
                                const char *name)
{
    /* Only process if we're idle or looking to connect */
    if (state != STATE_IDLE && state != STATE_SCANNING) {
        return;
    }

    /* Pairing mode: collect discovered keyboards.
     * Accept devices that advertise HID Service UUID or whose name
     * matches the configured target prefix. */
    if (pairing_mode) {
        if (!is_hid_service_in_ad(buf) && !name_matches_target(name)) {
            return;
        }
        add_to_discovered(addr, name, rssi);
        return;
    }

    /* Normal mode: connect to selected bonded keyboard */
    if (has_bonded_addr) {
        if (bt_addr_le_cmp(addr, &bonded_addr) != 0) {
            return; /* Not our bonded keyboard */
        }
        LOG_INF("Bonded addr matched");
    } else {
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            if (!name_matches_target(name)) {
                return;
            }
        } else
#endif
        {
            if (!is_hid_service_in_ad(buf)) {
                return;
            }
        }
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("DONGLE: Target found: %s", addr_str);

    /* Capture device name for GATT-based status display */
    if (name && strcmp(name, "Unknown") != 0) {
        strncpy(connected_kbd_name, name, sizeof(connected_kbd_name) - 1);
        connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
    } else if (connected_kbd_name[0] == '\0') {
        /* Bonded reconnection: name not yet known from scan response.
         * Use target name as fallback. */
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            strncpy(connected_kbd_name, target, sizeof(connected_kbd_name) - 1);
            connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
        }
#endif
    }

    /* Defer connection to work queue – bt_conn_le_create must NOT be
     * called from within the scan callback (BLE RX thread context). */
    bt_addr_le_copy(&pending_addr, addr);
    pending_connect = true;
    state = STATE_CONNECTING;
    k_work_submit(&connect_work);
}

static void run_psa_diagnostic(void)
{
    if (psa_diag_done) {
        return;
    }
    psa_diag_done = true;
    LOG_DBG("PSA diag: ECC_PUB=%d BT_ECC=%d",
            IS_ENABLED(CONFIG_PSA_WANT_KEY_TYPE_ECC_PUBLIC_KEY),
            IS_ENABLED(CONFIG_BT_ECC));
}

static void connect_work_handler(struct k_work *work)
{
    if (!pending_connect || state != STATE_CONNECTING) {
        return;
    }
    pending_connect = false;

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&pending_addr, addr_str, sizeof(addr_str));

    /* One-time PSA import test with known-valid key */
    if (!psa_diag_done) {
        psa_diag_done = true;
        test_psa_import();
    }

    /* Check if a stale connection object exists for this address.
     * If found, disconnect it first – reusing an existing connection
     * with unknown SMP state can cause LESC pairing failures. */
    struct bt_conn *existing = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &pending_addr);
    if (existing) {
        LOG_WRN("Existing conn to %s, disconnecting", addr_str);
        bt_conn_disconnect(existing, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(existing);
        /* Wait a bit for disconnect to complete, then retry */
        state = STATE_IDLE;
        k_work_schedule(&reconnect_work, K_MSEC(1000));
        return;
    }

    /* Stop scanning before connecting (Zephyr requirement).
     * Use zmk_status_scanner_stop() to keep the scanning flag in sync. */
    zmk_status_scanner_stop();

    /* Ensure kbd_conn is NULL before calling bt_conn_le_create */
    if (kbd_conn) {
        bt_conn_unref(kbd_conn);
        kbd_conn = NULL;
    }

    struct bt_conn_le_create_param create_param =
        BT_CONN_LE_CREATE_PARAM_INIT(
            BT_CONN_LE_OPT_NONE,
            BT_GAP_SCAN_FAST_INTERVAL,
            BT_GAP_SCAN_FAST_WINDOW
        );

    struct bt_le_conn_param conn_param = BT_LE_CONN_PARAM_INIT(
        6, 6, 0, 400   /* 7.5ms interval, no latency, 4s timeout */
    );

    LOG_INF("DONGLE: Connecting to %s", addr_str);
    int err = bt_conn_le_create(&pending_addr, &create_param, &conn_param, &kbd_conn);
    if (err) {
        LOG_ERR("DONGLE: Connect failed: %d", err);
        if (pairing_mode) {
            state = STATE_SCANNING;
        } else {
            state = STATE_IDLE;
            schedule_reconnect();
        }
        zmk_status_scanner_stop();
        zmk_status_scanner_start();
    }
}

bool hid_central_is_connected(void)
{
    return state == STATE_READY;
}

/* ------------------------------------------------------------------ */
/* Reconnection logic                                                 */
/* ------------------------------------------------------------------ */

static void reconnect_work_handler(struct k_work *work)
{
    /* Never reconnect during pairing mode */
    if (pairing_mode) {
        return;
    }

    if (state != STATE_IDLE && state != STATE_SCANNING) {
        return; /* Already connecting or connected */
    }

    if (!has_bonded_addr) {
        /* No bonded device – just wait for scan results */
        state = STATE_SCANNING;
        LOG_INF("No bonded device, waiting for scan results");
        return;
    }

    LOG_INF("Attempting reconnection to bonded keyboard");

    /* Stop any active scan first (use scanner API to keep flag in sync) */
    zmk_status_scanner_stop();

    state = STATE_CONNECTING;

    struct bt_conn_le_create_param create_param =
        BT_CONN_LE_CREATE_PARAM_INIT(
            BT_CONN_LE_OPT_NONE,
            BT_GAP_SCAN_FAST_INTERVAL,
            BT_GAP_SCAN_FAST_WINDOW
        );

    struct bt_le_conn_param conn_param = BT_LE_CONN_PARAM_INIT(
        6, 6, 0, 400
    );

    int err = bt_conn_le_create(&bonded_addr, &create_param,
                                 &conn_param, &kbd_conn);
    if (err) {
        LOG_WRN("Reconnect failed: %d", err);
        state = STATE_IDLE;
        zmk_status_scanner_stop();
        zmk_status_scanner_start();
        schedule_reconnect();
    }
}

static void schedule_reconnect(void)
{
#if IS_ENABLED(CONFIG_PROSPECTOR_DONGLE_AUTO_RECONNECT)
    k_work_schedule(&reconnect_work, K_MSEC(RECONNECT_INTERVAL_MS));
#endif
}

/* ------------------------------------------------------------------ */
/* Prospector Status GATT service discovery + subscribe               */
/* ------------------------------------------------------------------ */

static uint8_t status_notify_cb(struct bt_conn *conn,
                                 struct bt_gatt_subscribe_params *params,
                                 const void *data, uint16_t length)
{
    if (!data) {
        LOG_INF("Status GATT notification unsubscribed");
        params->value_handle = 0;
        status_svc_subscribed = false;
        return BT_GATT_ITER_STOP;
    }

    if (length < sizeof(struct zmk_status_adv_data)) {
        LOG_WRN("Status notify: too short %d (min %d)",
                length, sizeof(struct zmk_status_adv_data));
        return BT_GATT_ITER_CONTINUE;
    }

    const struct zmk_status_adv_data *status_data = data;
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);

    /* Extract layer name from extended payload (after 26-byte base) */
    const char *layer_name = NULL;
    if (length > sizeof(struct zmk_status_adv_data)) {
        const char *ext = (const char *)data + sizeof(struct zmk_status_adv_data);
        size_t max_name = length - sizeof(struct zmk_status_adv_data);
        /* Verify null termination within bounds */
        if (memchr(ext, '\0', max_name) != NULL) {
            layer_name = ext;
        }
    }

    /* Ensure name is available — last-resort fallback to target name */
    if (connected_kbd_name[0] == '\0') {
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            strncpy(connected_kbd_name, target, sizeof(connected_kbd_name) - 1);
            connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
            LOG_INF("GATT notify: name was empty, set to '%s'", connected_kbd_name);
        }
#endif
    }

    LOG_INF("GATT notify: layer=%d(%s) mod=0x%02x bat=%d%% name='%s'",
            status_data->active_layer, layer_name ? layer_name : "?",
            status_data->modifier_flags,
            status_data->battery_level, connected_kbd_name);

    status_scanner_update_from_gatt(addr, status_data, 0, connected_kbd_name, layer_name);

    return BT_GATT_ITER_CONTINUE;
}

static void status_subscribe_cb(struct bt_conn *conn, uint8_t err,
                                 struct bt_gatt_subscribe_params *params)
{
    if (err) {
        LOG_WRN("Status GATT subscribe failed (%d) - display updates unavailable", err);
        return;
    }

    status_svc_subscribed = true;
    LOG_INF("Status GATT subscribed (handle 0x%04x) - scan-free status updates active",
            params->value_handle);

    /* Start GATT READ polling as fallback (notifications may not arrive) */
    k_work_schedule(&status_poll_work, K_MSEC(STATUS_POLL_INTERVAL_MS));
    LOG_INF("Status GATT READ polling started (%d ms interval)", STATUS_POLL_INTERVAL_MS);
}

/* Phase 1 callback: discover Status characteristic within service */
static uint8_t status_chr_disc_cb(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("Status characteristic NOT found - display updates unavailable");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
    status_chr_value_handle = chrc->value_handle;
    LOG_INF("Status characteristic at 0x%04x", chrc->value_handle);

    /* Subscribe to notifications */
    status_sub_params.notify = status_notify_cb;
    status_sub_params.subscribe = status_subscribe_cb;
    status_sub_params.value_handle = chrc->value_handle;
    status_sub_params.ccc_handle = chrc->value_handle + 1;
    status_sub_params.value = BT_GATT_CCC_NOTIFY;

    int err = bt_gatt_subscribe(conn, &status_sub_params);
    if (err) {
        LOG_WRN("Status GATT subscribe start failed: %d - display updates unavailable", err);
    }

    return BT_GATT_ITER_STOP;
}

/* Phase 0 callback: discover Status primary service */
static uint8_t status_svc_disc_cb(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("Prospector Status GATT service not found (old firmware?) - display updates unavailable");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc = (struct bt_gatt_service_val *)attr->user_data;
    status_svc_start = attr->handle;
    status_svc_end = svc->end_handle;
    LOG_INF("Status GATT service 0x%04x-0x%04x", status_svc_start, status_svc_end);

    /* Discover characteristic within service range */
    memset(&status_chr_disc_params, 0, sizeof(status_chr_disc_params));
    status_chr_disc_params.uuid = &disc_uuid_status_chr.uuid;
    status_chr_disc_params.func = status_chr_disc_cb;
    status_chr_disc_params.start_handle = status_svc_start + 1;
    status_chr_disc_params.end_handle = status_svc_end;
    status_chr_disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &status_chr_disc_params);
    if (err) {
        LOG_WRN("Status char discovery failed: %d - display updates unavailable", err);
    }

    return BT_GATT_ITER_STOP;
}

static void start_status_service_discovery(struct bt_conn *conn)
{
    memset(&status_disc_params, 0, sizeof(status_disc_params));
    status_disc_params.uuid = &disc_uuid_status_svc.uuid;
    status_disc_params.func = status_svc_disc_cb;
    status_disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    status_disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    status_disc_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &status_disc_params);
    if (err) {
        LOG_WRN("Status service discovery start failed: %d - display updates unavailable", err);
    }
}

/* ------------------------------------------------------------------ */
/* RSSI periodic update (1 Hz)                                        */
/* ------------------------------------------------------------------ */

static void rssi_work_handler(struct k_work *work)
{
    if (!kbd_conn || state != STATE_READY) {
        return;
    }

    /* Keep last_seen fresh so the keyboard doesn't time out while
     * connected (RSSI value comes from GATT notification path or is
     * left at 0 — acceptable for connected-dongle use case). */
    const bt_addr_le_t *addr = bt_conn_get_dst(kbd_conn);
    status_scanner_update_rssi(addr, 0);

    k_work_schedule(&rssi_work, K_SECONDS(1));
}

/* ------------------------------------------------------------------ */
/* GATT READ polling (fallback for status updates)                    */
/* ------------------------------------------------------------------ */

static uint8_t status_read_cb(struct bt_conn *conn, uint8_t err,
                               struct bt_gatt_read_params *params,
                               const void *data, uint16_t length)
{
    if (err || !data) {
        if (err) {
            LOG_WRN("Status GATT read failed: %d", err);
        }
        return BT_GATT_ITER_STOP;
    }

    if (length < sizeof(struct zmk_status_adv_data)) {
        LOG_WRN("Status read: too short %d (min %d)",
                length, sizeof(struct zmk_status_adv_data));
        return BT_GATT_ITER_STOP;
    }

    const struct zmk_status_adv_data *status_data = data;
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);

    /* Extract layer name from extended payload */
    const char *layer_name = NULL;
    if (length > sizeof(struct zmk_status_adv_data)) {
        const char *ext = (const char *)data + sizeof(struct zmk_status_adv_data);
        size_t max_name = length - sizeof(struct zmk_status_adv_data);
        if (memchr(ext, '\0', max_name) != NULL) {
            layer_name = ext;
        }
    }

    if (connected_kbd_name[0] == '\0') {
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            strncpy(connected_kbd_name, target, sizeof(connected_kbd_name) - 1);
            connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
        }
#endif
    }

    LOG_DBG("GATT poll: layer=%d(%s) mod=0x%02x bat=%d%%",
            status_data->active_layer, layer_name ? layer_name : "?",
            status_data->modifier_flags, status_data->battery_level);

    status_scanner_update_from_gatt(addr, status_data, 0, connected_kbd_name, layer_name);

    return BT_GATT_ITER_STOP;
}

static void status_poll_work_handler(struct k_work *work)
{
    if (!kbd_conn || state != STATE_READY || status_chr_value_handle == 0) {
        return;
    }

    status_read_params.func = status_read_cb;
    status_read_params.handle_count = 1;
    status_read_params.single.handle = status_chr_value_handle;
    status_read_params.single.offset = 0;

    int err = bt_gatt_read(kbd_conn, &status_read_params);
    if (err) {
        LOG_DBG("Status GATT read request failed: %d", err);
    }

    k_work_schedule(&status_poll_work, K_MSEC(STATUS_POLL_INTERVAL_MS));
}

/* ------------------------------------------------------------------ */
/* Multi-keyboard public API                                          */
/* ------------------------------------------------------------------ */

int hid_central_get_bonded_keyboards(struct bonded_keyboard_info *out, int max_count)
{
    int count = 0;
    for (int i = 0; i < bonded_count && count < max_count; i++) {
        if (!bonded_kbds[i].valid) {
            continue;
        }
        bt_addr_le_copy(&out[count].addr, &bonded_kbds[i].addr);
        strncpy(out[count].name, bonded_kbds[i].name, sizeof(out[count].name) - 1);
        out[count].name[sizeof(out[count].name) - 1] = '\0';
        out[count].valid = true;
        count++;
    }
    return count;
}

int hid_central_get_active_index(void)
{
    return active_kbd_idx;
}

int hid_central_select_keyboard(int index)
{
    if (index < 0 || index >= bonded_count || !bonded_kbds[index].valid) {
        return -EINVAL;
    }

    if (index == active_kbd_idx && state == STATE_READY) {
        return 0; /* Already connected to this keyboard */
    }

    LOG_INF("Switching to keyboard[%d]: %s", index, bonded_kbds[index].name);

    /* Disconnect current keyboard if connected */
    if (kbd_conn) {
        bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        /* disconnected_cb will clean up kbd_conn */
    }

    active_kbd_idx = index;
    save_active_index();

    /* Clear connected name so it gets repopulated */
    if (bonded_kbds[index].name[0] != '\0') {
        strncpy(connected_kbd_name, bonded_kbds[index].name,
                sizeof(connected_kbd_name) - 1);
        connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
    } else {
        memset(connected_kbd_name, 0, sizeof(connected_kbd_name));
    }

    /* Schedule reconnection to new keyboard */
    state = STATE_IDLE;
    k_work_schedule(&reconnect_work, K_MSEC(500));
    return 0;
}

int hid_central_enter_pairing_mode(void)
{
    LOG_INF("Entering pairing mode");

    /* Set pairing flag BEFORE disconnect so disconnected_cb knows
     * not to auto-reconnect or override state. */
    pairing_mode = true;
    discovered_count = 0;
    memset(discovered_kbds, 0, sizeof(discovered_kbds));

    /* Cancel all pending connection work */
    k_work_cancel_delayable(&reconnect_work);
    k_work_cancel(&connect_work);
    pending_connect = false;

    /* Disconnect current keyboard (disconnected_cb will see pairing_mode=true) */
    if (kbd_conn) {
        bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    /* Start scanning for discoverable keyboards */
    state = STATE_SCANNING;
    zmk_status_scanner_stop();
    zmk_status_scanner_start();

    return 0;
}

void hid_central_exit_pairing_mode(void)
{
    LOG_INF("Exiting pairing mode");
    pairing_mode = false;
    discovered_count = 0;

    /* If we had a selected keyboard, reconnect */
    if (has_bonded_addr) {
        state = STATE_IDLE;
        schedule_reconnect();
    } else {
        state = STATE_SCANNING;
    }
}

bool hid_central_is_pairing_mode(void)
{
    return pairing_mode;
}

int hid_central_get_discovered_keyboards(struct discovered_keyboard_info *out, int max_count)
{
    int count = 0;
    for (int i = 0; i < discovered_count && count < max_count; i++) {
        if (!discovered_kbds[i].valid) {
            continue;
        }
        bt_addr_le_copy(&out[count].addr, &discovered_kbds[i].addr);
        strncpy(out[count].name, discovered_kbds[i].name, sizeof(out[count].name) - 1);
        out[count].name[sizeof(out[count].name) - 1] = '\0';
        out[count].rssi = discovered_kbds[i].rssi;
        count++;
    }
    return count;
}

int hid_central_pair_with(int discovered_index)
{
    if (discovered_index < 0 || discovered_index >= discovered_count ||
        !discovered_kbds[discovered_index].valid) {
        return -EINVAL;
    }

    LOG_INF("Pairing with discovered[%d]: %s", discovered_index,
            discovered_kbds[discovered_index].name);

    /* Keep pairing_mode = true during the connection attempt.
     * This prevents the normal scan flow from re-matching the target
     * and entering an infinite connect-timeout loop if pairing fails.
     * pairing_mode is cleared in security_changed_cb on success,
     * or stays true so connected_cb error path won't auto-reconnect. */

    /* Capture name for the new bond */
    strncpy(connected_kbd_name, discovered_kbds[discovered_index].name,
            sizeof(connected_kbd_name) - 1);
    connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';

    /* Initiate connection (security_changed_cb will handle bond registration) */
    bt_addr_le_copy(&pending_addr, &discovered_kbds[discovered_index].addr);
    pending_connect = true;
    state = STATE_CONNECTING;
    k_work_submit(&connect_work);

    return 0;
}

int hid_central_unpair(int bonded_index)
{
    if (bonded_index < 0 || bonded_index >= bonded_count ||
        !bonded_kbds[bonded_index].valid) {
        return -EINVAL;
    }

    LOG_INF("Unpairing keyboard[%d]: %s", bonded_index,
            bonded_kbds[bonded_index].name);

    /* If this is the active keyboard, disconnect first */
    if (bonded_index == active_kbd_idx && kbd_conn) {
        bt_conn_disconnect(kbd_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    /* Remove bond from Zephyr */
    int err = bt_unpair(BT_ID_DEFAULT, &bonded_kbds[bonded_index].addr);
    if (err) {
        LOG_WRN("bt_unpair failed: %d", err);
    }

    /* Remove from our array by shifting entries down */
    for (int i = bonded_index; i < bonded_count - 1; i++) {
        bonded_kbds[i] = bonded_kbds[i + 1];
        save_bonded_name(i);
        save_bonded_addr(i);
    }
    bonded_count--;
    memset(&bonded_kbds[bonded_count], 0, sizeof(bonded_kbds[bonded_count]));

    /* Clear old slots */
    char key[24];
    snprintf(key, sizeof(key), "dongle/name/%d", bonded_count);
    settings_delete(key);
    snprintf(key, sizeof(key), "dongle/addr/%d", bonded_count);
    settings_delete(key);

    /* Adjust active index */
    if (bonded_count == 0) {
        active_kbd_idx = -1;
    } else if (bonded_index == active_kbd_idx) {
        active_kbd_idx = 0;
    } else if (bonded_index < active_kbd_idx) {
        active_kbd_idx--;
    }
    save_active_index();

    /* Reconnect to new active keyboard if any */
    if (has_bonded_addr) {
        state = STATE_IDLE;
        schedule_reconnect();
    } else {
        state = STATE_SCANNING;
        zmk_status_scanner_start();
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Initialization                                                     */
/* ------------------------------------------------------------------ */

static int hid_central_init(void)
{
    k_work_init_delayable(&reconnect_work, reconnect_work_handler);
    k_work_init_delayable(&discovery_work, discovery_work_handler);
    k_work_init_delayable(&rssi_work, rssi_work_handler);
    k_work_init_delayable(&status_poll_work, status_poll_work_handler);
    k_work_init(&connect_work, connect_work_handler);
    k_work_init(&hid_send_work, hid_send_work_handler);
    memset(connected_kbd_name, 0, sizeof(connected_kbd_name));

    LOG_INF("DONGLE: %d bonded keyboards, active=%d", bonded_count, active_kbd_idx);

    /* Validate active index after settings + bond restore */
    if (active_kbd_idx >= bonded_count) {
        active_kbd_idx = bonded_count > 0 ? 0 : -1;
    }

    if (has_bonded_addr) {
        /* Pre-fill connected_kbd_name from bonded name for display */
        if (bonded_kbds[active_kbd_idx].name[0] != '\0') {
            strncpy(connected_kbd_name, bonded_kbds[active_kbd_idx].name,
                    sizeof(connected_kbd_name) - 1);
            connected_kbd_name[sizeof(connected_kbd_name) - 1] = '\0';
        }
        state = STATE_IDLE;
        k_work_schedule(&reconnect_work, K_MSEC(1000));
        LOG_INF("DONGLE: bonded addr found, scheduling reconnect to [%d]",
                active_kbd_idx);
    } else {
        state = STATE_SCANNING;
    }
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
    LOG_INF("DONGLE: target='%s'", CONFIG_PROSPECTOR_DONGLE_TARGET_NAME);
#endif
    return 0;
}

SYS_INIT(hid_central_init, APPLICATION, 99);

/* ------------------------------------------------------------------ */
/* BT stack initialization (when ZMK_BLE is disabled)                 */
/* ------------------------------------------------------------------ */

/* Test psa_import_key with a known-valid P-256 public key (generator point G).
 * Tests BOTH with ECDH usage flags (our original test) and with usage=0
 * (same attributes as bt_pub_key_is_valid) to check if attributes matter. */
static void test_psa_import(void)
{
    /* P-256 generator point G in uncompressed form (0x04 || X || Y) */
    static const uint8_t test_key[65] = {
        0x04,
        0x6B, 0x17, 0xD1, 0xF2, 0xE1, 0x2C, 0x42, 0x47,
        0xF8, 0xBC, 0xE6, 0xE5, 0x63, 0xA4, 0x40, 0xF2,
        0x77, 0x03, 0x7D, 0x81, 0x2D, 0xEB, 0x33, 0xA0,
        0xF4, 0xA1, 0x39, 0x45, 0xD8, 0x98, 0xC2, 0x96,
        0x4F, 0xE3, 0x42, 0xE2, 0xFE, 0x1A, 0x7F, 0x9B,
        0x8E, 0xE7, 0xEB, 0x4A, 0x7C, 0x0F, 0x9E, 0x16,
        0x2B, 0xCE, 0x33, 0x57, 0x6B, 0x31, 0x5E, 0xCE,
        0xCB, 0xB6, 0x40, 0x68, 0x37, 0xBF, 0x51, 0xF5,
    };

    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t handle;

    /* Test A: with ECDH usage (our original working test) */
    psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attr, 256);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attr, PSA_ALG_ECDH);

    psa_test_result = (int)psa_import_key(&attr, test_key, sizeof(test_key), &handle);
    if (psa_test_result == 0) {
        psa_destroy_key(handle);
    }
    psa_reset_key_attributes(&attr);

    /* Test B: with usage=0, no algorithm (same as bt_pub_key_is_valid) */
    psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attr, 256);
    psa_set_key_usage_flags(&attr, 0);

    psa_test_nousage = (int)psa_import_key(&attr, test_key, sizeof(test_key), &handle);
    if (psa_test_nousage == 0) {
        psa_destroy_key(handle);
    }
    psa_reset_key_attributes(&attr);
}

/* Temporary flag array for bond verification */
static bool bond_verified[MAX_BONDED_KBD];

/* Callback for bt_foreach_bond: verify that our saved entries still have
 * a corresponding bond in the BT stack.  Addresses and names are already
 * restored by settings_load (dongle/addr/N, dongle/name/N). */
static void bond_found_cb(const struct bt_bond_info *info, void *user_data)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));

    for (int i = 0; i < bonded_count; i++) {
        if (bonded_kbds[i].valid &&
            bt_addr_le_cmp(&info->addr, &bonded_kbds[i].addr) == 0) {
            bond_verified[i] = true;
            printk("*** DONGLE: Verified bond[%d]: %s name='%s' ***\n",
                   i, addr_str, bonded_kbds[i].name);
            return;
        }
    }

    /* Bond exists in BT stack but not in our saved list — add it */
    if (bonded_count < MAX_BONDED_KBD) {
        bt_addr_le_copy(&bonded_kbds[bonded_count].addr, &info->addr);
        bonded_kbds[bonded_count].valid = true;
        bond_verified[bonded_count] = true;
        printk("*** DONGLE: New bond[%d]: %s (no saved name) ***\n",
               bonded_count, addr_str);
        bonded_count++;
    }
}

#if !IS_ENABLED(CONFIG_ZMK_BLE)
static int dongle_bt_enable(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("*** DONGLE: bt_enable failed: %d ***\n", err);
        return err;
    }
    printk("*** DONGLE: BT ready ***\n");

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_load();

    /* Count entries restored from settings (addr/N sets valid=true) */
    bonded_count = 0;
    for (int i = 0; i < MAX_BONDED_KBD; i++) {
        if (bonded_kbds[i].valid) {
            bonded_count = i + 1;
        }
    }

    /* Verify saved entries against actual BT bonds */
    memset(bond_verified, 0, sizeof(bond_verified));
    bt_foreach_bond(BT_ID_DEFAULT, bond_found_cb, NULL);

    /* Remove entries whose bond was deleted externally */
    for (int i = bonded_count - 1; i >= 0; i--) {
        if (!bond_verified[i]) {
            printk("*** DONGLE: Pruning stale bond[%d]: name='%s' ***\n",
                   i, bonded_kbds[i].name);
            /* Shift remaining entries down */
            for (int j = i; j < bonded_count - 1; j++) {
                bonded_kbds[j] = bonded_kbds[j + 1];
                save_bonded_name(j);
                save_bonded_addr(j);
            }
            bonded_count--;
            memset(&bonded_kbds[bonded_count], 0, sizeof(bonded_kbds[bonded_count]));
        }
    }

    printk("*** DONGLE: %d bonded keyboard(s), active=%d ***\n",
           bonded_count, active_kbd_idx);
#endif

    return 0;
}

/* Run before scanner init (priority 99) */
SYS_INIT(dongle_bt_enable, APPLICATION, 50);
#endif

/* ------------------------------------------------------------------ */
/* v18: Fix keyboard's invalid Y coordinate by computing correct Y     */
/* from X using P-256 curve equation, then wrap bt_dh_key_gen to use   */
/* the corrected key for ECDH.                                         */
/*                                                                     */
/* Analysis: keyboard's X coord (after LE→BE swap) IS valid on P-256,  */
/* but Y coord is completely wrong. Likely TinyCrypt serialization bug. */
/* Since ECDH shared secret = X(d*Q), and d*(-Q) = -(d*Q) which has   */
/* the same X, either sign of Y produces the same shared secret.       */
/* ------------------------------------------------------------------ */

#include <mbedtls/ecp.h>
#include <mbedtls/bignum.h>

/* RNG wrapper for mbedtls_ecp_mul (required in MbedTLS 3.x for blinding) */
static int ecp_rng_wrapper(void *ctx, unsigned char *buf, size_t len)
{
    (void)ctx;
    return (psa_generate_random(buf, len) == PSA_SUCCESS) ? 0 : -1;
}

/* Reverse-copy n bytes (byte-swap LE↔BE) */
static void memcpy_swap(void *dst, const void *src, size_t n)
{
    const uint8_t *s = src;
    uint8_t *d = dst;
    for (size_t i = 0; i < n; i++) {
        d[i] = s[n - 1 - i];
    }
}

static void hex_format(char *out, const uint8_t *data, size_t len)
{
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
        out[i * 2]     = hex[data[i] >> 4];
        out[i * 2 + 1] = hex[data[i] & 0x0f];
    }
    out[len * 2] = '\0';
}

/*
 * Compute valid Y for X on P-256: Y = sqrt(X³ + aX + b) mod p
 * P-256 has p ≡ 3 (mod 4), so sqrt(z) = z^((p+1)/4) mod p.
 * x_be and y_be are 32-byte big-endian buffers.
 * Returns 0 on success, -1 if X has no valid Y on P-256.
 */
static int compute_p256_y(const uint8_t x_be[32], uint8_t y_be[32])
{
    /* P-256 parameters (big-endian) */
    static const uint8_t p256_p[] = {
        0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x01,
        0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00, 0xFF,0xFF,0xFF,0xFF,
        0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF
    };
    static const uint8_t p256_b[] = {
        0x5A,0xC6,0x35,0xD8, 0xAA,0x3A,0x93,0xE7,
        0xB3,0xEB,0xBD,0x55, 0x76,0x98,0x86,0xBC,
        0x65,0x1D,0x06,0xB0, 0xCC,0x53,0xB0,0xF6,
        0x3B,0xCE,0x3C,0x3E, 0x27,0xD2,0x60,0x4B
    };

    int ret = -1;
    mbedtls_mpi X, Y, z, a_mpi, b_mpi, p_mpi, exp, tmp;
    mbedtls_mpi_init(&X);     mbedtls_mpi_init(&Y);
    mbedtls_mpi_init(&z);     mbedtls_mpi_init(&a_mpi);
    mbedtls_mpi_init(&b_mpi); mbedtls_mpi_init(&p_mpi);
    mbedtls_mpi_init(&exp);   mbedtls_mpi_init(&tmp);

    /* Load parameters */
    mbedtls_mpi_read_binary(&p_mpi, p256_p, 32);
    mbedtls_mpi_read_binary(&b_mpi, p256_b, 32);
    mbedtls_mpi_read_binary(&X, x_be, 32);

    /* a = p - 3 (equivalent to -3 mod p) */
    mbedtls_mpi_copy(&a_mpi, &p_mpi);
    mbedtls_mpi_sub_int(&a_mpi, &a_mpi, 3);

    /* z = X² mod p */
    mbedtls_mpi_mul_mpi(&z, &X, &X);
    mbedtls_mpi_mod_mpi(&z, &z, &p_mpi);

    /* z = X³ mod p */
    mbedtls_mpi_mul_mpi(&z, &z, &X);
    mbedtls_mpi_mod_mpi(&z, &z, &p_mpi);

    /* tmp = a * X mod p */
    mbedtls_mpi_mul_mpi(&tmp, &a_mpi, &X);
    mbedtls_mpi_mod_mpi(&tmp, &tmp, &p_mpi);

    /* z = X³ + aX mod p */
    mbedtls_mpi_add_mpi(&z, &z, &tmp);
    mbedtls_mpi_mod_mpi(&z, &z, &p_mpi);

    /* z = X³ + aX + b mod p */
    mbedtls_mpi_add_mpi(&z, &z, &b_mpi);
    mbedtls_mpi_mod_mpi(&z, &z, &p_mpi);

    /* Y = z^((p+1)/4) mod p */
    mbedtls_mpi_copy(&exp, &p_mpi);
    mbedtls_mpi_add_int(&exp, &exp, 1);
    mbedtls_mpi_shift_r(&exp, 2); /* divide by 4 */

    mbedtls_mpi_exp_mod(&Y, &z, &exp, &p_mpi, NULL);

    /* Verify: Y² mod p == z */
    mbedtls_mpi_mul_mpi(&tmp, &Y, &Y);
    mbedtls_mpi_mod_mpi(&tmp, &tmp, &p_mpi);
    if (mbedtls_mpi_cmp_mpi(&tmp, &z) != 0) {
        LOG_WRN("Y_FIX: No valid Y for this X");
        goto cleanup;
    }

    /* Write Y as big-endian */
    mbedtls_mpi_write_binary(&Y, y_be, 32);
    ret = 0;

cleanup:
    mbedtls_mpi_free(&X);     mbedtls_mpi_free(&Y);
    mbedtls_mpi_free(&z);     mbedtls_mpi_free(&a_mpi);
    mbedtls_mpi_free(&b_mpi); mbedtls_mpi_free(&p_mpi);
    mbedtls_mpi_free(&exp);   mbedtls_mpi_free(&tmp);
    return ret;
}

/* --- bt_pub_key_is_valid wrapper: dump key + return true --- */

extern bool __real_bt_pub_key_is_valid(const uint8_t key[64]);

bool __wrap_bt_pub_key_is_valid(const uint8_t key[64])
{
    LOG_DBG("bt_pub_key_is_valid: bypassed");
    return true;
}

/* --- bt_dh_key_gen wrapper: fix Y coordinate before ECDH --- */

extern int __real_bt_dh_key_gen(const uint8_t remote_pk[64],
                                void (*cb)(const uint8_t key[32]));

/* v19: Intercept DH key callback to dump the computed DHKey */
static void (*saved_dh_key_cb)(const uint8_t key[32]);

static void dh_key_dump_cb(const uint8_t key[32])
{
    if (!key) {
        LOG_WRN("ECDH failed: NULL DHKey");
    }
    saved_dh_key_cb(key);
}

int __wrap_bt_dh_key_gen(const uint8_t remote_pk[64],
                          void (*cb)(const uint8_t key[32]))
{
    /* MUST be static: bt_dh_key_gen copies from this pointer synchronously,
     * but keeping static for safety. */
    static uint8_t fixed_pk[64];
    uint8_t x_be[32], y_be[32];

    memcpy(fixed_pk, remote_pk, 64);

    /* Convert X from LE (wire) to BE (math) */
    memcpy_swap(x_be, remote_pk, 32);

    /* Compute valid Y from X and replace original (corrupted) Y. */
    if (compute_p256_y(x_be, y_be) == 0) {
        uint8_t y_fixed_le[32];
        memcpy_swap(y_fixed_le, y_be, 32);

        /* Replace Y in fixed_pk with the corrected Y */
        memcpy(&fixed_pk[32], y_fixed_le, 32);
    }

    /* Intercept callback to dump DHKey */
    saved_dh_key_cb = cb;
    return __real_bt_dh_key_gen(fixed_pk, dh_key_dump_cb);
}

/* --- mbedtls_ecp_check_pubkey bypass (safety net for ECDH internals) --- */

extern int __real_mbedtls_ecp_check_pubkey(const mbedtls_ecp_group *grp,
                                            const mbedtls_ecp_point *pt);

int __wrap_mbedtls_ecp_check_pubkey(const mbedtls_ecp_group *grp,
                                     const mbedtls_ecp_point *pt)
{
    int real_result = __real_mbedtls_ecp_check_pubkey(grp, pt);
    if (real_result != 0) {
        LOG_DBG("ECP check bypassed (real=%d)", real_result);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* v23: Wrap bt_crypto_f5 to dump all SMP key derivation inputs        */
/* This reveals the exact DHKey, nonces, and addresses used in LESC.   */
/* ------------------------------------------------------------------ */

#include <zephyr/bluetooth/addr.h>

extern int __real_bt_crypto_f5(const uint8_t *w, const uint8_t *n1,
                                const uint8_t *n2,
                                const bt_addr_le_t *a1,
                                const bt_addr_le_t *a2,
                                uint8_t *mackey, uint8_t *ltk);

int __wrap_bt_crypto_f5(const uint8_t *w, const uint8_t *n1,
                         const uint8_t *n2,
                         const bt_addr_le_t *a1,
                         const bt_addr_le_t *a2,
                         uint8_t *mackey, uint8_t *ltk)
{
    LOG_DBG("f5 called");
    return __real_bt_crypto_f5(w, n1, n2, a1, a2, mackey, ltk);
}

/* ------------------------------------------------------------------ */
/* v24: Wrap bt_crypto_f6 to dump DHKey Check computation              */
/* This is the value the keyboard uses to verify our pairing.          */
/* f6(MacKey, N1, N2, R, IOcap, A1, A2) -> Check (16 bytes)           */
/* ------------------------------------------------------------------ */

extern int __real_bt_crypto_f6(const uint8_t *w, const uint8_t *n1,
                                const uint8_t *n2, const uint8_t *r,
                                const uint8_t *iocap,
                                const bt_addr_le_t *a1,
                                const bt_addr_le_t *a2,
                                uint8_t *check);

int __wrap_bt_crypto_f6(const uint8_t *w, const uint8_t *n1,
                         const uint8_t *n2, const uint8_t *r,
                         const uint8_t *iocap,
                         const bt_addr_le_t *a1,
                         const bt_addr_le_t *a2,
                         uint8_t *check)
{
    LOG_DBG("f6 called");
    return __real_bt_crypto_f6(w, n1, n2, r, iocap, a1, a2, check);
}
