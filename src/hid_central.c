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

/* Bonded keyboard address (for reconnection) */
static bt_addr_le_t bonded_addr;
static bool has_bonded_addr;

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

/* Flag: discovery is waiting for security to be established */
static bool discover_after_security;

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
/* USB HID report forwarding                                          */
/* ------------------------------------------------------------------ */

/*
 * Map BLE HID report to the USB forwarder.
 *
 * The BLE report is raw (no Report ID prefix).  Our USB descriptor uses
 * Report IDs, so we prepend the appropriate ID.
 *
 * Report ID 1 = Keyboard (8 bytes: modifiers + reserved + 6 keys)
 * Report ID 2 = Consumer Control (2 bytes: usage code LE)
 */
static void forward_hid_report(uint8_t report_id, const uint8_t *data,
                                uint16_t len)
{
    if (!usb_hid_forwarder_is_ready()) {
        LOG_DBG("USB not ready, dropping report");
        return;
    }

    uint8_t buf[16]; /* enough for Report-ID + max report */

    if (report_id == 1 && len == 8) {
        /* Keyboard report */
        buf[0] = 0x01;
        memcpy(&buf[1], data, 8);
        usb_hid_forwarder_send(buf, 9);
    } else if (report_id == 2 && len >= 2) {
        /* Consumer control report */
        buf[0] = 0x02;
        memcpy(&buf[1], data, 2);
        usb_hid_forwarder_send(buf, 3);
    } else {
        /* Best-effort: try keyboard report for 8-byte payloads */
        if (len == 8) {
            buf[0] = 0x01;
            memcpy(&buf[1], data, 8);
            usb_hid_forwarder_send(buf, 9);
        } else {
            LOG_WRN("Unknown report id=%d len=%d, dropped", report_id, len);
        }
    }
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

    LOG_DBG("HID notify: id=%d len=%d", report_id, length);
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
        LOG_WRN("Report Reference read failed (err %d, len %d)", err, length);
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
            LOG_INF("Report handle 0x%04x -> id=%d type=%d",
                    reports[i].handle, report_id, report_type);
            break;
        }
    }

done:
    pending_ref_reads--;
    if (pending_ref_reads <= 0) {
        /* All Report References read – subscribe to Input Reports */
        LOG_INF("All report references read, subscribing to input reports");
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
 *  1. Discover all characteristics in HID service
 *  2. For each Report characteristic, read its Report Reference descriptor
 *  3. Subscribe to Input Report notifications
 */

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

static uint8_t discover_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("HID discovery complete, found %d reports", report_count);
        if (report_count > 0) {
            read_report_references(conn);
        } else {
            LOG_ERR("No HID Report characteristics found");
            state = STATE_IDLE;
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

    /* Look for HID Report characteristics */
    if (bt_uuid_cmp(chrc->uuid, BT_UUID_HID_REPORT) == 0) {
        if (report_count < MAX_HID_REPORTS) {
            reports[report_count].handle = chrc->value_handle;
            reports[report_count].ccc_handle = 0; /* will be derived */
            reports[report_count].report_id = 0;
            reports[report_count].report_type = 0;
            reports[report_count].subscribed = false;
            LOG_INF("Found HID Report char at handle 0x%04x (#%d)",
                    chrc->value_handle, report_count);
            report_count++;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static void start_hid_discovery(struct bt_conn *conn)
{
    report_count = 0;
    memset(reports, 0, sizeof(reports));

    state = STATE_DISCOVERING;

    disc_params.uuid = BT_UUID_HID_REPORT;
    disc_params.func = discover_cb;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

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
    printk("*** DONGLE: READY - forwarding HID reports to USB ***\n");
    LOG_INF("HID Central ready – forwarding reports to USB");
}

/* ------------------------------------------------------------------ */
/* BLE connection callbacks                                           */
/* ------------------------------------------------------------------ */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (conn != kbd_conn) {
        return;
    }

    if (err) {
        printk("*** DONGLE: Connection failed: %d ***\n", err);
        LOG_ERR("Connection failed: %d", err);
        bt_conn_unref(kbd_conn);
        kbd_conn = NULL;
        state = STATE_IDLE;
        schedule_reconnect();
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("*** DONGLE: Connected to %s ***\n", addr_str);
    LOG_INF("Connected to %s", addr_str);

    /* v19: Dump our own BLE address and public key for DHKey verification */
    {
        struct bt_le_oob oob;
        if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob) == 0) {
            bt_addr_le_to_str(&oob.addr, addr_str, sizeof(addr_str));
            printk("*** DONGLE: Our addr: %s ***\n", addr_str);
        }
        const uint8_t *our_pk = bt_pub_key_get();
        if (our_pk) {
            char hbuf[65];
            hex_format(hbuf, our_pk, 32);
            printk("*** OUR_PK_X: %s ***\n", hbuf);
            hex_format(hbuf, our_pk + 32, 32);
            printk("*** OUR_PK_Y: %s ***\n", hbuf);
        }
    }

    state = STATE_CONNECTING;

    /* Request security – GATT discovery will start in security_changed_cb
     * because HID attributes require encryption to be visible. */
    discover_after_security = true;
    int sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err) {
        printk("*** DONGLE: Security request failed: %d, trying discovery anyway ***\n", sec_err);
        LOG_WRN("Security request failed: %d", sec_err);
        discover_after_security = false;
        start_hid_discovery(conn);
    } else {
        printk("*** DONGLE: Security requested, waiting for pairing ***\n");
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

    printk("*** DONGLE: Disconnected (reason 0x%02x) ***\n", reason);
    LOG_INF("Disconnected (reason 0x%02x)", reason);

    bt_conn_unref(kbd_conn);
    kbd_conn = NULL;
    state = STATE_IDLE;

    /* Reset subscribe params */
    for (int i = 0; i < MAX_HID_REPORTS; i++) {
        sub_params[i].value_handle = 0;
    }

    /* Send empty keyboard report to release all keys */
    if (usb_hid_forwarder_is_ready()) {
        static const uint8_t release_kb[] = {0x01, 0,0,0,0,0,0,0,0};
        usb_hid_forwarder_send(release_kb, sizeof(release_kb));
    }

    schedule_reconnect();
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                 enum bt_security_err err)
{
    if (conn != kbd_conn) {
        return;
    }

    /* Restart scanning now that pairing is done (success or fail).
     * We delayed this from connected_cb to avoid RF interference
     * during LESC key exchange. */
    int scan_err = status_scanner_restart_scanning();
    if (scan_err) {
        LOG_WRN("Failed to restart scanning after security: %d", scan_err);
    }

    if (err) {
        printk("*** DONGLE: Security failed: level %d err %d ***\n", level, err);
        LOG_WRN("Security change failed: level %d err %d", level, err);
        /* Try discovery anyway – some devices allow unencrypted HID access */
        if (discover_after_security) {
            discover_after_security = false;
            start_hid_discovery(conn);
        }
    } else {
        printk("*** DONGLE: Security level %d established ***\n", level);
        LOG_INF("Security level %d established", level);
        /* Store bonded address for reconnection */
        bt_addr_le_copy(&bonded_addr, bt_conn_get_dst(conn));
        has_bonded_addr = true;

        /* If we were waiting to discover, do it now */
        if (discover_after_security) {
            discover_after_security = false;
            printk("*** DONGLE: Starting HID discovery after security ***\n");
            start_hid_discovery(conn);
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

void hid_central_on_scan_result(const bt_addr_le_t *addr, int8_t rssi,
                                uint8_t type, struct net_buf_simple *buf,
                                const char *name)
{
    /* Only process if we're idle or looking to connect */
    if (state != STATE_IDLE && state != STATE_SCANNING) {
        return;
    }

    /* If we have a bonded address, prefer reconnecting to it */
    if (has_bonded_addr) {
        if (bt_addr_le_cmp(addr, &bonded_addr) != 0) {
            return; /* Not our bonded keyboard */
        }
        printk("*** DONGLE: Bonded addr matched, connecting ***\n");
    } else {
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
        const char *target = CONFIG_PROSPECTOR_DONGLE_TARGET_NAME;
        if (target[0] != '\0') {
            /* Target name is set – REQUIRE name match.
             * Don't connect based on HID service UUID alone,
             * otherwise we'd grab random BLE HID mice/keyboards. */
            if (!name_matches_target(name)) {
                return;
            }
            printk("*** DONGLE: Name '%s' matched target '%s' ***\n",
                   name ? name : "(null)", target);
        } else
#endif
        {
            /* No target name – accept any device with HID service */
            if (!is_hid_service_in_ad(buf)) {
                return;
            }
            printk("*** DONGLE: HID service found, connecting ***\n");
        }
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    printk("*** DONGLE: Target found: %s (RSSI %d, type 0x%02x) ***\n",
           addr_str, rssi, type);
    LOG_INF("Found target keyboard: %s (RSSI %d)", addr_str, rssi);

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

    printk("*** PSA_DIAG: start ***\n");
    printk("*** PSA_DIAG: ECC_PUB=%d BT_ECC=%d ***\n",
           IS_ENABLED(CONFIG_PSA_WANT_KEY_TYPE_ECC_PUBLIC_KEY),
           IS_ENABLED(CONFIG_BT_ECC));
    printk("*** PSA_DIAG: done ***\n");
}

static void connect_work_handler(struct k_work *work)
{
    if (!pending_connect || state != STATE_CONNECTING) {
        return;
    }
    pending_connect = false;

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&pending_addr, addr_str, sizeof(addr_str));

    printk("*** DONGLE v19: PSA_USE=%d PSA_NOUSE=%d P256M=%d Y_FIX+DHKEY_DUMP ***\n",
           psa_test_result, psa_test_nousage,
           IS_ENABLED(CONFIG_MBEDTLS_PSA_P256M_DRIVER_ENABLED));

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
        printk("*** DONGLE v6: Existing conn found to %s, disconnecting first ***\n", addr_str);
        bt_conn_disconnect(existing, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(existing);
        /* Wait a bit for disconnect to complete, then retry */
        state = STATE_IDLE;
        k_work_schedule(&reconnect_work, K_MSEC(1000));
        return;
    }

    /* Stop scanning before connecting (Zephyr requirement) */
    int err = bt_le_scan_stop();
    if (err) {
        printk("*** DONGLE: Scan stop failed: %d ***\n", err);
        LOG_WRN("Scan stop failed: %d", err);
    }

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

    printk("*** DONGLE: Calling bt_conn_le_create for %s ***\n", addr_str);
    err = bt_conn_le_create(&pending_addr, &create_param, &conn_param, &kbd_conn);
    if (err) {
        printk("*** DONGLE: bt_conn_le_create failed: %d ***\n", err);
        LOG_ERR("Create connection failed: %d", err);
        state = STATE_IDLE;
        /* Restart scanning */
        status_scanner_restart_scanning();
        schedule_reconnect();
    } else {
        printk("*** DONGLE: Connection initiated to %s ***\n", addr_str);
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
    if (state != STATE_IDLE) {
        return; /* Already connecting or connected */
    }

    if (!has_bonded_addr) {
        /* No bonded device – just wait for scan results */
        state = STATE_SCANNING;
        LOG_INF("No bonded device, waiting for scan results");
        return;
    }

    LOG_INF("Attempting reconnection to bonded keyboard");

    /* Stop any active scan first */
    bt_le_scan_stop();

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
        printk("*** DONGLE: Reconnect failed: %d ***\n", err);
        LOG_WRN("Reconnect create failed: %d, will retry", err);
        state = STATE_IDLE;
        status_scanner_restart_scanning();
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
/* Initialization                                                     */
/* ------------------------------------------------------------------ */

static int hid_central_init(void)
{
    k_work_init_delayable(&reconnect_work, reconnect_work_handler);
    k_work_init(&connect_work, connect_work_handler);

    state = STATE_SCANNING;
#ifdef CONFIG_PROSPECTOR_DONGLE_TARGET_NAME
    printk("*** DONGLE: HID Central initialized, target='%s' ***\n",
           CONFIG_PROSPECTOR_DONGLE_TARGET_NAME);
#else
    printk("*** DONGLE: HID Central initialized, target='(any)' ***\n");
#endif
    LOG_INF("HID Central initialized, waiting for scan results");
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

#if !IS_ENABLED(CONFIG_ZMK_BLE)
static int dongle_bt_enable(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("*** DONGLE: bt_enable failed: %d ***\n", err);
        return err;
    }
    printk("*** DONGLE: BT stack initialized (ZMK_BLE disabled) ***\n");

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_load();
    printk("*** DONGLE: Settings loaded (bonds) ***\n");
#endif

    /* Clear all bonds to force fresh pairing.
     * Previous failed pairing attempts left stale bonds on keyboard side.
     * Without this, keyboard tries LTK re-encryption → PIN_OR_KEY_MISSING. */
    err = bt_unpair(BT_ID_DEFAULT, NULL);
    printk("*** DONGLE: Bonds cleared: %d ***\n", err);

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
        printk("*** Y_FIX: No valid Y for this X ***\n");
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
    char hex_buf[65];

    hex_format(hex_buf, &key[0], 32);
    printk("*** KEY_X: %s ***\n", hex_buf);

    hex_format(hex_buf, &key[32], 32);
    printk("*** KEY_Y_ORIG: %s ***\n", hex_buf);

    printk("*** KEY_VALID: FORCED TRUE ***\n");
    return true;
}

/* --- bt_dh_key_gen wrapper: fix Y coordinate before ECDH --- */

extern int __real_bt_dh_key_gen(const uint8_t remote_pk[64],
                                void (*cb)(const uint8_t key[32]));

/* v19: Intercept DH key callback to dump the computed DHKey */
static void (*saved_dh_key_cb)(const uint8_t key[32]);

static void dh_key_dump_cb(const uint8_t key[32])
{
    if (key) {
        char hex_buf[65];
        hex_format(hex_buf, key, 32);
        printk("*** DHKEY_RESULT: %s ***\n", hex_buf);
    } else {
        printk("*** DHKEY_RESULT: NULL (ECDH failed) ***\n");
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
    char hex_buf[65];

    memcpy(fixed_pk, remote_pk, 64);

    /* Convert X from LE (wire) to BE (math) */
    memcpy_swap(x_be, remote_pk, 32);

    /* Compute valid Y on P-256 for this X */
    if (compute_p256_y(x_be, y_be) == 0) {
        /* Convert Y from BE back to LE (wire format) */
        memcpy_swap(&fixed_pk[32], y_be, 32);

        hex_format(hex_buf, &fixed_pk[32], 32);
        printk("*** KEY_Y_FIXED: %s ***\n", hex_buf);
        printk("*** ECDH_FIX: Y coordinate corrected ***\n");
    } else {
        printk("*** ECDH_FIX: Cannot compute Y, using original ***\n");
    }

    /* v19: Dump the full key passed to real bt_dh_key_gen */
    hex_format(hex_buf, fixed_pk, 32);
    printk("*** ECDH_INPUT_X: %s ***\n", hex_buf);
    hex_format(hex_buf, &fixed_pk[32], 32);
    printk("*** ECDH_INPUT_Y: %s ***\n", hex_buf);

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
        printk("*** ECP_CHECK: BYPASSED (real=%d) ***\n", real_result);
    }
    return 0;
}
