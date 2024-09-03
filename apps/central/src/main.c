#include <stdio.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define UART_SERVICE_UUID                                                                          \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0000, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))
#define UART_TX_UUID                                                                               \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0200, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))
#define UART_RX_UUID                                                                               \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0100, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))

#define MEASURE_DURATION_MS 30000

K_THREAD_STACK_DEFINE(my_stack_area, 5000);
struct k_work_q bench_work_queue;

static struct bt_conn *default_conn;
static struct bt_uuid_128 uuid = BT_UUID_INIT_128(0);
static bool subscribe_params_valid = false;
static bool subscribed = false;
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_discover_params discover_params;
static uint16_t uart_svc_handle;
static uint16_t write_handle;

uint32_t c_start = 0;
uint32_t payload_sum = 0;
static int payload_size = 16;

static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *_data, uint16_t len) {
    uint8_t *data = (uint8_t *) _data;

    if (len < 10) {
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t flag = data[0];

    switch (flag) {
        case 0x10:
            payload_sum += len;
            break;
        case 0x11:
            c_start = k_cycle_get_32();
            payload_sum = 0;
            break;
        case 0x12:
            if (c_start == 0) {
                break;
            }
            uint32_t k_bytes = payload_sum / MEASURE_DURATION_MS;
            uint32_t k_bits =  (payload_sum * 8) / MEASURE_DURATION_MS;
            LOG_INF("Bench result: %d bytes. %d KB/s = %d kbps", payload_sum, k_bytes, k_bits);
            c_start = 0;
            break;
        default:
            LOG_ERR("Unknown flag 0x%x", flag);
    }

    return BT_GATT_ITER_CONTINUE;
}

// --- Scanning ---

static void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match,
                              bool connectable) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %s", addr, connectable ? "true" : "false");
}

static void scan_connecting_error(struct bt_scan_device_info *device_info) {
    LOG_ERR("Connecting failed");
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn) {
    default_conn = bt_conn_ref(conn);
}

static void scan_filter_no_match(struct bt_scan_device_info *device_info, bool connectable) {
    int err;
    struct bt_conn *conn;
    char addr[BT_ADDR_LE_STR_LEN];

    if (device_info->recv_info->adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
        LOG_INF("Direct advertising received from %s", addr);
        bt_scan_stop();

        err = bt_conn_le_create(device_info->recv_info->addr, BT_CONN_LE_CREATE_CONN, device_info->conn_param, &conn);
        if (!err) {
            default_conn = bt_conn_ref(conn);
            bt_conn_unref(conn);
        }
    }
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match, scan_connecting_error, scan_connecting);

static void scan_init(void) {
    int err;

    struct bt_le_scan_param scan_param = {
            .type = BT_LE_SCAN_TYPE_ACTIVE,
            .options = BT_LE_SCAN_OPT_NONE,
            .interval = BT_GAP_SCAN_FAST_INTERVAL,
            .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    struct bt_scan_init_param scan_init = {
            .connect_if_match = true,
            .scan_param = &scan_param,
            .conn_param = BT_LE_CONN_PARAM_DEFAULT,
    };

    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_NAME, "DEVICE");
    if (err) {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);

        return;
    }

    err = bt_scan_filter_enable(BT_SCAN_NAME_FILTER, false);
    if (err) {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
    }
}

// --- MTU ---

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params) {
    int err;

    if (!attr) {
        LOG_INF("Discover complete");
        (void) memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    if (!bt_uuid_cmp(discover_params.uuid, UART_SERVICE_UUID)) {
        memcpy(&uuid, UART_RX_UUID, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        uart_svc_handle = attr->handle;
        LOG_DBG("Discovered UART_SERVICE_UUID %u", attr->handle);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
        }
    } else if (!bt_uuid_cmp(discover_params.uuid, UART_RX_UUID)) {
        memcpy(&uuid, UART_TX_UUID, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.start_handle = uart_svc_handle + 1;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        write_handle = bt_gatt_attr_value_handle(attr);

        LOG_DBG("Discovered UART_RX_UUID attr_handle %u, write_handle %u", attr->handle,
                write_handle);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
        }
    } else if (!bt_uuid_cmp(discover_params.uuid, UART_TX_UUID)) {
        memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

        LOG_DBG("Discovered UART_TX_UUID %u", attr->handle);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
        }
    } else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CCC)) {
        subscribe_params.notify = notify_func;
        subscribe_params.value = BT_GATT_CCC_NOTIFY;
        subscribe_params.ccc_handle = attr->handle;
        subscribe_params_valid = true;

        err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err && err != -EALREADY) {
            LOG_ERR("Subscribe failed (err %d)", err);
        } else {
            subscribed = true;
            LOG_DBG("[SUBSCRIBED BT_GATT_CCC_NOTIFY] %u", attr->handle);
        }
    }

    return BT_GATT_ITER_STOP;
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t mtu_err,
                            struct bt_gatt_exchange_params *params) {
    int err;

    if (mtu_err != 0) {
        LOG_ERR("BT error while mtu exchange: %d", mtu_err);
        bt_conn_disconnect(conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);
        return;
    }

    if (conn == default_conn) {
        memcpy(&uuid, UART_SERVICE_UUID, sizeof(uuid));
        discover_params.uuid = &uuid.uuid;
        discover_params.func = discover_func;
        discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
        discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        discover_params.type = BT_GATT_DISCOVER_PRIMARY;

        err = bt_gatt_discover(default_conn, &discover_params);
        if (err) {
            LOG_ERR("Discover failed (err %d)", err);
            return;
        }
    }
}

static struct bt_gatt_exchange_params mtu_exchange = {.func = mtu_exchange_cb};

// --- Connection handling ---

static void connected(struct bt_conn *conn, uint8_t conn_err) {
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, conn_err);
        if (conn == default_conn) {
            bt_conn_unref(default_conn);
            default_conn = NULL;

            /* This demo doesn't require active scan */
            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if (err) {
                LOG_ERR("Scanning failed to start (err %d)", err);
            }
        }

        return;
    }

    LOG_INF("Connected: %s", addr);

//    err = bt_conn_set_security(conn, BT_SECURITY_L2);
//    if (err) {
//        LOG_ERR("Failed to set security: %d", err);
//
//        gatt_discover(conn);
//    }

    err = bt_gatt_exchange_mtu(conn, &mtu_exchange);
    if (err) {
        LOG_ERR("MTU exchange failed (err %d)", err);
        bt_conn_disconnect(conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);
    }

}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    if (default_conn != conn) {
        return;
    }

    bt_conn_unref(default_conn);
    default_conn = NULL;

    /* This demo doesn't require active scan */
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u", addr, level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", addr, level, err);
    }
}

static void conn_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout) {
    LOG_DBG("Connection interval updated: %d units (%d ms)", interval, (int) (interval * 1.25));
    LOG_DBG("Connection latency: %d", latency);
    LOG_DBG("Connection supervision timeout: %d units", timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
        .connected = connected,
        .disconnected = disconnected,
        .security_changed = security_changed,
        .le_param_updated = conn_param_updated,
};

// --- Pairing ---

static void pairing_complete(struct bt_conn *conn, bool bonded) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);
}

static void passkey_display(struct bt_conn *conn, unsigned int passkey) {
    LOG_INF("Passkey is %u", passkey);
}

static void passkey_confirm(struct bt_conn *conn, unsigned int passkey) {
    if (conn != default_conn) {
        return;
    }

    LOG_INF("Confirm passkey %u", passkey);
}

static void passkey_entry(struct bt_conn *conn) {
    if (conn != default_conn) {
        return;
    }

    LOG_INF("Please enter a passkey");
}

static void auth_cancel(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_WRN("Pairing cancelled: %s", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
        .passkey_display = passkey_display,
        .passkey_confirm = passkey_confirm,
        .passkey_entry = passkey_entry,
        .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
        .pairing_complete = pairing_complete,
        .pairing_failed = pairing_failed
};

// --- Bench work ---

static void bench_work_handler(struct k_work *work) {
    int err;

    char payload[payload_size];
    memset(payload, 0x10, payload_size);

    LOG_INF("Starting benchmark with payload size %d...", payload_size);
    // Send start command
    payload[0] = 0x11;
    for (int i = 0; i < 10; ++i) {
        bt_gatt_write_without_response(default_conn, write_handle, payload, payload_size, false);
    }

    // Main benchmark
    uint32_t c_start = k_cycle_get_32();
    payload[0] = 0x10;
    while (k_cyc_to_ms_ceil32(k_cycle_get_32() - c_start) < MEASURE_DURATION_MS) {
        err = bt_gatt_write_without_response(default_conn, write_handle, payload, payload_size, false);
        if (err) {
            LOG_ERR("gatt_write failed with code %d", err);
        }
    }
    uint32_t c_stop = k_cycle_get_32();

    // Send stop command
    payload[0] = 0x12;
    for (int i = 0; i < 10; ++i) {
        bt_gatt_write_without_response(default_conn, write_handle, payload, payload_size, false);
    }

    LOG_INF("Finished benchmark after %d ms.", k_cyc_to_ms_ceil32(c_stop - c_start));
}

K_WORK_DEFINE(bench_work, bench_work_handler);

// --- Shell ---

static int main_cmd_send(const struct shell *sh, size_t argc, char **argv) {
    int ret;

    if (argc != 2) {
        shell_print(sh, "Missing argument: <payload>");
        return -EINVAL;
    }
    const char *const payload = argv[1];
    const size_t payload_len = strlen(payload);

    if (default_conn == NULL) {
        shell_error(sh, "Not connected");
        return -1;
    }

    ret = bt_gatt_write_without_response(default_conn, write_handle, payload, payload_len, false);

    LOG_INF("Written payload (%d) %s", payload_len, payload);
    if (ret) {
        LOG_ERR("Failed to notify: %d", ret);
    }

    return 0;
}

static int main_cmd_bench(const struct shell *sh, size_t argc, char **argv) {
    int ret;

    if (argc != 2) {
        shell_print(sh, "Missing argument: <payload_size>");
        return -EINVAL;
    }
    const char *const payload_size_str = argv[1];
    payload_size = atoi(payload_size_str);

    if (default_conn == NULL) {
        shell_error(sh, "Not connected");
        return -1;
    }

    k_work_submit_to_queue(&bench_work_queue, &bench_work);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
        sub_main,
        SHELL_CMD(send, NULL, "Send data", main_cmd_send),
        SHELL_CMD(bench, NULL, "Start benchmark", main_cmd_bench),
        SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(main, &sub_main, "Main commands", NULL);

int main(void) {
    int err;

    LOG_INF("Running main...");

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized");

    scan_init();

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        LOG_ERR("Failed to register authorization callbacks.");
        return 0;
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        LOG_ERR("Failed to register authorization info callbacks.");
        return 0;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return 0;
    }

    LOG_INF("Scanning successfully started");
    return 0;
}

// --- Init stuff ---

static int k_workqueue_init(void) {
    struct k_work_queue_config config = {
            .name = "bench_q",
    };
    k_work_queue_init(&bench_work_queue);
    k_work_queue_start(&bench_work_queue,
                       my_stack_area,
                       K_THREAD_STACK_SIZEOF(my_stack_area),
                       8, &config
    );

    return 0;
}

SYS_INIT(k_workqueue_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
