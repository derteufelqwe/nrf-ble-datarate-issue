#include <stdlib.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define UART_SERVICE_UUID                                                                          \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0000, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))
#define UART_RX_UUID                                                                               \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0100, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))
#define UART_TX_UUID                                                                               \
    BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x04BC0200, 0x0E7F, 0x46FE, 0x89AA, 0xA698E16CA002))

#define MEASURE_DURATION_MS 30000

K_THREAD_STACK_DEFINE(my_stack_area, 5000);
struct k_work_q bench_work_queue;

static const struct bt_data advertising_data[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, "DEVICE", 6),
};

static const struct bt_data sd_data[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, NULL, 0),
};

static struct bt_conn *default_conn = NULL;

// --- GATT stuff ---

uint32_t c_start = 0;
uint32_t payload_sum = 0;
static int payload_size = 16;

static ssize_t gatt_write_rx(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             const void *buf_, uint16_t buf_len, uint16_t offset, uint8_t flags) {
    const uint8_t *const buf = buf_;
    int ret = buf_len;

    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

//    LOG_HEXDUMP_DBG(buf, buf_len, "rx_buffer");

    if (offset != 0) {
        return -BT_ATT_ERR_INVALID_OFFSET;
    }

    if (buf_len < 10) {
        return ret;
    }

    uint8_t flag = buf[0];

    switch (flag) {
        case 0x10:
            payload_sum += buf_len;
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

    return ret;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    ARG_UNUSED(value);

    LOG_INF("Handle %d, ccc value 0x%04x", attr->handle, value);
}

BT_GATT_SERVICE_DEFINE(uart_svc, BT_GATT_PRIMARY_SERVICE(UART_SERVICE_UUID),
                       BT_GATT_CHARACTERISTIC(UART_TX_UUID, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ, NULL, NULL, NULL),
                       BT_GATT_CCC(ccc_changed, (BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)),
                       BT_GATT_CHARACTERISTIC(UART_RX_UUID,
                                              BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE, NULL, gatt_write_rx, NULL),);

// --- Connection handling ---

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    LOG_INF("Connected");
    default_conn = conn;
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
    default_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
        .connected        = connected,
        .disconnected     = disconnected,
};

// --- Bench work ---

static inline int send_notify(const uint8_t *payload, size_t payload_len) {
    struct bt_gatt_notify_params params = {
            .attr = &uart_svc.attrs[1],
            .data = payload,
            .len = payload_len,
    };

    return bt_gatt_notify_cb(default_conn, &params);
}

static void bench_work_handler(struct k_work *work) {
    int err;

    char payload[payload_size];
    memset(payload, 0x10, payload_size);

    LOG_INF("Starting benchmark with payload size %d...", payload_size);
    // Send start command
    payload[0] = 0x11;
    for (int i = 0; i < 10; ++i) {
        send_notify(payload, payload_size);
    }

    // Main benchmark
    uint32_t c_start = k_cycle_get_32();
    payload[0] = 0x10;
    while (k_cyc_to_ms_ceil32(k_cycle_get_32() - c_start) < MEASURE_DURATION_MS) {
        err = send_notify(payload, payload_size);
        if (err) {
            LOG_ERR("gatt_notify failed with code %d", err);
        }
    }
    uint32_t c_stop = k_cycle_get_32();

    // Send stop command
    payload[0] = 0x12;
    for (int i = 0; i < 10; ++i) {
        send_notify(payload, payload_size);
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

    struct bt_gatt_notify_params params = {
            .attr = &uart_svc.attrs[1],
            .data = payload,
            .len = payload_len,
    };

    ret = bt_gatt_notify_cb(default_conn, &params);
    LOG_INF("Notified payload (%d) %s", payload_len, payload);
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

    err = bt_le_adv_start(BT_LE_ADV_CONN, advertising_data, ARRAY_SIZE(advertising_data), sd_data, ARRAY_SIZE(sd_data));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return 0;
    }

    LOG_INF("Advertising successfully started");

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
