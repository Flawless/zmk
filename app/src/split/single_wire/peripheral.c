/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zmk/split/transport/peripheral.h>

#include "../wired/wired.h"
#include "single_wire.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_single_wire_split

#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct command_envelope) + sizeof(struct msg_postfix)) *                              \
     CONFIG_ZMK_SPLIT_SINGLE_WIRE_CMD_BUFFER_ITEMS)
#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct event_envelope) + sizeof(struct msg_postfix)) *                                \
     CONFIG_ZMK_SPLIT_SINGLE_WIRE_EVENT_BUFFER_ITEMS)

RING_BUF_DECLARE(rx_buf, RX_BUFFER_SIZE);
RING_BUF_DECLARE(tx_buf, TX_BUFFER_SIZE);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static const struct gpio_dt_spec data_gpio = GPIO_DT_SPEC_INST_GET(0, data_gpios);

#else

#error                                                                                             \
    "Need to create a node with compatible 'zmk,single-wire-split' and a data-gpios property."

#endif

static struct k_mutex line_mutex;
static struct k_work_delayable rx_work;
static bool transport_enabled = true;
static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static struct zmk_split_transport_status split_peripheral_singlewire_get_status(void);
extern struct zmk_split_transport_peripheral singlewire_peripheral;

static int send_frame(const uint8_t *frame, size_t frame_len) {
    LOG_HEXDUMP_DBG(frame, frame_len, "Single-wire TX frame");
    return zmk_split_single_wire_tx(&data_gpio, frame, frame_len,
                                    CONFIG_ZMK_SPLIT_SINGLE_WIRE_BIT_TIME_US);
}

static void send_pending_events(void) {
    size_t pending = ring_buf_size_get(&tx_buf);
    if (pending == 0) {
        return;
    }

    if (pending > TX_BUFFER_SIZE) {
        pending = TX_BUFFER_SIZE;
    }

    static uint8_t tx_frame_buffer[TX_BUFFER_SIZE];
    uint32_t read = ring_buf_get(&tx_buf, tx_frame_buffer, pending);
    if (read == 0) {
        return;
    }

    send_frame(tx_frame_buffer, read);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(evt->data.input);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return sizeof(evt->data.key_position);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(evt->data.sensor);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(evt->data.battery);
    default:
        return -ENOTSUP;
    }
}

static int split_peripheral_singlewire_report_event(
    const struct zmk_split_transport_peripheral_event *event) {
    ssize_t data_size = get_payload_data_size(event);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    size_t payload_size =
        data_size + sizeof(uint8_t) + sizeof(enum zmk_split_transport_peripheral_event_type);

    if (ring_buf_space_get(&tx_buf) < MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to queue event for the central");
        return -ENOSPC;
    }

    struct event_envelope env = {.prefix =
                                     {
                                         .magic_prefix = ZMK_SPLIT_WIRED_ENVELOPE_MAGIC_PREFIX,
                                         .payload_size = payload_size,
                                     },
                                 .payload =
                                     {
                                         .source = 0,
                                         .event = *event,
                                     }};

    struct msg_postfix postfix = {.crc =
                                      crc32_ieee((void *)&env, sizeof(env.prefix) + payload_size)};

    ring_buf_put(&tx_buf, (uint8_t *)&env, sizeof(env.prefix) + payload_size);
    ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));

    return 0;
}

static void process_commands(void) {
    while (ring_buf_size_get(&rx_buf) > MSG_EXTRA_SIZE) {
        struct command_envelope env;
        int item_err =
            zmk_split_wired_get_item(&rx_buf, (uint8_t *)&env, sizeof(struct command_envelope));
        switch (item_err) {
        case 0:
            LOG_DBG("Single-wire RX command %u", env.payload.cmd.type);
            zmk_split_transport_peripheral_command_handler(&singlewire_peripheral, env.payload.cmd);
            if (env.payload.cmd.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS) {
                send_pending_events();
            }
            break;
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching a command from the RX buffer: %d", item_err);
            return;
        }
    }
}

static void rx_work_handler(struct k_work *work) {
    if (!transport_enabled) {
        return;
    }

    if (k_mutex_lock(&line_mutex, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_MUTEX_TIMEOUT_MS)) != 0) {
        goto reschedule;
    }

    int err = zmk_split_single_wire_rx(&rx_buf, &data_gpio, CONFIG_ZMK_SPLIT_SINGLE_WIRE_BIT_TIME_US,
                                       CONFIG_ZMK_SPLIT_SINGLE_WIRE_START_TIMEOUT_US,
                                       CONFIG_ZMK_SPLIT_SINGLE_WIRE_RX_TIMEOUT_US);

    if (err == 0 && ring_buf_size_get(&rx_buf) > 0) {
        process_commands();
    } else if (err < 0 && err != -EAGAIN) {
        LOG_WRN("Single-wire RX failed (%d)", err);
    }

    k_mutex_unlock(&line_mutex);

reschedule:
    k_work_schedule(&rx_work, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_POLL_INTERVAL_MS));
}

static int split_peripheral_singlewire_set_enabled(bool enabled) {
    transport_enabled = enabled;

    if (enabled) {
        k_work_schedule(&rx_work, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&rx_work);
    }

    if (transport_status_cb) {
        transport_status_cb(&singlewire_peripheral, split_peripheral_singlewire_get_status());
    }

    return 0;
}

static struct zmk_split_transport_status split_peripheral_singlewire_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = transport_enabled,
        .connections = transport_enabled ? ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED
                                         : ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED,
    };
}

static int split_peripheral_singlewire_set_status_callback(
    zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_singlewire_report_event,
    .set_enabled = split_peripheral_singlewire_set_enabled,
    .get_status = split_peripheral_singlewire_get_status,
    .set_status_callback = split_peripheral_singlewire_set_status_callback,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(singlewire_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_SINGLE_WIRE_PRIORITY);

static int zmk_split_singlewire_peripheral_init(void) {
    int ret = zmk_split_single_wire_init(&data_gpio);
    if (ret < 0) {
        return ret;
    }

    k_mutex_init(&line_mutex);
    k_work_init_delayable(&rx_work, rx_work_handler);

    k_work_schedule(&rx_work, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_POLL_INTERVAL_MS));

    return 0;
}

SYS_INIT(zmk_split_singlewire_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
