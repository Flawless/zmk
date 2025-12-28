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
#include <string.h>

#include <zmk/split/transport/central.h>

#include "../wired/wired.h"
#include "single_wire.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_single_wire_split

#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct event_envelope) + sizeof(struct msg_postfix)) *                                \
     CONFIG_ZMK_SPLIT_SINGLE_WIRE_EVENT_BUFFER_ITEMS)

RING_BUF_DECLARE(rx_buf, RX_BUFFER_SIZE);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static const struct gpio_dt_spec data_gpio = GPIO_DT_SPEC_INST_GET(0, data_gpios);

#else

#error                                                                                             \
    "Need to create a node with compatible 'zmk,single-wire-split' and a data-gpios property."

#endif

static struct k_mutex line_mutex;
static struct k_work publish_events_work;
static struct k_work_delayable poll_work;
static bool transport_enabled = true;
static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static struct zmk_split_transport_status split_central_singlewire_get_status(void);
extern struct zmk_split_transport_central singlewire_central;

static ssize_t get_payload_data_size(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

static int send_frame(const uint8_t *frame, size_t frame_len) {
    LOG_HEXDUMP_DBG(frame, frame_len, "Single-wire TX frame");
    return zmk_split_single_wire_tx(&data_gpio, frame, frame_len,
                                    CONFIG_ZMK_SPLIT_SINGLE_WIRE_BIT_TIME_US);
}

static int send_command_frame(uint8_t source, struct zmk_split_transport_central_command cmd) {
    ssize_t data_size = get_payload_data_size(&cmd);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    size_t payload_size =
        data_size + sizeof(source) + sizeof(enum zmk_split_transport_central_command_type);

    struct command_envelope env = {.prefix =
                                       {
                                           .magic_prefix = ZMK_SPLIT_WIRED_ENVELOPE_MAGIC_PREFIX,
                                           .payload_size = payload_size,
                                       },
                                   .payload =
                                       {
                                           .source = source,
                                           .cmd = cmd,
                                       }};

    struct msg_postfix postfix = {.crc =
                                      crc32_ieee((void *)&env, sizeof(env.prefix) + payload_size)};

    uint8_t frame[sizeof(env) + sizeof(postfix)];
    size_t frame_len = sizeof(env.prefix) + payload_size + sizeof(postfix);

    memcpy(frame, &env, sizeof(env.prefix) + payload_size);
    memcpy(frame + sizeof(env.prefix) + payload_size, &postfix, sizeof(postfix));

    return send_frame(frame, frame_len);
}

static void publish_events(struct k_work *work) {
    while (ring_buf_size_get(&rx_buf) > MSG_EXTRA_SIZE) {
        struct event_envelope env;
        int item_err =
            zmk_split_wired_get_item(&rx_buf, (uint8_t *)&env, sizeof(struct event_envelope));
        switch (item_err) {
        case 0:
            LOG_DBG("Single-wire RX event from %u", env.payload.source);
            zmk_split_transport_central_peripheral_event_handler(&singlewire_central,
                                                                 env.payload.source,
                                                                 env.payload.event);
            break;
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}

static void poll_work_handler(struct k_work *work) {
    if (!transport_enabled) {
        return;
    }

    if (k_mutex_lock(&line_mutex, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_MUTEX_TIMEOUT_MS)) != 0) {
        goto reschedule;
    }

    int err = send_command_frame(0,
                                 (struct zmk_split_transport_central_command){
                                     .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS,
                                 });
    if (err < 0) {
        LOG_WRN("Single-wire poll send failed (%d)", err);
        k_mutex_unlock(&line_mutex);
        goto reschedule;
    }

    err = zmk_split_single_wire_rx(&rx_buf, &data_gpio, CONFIG_ZMK_SPLIT_SINGLE_WIRE_BIT_TIME_US,
                                   CONFIG_ZMK_SPLIT_SINGLE_WIRE_START_TIMEOUT_US,
                                   CONFIG_ZMK_SPLIT_SINGLE_WIRE_RX_TIMEOUT_US);
    k_mutex_unlock(&line_mutex);

    if (err == 0 && ring_buf_size_get(&rx_buf) > 0) {
        k_work_submit(&publish_events_work);
    } else if (err < 0 && err != -EAGAIN) {
        LOG_WRN("Single-wire RX failed (%d)", err);
    }

reschedule:
    k_work_schedule(&poll_work, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_POLL_INTERVAL_MS));
}

static int split_central_singlewire_send_command(uint8_t source,
                                                 struct zmk_split_transport_central_command cmd) {
    if (source != 0) {
        return -EINVAL;
    }

    if (k_mutex_lock(&line_mutex, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_MUTEX_TIMEOUT_MS)) != 0) {
        return -EBUSY;
    }

    int ret = send_command_frame(source, cmd);
    k_mutex_unlock(&line_mutex);

    return ret;
}

static int split_central_singlewire_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;
    return 1;
}

static int split_central_singlewire_set_enabled(bool enabled) {
    transport_enabled = enabled;

    if (enabled) {
        k_work_schedule(&poll_work, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&poll_work);
    }

    if (transport_status_cb) {
        transport_status_cb(&singlewire_central, split_central_singlewire_get_status());
    }

    return 0;
}

static struct zmk_split_transport_status split_central_singlewire_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = transport_enabled,
        .connections = transport_enabled ? ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED
                                         : ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED,
    };
}

static int split_central_singlewire_set_status_callback(
    zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_singlewire_send_command,
    .get_available_source_ids = split_central_singlewire_get_available_source_ids,
    .set_enabled = split_central_singlewire_set_enabled,
    .get_status = split_central_singlewire_get_status,
    .set_status_callback = split_central_singlewire_set_status_callback,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(singlewire_central, &central_api,
                                     CONFIG_ZMK_SPLIT_SINGLE_WIRE_PRIORITY);

static int zmk_split_singlewire_central_init(void) {
    int ret = zmk_split_single_wire_init(&data_gpio);
    if (ret < 0) {
        return ret;
    }

    k_mutex_init(&line_mutex);
    k_work_init(&publish_events_work, publish_events);
    k_work_init_delayable(&poll_work, poll_work_handler);

    k_work_schedule(&poll_work, K_MSEC(CONFIG_ZMK_SPLIT_SINGLE_WIRE_POLL_INTERVAL_MS));

    return 0;
}

SYS_INIT(zmk_split_singlewire_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
