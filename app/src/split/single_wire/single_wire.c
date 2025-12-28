/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "single_wire.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SINGLE_WIRE_SAMPLE_DELAY_US 2

static uint32_t elapsed_us(uint32_t start_cycle) {
    return k_cyc_to_us_floor32(k_cycle_get_32() - start_cycle);
}

static int configure_line_idle(const struct gpio_dt_spec *data_gpio) {
    return gpio_pin_configure_dt(data_gpio, GPIO_INPUT | GPIO_PULL_UP);
}

static int configure_line_drive(const struct gpio_dt_spec *data_gpio) {
    return gpio_pin_configure_dt(data_gpio, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
}

int zmk_split_single_wire_init(const struct gpio_dt_spec *data_gpio) {
    if (!device_is_ready(data_gpio->port)) {
        LOG_ERR("Single-wire GPIO device not ready");
        return -ENODEV;
    }

    int ret = configure_line_idle(data_gpio);
    if (ret < 0) {
        LOG_ERR("Failed to configure single-wire GPIO (%d)", ret);
    }

    return ret;
}

static int wait_for_start_bit(const struct gpio_dt_spec *data_gpio, uint32_t timeout_us) {
    uint32_t start = k_cycle_get_32();

    while (elapsed_us(start) < timeout_us) {
        int value = gpio_pin_get_dt(data_gpio);
        if (value < 0) {
            return value;
        }

        if (value == 0) {
            return 0;
        }

        k_busy_wait(SINGLE_WIRE_SAMPLE_DELAY_US);
    }

    return -EAGAIN;
}

static int read_byte(const struct gpio_dt_spec *data_gpio, uint8_t *byte, uint32_t bit_time_us,
                     uint32_t timeout_us) {
    int ret = wait_for_start_bit(data_gpio, timeout_us);
    if (ret < 0) {
        return ret;
    }

    unsigned int key = irq_lock();

    k_busy_wait(bit_time_us / 2U);

    uint8_t value = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        k_busy_wait(bit_time_us);
        int bit = gpio_pin_get_dt(data_gpio);
        if (bit < 0) {
            irq_unlock(key);
            return bit;
        }

        if (bit) {
            value |= BIT(i);
        }
    }

    k_busy_wait(bit_time_us);
    int stop_bit = gpio_pin_get_dt(data_gpio);

    irq_unlock(key);

    if (stop_bit <= 0) {
        return -EIO;
    }

    *byte = value;
    return 0;
}

int zmk_split_single_wire_tx(const struct gpio_dt_spec *data_gpio, const uint8_t *buf, size_t len,
                             uint32_t bit_time_us) {
    int ret = configure_line_drive(data_gpio);
    if (ret < 0) {
        return ret;
    }

    unsigned int key = irq_lock();

    for (size_t idx = 0; idx < len; idx++) {
        uint8_t byte = buf[idx];

        gpio_pin_set_dt(data_gpio, 0);
        k_busy_wait(bit_time_us);

        for (uint8_t bit = 0U; bit < 8U; bit++) {
            gpio_pin_set_dt(data_gpio, (byte >> bit) & 0x01);
            k_busy_wait(bit_time_us);
        }

        gpio_pin_set_dt(data_gpio, 1);
        k_busy_wait(bit_time_us);
    }

    irq_unlock(key);

    ret = configure_line_idle(data_gpio);
    if (ret < 0) {
        LOG_ERR("Failed to release single-wire line (%d)", ret);
    }

    return ret;
}

int zmk_split_single_wire_rx(struct ring_buf *rx_buf, const struct gpio_dt_spec *data_gpio,
                             uint32_t bit_time_us, uint32_t start_timeout_us,
                             uint32_t idle_timeout_us) {
    int ret = configure_line_idle(data_gpio);
    if (ret < 0) {
        return ret;
    }

    bool received_any = false;

    while (true) {
        uint8_t byte = 0U;
        ret = read_byte(data_gpio, &byte, bit_time_us,
                        received_any ? idle_timeout_us : start_timeout_us);
        if (ret == -EAGAIN) {
            return received_any ? 0 : -EAGAIN;
        }
        if (ret < 0) {
            LOG_WRN("Single-wire RX error %d", ret);
            return ret;
        }

        received_any = true;
        uint32_t wrote = ring_buf_put(rx_buf, &byte, 1);
        if (wrote != 1U) {
            LOG_WRN("Single-wire RX buffer full, dropping byte");
            return -ENOSPC;
        }
    }
}
