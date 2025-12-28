/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>

int zmk_split_single_wire_init(const struct gpio_dt_spec *data_gpio);

int zmk_split_single_wire_tx(const struct gpio_dt_spec *data_gpio, const uint8_t *buf, size_t len,
                             uint32_t bit_time_us);

int zmk_split_single_wire_rx(struct ring_buf *rx_buf, const struct gpio_dt_spec *data_gpio,
                             uint32_t bit_time_us, uint32_t start_timeout_us,
                             uint32_t idle_timeout_us);
