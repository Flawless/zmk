# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources(single_wire.c)

if (CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    zephyr_library_sources(central.c)
else()
    zephyr_library_sources(peripheral.c)
endif()
