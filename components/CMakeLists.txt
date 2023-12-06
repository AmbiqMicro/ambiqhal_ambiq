# Ambiq HAL Components
#
# Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(bluetooth/am_devices_cooper.c)
zephyr_include_directories(bluetooth)
