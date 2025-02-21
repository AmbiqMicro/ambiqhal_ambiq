# Ambiq HAL Components
#
# Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_SOC_APOLLO4P_BLUE)
    zephyr_library_sources(am_devices_cooper.c)
elseif(CONFIG_SOC_APOLLO3P_BLUE OR CONFIG_SOC_APOLLO3_BLUE)
    zephyr_library_sources(am_apollo3_bt_support.c)
endif()
