# Ambiq HAL Components
#
# Copyright (c) 2025 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_AMBIQ_COMPONENT_USE_BT)
    add_subdirectory(bluetooth)
    zephyr_include_directories(bluetooth)
endif()
