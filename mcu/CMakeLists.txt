# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_APOLLO4P)
    zephyr_include_directories(apollo4p)
    add_subdirectory(apollo4p)
endif()
