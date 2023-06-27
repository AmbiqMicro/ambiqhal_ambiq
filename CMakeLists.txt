# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_AMBIQ_HAL)
    zephyr_include_directories(
        CMSIS/AmbiqMicro/Include
        utils
        )
    add_subdirectory(mcu)
endif()
