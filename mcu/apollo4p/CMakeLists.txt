# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# Sources and headers necessary for every build.
# These contain definitions and implementation used mostly for
# initializing the SoC, and therefore are always required.
zephyr_sources(hal/am_hal_global.c)
zephyr_sources(hal/am_hal_pwrctrl.c)
zephyr_sources(hal/am_hal_queue.c)
zephyr_sources(hal/mcu/am_hal_bootrom_helper.c)
zephyr_sources(hal/mcu/am_hal_cachectrl.c)
zephyr_sources(hal/mcu/am_hal_interrupt.c)
zephyr_sources(hal/mcu/am_hal_mram.c)
zephyr_sources(hal/mcu/am_hal_rtc.c)
zephyr_sources(hal/mcu/am_hal_utils.c)

if(CONFIG_AMBIQ_HAL_USE_GPIO)
    zephyr_sources(hal/am_hal_gpio.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_STIMER)
    zephyr_sources(hal/am_hal_stimer.c)
endif()
