# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# Sources and headers necessary for every build.
# These contain definitions and implementation used mostly for
# initializing the SoC, and therefore are always required.
zephyr_library()
zephyr_library_sources(hal/am_hal_global.c)
zephyr_library_sources(hal/am_hal_pwrctrl.c)
zephyr_library_sources(hal/am_hal_queue.c)
zephyr_library_sources(hal/mcu/am_hal_bootrom_helper.c)
zephyr_library_sources(hal/mcu/am_hal_cachectrl.c)
zephyr_library_sources(hal/mcu/am_hal_interrupt.c)
zephyr_library_sources(hal/mcu/am_hal_mram.c)
zephyr_library_sources(hal/mcu/am_hal_rtc.c)
zephyr_library_sources(hal/mcu/am_hal_utils.c)

if(CONFIG_AMBIQ_HAL_USE_GPIO)
    zephyr_library_sources(hal/am_hal_gpio.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_STIMER)
    zephyr_library_sources(hal/am_hal_stimer.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_TIMER)
    zephyr_library_sources(hal/am_hal_timer.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_WDT)
    zephyr_library_sources(hal/am_hal_wdt.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_I2C OR CONFIG_AMBIQ_HAL_USE_SPI)
    zephyr_library_sources(hal/mcu/am_hal_iom.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
    zephyr_library_sources(hal/mcu/am_hal_fault.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_MSPI)
    zephyr_library_sources(hal/mcu/am_hal_mspi.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
endif()
