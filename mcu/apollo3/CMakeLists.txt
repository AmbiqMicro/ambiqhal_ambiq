# Ambiq HAL
#
# Copyright (c) 2024 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

# Sources and headers necessary for every build.
# These contain definitions and implementation used mostly for
# initializing the SoC, and therefore are always required.
zephyr_library()
zephyr_library_sources(hal/am_hal_global.c)
zephyr_library_sources(hal/am_hal_pwrctrl.c)
zephyr_library_sources(hal/am_hal_queue.c)
zephyr_library_sources(hal/am_hal_cachectrl.c)
zephyr_library_sources(hal/am_hal_interrupt.c)
zephyr_library_sources(hal/am_hal_flash.c)
zephyr_library_sources(hal/am_hal_rtc.c)
zephyr_library_sources(hal/am_hal_mcuctrl.c)
zephyr_library_sources(hal/am_hal_burst.c)
zephyr_library_sources(hal/am_hal_reset.c)
zephyr_library_sources(hal/am_hal_clkgen.c)
zephyr_library_sources(hal/am_hal_sysctrl.c)
zephyr_library_sources(hal/am_hal_debug.c)
zephyr_library_sources(hal/am_hal_itm.c)
zephyr_library_sources(hal/am_hal_security.c)
zephyr_library_sources(hal/am_hal_systick.c)
zephyr_library_sources(hal/am_hal_tpiu.c)
zephyr_library_sources(hal/am_hal_uart.c)

if(CONFIG_AMBIQ_HAL_USE_GPIO)
    zephyr_library_sources(hal/am_hal_gpio.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_STIMER)
    zephyr_library_sources(hal/am_hal_stimer.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_TIMER)
    zephyr_library_sources(hal/am_hal_ctimer.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_WDT)
    zephyr_library_sources(hal/am_hal_wdt.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_I2C OR CONFIG_AMBIQ_HAL_USE_SPIC OR CONFIG_AMBIQ_HAL_USE_SPI)
    zephyr_library_sources(hal/am_hal_iom.c)
    zephyr_library_sources(hal/am_hal_cmdq.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_SPID)
    zephyr_library_sources(hal/am_hal_ios.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_MSPI)
    zephyr_library_sources(hal/am_hal_mspi.c)
    zephyr_library_sources(hal/am_hal_cmdq.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_BLEIF)
    zephyr_library_sources(hal/am_hal_ble.c)
    zephyr_library_sources(hal/am_hal_ble_patch.c)
    zephyr_library_sources(hal/am_hal_ble_patch_b0.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_ADC)
    zephyr_library_sources(hal/am_hal_adc.c)
endif()
