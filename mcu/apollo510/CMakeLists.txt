# Ambiq HAL
#
# Copyright (c) 2025 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

# Sources and headers necessary for every build.
# These contain definitions and implementation used mostly for
# initializing the SoC, and therefore are always required.
zephyr_library()
zephyr_library_sources(hal/am_hal_global.c)
zephyr_library_sources(hal/am_hal_pwrctrl.c)
zephyr_library_sources(hal/am_hal_clkmgr.c)
zephyr_library_sources(hal/am_hal_spotmgr.c)
zephyr_library_sources(hal/am_hal_spotmgr_pcm0_7.c)
zephyr_library_sources(hal/am_hal_spotmgr_pcm2_0.c)
zephyr_library_sources(hal/am_hal_spotmgr_pcm2_1.c)
zephyr_library_sources(hal/am_hal_spotmgr_pcm2_2.c)
zephyr_library_sources(hal/am_hal_stimer.c)
zephyr_library_sources(hal/am_hal_queue.c)
zephyr_library_sources(hal/am_hal_info.c)
zephyr_library_sources(hal/mcu/am_hal_bootrom_helper.c)
zephyr_library_sources(hal/mcu/am_hal_cachectrl.c)
zephyr_library_sources(hal/mcu/am_hal_clkgen.c)
zephyr_library_sources(hal/mcu/am_hal_debug.c)
zephyr_library_sources(hal/mcu/am_hal_interrupt.c)
zephyr_library_sources(hal/mcu/am_hal_rtc.c)
zephyr_library_sources(hal/mcu/am_hal_utils.c)
zephyr_library_sources(hal/mcu/am_hal_mcuctrl.c)
zephyr_library_sources(hal/mcu/am_hal_sysctrl.c)
zephyr_library_sources(hal/mcu/am_hal_syspll.c)
zephyr_library_sources(hal/mcu/am_hal_itm.c)

if(CONFIG_AMBIQ_HAL_USE_ADC)
    zephyr_library_sources(hal/am_hal_adc.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_SDIO)
zephyr_library_sources(hal/mcu/am_hal_card.c)
zephyr_library_sources(hal/mcu/am_hal_card_host.c)
zephyr_library_sources(hal/mcu/am_hal_sdhc.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_TPIU)
zephyr_library_sources(hal/mcu/am_hal_tpiu.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_DCU)
zephyr_library_sources(hal/am_hal_dcu.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_GPIO)
    zephyr_library_sources(hal/am_hal_gpio.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_HWINFO)
    zephyr_library_sources(hal/mcu/am_hal_reset.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_TIMER)
    zephyr_library_sources(hal/am_hal_timer.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_WDT)
    zephyr_library_sources(hal/am_hal_wdt.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_UART)
    zephyr_library_sources(hal/mcu/am_hal_uart.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_I2C OR CONFIG_AMBIQ_HAL_USE_SPIC OR CONFIG_AMBIQ_HAL_USE_SPI)
    zephyr_library_sources(hal/mcu/am_hal_iom.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_I2S)
    zephyr_library_sources(hal/am_hal_i2s.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_MSPI)
    zephyr_library_sources(hal/mcu/am_hal_mspi.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_PDM)
    zephyr_library_sources(hal/am_hal_pdm.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_USB)
    zephyr_library_sources(hal/am_hal_usb.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_DSI)
    zephyr_library_sources(hal/mcu/am_hal_dsi.c)
endif()

zephyr_library_compile_options(-w)
