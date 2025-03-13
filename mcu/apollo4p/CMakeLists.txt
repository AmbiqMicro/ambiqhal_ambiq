# Ambiq HAL
#
# Copyright (c) 2023 Antmicro Ltd <www.antmicro.com>
# Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
#
# SPDX-License-Identifier: Apache-2.0

# Sources and headers necessary for every build.
# These contain definitions and implementation used mostly for
# initializing the SoC, and therefore are always required.
zephyr_library()
zephyr_library_sources(hal/am_hal_global.c)
zephyr_library_sources(hal/am_hal_pin.c)
zephyr_library_sources(hal/am_hal_pwrctrl.c)
zephyr_library_sources(hal/am_hal_queue.c)
zephyr_library_sources(hal/am_hal_security.c)
zephyr_library_sources(hal/mcu/am_hal_bootrom_helper.c)
zephyr_library_sources(hal/mcu/am_hal_cachectrl.c)
zephyr_library_sources(hal/mcu/am_hal_clkgen.c)
zephyr_library_sources(hal/mcu/am_hal_interrupt.c)
zephyr_library_sources(hal/mcu/am_hal_itm.c)
zephyr_library_sources(hal/mcu/am_hal_mcuctrl.c)
zephyr_library_sources(hal/mcu/am_hal_mpu.c)
zephyr_library_sources(hal/mcu/am_hal_mram.c)
zephyr_library_sources(hal/mcu/am_hal_rtc.c)
zephyr_library_sources(hal/mcu/am_hal_sysctrl.c)
zephyr_library_sources(hal/mcu/am_hal_tpiu.c)
zephyr_library_sources(hal/mcu/am_hal_uart.c)
zephyr_library_sources(hal/mcu/am_hal_utils.c)

if(CONFIG_AMBIQ_HAL_USE_SDIO)
zephyr_library_sources(hal/mcu/am_hal_card.c)
zephyr_library_sources(hal/mcu/am_hal_card_host.c)
zephyr_library_sources(hal/mcu/am_hal_sdhc.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_ADC)
    zephyr_library_sources(hal/am_hal_adc.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_GPIO)
    zephyr_library_sources(hal/am_hal_gpio.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_HWINFO)
    zephyr_library_sources(hal/mcu/am_hal_reset.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_I2C OR CONFIG_AMBIQ_HAL_USE_SPIC OR CONFIG_AMBIQ_HAL_USE_SPI)
    zephyr_library_sources(hal/mcu/am_hal_iom.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
    zephyr_library_sources(hal/mcu/am_hal_fault.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_SPID)
    zephyr_library_sources(hal/mcu/am_hal_ios.c)
endif()

if(CONFIG_AMBIQ_HAL_USE_MSPI)
    zephyr_library_sources(hal/mcu/am_hal_mspi.c)
    zephyr_library_sources(hal/mcu/am_hal_cmdq.c)
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

if(CONFIG_AMBIQ_HAL_USE_USB)
    zephyr_library_sources(hal/am_hal_usb.c)
	add_compile_definitions(AM_HAL_USB_CTRL_XFR_WAIT_STATUS_ACK_ZLP_FROM_STACK)
endif()

zephyr_include_directories(hal)
