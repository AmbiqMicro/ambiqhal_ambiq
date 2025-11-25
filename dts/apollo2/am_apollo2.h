//*****************************************************************************
//
//! @file am_apollo2.h
//!
//! @brief Top DTS Include for Apollo2 class devices.
//!
//! This file provides all bus addresses of an apollo2 device for device tree.
//!
//! @addtogroup hal
//
//! @defgroup apollo2
//! @ingroup hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef AM_APOLLO2_H
#define AM_APOLLO2_H

//-----------------------------------------------------------------------------
// Apollo2 Interrupt Numbers
//-----------------------------------------------------------------------------
#define APOLLO2_BROWNOUT_IRQ              0
#define APOLLO2_WDT_IRQ                   1
#define APOLLO2_RTC_IRQ                   2 // Part of Clock Control and RTC IRQ2
#define APOLLO2_VCOMP_IRQ                 3
#define APOLLO2_IOSLAVE_IRQ               4
#define APOLLO2_IOSLAVEACC_IRQ            5 // I2C/SPI Slave Register Access
#define APOLLO2_IOMSTR0_IRQ               6 // I2C/SPI Master0
#define APOLLO2_IOMSTR1_IRQ               7 // I2C/SPI Master1
#define APOLLO2_IOMSTR2_IRQ               8 // I2C/SPI Master2
#define APOLLO2_IOMSTR3_IRQ               9 // I2C/SPI Master3
#define APOLLO2_IOMSTR4_IRQ               10 // I2C/SPI Master4
#define APOLLO2_IOMSTR5_IRQ               11 // I2C/SPI Master5
#define APOLLO2_GPIO_IRQ                  12
#define APOLLO2_CTIMER_IRQ                13 // Counter/Timers
#define APOLLO2_UART0_IRQ                 14
#define APOLLO2_UART1_IRQ                 15
#define APOLLO2_ADC_IRQ                   16
#define APOLLO2_PDM_IRQ                   17
#define APOLLO2_STIMER_IRQ                18 // STimer Capture/Overflow
#define APOLLO2_STIMER_CMPR0_IRQ          19 // STimer Compare [0:7] starts at IRQ19
#define APOLLO2_STIMER_CMPR1_IRQ          20
#define APOLLO2_STIMER_CMPR2_IRQ          21
#define APOLLO2_STIMER_CMPR3_IRQ          22
#define APOLLO2_STIMER_CMPR4_IRQ          23
#define APOLLO2_STIMER_CMPR5_IRQ          24
#define APOLLO2_STIMER_CMPR6_IRQ          25
#define APOLLO2_STIMER_CMPR7_IRQ          26
#define APOLLO2_SW_INT0_IRQ               28 // SW INT[0-3] starts at IRQ28
#define APOLLO2_SW_INT1_IRQ               29
#define APOLLO2_SW_INT2_IRQ               30
#define APOLLO2_SW_INT3_IRQ               31
#define APOLLO2_MAX_IRQ                   32 // Reflects max peripheral IRQ + 1

//-----------------------------------------------------------------------------
// Apollo2 Peripheral Base Addresses
//-----------------------------------------------------------------------------
#define APOLLO2_RSTGEN                    40000000 // Reset / BoD Control
#define APOLLO2_CLKGEN                    40004000 // Clock Generator / RTC
#define APOLLO2_CTIMER                    40008000 // Timers
#define APOLLO2_VCOMP                     4000c000 // Voltage Comparator
#define APOLLO2_GPIO                      40010000 // GPIO Control
#define APOLLO2_APBDMA                    40011000
#define APOLLO2_CACHECTRL                 40018000 // Flash Cache Control
#define APOLLO2_UART0                     4001c000
#define APOLLO2_UART1                     4001d000
#define APOLLO2_MCUCTRL                   40020000
#define APOLLO2_PWRCTRL                   40021000 // Power Control
#define APOLLO2_WDT                       40024000 // Watchdog Timer

#define APOLLO2_IOS                       50000000 // I2C/SPI Slave
#define APOLLO2_IOM0                      50004000 // I2C/SPI Master0
#define APOLLO2_IOM1                      50005000 // I2C/SPI Master1
#define APOLLO2_IOM2                      50006000 // I2C/SPI Master2
#define APOLLO2_IOM3                      50007000 // I2C/SPI Master3
#define APOLLO2_IOM4                      50008000 // I2C/SPI Master4
#define APOLLO2_IOM5                      50009000 // I2C/SPI Master5
#define APOLLO2_ADC                       50010000
#define APOLLO2_PDM                       50011000
#define APOLLO2_FLASH_OTP                 50020000

#define APOLLO2_RSTGEN_BASE               0x40000000UL
#define APOLLO2_CLKGEN_BASE               0x40004000UL
#define APOLLO2_CTIMER_BASE               0x40008000UL
#define APOLLO2_VCOMP_BASE                0x4000c000UL
#define APOLLO2_GPIO_BASE                 0x40010000UL
#define APOLLO2_APBDMA_BASE               0x40011000UL
#define APOLLO2_CACHECTRL_BASE            0x40018000UL
#define APOLLO2_UART0_BASE                0x4001c000UL
#define APOLLO2_UART1_BASE                0x4001d000UL
#define APOLLO2_MCUCTRL_BASE              0x40020000UL
#define APOLLO2_PWRCTRL_BASE              0x40021000UL
#define APOLLO2_WDT_BASE                  0x40024000UL

#define APOLLO2_IOS_BASE                  0x50000000UL
#define APOLLO2_IOM0_BASE                 0x50004000UL
#define APOLLO2_IOM1_BASE                 0x50005000UL
#define APOLLO2_IOM2_BASE                 0x50006000UL
#define APOLLO2_IOM3_BASE                 0x50007000UL
#define APOLLO2_IOM4_BASE                 0x50008000UL
#define APOLLO2_IOM5_BASE                 0x50009000UL
#define APOLLO2_ADC_BASE                  0x50010000UL
#define APOLLO2_PDM_BASE                  0x50011000UL
#define APOLLO2_FLASH_OTP_BASE            0x50020000UL

//-----------------------------------------------------------------------------
// Apollo2 Peripheral Sizes
//-----------------------------------------------------------------------------
#define APOLLO2_RSTGEN_SIZE               0x400UL // 0x400003FF - 0x40000000 + 1 = 0x400
#define APOLLO2_CLKGEN_SIZE               0x400UL // 0x400041FF - 0x40004000 + 1 = 0x200 (using 0x400 boundary)
#define APOLLO2_CTIMER_SIZE               0x400UL // 0x400083FF - 0x40008000 + 1 = 0x400
#define APOLLO2_VCOMP_SIZE                0x400UL // 0x4000C3FF - 0x4000C000 + 1 = 0x400
#define APOLLO2_GPIO_SIZE                 0x400UL // 0x400103FF - 0x40010000 + 1 = 0x400
#define APOLLO2_APBDMA_SIZE               0x400UL
#define APOLLO2_CACHECTRL_SIZE            0x400UL // 0x40018FFF - 0x40018000 + 1 = 0x1000 (using 0x400 boundary)
#define APOLLO2_UART0_SIZE                0x400UL // 0x4001C3FF - 0x4001C000 + 1 = 0x400
#define APOLLO2_UART1_SIZE                0x400UL // 0x4001D3FF - 0x4001D000 + 1 = 0x400
#define APOLLO2_MCUCTRL_SIZE              0x400UL // 0x400203FF - 0x40020000 + 1 = 0x400
#define APOLLO2_PWRCTRL_SIZE              0x400UL // 0x400213FF - 0x40021000 + 1 = 0x400
#define APOLLO2_WDT_SIZE                  0x400UL // 0x400243FF - 0x40024000 + 1 = 0x400

#define APOLLO2_IOS_SIZE                  0x400UL // 0x500003FF - 0x50000000 + 1 = 0x400
#define APOLLO2_IOM0_SIZE                 0x400UL // 0x500043FF - 0x50004000 + 1 = 0x400
#define APOLLO2_IOM1_SIZE                 0x400UL // 0x500053FF - 0x50005000 + 1 = 0x400
#define APOLLO2_IOM2_SIZE                 0x400UL // 0x500063FF - 0x50006000 + 1 = 0x400
#define APOLLO2_IOM3_SIZE                 0x400UL // 0x500073FF - 0x50007000 + 1 = 0x400
#define APOLLO2_IOM4_SIZE                 0x400UL // 0x500083FF - 0x50008000 + 1 = 0x400
#define APOLLO2_IOM5_SIZE                 0x400UL // 0x500093FF - 0x50009000 + 1 = 0x400
#define APOLLO2_ADC_SIZE                  0x400UL // 0x500103FF - 0x50010000 + 1 = 0x400
#define APOLLO2_PDM_SIZE                  0x400UL // 0x500113FF - 0x50011000 + 1 = 0x400
#define APOLLO2_FLASH_OTP_SIZE            0x10000UL // 0x5002FFFF - 0x50020000 + 1 = 0x10000

//-----------------------------------------------------------------------------
// Apollo2 Memory Regions
//-----------------------------------------------------------------------------
#define APOLLO2_SRAM                      10000000 // Low-power SRAM starts at 0x10000000
#define APOLLO2_SRAM_BASE                 0x10000000UL
#define APOLLO2_SRAM_SIZE                 0x40000UL // 256KB = 0x40000 bytes

#define APOLLO2_FLASH                     0 // Flash Memory starts at 0x00000000
#define APOLLO2_FLASH_BASE                0x00000000UL
#define APOLLO2_FLASH_SIZE                0x100000UL // 1MB = 0x100000 bytes

#define APOLLO2_FLASH_BOOT                0
#define APOLLO2_FLASH_BOOT_BASE           0x00000000UL
#define APOLLO2_FLASH_BOOT_SIZE           0x8000UL

#define APOLLO2_FLASH_APP                 8000
#define APOLLO2_FLASH_APP_BASE            0x00008000UL
#define APOLLO2_FLASH_APP_SIZE            0xf8000UL

#endif // AM_APOLLO2_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
