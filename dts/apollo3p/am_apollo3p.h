//*****************************************************************************
//
//! @file am_apollo3p.h
//!
//! @brief Top DTS Include for Apollo3 Plus devices.
//!
//! This file provides all bus addresses of an apollo device for device tree.
//!
//! @addtogroup hal
//
//! @defgroup apollo3p
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

#ifndef AM_APOLLO3P_H
#define AM_APOLLO3P_H

#define APOLLO3P_BROWNOUT_IRQ            0
#define APOLLO3P_WDT_IRQ                 1
#define APOLLO3P_RTC_IRQ                 2
#define APOLLO3P_VCOMP_IRQ               3
#define APOLLO3P_IOSLAVE_IRQ             4
#define APOLLO3P_IOSLAVEACC_IRQ          5
#define APOLLO3P_IOMSTR0_IRQ             6
#define APOLLO3P_IOMSTR1_IRQ             7
#define APOLLO3P_IOMSTR2_IRQ             8
#define APOLLO3P_IOMSTR3_IRQ             9
#define APOLLO3P_IOMSTR4_IRQ             10
#define APOLLO3P_IOMSTR5_IRQ             11
#define APOLLO3P_BLE_IRQ                 12
#define APOLLO3P_GPIO_IRQ                13
#define APOLLO3P_CTIMER_IRQ              14
#define APOLLO3P_UART0_IRQ               15
#define APOLLO3P_UART1_IRQ               16
#define APOLLO3P_SCARD_IRQ               17
#define APOLLO3P_ADC_IRQ                 18
#define APOLLO3P_PDM_IRQ                 19
#define APOLLO3P_MSPI0_IRQ               20
#define APOLLO3P_STIMER_IRQ              22
#define APOLLO3P_STIMER_CMPR0_IRQ        23
#define APOLLO3P_STIMER_CMPR1_IRQ        24
#define APOLLO3P_STIMER_CMPR2_IRQ        25
#define APOLLO3P_STIMER_CMPR3_IRQ        26
#define APOLLO3P_STIMER_CMPR4_IRQ        27
#define APOLLO3P_STIMER_CMPR5_IRQ        28
#define APOLLO3P_STIMER_CMPR6_IRQ        29
#define APOLLO3P_STIMER_CMPR7_IRQ        30
#define APOLLO3P_CLKGEN_IRQ              31
#define APOLLO3P_MSPI1_IRQ               32
#define APOLLO3P_MSPI2_IRQ               33
#define APOLLO3P_MAX_IRQ                 34

#define APOLLO3P_ADC                     50010000
#define APOLLO3P_APBDMA                  40011000
#define APOLLO3P_BLEIF                   5000c000
#define APOLLO3P_CACHECTRL               40018000
#define APOLLO3P_CLKGEN                  40004000
#define APOLLO3P_CTIMER0                 40008000
#define APOLLO3P_CTIMER1                 40008020
#define APOLLO3P_CTIMER2                 40008040
#define APOLLO3P_CTIMER3                 40008060
#define APOLLO3P_CTIMER4                 40008080
#define APOLLO3P_CTIMER5                 400080a0
#define APOLLO3P_CTIMER6                 400080c0
#define APOLLO3P_CTIMER7                 400080e0
#define APOLLO3P_STIMER                  40008140
#define APOLLO3P_GPIO                    40010000
#define APOLLO3P_IOM0                    50004000
#define APOLLO3P_IOM1                    50005000
#define APOLLO3P_IOM2                    50006000
#define APOLLO3P_IOM3                    50007000
#define APOLLO3P_IOM4                    50008000
#define APOLLO3P_IOM5                    50009000
#define APOLLO3P_IOS                     50000000
#define APOLLO3P_MCUCTRL                 40020000
#define APOLLO3P_MSPI0                   50014000
#define APOLLO3P_MSPI1                   50015000
#define APOLLO3P_MSPI2                   50016000
#define APOLLO3P_PDM                     50011000
#define APOLLO3P_PWRCTRL                 40021000
#define APOLLO3P_RSTGEN                  40000000
#define APOLLO3P_RTC                     40004200
#define APOLLO3P_SCARD                   40080000
#define APOLLO3P_SECURITY                40030000
#define APOLLO3P_UART0                   4001c000
#define APOLLO3P_UART1                   4001d000
#define APOLLO3P_VCOMP                   4000c000
#define APOLLO3P_WDT                     40024000

#define APOLLO3P_ADC_BASE                0x50010000UL
#define APOLLO3P_APBDMA_BASE             0x40011000UL
#define APOLLO3P_BLEIF_BASE              0x5000c000UL
#define APOLLO3P_CACHECTRL_BASE          0x40018000UL
#define APOLLO3P_CLKGEN_BASE             0x40004000UL
#define APOLLO3P_CTIMER0_BASE            0x40008000UL
#define APOLLO3P_CTIMER1_BASE            0x40008020UL
#define APOLLO3P_CTIMER2_BASE            0x40008040UL
#define APOLLO3P_CTIMER3_BASE            0x40008060UL
#define APOLLO3P_CTIMER4_BASE            0x40008080UL
#define APOLLO3P_CTIMER5_BASE            0x400080a0UL
#define APOLLO3P_CTIMER6_BASE            0x400080c0UL
#define APOLLO3P_CTIMER7_BASE            0x400080e0UL
#define APOLLO3P_STIMER_BASE             0x40008140UL
#define APOLLO3P_GPIO_BASE               0x40010000UL
#define APOLLO3P_IOM0_BASE               0x50004000UL
#define APOLLO3P_IOM1_BASE               0x50005000UL
#define APOLLO3P_IOM2_BASE               0x50006000UL
#define APOLLO3P_IOM3_BASE               0x50007000UL
#define APOLLO3P_IOM4_BASE               0x50008000UL
#define APOLLO3P_IOM5_BASE               0x50009000UL
#define APOLLO3P_IOS_BASE                0x50000000UL
#define APOLLO3P_MCUCTRL_BASE            0x40020000UL
#define APOLLO3P_MSPI0_BASE              0x50014000UL
#define APOLLO3P_MSPI1_BASE              0x50015000UL
#define APOLLO3P_MSPI2_BASE              0x50016000UL
#define APOLLO3P_PDM_BASE                0x50011000UL
#define APOLLO3P_PWRCTRL_BASE            0x40021000UL
#define APOLLO3P_RSTGEN_BASE             0x40000000UL
#define APOLLO3P_RTC_BASE                0x40004200UL
#define APOLLO3P_SCARD_BASE              0x40080000UL
#define APOLLO3P_SECURITY_BASE           0x40030000UL
#define APOLLO3P_UART0_BASE              0x4001c000UL
#define APOLLO3P_UART1_BASE              0x4001d000UL
#define APOLLO3P_VCOMP_BASE              0x4000c000UL
#define APOLLO3P_WDT_BASE                0x40024000UL

#define APOLLO3P_ADC_SIZE                0x294UL
#define APOLLO3P_APBDMA_SIZE             0x44UL
#define APOLLO3P_BLEIF_SIZE              0x414UL
#define APOLLO3P_CACHECTRL_SIZE          0x110UL
#define APOLLO3P_CLKGEN_SIZE             0x110UL
#define APOLLO3P_CTIMER0_SIZE            0x20UL
#define APOLLO3P_CTIMER1_SIZE            0x20UL
#define APOLLO3P_CTIMER2_SIZE            0x20UL
#define APOLLO3P_CTIMER3_SIZE            0x20UL
#define APOLLO3P_CTIMER4_SIZE            0x20UL
#define APOLLO3P_CTIMER5_SIZE            0x20UL
#define APOLLO3P_CTIMER6_SIZE            0x20UL
#define APOLLO3P_CTIMER7_SIZE            0x20UL
#define APOLLO3P_STIMER_SIZE             0x1d0UL
#define APOLLO3P_GPIO_SIZE               0x254UL
#define APOLLO3P_IOM0_SIZE               0x414UL
#define APOLLO3P_IOM1_SIZE               0x414UL
#define APOLLO3P_IOM2_SIZE               0x414UL
#define APOLLO3P_IOM3_SIZE               0x414UL
#define APOLLO3P_IOM4_SIZE               0x414UL
#define APOLLO3P_IOM5_SIZE               0x414UL
#define APOLLO3P_IOS_SIZE                0x220UL
#define APOLLO3P_MCUCTRL_SIZE            0x3dcUL
#define APOLLO3P_MSPI0_SIZE              0x2c8UL
#define APOLLO3P_MSPI1_SIZE              0x2c8UL
#define APOLLO3P_MSPI2_SIZE              0x2c8UL
#define APOLLO3P_PDM_SIZE                0x294UL
#define APOLLO3P_PWRCTRL_SIZE            0x30UL
#define APOLLO3P_RSTGEN_SIZE             0x210UL
#define APOLLO3P_RTC_SIZE                0x110UL
#define APOLLO3P_SCARD_SIZE              0x104UL
#define APOLLO3P_SECURITY_SIZE           0x90UL
#define APOLLO3P_UART0_SIZE              0x48UL
#define APOLLO3P_UART1_SIZE              0x48UL
#define APOLLO3P_VCOMP_SIZE              0x210UL
#define APOLLO3P_WDT_SIZE                0x210UL

#define APOLLO3P_TCM                     10000000
#define APOLLO3P_TCM_BASE                0x10000000UL
#define APOLLO3P_TCM_SIZE                0x10000UL

#define APOLLO3P_SRAM                    10010000
#define APOLLO3P_SRAM_BASE               0x10010000UL
#define APOLLO3P_SRAM_SIZE               0xb0000UL

#define APOLLO3P_FLASH                   0
#define APOLLO3P_FLASH_BASE              0x00000000UL
#define APOLLO3P_FLASH_SIZE              0x200000UL

#define APOLLO3P_FLASH_BOOT              0
#define APOLLO3P_FLASH_BOOT_BASE         0x00000000UL
#define APOLLO3P_FLASH_BOOT_SIZE         0xc000UL

#define APOLLO3P_FLASH_APP               c000
#define APOLLO3P_FLASH_APP_BASE          0x0000c000UL
#define APOLLO3P_FLASH_APP_SIZE          0x1f4000UL

#define APOLLO3P_MSPI0_XIP_APERTURE       2000000
#define APOLLO3P_MSPI0_XIP_APERTURE_BASE  0x02000000UL
#define APOLLO3P_MSPI0_XIP_APERTURE_SIZE  0x02000000UL

#define APOLLO3P_MSPI1_XIP_APERTURE       4000000
#define APOLLO3P_MSPI1_XIP_APERTURE_BASE  0x04000000UL
#define APOLLO3P_MSPI1_XIP_APERTURE_SIZE  0x02000000UL

#define APOLLO3P_MSPI2_XIP_APERTURE       6000000
#define APOLLO3P_MSPI2_XIP_APERTURE_BASE  0x06000000UL
#define APOLLO3P_MSPI2_XIP_APERTURE_SIZE  0x02000000UL

#define APOLLO3P_MSPI0_XIPMM_APERTURE      52000000
#define APOLLO3P_MSPI0_XIPMM_APERTURE_BASE 0x52000000UL
#define APOLLO3P_MSPI0_XIPMM_APERTURE_SIZE 0x02000000UL

#define APOLLO3P_MSPI1_XIPMM_APERTURE      54000000
#define APOLLO3P_MSPI1_XIPMM_APERTURE_BASE 0x54000000UL
#define APOLLO3P_MSPI1_XIPMM_APERTURE_SIZE 0x02000000UL

#define APOLLO3P_MSPI2_XIPMM_APERTURE      56000000
#define APOLLO3P_MSPI2_XIPMM_APERTURE_BASE 0x56000000UL
#define APOLLO3P_MSPI2_XIPMM_APERTURE_SIZE 0x02000000UL

#endif // AM_APOLLO3P_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
