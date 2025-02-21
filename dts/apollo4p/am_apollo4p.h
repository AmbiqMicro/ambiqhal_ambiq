//*****************************************************************************
//
//! @file am_apollo4p.h
//!
//! @brief Top DTS Include for Apollo4 Plus devices.
//!
//! This file provides all bus addresses of an apollo device for device tree.
//!
//! @addtogroup hal
//
//! @defgroup apollo4p
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

#ifndef AM_APOLLO4P_H
#define AM_APOLLO4P_H

#define APOLLO4P_BROWNOUT_IRQ            0
#define APOLLO4P_WDT_IRQ                 1
#define APOLLO4P_RTC_IRQ                 2
#define APOLLO4P_VCOMP_IRQ               3
#define APOLLO4P_IOSLAVE_IRQ             4
#define APOLLO4P_IOSLAVEACC_IRQ          5
#define APOLLO4P_IOMSTR0_IRQ             6
#define APOLLO4P_IOMSTR1_IRQ             7
#define APOLLO4P_IOMSTR2_IRQ             8
#define APOLLO4P_IOMSTR3_IRQ             9
#define APOLLO4P_IOMSTR4_IRQ             10
#define APOLLO4P_IOMSTR5_IRQ             11
#define APOLLO4P_IOMSTR6_IRQ             12
#define APOLLO4P_IOMSTR7_IRQ             13
#define APOLLO4P_TIMER_IRQ               14
#define APOLLO4P_UART0_IRQ               15
#define APOLLO4P_UART1_IRQ               16
#define APOLLO4P_UART2_IRQ               17
#define APOLLO4P_UART3_IRQ               18
#define APOLLO4P_ADC_IRQ                 19
#define APOLLO4P_MSPI0_IRQ               20
#define APOLLO4P_MSPI1_IRQ               21
#define APOLLO4P_MSPI2_IRQ               22
#define APOLLO4P_CLKGEN_IRQ              23
#define APOLLO4P_CRYPTOSEC_IRQ           24
#define APOLLO4P_SDIO_IRQ                26
#define APOLLO4P_USB_IRQ                 27
#define APOLLO4P_GPU_IRQ                 28
#define APOLLO4P_DC_IRQ                  29
#define APOLLO4P_DSI_IRQ                 30
#define APOLLO4P_STIMER_CMPR0_IRQ        32
#define APOLLO4P_STIMER_CMPR1_IRQ        33
#define APOLLO4P_STIMER_CMPR2_IRQ        34
#define APOLLO4P_STIMER_CMPR3_IRQ        35
#define APOLLO4P_STIMER_CMPR4_IRQ        36
#define APOLLO4P_STIMER_CMPR5_IRQ        37
#define APOLLO4P_STIMER_CMPR6_IRQ        38
#define APOLLO4P_STIMER_CMPR7_IRQ        39
#define APOLLO4P_STIMER_OVF_IRQ          40
#define APOLLO4P_AUDADC_IRQ              42
#define APOLLO4P_I2S0_IRQ                44
#define APOLLO4P_I2S1_IRQ                45
#define APOLLO4P_PDM0_IRQ                48
#define APOLLO4P_PDM1_IRQ                49
#define APOLLO4P_PDM2_IRQ                50
#define APOLLO4P_PDM3_IRQ                51
#define APOLLO4P_GPIO0_001F_IRQ          56
#define APOLLO4P_GPIO0_203F_IRQ          57
#define APOLLO4P_GPIO0_405F_IRQ          58
#define APOLLO4P_GPIO0_607F_IRQ          59
#define APOLLO4P_GPIO1_001F_IRQ          60
#define APOLLO4P_GPIO1_203F_IRQ          61
#define APOLLO4P_GPIO1_405F_IRQ          62
#define APOLLO4P_GPIO1_607F_IRQ          63
#define APOLLO4P_TIMER0_IRQ              67
#define APOLLO4P_TIMER1_IRQ              68
#define APOLLO4P_TIMER2_IRQ              69
#define APOLLO4P_TIMER3_IRQ              70
#define APOLLO4P_TIMER4_IRQ              71
#define APOLLO4P_TIMER5_IRQ              72
#define APOLLO4P_TIMER6_IRQ              73
#define APOLLO4P_TIMER7_IRQ              74
#define APOLLO4P_TIMER8_IRQ              75
#define APOLLO4P_TIMER9_IRQ              76
#define APOLLO4P_TIMER10_IRQ             77
#define APOLLO4P_TIMER11_IRQ             78
#define APOLLO4P_TIMER12_IRQ             79
#define APOLLO4P_TIMER13_IRQ             80
#define APOLLO4P_TIMER14_IRQ             81
#define APOLLO4P_TIMER15_IRQ             82
#define APOLLO4P_CACHE_IRQ               83
#define APOLLO4P_MAX_IRQ                 84

#define APOLLO4P_ADC                     40038000
#define APOLLO4P_APBDMA                  40011000
#define APOLLO4P_AUDADC                  40210000
#define APOLLO4P_CLKGEN                  40004000
#define APOLLO4P_CPU                     48000000
#define APOLLO4P_CRYPTO                  400c0000
#define APOLLO4P_DC                      400a0000
#define APOLLO4P_DSI                     400a8000
#define APOLLO4P_FPIO                    48001000
#define APOLLO4P_GPIO                    40010000
#define APOLLO4P_GPU                     40090000
#define APOLLO4P_I2S0                    40208000
#define APOLLO4P_I2S1                    40209000
#define APOLLO4P_IOM0                    40050000
#define APOLLO4P_IOM1                    40051000
#define APOLLO4P_IOM2                    40052000
#define APOLLO4P_IOM3                    40053000
#define APOLLO4P_IOM4                    40054000
#define APOLLO4P_IOM5                    40055000
#define APOLLO4P_IOM6                    40056000
#define APOLLO4P_IOM7                    40057000
#define APOLLO4P_IOS                     40034000
#define APOLLO4P_MCUCTRL                 40020000
#define APOLLO4P_MSPI0                   40060000
#define APOLLO4P_MSPI1                   40061000
#define APOLLO4P_MSPI2                   40062000
#define APOLLO4P_PDM0                    40201000
#define APOLLO4P_PDM1                    40202000
#define APOLLO4P_PDM2                    40203000
#define APOLLO4P_PWRCTRL                 40021000
#define APOLLO4P_RSTGEN                  40000000
#define APOLLO4P_RTC                     40004800
#define APOLLO4P_SDIO                    40070000
#define APOLLO4P_SECURITY                40030000
#define APOLLO4P_STIMER                  40008800
#define APOLLO4P_TIMER0                  40008200
#define APOLLO4P_TIMER1                  40008220
#define APOLLO4P_TIMER2                  40008240
#define APOLLO4P_TIMER3                  40008260
#define APOLLO4P_TIMER4                  40008280
#define APOLLO4P_TIMER5                  400082a0
#define APOLLO4P_TIMER6                  400082c0
#define APOLLO4P_TIMER7                  400082e0
#define APOLLO4P_TIMER8                  40008300
#define APOLLO4P_TIMER9                  40008320
#define APOLLO4P_TIMER10                 40008340
#define APOLLO4P_TIMER11                 40008360
#define APOLLO4P_TIMER12                 40008380
#define APOLLO4P_TIMER13                 400083a0
#define APOLLO4P_TIMER14                 400083c0
#define APOLLO4P_TIMER15                 400083e0
#define APOLLO4P_UART0                   4001c000
#define APOLLO4P_UART1                   4001d000
#define APOLLO4P_UART2                   4001e000
#define APOLLO4P_UART3                   4001f000
#define APOLLO4P_USBPHY                  400b4000
#define APOLLO4P_USB                     400b0000
#define APOLLO4P_VCOMP                   4000c000
#define APOLLO4P_WDT                     40024000

#define APOLLO4P_ADC_BASE                0x40038000UL
#define APOLLO4P_APBDMA_BASE             0x40011000UL
#define APOLLO4P_AUDADC_BASE             0x40210000UL
#define APOLLO4P_CLKGEN_BASE             0x40004000UL
#define APOLLO4P_CPU_BASE                0x48000000UL
#define APOLLO4P_CRYPTO_BASE             0x400c0000UL
#define APOLLO4P_DC_BASE                 0x400a0000UL
#define APOLLO4P_DSI_BASE                0x400a8000UL
#define APOLLO4P_FPIO_BASE               0x48001000UL
#define APOLLO4P_GPIO_BASE               0x40010000UL
#define APOLLO4P_GPU_BASE                0x40090000UL
#define APOLLO4P_I2S0_BASE               0x40208000UL
#define APOLLO4P_I2S1_BASE               0x40209000UL
#define APOLLO4P_IOM0_BASE               0x40050000UL
#define APOLLO4P_IOM1_BASE               0x40051000UL
#define APOLLO4P_IOM2_BASE               0x40052000UL
#define APOLLO4P_IOM3_BASE               0x40053000UL
#define APOLLO4P_IOM4_BASE               0x40054000UL
#define APOLLO4P_IOM5_BASE               0x40055000UL
#define APOLLO4P_IOM6_BASE               0x40056000UL
#define APOLLO4P_IOM7_BASE               0x40057000UL
#define APOLLO4P_IOS_BASE                0x40034000UL
#define APOLLO4P_MCUCTRL_BASE            0x40020000UL
#define APOLLO4P_MSPI0_BASE              0x40060000UL
#define APOLLO4P_MSPI1_BASE              0x40061000UL
#define APOLLO4P_MSPI2_BASE              0x40062000UL
#define APOLLO4P_PDM0_BASE               0x40201000UL
#define APOLLO4P_PDM1_BASE               0x40202000UL
#define APOLLO4P_PDM2_BASE               0x40203000UL
#define APOLLO4P_PDM3_BASE               0x40204000UL
#define APOLLO4P_PWRCTRL_BASE            0x40021000UL
#define APOLLO4P_RSTGEN_BASE             0x40000000UL
#define APOLLO4P_RTC_BASE                0x40004800UL
#define APOLLO4P_SDIO_BASE               0x40070000UL
#define APOLLO4P_SECURITY_BASE           0x40030000UL
#define APOLLO4P_STIMER_BASE             0x40008800UL
#define APOLLO4P_TIMER0_BASE             0x40008200UL
#define APOLLO4P_TIMER1_BASE             0x40008220UL
#define APOLLO4P_TIMER2_BASE             0x40008240UL
#define APOLLO4P_TIMER3_BASE             0x40008260UL
#define APOLLO4P_TIMER4_BASE             0x40008280UL
#define APOLLO4P_TIMER5_BASE             0x400082a0UL
#define APOLLO4P_TIMER6_BASE             0x400082c0UL
#define APOLLO4P_TIMER7_BASE             0x400082e0UL
#define APOLLO4P_TIMER8_BASE             0x40008300UL
#define APOLLO4P_TIMER9_BASE             0x40008320UL
#define APOLLO4P_TIMER10_BASE            0x40008340UL
#define APOLLO4P_TIMER11_BASE            0x40008360UL
#define APOLLO4P_TIMER12_BASE            0x40008380UL
#define APOLLO4P_TIMER13_BASE            0x400083a0UL
#define APOLLO4P_TIMER14_BASE            0x400083c0UL
#define APOLLO4P_TIMER15_BASE            0x400083e0UL
#define APOLLO4P_UART0_BASE              0x4001c000UL
#define APOLLO4P_UART1_BASE              0x4001d000UL
#define APOLLO4P_UART2_BASE              0x4001e000UL
#define APOLLO4P_UART3_BASE              0x4001f000UL
#define APOLLO4P_USBPHY_BASE             0x400b4000UL
#define APOLLO4P_USB_BASE                0x400b0000UL
#define APOLLO4P_VCOMP_BASE              0x4000c000UL
#define APOLLO4P_WDT_BASE                0x40024000UL

#define APOLLO4P_ADC_SIZE                0x294UL
#define APOLLO4P_APBDMA_SIZE             0x44UL
#define APOLLO4P_AUDADC_SIZE             0x294UL
#define APOLLO4P_CLKGEN_SIZE             0x110UL
#define APOLLO4P_CPU_SIZE                0x120UL
#define APOLLO4P_CRYPTO_SIZE             0x1f30UL
#define APOLLO4P_DC_SIZE                 0x2000UL
#define APOLLO4P_DSI_SIZE                0xa4UL
#define APOLLO4P_FPIO_SIZE               0x70UL
#define APOLLO4P_GPIO_SIZE               0x440UL
#define APOLLO4P_GPU_SIZE                0xff4UL
#define APOLLO4P_I2S0_SIZE               0x404UL
#define APOLLO4P_I2S1_SIZE               0x404UL
#define APOLLO4P_IOM0_SIZE               0x38cUL
#define APOLLO4P_IOM1_SIZE               0x38cUL
#define APOLLO4P_IOM2_SIZE               0x38cUL
#define APOLLO4P_IOM3_SIZE               0x38cUL
#define APOLLO4P_IOM4_SIZE               0x38cUL
#define APOLLO4P_IOM5_SIZE               0x38cUL
#define APOLLO4P_IOM6_SIZE               0x38cUL
#define APOLLO4P_IOM7_SIZE               0x38cUL
#define APOLLO4P_IOS_SIZE                0x220UL
#define APOLLO4P_MCUCTRL_SIZE            0x458UL
#define APOLLO4P_MSPI0_SIZE              0x314UL
#define APOLLO4P_MSPI1_SIZE              0x314UL
#define APOLLO4P_MSPI2_SIZE              0x314UL
#define APOLLO4P_PDM0_SIZE               0x254UL
#define APOLLO4P_PDM1_SIZE               0x254UL
#define APOLLO4P_PDM2_SIZE               0x254UL
#define APOLLO4P_PDM3_SIZE               0x254UL
#define APOLLO4P_PWRCTRL_SIZE            0x250UL
#define APOLLO4P_RSTGEN_SIZE             0x210UL
#define APOLLO4P_RTC_SIZE                0x210UL
#define APOLLO4P_SDIO_SIZE               0x100UL
#define APOLLO4P_SECURITY_SIZE           0x90UL
#define APOLLO4P_STIMER_SIZE             0x110UL
#define APOLLO4P_TIMER0_SIZE             0x20UL
#define APOLLO4P_TIMER1_SIZE             0x20UL
#define APOLLO4P_TIMER2_SIZE             0x20UL
#define APOLLO4P_TIMER3_SIZE             0x20UL
#define APOLLO4P_TIMER4_SIZE             0x20UL
#define APOLLO4P_TIMER5_SIZE             0x20UL
#define APOLLO4P_TIMER6_SIZE             0x20UL
#define APOLLO4P_TIMER7_SIZE             0x20UL
#define APOLLO4P_TIMER8_SIZE             0x20UL
#define APOLLO4P_TIMER9_SIZE             0x20UL
#define APOLLO4P_TIMER10_SIZE            0x20UL
#define APOLLO4P_TIMER11_SIZE            0x20UL
#define APOLLO4P_TIMER12_SIZE            0x20UL
#define APOLLO4P_TIMER13_SIZE            0x20UL
#define APOLLO4P_TIMER14_SIZE            0x20UL
#define APOLLO4P_TIMER15_SIZE            0x20UL
#define APOLLO4P_UART0_SIZE              0x48UL
#define APOLLO4P_UART1_SIZE              0x48UL
#define APOLLO4P_UART2_SIZE              0x48UL
#define APOLLO4P_UART3_SIZE              0x48UL
#define APOLLO4P_USBPHY_SIZE             0x88UL
#define APOLLO4P_USB_SIZE                0x202cUL
#define APOLLO4P_VCOMP_SIZE              0x210UL
#define APOLLO4P_WDT_SIZE                0x230UL

#define APOLLO4P_TCM                     10000000
#define APOLLO4P_TCM_BASE                0x10000000UL
#define APOLLO4P_TCM_SIZE                0x60000UL

#define APOLLO4P_SRAM0                   10060000
#define APOLLO4P_SRAM0_BASE              0x10060000UL
#define APOLLO4P_SRAM0_SIZE              0x100000UL

#define APOLLO4P_SRAM1                   101c0000
#define APOLLO4P_SRAM1_BASE              0x101c0000UL
#define APOLLO4P_SRAM1_SIZE              0x100000UL

#define APOLLO4P_FLASH                   0
#define APOLLO4P_FLASH_BASE              0x00000000UL
#define APOLLO4P_FLASH_SIZE              0x200000UL

#define APOLLO4P_FLASH_BOOT              0
#define APOLLO4P_FLASH_BOOT_BASE         0x00000000UL
#define APOLLO4P_FLASH_BOOT_SIZE         0x18000UL

#define APOLLO4P_FLASH_APP               18000
#define APOLLO4P_FLASH_APP_BASE          0x00018000UL
#define APOLLO4P_FLASH_APP_SIZE          0x1e8000UL

#define APOLLO4P_MSPI0_XIP_APERTURE       14000000
#define APOLLO4P_MSPI0_XIP_APERTURE_BASE  0x14000000UL
#define APOLLO4P_MSPI0_XIP_APERTURE_SIZE  0x04000000UL

#define APOLLO4P_MSPI1_XIP_APERTURE       18000000
#define APOLLO4P_MSPI1_XIP_APERTURE_BASE  0x18000000UL
#define APOLLO4P_MSPI1_XIP_APERTURE_SIZE  0x04000000UL

#define APOLLO4P_MSPI2_XIP_APERTURE       1c000000
#define APOLLO4P_MSPI2_XIP_APERTURE_BASE  0x1c000000UL
#define APOLLO4P_MSPI2_XIP_APERTURE_SIZE  0x04000000UL

#endif // AM_APOLLO4P_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
