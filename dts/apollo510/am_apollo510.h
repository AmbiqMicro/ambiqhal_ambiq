//*****************************************************************************
//
//! @file am_apollo510.h
//!
//! @brief Top DTS Include for Apollo510 class devices.
//!
//! This file provides all bus addresses of an apollo device for device tree.
//!
//! @addtogroup hal
//
//! @defgroup apollo510
//! @ingroup hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2024, Ambiq Micro, Inc.
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
// This is part of revision stable-ab51288298 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_APOLLO510_H
#define AM_APOLLO510_H

#define ADC_BASE_NAME                   40038000
#define AUDADC_BASE_NAME                40210000
#define CLKGEN_BASE_NAME                40004000
#define CRYPTO_BASE_NAME                400c0000
#define DC_BASE_NAME                    400a0000
#define DSI_BASE_NAME                   400a8000
#define FPIO_BASE_NAME                  40011000
#define GPIO_BASE_NAME                  40010000
#define GPU_BASE_NAME                   40090000
#define I2S0_BASE_NAME                  40208000
#define I2S1_BASE_NAME                  40209000
#define IOM0_BASE_NAME                  40050000
#define IOM1_BASE_NAME                  40051000
#define IOM2_BASE_NAME                  40052000
#define IOM3_BASE_NAME                  40053000
#define IOM4_BASE_NAME                  40054000
#define IOM5_BASE_NAME                  40055000
#define IOM6_BASE_NAME                  40056000
#define IOM7_BASE_NAME                  40057000
#define IOSLAVEFD0_BASE_NAME            40035000
#define IOSLAVEFD1_BASE_NAME            40036000
#define IOSLAVE_BASE_NAME               40034000
#define MCUCTRL_BASE_NAME               40020000
#define MSPI0_BASE_NAME                 40060000
#define MSPI1_BASE_NAME                 40061000
#define MSPI2_BASE_NAME                 40062000
#define MSPI3_BASE_NAME                 40063000
#define OTP_BASE_NAME                   40014800
#define PDM0_BASE_NAME                  40201000
#define PDM1_BASE_NAME                  40202000
#define PDM2_BASE_NAME                  40203000
#define PDM3_BASE_NAME                  40204000
#define PWRCTRL_BASE_NAME               40021000
#define RSTGEN_BASE_NAME                40000000
#define RTC_BASE_NAME                   40004800
#define SDIO0_BASE_NAME                 40070000
#define SDIO1_BASE_NAME                 40071000
#define SECURITY_BASE_NAME              40030000
#define SSC_BASE_NAME                   40025000
#define STIMER_BASE_NAME                40008800
#define TIMER_BASE_NAME                 40008000
#define UART0_BASE_NAME                 40039000
#define UART1_BASE_NAME                 4003a000
#define UART2_BASE_NAME                 4003b000
#define UART3_BASE_NAME                 4003c000
#define USBPHY_BASE_NAME                400b4000
#define USB_BASE_NAME                   400b0000
#define VCOMP_BASE_NAME                 4000c000
#define WDT_BASE_NAME                   40024000

#define ADC_REG_BASE                    0x40038000UL
#define AUDADC_REG_BASE                 0x40210000UL
#define CLKGEN_REG_BASE                 0x40004000UL
#define CRYPTO_REG_BASE                 0x400C0000UL
#define DC_REG_BASE                     0x400A0000UL
#define DSI_REG_BASE                    0x400A8000UL
#define FPIO_REG_BASE                   0x40011000UL
#define GPIO_REG_BASE                   0x40010000UL
#define GPU_REG_BASE                    0x40090000UL
#define I2S0_REG_BASE                   0x40208000UL
#define I2S1_REG_BASE                   0x40209000UL
#define IOM0_REG_BASE                   0x40050000UL
#define IOM1_REG_BASE                   0x40051000UL
#define IOM2_REG_BASE                   0x40052000UL
#define IOM3_REG_BASE                   0x40053000UL
#define IOM4_REG_BASE                   0x40054000UL
#define IOM5_REG_BASE                   0x40055000UL
#define IOM6_REG_BASE                   0x40056000UL
#define IOM7_REG_BASE                   0x40057000UL
#define IOSLAVEFD0_REG_BASE             0x40035000UL
#define IOSLAVEFD1_REG_BASE             0x40036000UL
#define IOSLAVE_REG_BASE                0x40034000UL
#define MCUCTRL_REG_BASE                0x40020000UL
#define MSPI0_REG_BASE                  0x40060000UL
#define MSPI1_REG_BASE                  0x40061000UL
#define MSPI2_REG_BASE                  0x40062000UL
#define MSPI3_REG_BASE                  0x40063000UL
#define OTP_REG_BASE                    0x40014800UL
#define PDM0_REG_BASE                   0x40201000UL
#define PDM1_REG_BASE                   0x40202000UL
#define PDM2_REG_BASE                   0x40203000UL
#define PDM3_REG_BASE                   0x40204000UL
#define PWRCTRL_REG_BASE                0x40021000UL
#define RSTGEN_REG_BASE                 0x40000000UL
#define RTC_REG_BASE                    0x40004800UL
#define SDIO0_REG_BASE                  0x40070000UL
#define SDIO1_REG_BASE                  0x40071000UL
#define SECURITY_REG_BASE               0x40030000UL
#define SSC_REG_BASE                    0x40025000UL
#define STIMER_REG_BASE                 0x40008800UL
#define TIMER_REG_BASE                  0x40008000UL
#define UART0_REG_BASE                  0x40039000UL
#define UART1_REG_BASE                  0x4003A000UL
#define UART2_REG_BASE                  0x4003B000UL
#define UART3_REG_BASE                  0x4003C000UL
#define USBPHY_REG_BASE                 0x400B4000UL
#define USB_REG_BASE                    0x400B0000UL
#define VCOMP_REG_BASE                  0x4000C000UL
#define WDT_REG_BASE                    0x40024000UL

#define ADC_REG_SIZE                    0x2a0UL
#define AUDADC_REG_SIZE                 0x2a0UL
#define CLKGEN_REG_SIZE                 0x124UL
#define CRYPTO_REG_SIZE                 0x1f30UL
#define DC_REG_SIZE                     0x1c04UL
#define DSI_REG_SIZE                    0xa4UL
#define FPIO_REG_SIZE                   0xc4UL
#define GPIO_REG_SIZE                   0x610UL
#define GPU_REG_SIZE                    0xff4UL
#define I2S0_REG_SIZE                   0x310UL
#define I2S1_REG_SIZE                   0x310UL
#define IOM0_REG_SIZE                   0x38cUL
#define IOM1_REG_SIZE                   0x38cUL
#define IOM2_REG_SIZE                   0x38cUL
#define IOM3_REG_SIZE                   0x38cUL
#define IOM4_REG_SIZE                   0x38cUL
#define IOM5_REG_SIZE                   0x38cUL
#define IOM6_REG_SIZE                   0x38cUL
#define IOM7_REG_SIZE                   0x38cUL
#define IOSLAVEFD0_REG_SIZE             0x220UL
#define IOSLAVEFD1_REG_SIZE             0x220UL
#define IOSLAVE_REG_SIZE                0x220UL
#define MCUCTRL_REG_SIZE                0x4ecUL
#define MSPI0_REG_SIZE                  0x2c8UL
#define MSPI1_REG_SIZE                  0x2c8UL
#define MSPI2_REG_SIZE                  0x2c8UL
#define MSPI3_REG_SIZE                  0x2c8UL
#define OTP_REG_SIZE                    0x2c8UL
#define PDM0_REG_SIZE                   0x254UL
#define PDM1_REG_SIZE                   0x254UL
#define PDM2_REG_SIZE                   0x254UL
#define PDM3_REG_SIZE                   0x254UL
#define PWRCTRL_REG_SIZE                0x250UL
#define RSTGEN_REG_SIZE                 0x8860UL
#define RTC_REG_SIZE                    0x210UL
#define SDIO0_REG_SIZE                  0x104UL
#define SDIO1_REG_SIZE                  0x104UL
#define SECURITY_REG_SIZE               0x90UL
#define SSC_REG_SIZE                    0x8UL
#define STIMER_REG_SIZE                 0x110UL
#define TIMER_REG_SIZE                  0x3fcUL
#define UART0_REG_SIZE                  0x54UL
#define UART1_REG_SIZE                  0x54UL
#define UART2_REG_SIZE                  0x54UL
#define UART3_REG_SIZE                  0x54UL
#define USBPHY_REG_SIZE                 0x88UL
#define USB_REG_SIZE                    0x2428UL
#define VCOMP_REG_SIZE                  0x210UL
#define WDT_REG_SIZE                    0x210UL

#define ITCM_BASE_NAME                  0
#define ITCM_BASE_ADDR                  0x00000000UL
#define ITCM_MAX_SIZE                   0x40000UL

#define DTCM_BASE_NAME                  20000000
#define DTCM_BASE_ADDR                  0x20000000UL
#define DTCM_MAX_SIZE                   0x7C000UL

#define SSRAM_BASE_NAME                 20080000
#define SSRAM_BASE_ADDR                 0x20080000UL
#define SSRAM_MAX_SIZE                  0x300000UL

#define MRAM_BASE_NAME                  410000
#define MRAM_BASE_ADDR                  0x410000UL
#define MRAM_MAX_SIZE                   0x3F0000UL

#define MSPI0_APERTURE_BASE_NAME        60000000
#define MSPI0_APERTURE_BASE_ADDR        0x60000000UL
#define MSPI0_APERTURE_MAX_SIZE         0x10000000UL

#define MSPI1_APERTURE_BASE_NAME        80000000
#define MSPI1_APERTURE_BASE_ADDR        0x80000000UL
#define MSPI1_APERTURE_MAX_SIZE         0x4000000UL

#define MSPI2_APERTURE_BASE_NAME        84000000
#define MSPI2_APERTURE_BASE_ADDR        0x84000000UL
#define MSPI2_APERTURE_MAX_SIZE         0x4000000UL

#define MSPI3_APERTURE_BASE_NAME        88000000
#define MSPI3_APERTURE_BASE_ADDR        0x88000000UL
#define MSPI3_APERTURE_MAX_SIZE         0x8000000UL

#endif // AM_APOLLO510_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
