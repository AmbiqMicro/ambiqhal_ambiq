//*****************************************************************************
//
//! @file am_util_regdump_apollo510L.h
//!
//! @brief Functions to aid register dumping and debugging (AP510L specific)
//!
//! @addtogroup regdump_ap510l_utils Register Dump Functionality - Apollo510L
//! @ingroup utils
//! @{
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2026, Ambiq Micro, Inc.
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
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
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
// This is part of revision v5.2.0-zephyr-685438d73f of the AmbiqSuite Development Package.
//
//*****************************************************************************

#if !defined(AM_UTIL_REGDUMP_APOLLO510L_H)
#define AM_UTIL_REGDUMP_APOLLO510L_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "am_util_stdio.h"
#include "am_bsp.h"

#if defined(__cplusplus)
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Configuration option for full register dump functionality
//!
//! Define AM_UTIL_REGDUMP_ALL to include all register blocks in the
//! am_util_get_all_registers() dump. Otherwise, define one or more
//! block macros to include only those register ranges (saves code and
//! data when only specific blocks are needed).
//!
//! Per-block macros (match register map block names):
//!     AM_UTIL_REGDUMP_ADC, AM_UTIL_REGDUMP_CLKGEN, AM_UTIL_REGDUMP_CM55_IPC,
//!     AM_UTIL_REGDUMP_CRM, AM_UTIL_REGDUMP_CRYPTO, AM_UTIL_REGDUMP_DC,
//!     AM_UTIL_REGDUMP_DSI, AM_UTIL_REGDUMP_FPIO, AM_UTIL_REGDUMP_GPIO,
//!     AM_UTIL_REGDUMP_GPU, AM_UTIL_REGDUMP_I2S, AM_UTIL_REGDUMP_I3C,
//!     AM_UTIL_REGDUMP_IOM, AM_UTIL_REGDUMP_IOSLAVEFD, AM_UTIL_REGDUMP_ITM,
//!     AM_UTIL_REGDUMP_MCUCTRL, AM_UTIL_REGDUMP_MRAMINFO0, AM_UTIL_REGDUMP_MRAMINFO1,
//!     AM_UTIL_REGDUMP_MSPI, AM_UTIL_REGDUMP_NVIC, AM_UTIL_REGDUMP_OTP,
//!     AM_UTIL_REGDUMP_OTPINFO0, AM_UTIL_REGDUMP_OTPINFO1, AM_UTIL_REGDUMP_OTPINFOC,
//!     AM_UTIL_REGDUMP_PDM, AM_UTIL_REGDUMP_PWRCTRL, AM_UTIL_REGDUMP_RSTGEN,
//!     AM_UTIL_REGDUMP_RTC, AM_UTIL_REGDUMP_SDIO, AM_UTIL_REGDUMP_SECURITY,
//!     AM_UTIL_REGDUMP_SSC, AM_UTIL_REGDUMP_STIMER, AM_UTIL_REGDUMP_SYSCTRL,
//!     AM_UTIL_REGDUMP_SYSTICK, AM_UTIL_REGDUMP_TIMER, AM_UTIL_REGDUMP_TPIU,
//!     AM_UTIL_REGDUMP_UART, AM_UTIL_REGDUMP_USB, AM_UTIL_REGDUMP_USBPHY,
//!     AM_UTIL_REGDUMP_VCOMP, AM_UTIL_REGDUMP_WDT
//!
//! Example - dump only RSTGEN and CLKGEN:
//!     #define AM_UTIL_REGDUMP_RSTGEN
//!     #define AM_UTIL_REGDUMP_CLKGEN
//!
//! Example - dump all blocks:
//!     #define AM_UTIL_REGDUMP_ALL
//!
//*****************************************************************************
#if defined(AM_UTIL_REGDUMP_ALL)
#define AM_UTIL_REGDUMP_ANY
#define AM_UTIL_REGDUMP_ADC
#define AM_UTIL_REGDUMP_CLKGEN
#define AM_UTIL_REGDUMP_CM55_IPC
#define AM_UTIL_REGDUMP_CRM
#define AM_UTIL_REGDUMP_CRYPTO
#define AM_UTIL_REGDUMP_DC
#define AM_UTIL_REGDUMP_DSI
#define AM_UTIL_REGDUMP_FPIO
#define AM_UTIL_REGDUMP_GPIO
#define AM_UTIL_REGDUMP_GPU
#define AM_UTIL_REGDUMP_I2S
#define AM_UTIL_REGDUMP_I3C
#define AM_UTIL_REGDUMP_IOM
#define AM_UTIL_REGDUMP_IOSLAVEFD
#define AM_UTIL_REGDUMP_ITM
#define AM_UTIL_REGDUMP_MCUCTRL
#define AM_UTIL_REGDUMP_MRAMINFO0
#define AM_UTIL_REGDUMP_MRAMINFO1
#define AM_UTIL_REGDUMP_MSPI
#define AM_UTIL_REGDUMP_NVIC
#define AM_UTIL_REGDUMP_OTP
#define AM_UTIL_REGDUMP_OTPINFO0
#define AM_UTIL_REGDUMP_OTPINFO1
#define AM_UTIL_REGDUMP_OTPINFOC
#define AM_UTIL_REGDUMP_PDM
#define AM_UTIL_REGDUMP_PWRCTRL
#define AM_UTIL_REGDUMP_RSTGEN
#define AM_UTIL_REGDUMP_RTC
#define AM_UTIL_REGDUMP_SDIO
#define AM_UTIL_REGDUMP_SECURITY
#define AM_UTIL_REGDUMP_SSC
#define AM_UTIL_REGDUMP_STIMER
#define AM_UTIL_REGDUMP_SYSCTRL
#define AM_UTIL_REGDUMP_SYSTICK
#define AM_UTIL_REGDUMP_TIMER
#define AM_UTIL_REGDUMP_TPIU
#define AM_UTIL_REGDUMP_UART
#define AM_UTIL_REGDUMP_USB
#define AM_UTIL_REGDUMP_USBPHY
#define AM_UTIL_REGDUMP_VCOMP
#define AM_UTIL_REGDUMP_WDT
#else
#if defined(AM_UTIL_REGDUMP_ADC) \
    || defined(AM_UTIL_REGDUMP_CLKGEN) \
    || defined(AM_UTIL_REGDUMP_CM55_IPC) \
    || defined(AM_UTIL_REGDUMP_CRM) \
    || defined(AM_UTIL_REGDUMP_CRYPTO) \
    || defined(AM_UTIL_REGDUMP_DC) \
    || defined(AM_UTIL_REGDUMP_DSI) \
    || defined(AM_UTIL_REGDUMP_FPIO) \
    || defined(AM_UTIL_REGDUMP_GPIO) \
    || defined(AM_UTIL_REGDUMP_GPU) \
    || defined(AM_UTIL_REGDUMP_I2S) \
    || defined(AM_UTIL_REGDUMP_I3C) \
    || defined(AM_UTIL_REGDUMP_IOM) \
    || defined(AM_UTIL_REGDUMP_IOSLAVEFD) \
    || defined(AM_UTIL_REGDUMP_ITM) \
    || defined(AM_UTIL_REGDUMP_MCUCTRL) \
    || defined(AM_UTIL_REGDUMP_MRAMINFO0) \
    || defined(AM_UTIL_REGDUMP_MRAMINFO1) \
    || defined(AM_UTIL_REGDUMP_MSPI) \
    || defined(AM_UTIL_REGDUMP_NVIC) \
    || defined(AM_UTIL_REGDUMP_OTP) \
    || defined(AM_UTIL_REGDUMP_OTPINFO0) \
    || defined(AM_UTIL_REGDUMP_OTPINFO1) \
    || defined(AM_UTIL_REGDUMP_OTPINFOC) \
    || defined(AM_UTIL_REGDUMP_PDM) \
    || defined(AM_UTIL_REGDUMP_PWRCTRL) \
    || defined(AM_UTIL_REGDUMP_RSTGEN) \
    || defined(AM_UTIL_REGDUMP_RTC) \
    || defined(AM_UTIL_REGDUMP_SDIO) \
    || defined(AM_UTIL_REGDUMP_SECURITY) \
    || defined(AM_UTIL_REGDUMP_SSC) \
    || defined(AM_UTIL_REGDUMP_STIMER) \
    || defined(AM_UTIL_REGDUMP_SYSCTRL) \
    || defined(AM_UTIL_REGDUMP_SYSTICK) \
    || defined(AM_UTIL_REGDUMP_TIMER) \
    || defined(AM_UTIL_REGDUMP_TPIU) \
    || defined(AM_UTIL_REGDUMP_UART) \
    || defined(AM_UTIL_REGDUMP_USB) \
    || defined(AM_UTIL_REGDUMP_USBPHY) \
    || defined(AM_UTIL_REGDUMP_VCOMP) \
    || defined(AM_UTIL_REGDUMP_WDT)
#define AM_UTIL_REGDUMP_ANY
#endif
#endif

#if defined(AM_UTIL_REGDUMP_ANY)

//*****************************************************************************
//
//! @brief Register dump data structures and definitions (when AM_UTIL_REGDUMP_ANY)
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Power status bit definitions for DEVPWRSTATUS register
//!
//! These defines map to the power status bits in the DEVPWRSTATUS register
//! (0x4000C008) to determine which peripherals are powered and accessible.
//
//*****************************************************************************
#define AM_UTIL_REGDUMP_PWRSTIOM0            (1 << 1)    //!< Power status IO Master 0
#define AM_UTIL_REGDUMP_PWRSTIOM1            (1 << 2)    //!< Power status IO Master 1
#define AM_UTIL_REGDUMP_PWRSTIOM2            (1 << 3)    //!< Power status IO Master 2
#define AM_UTIL_REGDUMP_PWRSTIOM3            (1 << 4)    //!< Power status IO Master 3
#define AM_UTIL_REGDUMP_PWRSTIOM4            (1 << 5)    //!< Power status IO Master 4
#define AM_UTIL_REGDUMP_PWRSTIOM5            (1 << 6)    //!< Power status IO Master 5
#define AM_UTIL_REGDUMP_PWRSTI3CPHY          (1 << 7)    //!< Power status I3C PHY
#define AM_UTIL_REGDUMP_PWRSTUART0           (1 << 9)    //!< Power status UART Controller 0
#define AM_UTIL_REGDUMP_PWRSTUART1           (1 << 10)   //!< Power status UART Controller 1
#define AM_UTIL_REGDUMP_PWRSTADC             (1 << 13)   //!< Power status ADC Digital Controller
#define AM_UTIL_REGDUMP_PWRSTMSPI0           (1 << 14)   //!< Power status MSPI Controller 0
#define AM_UTIL_REGDUMP_PWRSTMSPI1           (1 << 15)   //!< Power status MSPI Controller 1
#define AM_UTIL_REGDUMP_PWRSTMSPI2           (1 << 16)   //!< Power status MSPI Controller 2
#define AM_UTIL_REGDUMP_PWRSTPLL             (1 << 17)   //!< Power status PLL
#define AM_UTIL_REGDUMP_PWRSTGFX             (1 << 18)   //!< Power status GFX controller
#define AM_UTIL_REGDUMP_PWRSTDISP            (1 << 19)   //!< Power status DISP controller
#define AM_UTIL_REGDUMP_PWRSTDISPPHY         (1 << 20)   //!< Power status DISP PHY
#define AM_UTIL_REGDUMP_PWRSTCRYPTO          (1 << 21)   //!< Power status CRYPTO module
#define AM_UTIL_REGDUMP_PWRSTSDIO0           (1 << 22)   //!< Power status SDIO0 controller
#define AM_UTIL_REGDUMP_PWRSTSDIO1           (1 << 23)   //!< Power status SDIO1 controller
#define AM_UTIL_REGDUMP_PWRSTUSB             (1 << 24)   //!< Power status USB controller
#define AM_UTIL_REGDUMP_PWRSTUSBPHY          (1 << 25)   //!< Power status USB PHY
#define AM_UTIL_REGDUMP_PWRSTDBG             (1 << 26)   //!< Power status DBG subsystem
#define AM_UTIL_REGDUMP_PWRSTOTP             (1 << 27)   //!< Power status OTP
#define AM_UTIL_REGDUMP_PWRSTIOSFD0          (1 << 28)   //!< Power status IO FD Slave 0
#define AM_UTIL_REGDUMP_PWRSTIOSFD1          (1 << 29)   //!< Power status IO FD Slave 1
#define AM_UTIL_REGDUMP_PWRSTI3C             (1 << 30)   //!< Power status I3C
#define AM_UTIL_REGDUMP_PWRSTNETAOL          (1 << 31)   //!< Power status NETAOL

//*****************************************************************************
//
//! @brief Audio subsystem power status bit definitions for AUDSSPWRSTATUS register
//!
//! These defines map to the power status bits in the AUDSSPWRSTATUS register
//! (0x4000C010) to determine which audio peripherals are powered and accessible.
//
//*****************************************************************************
#define AM_UTIL_REGDUMP_PWRSTPDM0            (1 << 2)    //!< Power status audio subsystem PDM0 domain
#define AM_UTIL_REGDUMP_PWRSTI2S0            (1 << 6)    //!< Power status audio subsystem I2S0 domain

//*****************************************************************************
//
//! @brief CM4 power state bit definitions for CM4PWRSTATE register
//!
//! These defines map to the power state bits in the CM4PWRSTATE register
//! (0x4000C09C) to determine the CM4 processor power state.
//
//*****************************************************************************
#define AM_UTIL_REGDUMP_CM4PWRSTATUS_MASK    (0x3)       //!< CM4 power status mask (2 bits)
#define AM_UTIL_REGDUMP_CM4PWRSTATUS_OFF     (0x0)       //!< CM4 is OFF
#define AM_UTIL_REGDUMP_CM4PWRSTATUS_ON      (0x1)       //!< CM4 is ON
#define AM_UTIL_REGDUMP_CM4PWRSTATUS_RET     (0x2)       //!< CM4 is in retention

//*****************************************************************************
//
//! @brief Structure to hold register group information
//!
//! This structure contains the starting address and count for each contiguous
//! register group found in the AP510L register map.
//
//*****************************************************************************
typedef struct
{
    uint32_t start_address;    //!< Starting address of the register group
    uint32_t count;            //!< Number of registers in the group
}
am_util_register_group_t;

//*****************************************************************************
//
//! @brief Array of all contiguous register groups
//!
//! This array contains all the register group definitions for efficient
//! bulk reading of AP510L registers.
//
//*****************************************************************************
extern const am_util_register_group_t am_util_register_groups[];

//*****************************************************************************
//
//! @brief Total number of register groups
//!
//! This is calculated dynamically in the source file using sizeof().
//! The actual count is determined at compile time based on the array size.
//
//*****************************************************************************
extern const uint32_t am_util_register_group_count;

//*****************************************************************************
//
//! @brief Dump all available registers via UART
//!
//! @param void
//!
//! This function iterates through all register groups and prints each
//! register address and value using am_util_stdio_printf(). It processes
//! registers in batches to avoid buffer overflow.
//!
//! @note - before use this function, make sure the UART or SWO is enabled
//!
//! @returns None
//
//*****************************************************************************
extern void am_util_get_all_registers(void);

#endif // AM_UTIL_REGDUMP_ANY

//*****************************************************************************
//
//! @brief Dump registers within a specified address range via UART
//!
//! @param ui32StartAddr - Starting address of the register range (must be 4-byte aligned)
//! @param ui32EndAddr   - Ending address of the register range (inclusive, must be 4-byte aligned)
//!
//! This function reads and prints all contiguous 32-bit registers from the
//! start address to the end address (inclusive). It uses AM_REGVAL() macro
//! for direct register access.
//!
//! @note - Before using this function, make sure the UART or SWO is enabled
//! @note - Both addresses must be 4-byte aligned
//! @note - End address must be >= start address
//! @note - Caller is responsible for ensuring the address range is valid
//!         and the peripherals are powered before calling this function
//!
//! @returns None
//
//*****************************************************************************
extern void am_util_get_registers(uint32_t ui32StartAddr, uint32_t ui32EndAddr);

//*****************************************************************************
//
//! @brief Dump a specified number of registers starting from an address via UART
//!
//! @param ui32StartAddr - Starting address of the register range (must be 4-byte aligned)
//! @param ui32Count     - Number of 32-bit registers to read
//!
//! This function reads and prints ui32Count contiguous 32-bit registers starting
//! from the start address. It uses AM_REGVAL() macro for direct register access.
//!
//! @note - Before using this function, make sure the UART or SWO is enabled
//! @note - Start address must be 4-byte aligned
//! @note - Caller is responsible for ensuring the address range is valid
//!         and the peripherals are powered before calling this function
//!
//! @returns None
//
//*****************************************************************************
extern void am_util_get_n_registers(uint32_t ui32StartAddr, uint32_t ui32Count);

#if defined(__cplusplus)
}
#endif

#endif // AM_UTIL_REGDUMP_APOLLO510L_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
