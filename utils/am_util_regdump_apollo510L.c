//*****************************************************************************
//
//! @file am_util_regdump_apollo510L.c
//!
//! @brief Functions to aid register dumping and debugging (AP510L specific)
//!
//! @addtogroup regdump_ap510l_utils Register Dump Functionality - Apollo510L
//! @ingroup utils
//! @{
//!
//! Purpose: This module provides register dump utilities for Ambiq Micro
//!          AP510L devices. It enables comprehensive register reading and
//!          debugging for embedded applications requiring detailed system
//!          state analysis. The utilities support efficient bulk reading
//!          of all contiguous register groups found in the AP510L
//!          register map.
//!
//! @section utils_regdump_features Key Features
//!
//! 1. @b Comprehensive @b Coverage: All register groups.
//! 2. @b Efficient @b Reading: Bulk register group processing.
//! 3. @b UART @b Output: Formatted register dump via serial.
//! 4. @b Debug @b Support: Complete system state capture.
//! 5. @b Memory @b Efficient: Optimized data structures.
//!
//! @section utils_regdump_functionality Functionality
//!
//! - Dump all available registers
//! - Format output for debugging
//! - Support UART/SWO output
//! - Handle register group processing
//! - Provide comprehensive system state
//!
//! @section utils_regdump_usage Usage
//!
//! 1. Initialize UART/SWO output
//! 2. Call am_util_get_all_registers()
//! 3. Analyze register values
//! 4. Use for debugging purposes
//!
//! @section utils_regdump_configuration Configuration
//!
//! - Enable UART or SWO output
//! - Configure register groups
//! - Set debug output format
//! - Define memory constraints
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

#include "am_util_regdump_apollo510L.h"

#if defined(AM_UTIL_REGDUMP_ANY)

//*****************************************************************************
//
//! @brief Register addresses and power checking (when any register dump block is enabled)
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Power status register addresses
//!
//! Addresses of the power status registers used to determine
//! which peripherals are powered and accessible for register reading.
//
//*****************************************************************************
#define AM_UTIL_REGDUMP_DEVPWRSTATUS_ADDR     0x4000C008
#define AM_UTIL_REGDUMP_AUDSSPWRSTATUS_ADDR   0x4000C010
#define AM_UTIL_REGDUMP_CM4PWRSTATE_ADDR      0x4000C09C

//*****************************************************************************
//
//! @brief Check if a register group should be read based on power status
//!
//! @param group_start_address - Starting address of the register group
//! @param devpwrstatus - Device power status register value
//! @param audsspwrstatus - Audio subsystem power status register value
//! @param cm4pwrstate - CM4 power state register value
//!
//! @returns true if the register group is accessible and should be read
//
//*****************************************************************************
static bool
is_accessible(uint32_t group_start_address, uint32_t devpwrstatus, uint32_t audsspwrstatus, uint32_t cm4pwrstate)
{
    //! Map register group addresses to power status bits using defined constants (Apollo 510L register map).
    //!
    //! System and always-on registers (0x40000000-0x4002FFFF) - always accessible.
    if (group_start_address >= 0x40000000 && group_start_address < 0x40030000)
    {
        return true; //!< Always accessible
    }
    //! IOS0 registers (0x40030000-0x4003FFFF) - always accessible (no power status bit defined).
    else if (group_start_address >= 0x40030000 && group_start_address < 0x40040000)
    {
        return true; //!< Always accessible
    }
    //! I3C registers (0x40059000-0x40059FFF) - check PWRSTI3C bit (must be before IOM check).
    else if (group_start_address >= 0x40059000 && group_start_address < 0x4005A000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTI3C) != 0;
    }

    //! IOM registers (0x40050000-0x4005FFFF) - check IOM power bits.
    else if (group_start_address >= 0x40050000 && group_start_address < 0x40060000)
    {
        if (group_start_address >= 0x40050000 && group_start_address < 0x40051000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM0) != 0;  //!< IOM0 - PWRSTIOM0
        }
        else if (group_start_address >= 0x40051000 && group_start_address < 0x40052000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM1) != 0;  //!< IOM1 - PWRSTIOM1
        }
        else if (group_start_address >= 0x40052000 && group_start_address < 0x40053000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM2) != 0;  //!< IOM2 - PWRSTIOM2
        }
        else if (group_start_address >= 0x40053000 && group_start_address < 0x40054000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM3) != 0;  //!< IOM3 - PWRSTIOM3
        }
        else if (group_start_address >= 0x40054000 && group_start_address < 0x40055000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM4) != 0;  //!< IOM4 - PWRSTIOM4
        }
        else if (group_start_address >= 0x40055000 && group_start_address < 0x40056000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOM5) != 0;  //!< IOM5 - PWRSTIOM5
        }
        //! IOM6-15 registers - always accessible (IOM9 is I3C, handled above).
        else if (group_start_address >= 0x40056000 && group_start_address < 0x40060000)
        {
            return true; //!< Always accessible
        }
    }
    //! MSPI registers (0x40060000-0x40063FFF) - check MSPI power bits.
    else if (group_start_address >= 0x40060000 && group_start_address < 0x40064000)
    {
        if (group_start_address >= 0x40060000 && group_start_address < 0x40061000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTMSPI0) != 0;  //!< MSPI0
        }
        else if (group_start_address >= 0x40061000 && group_start_address < 0x40062000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTMSPI1) != 0;  //!< MSPI1
        }
        else if (group_start_address >= 0x40062000 && group_start_address < 0x40063000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTMSPI2) != 0;  //!< MSPI2
        }
        else if (group_start_address >= 0x40063000 && group_start_address < 0x40064000)
        {
            return true; //!< MSPI3 - always accessible
        }
    }
    //! SDIO registers (0x40070000-0x40071FFF) - check SDIO power bits.
    else if (group_start_address >= 0x40070000 && group_start_address < 0x40072000)
    {
        if (group_start_address >= 0x40070000 && group_start_address < 0x40071000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTSDIO0) != 0;  //!< SDIO0
        }
        else if (group_start_address >= 0x40071000 && group_start_address < 0x40072000)
        {
            return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTSDIO1) != 0;  //!< SDIO1
        }
    }
    //! GPU registers (0x40090000-0x40090FFF) - check PWRSTGFX bit.
    else if (group_start_address >= 0x40090000 && group_start_address < 0x40091000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTGFX) != 0;
    }

    //! DC registers (0x400A0000-0x400A1FFF) - check PWRSTDISP bit.
    else if (group_start_address >= 0x400A0000 && group_start_address < 0x400A2000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDISP) != 0;
    }
    //! DSI registers (0x400A8000-0x400A8FFF) - check PWRSTDISPPHY bit.
    else if (group_start_address >= 0x400A8000 && group_start_address < 0x400A9000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDISPPHY) != 0;
    }
    //! USB registers (0x400B0000-0x400B2FFF) - check PWRSTUSB bit.
    else if (group_start_address >= 0x400B0000 && group_start_address < 0x400B3000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTUSB) != 0;
    }
    //! USBPHY registers (0x400B4000-0x400B4FFF) - check PWRSTUSBPHY bit.
    else if (group_start_address >= 0x400B4000 && group_start_address < 0x400B5000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTUSBPHY) != 0;
    }
    //! CRYPTO registers (0x400C0000-0x400C1FFF) - check PWRSTCRYPTO bit.
    else if (group_start_address >= 0x400C0000 && group_start_address < 0x400C2000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTCRYPTO) != 0;
    }
    //! OTPINFOC registers (0x400C2000-0x400C23FF) - check PWRSTOTP bit.
    else if (group_start_address >= 0x400C2000 && group_start_address < 0x400C2400)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTOTP) != 0;
    }
    //! PDM registers (0x40201000-0x40202FFF) - check PWRSTPDM0 bit.
    else if (group_start_address >= 0x40201000 && group_start_address < 0x40203000)
    {
        return (audsspwrstatus & AM_UTIL_REGDUMP_PWRSTPDM0) != 0;
    }
    //! I2S registers (0x40208000-0x40209FFF) - check PWRSTI2S0 bit.
    else if (group_start_address >= 0x40208000 && group_start_address < 0x4020A000)
    {
        return (audsspwrstatus & AM_UTIL_REGDUMP_PWRSTI2S0) != 0;
    }
    //! CM55_IPC registers (0x40034000-0x40034FFF) - CM4 must be ON (not OFF or retention).
    else if (group_start_address >= 0x40034000 && group_start_address < 0x40035000)
    {
        uint32_t cm4_status = (cm4pwrstate & AM_UTIL_REGDUMP_CM4PWRSTATUS_MASK);
        return (cm4_status == AM_UTIL_REGDUMP_CM4PWRSTATUS_ON);
    }
    //! IOSLAVEFD registers (0x40035100-0x400352FF) - check PWRSTIOSFD0 bit (bit 28).
    else if (group_start_address >= 0x40035100 && group_start_address < 0x40035300)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOSFD0) != 0;
    }
    //! Additional IOSLAVEFD (0x40036100-0x400362FF) - check PWRSTIOSFD1 bit (bit 29).
    else if (group_start_address >= 0x40036100 && group_start_address < 0x40036300)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOSFD1) != 0;
    }
    //! UART registers (0x40039000-0x40039FFF) - check PWRSTUART0 bit.
    else if (group_start_address >= 0x40039000 && group_start_address < 0x4003A000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTUART0) != 0;
    }
    //! Additional UART registers (0x4003A000-0x4003AFFF) - check PWRSTUART1 bit.
    else if (group_start_address >= 0x4003A000 && group_start_address < 0x4003B000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTUART1) != 0;
    }
    //! ADC registers (0x40038000-0x40038FFF) - check PWRSTADC bit.
    else if (group_start_address >= 0x40038000 && group_start_address < 0x40039000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTADC) != 0;
    }
    //! OTP registers (0x40009AA0-0x40009ACF) - check PWRSTOTP bit.
    else if (group_start_address >= 0x40009AA0 && group_start_address < 0x40009AD0)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTOTP) != 0;
    }
    //! MRAMINFO0/1 registers (0x42000000-0x420034FF) - always accessible.
    else if (group_start_address >= 0x42000000 && group_start_address < 0x42003500)
    {
        return true; //!< Always accessible
    }
    //! OTPINFO0/1 registers (0x42004000-0x42006AFF) - check PWRSTOTP bit.
    else if (group_start_address >= 0x42004000 && group_start_address < 0x42006B00)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTOTP) != 0;
    }
    //! ITM registers (0xE0000000-0xE0000FFF) - check PWRSTDBG bit.
    else if (group_start_address >= 0xE0000000 && group_start_address < 0xE0001000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDBG) != 0;
    }
    //! TPIU registers (0xE0040000-0xE0040FFF) - check PWRSTDBG bit.
    else if (group_start_address >= 0xE0040000 && group_start_address < 0xE0041000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDBG) != 0;
    }
    //! Registers not covered by power status (e.g. system) - always readable.
    return true;
}

//*****************************************************************************
//
//! @brief Register group definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Array of all contiguous register groups
//!
//! This array contains all the register group definitions for efficient
//! bulk reading of AP510L registers. Each group contains a starting
//! address and count of contiguous registers.
//
//*****************************************************************************
const am_util_register_group_t am_util_register_groups[] = {
#if defined(AM_UTIL_REGDUMP_RSTGEN)
    //! Group 1: RSTGEN registers
    {0x40000000, 3},
    {0x40000014, 2},
    {0x40000200, 4},
    {0x4000885C, 1},
#endif

#if defined(AM_UTIL_REGDUMP_CLKGEN)
    //! Group 2: CLKGEN registers
    {0x4000400C, 2},
    {0x40004020, 1},
    {0x40004030, 3},
    {0x40004044, 1},
    {0x40004078, 1},
    {0x40004088, 2},
    {0x40004110, 1},
    {0x40004120, 1},
#endif

#if defined(AM_UTIL_REGDUMP_SSC)
    //! Group 3: SSC registers
    {0x40005000, 6},
#endif

#if defined(AM_UTIL_REGDUMP_CRM)
    //! Group 4: CRM registers
    {0x40006010, 16},
    {0x400061FC, 1},
#endif

#if defined(AM_UTIL_REGDUMP_OTP)
    //! Group 5: OTP registers
    //! OTP entries commented out - uncomment if needed.
    //! {0x40009AA0, 1},
    //! {0x40009AB0, 1},
    //! {0x40009AC4, 1},
#endif

#if defined(AM_UTIL_REGDUMP_MCUCTRL)
    //! Group 6: MCUCTRL registers
    {0x4000A800, 7},
    {0x4000A820, 1},
    {0x4000A828, 1},
    {0x4000A844, 5},
    {0x4000A860, 1},
    {0x4000A880, 1},
    {0x4000A888, 1},
    {0x4000A8C0, 1},
    {0x4000A8E0, 1},
    {0x4000A900, 1},
    {0x4000A908, 3},
    {0x4000A920, 2},
    {0x4000A92C, 1},
    {0x4000A980, 1},
    {0x4000A9AC, 2},
    {0x4000A9B8, 4},
    {0x4000AA00, 2},
    {0x4000AA1C, 1},
    {0x4000AA50, 1},
    {0x4000AA64, 1},
    {0x4000AA80, 2},
    {0x4000AB38, 6},
    {0x4000AB54, 7},
    {0x4000AB74, 4},
    {0x4000AB88, 1},
    {0x4000ABA8, 12},
    {0x4000ABF0, 4},
    {0x4000AC54, 3},
    {0x4000ACA0, 2},
    {0x4000ACB0, 1},
    {0x4000ACCC, 1},
    {0x4000ACD8, 5},
    {0x4000ACF0, 3},
#endif

#if defined(AM_UTIL_REGDUMP_PWRCTRL)
    //! Group 7: PWRCTRL registers
    {0x4000C000, 14},
    {0x4000C040, 1},
    {0x4000C050, 1},
    {0x4000C058, 1},
    {0x4000C084, 2},
    {0x4000C094, 3},
    {0x4000C100, 3},
    {0x4000C190, 2},
    {0x4000C1C8, 1},
    {0x4000C1D0, 1},
    {0x4000C200, 9},
    {0x4000C228, 8},
    {0x4000C24C, 1},
#endif

#if defined(AM_UTIL_REGDUMP_SECURITY)
    //! Group 8: SECURITY registers
    {0x4000E800, 1},
    {0x4000E810, 1},
    {0x4000E820, 1},
    {0x4000E830, 1},
    {0x4000E878, 6},
#endif

#if defined(AM_UTIL_REGDUMP_RTC)
    //! Group 9: RTC registers
    {0x40010000, 2},
    {0x40010020, 2},
    {0x40010030, 2},
    {0x40010200, 4},
#endif

#if defined(AM_UTIL_REGDUMP_VCOMP)
    //! Group 10: VCOMP registers
    {0x40011C00, 3},
    {0x40011E00, 4},
#endif

#if defined(AM_UTIL_REGDUMP_GPIO)
    //! Group 11: GPIO registers
    {0x40012800, 120},
    {0x40012C00, 46},
    {0x40012CC0, 32},
#endif

#if defined(AM_UTIL_REGDUMP_FPIO)
    //! Group 12: FPIO registers
    {0x40013800, 28},
#endif

#if defined(AM_UTIL_REGDUMP_WDT)
    //! Group 13: WDT registers
    {0x40027000, 4},
    {0x40027200, 4},
#endif

#if defined(AM_UTIL_REGDUMP_TIMER)
    //! Group 14: TIMER registers
    {0x40030000, 2},
    {0x40030010, 1},
    {0x40030060, 4},
    {0x40030080, 30},
    {0x40030200, 6},
    {0x40030220, 6},
    {0x40030240, 6},
    {0x40030260, 6},
    {0x40030280, 6},
    {0x400302A0, 6},
    {0x400302C0, 6},
    {0x400302E0, 6},
    {0x40030300, 6},
    {0x40030320, 6},
    {0x40030340, 6},
    {0x40030360, 6},
    {0x40030380, 6},
    {0x400303A0, 6},
    {0x400303C0, 6},
    {0x400303E0, 7},
#endif

#if defined(AM_UTIL_REGDUMP_STIMER)
    //! Group 15: STIMER registers
    {0x40030800, 2},
    {0x40030810, 19},
    {0x40030900, 4},
#endif

#if defined(AM_UTIL_REGDUMP_CM55_IPC)
    //! Group 16: CM55_IPC registers
    //! {0x40034000, 3},
    //! {0x40034010, 7},
#endif

#if defined(AM_UTIL_REGDUMP_IOSLAVEFD)
    //! Group 17: IOSLAVEFD registers
    {0x40035100, 11},
    {0x40035130, 4},
    {0x40035200, 8},
    {0x40036100, 11},
    {0x40036130, 4},
    {0x40036200, 8},
#endif

#if defined(AM_UTIL_REGDUMP_ADC)
    //! Group 18: ADC registers
    {0x40038000, 17},
    {0x40038060, 4},
    {0x400380A4, 4},
    {0x40038200, 4},
    {0x40038240, 2},
    {0x40038280, 1},
    {0x40038288, 6},
#endif

#if defined(AM_UTIL_REGDUMP_UART)
    //! Group 19: UART registers
    {0x40039000, 2},
    {0x40039018, 1},
    {0x40039020, 13},
    {0x4003A000, 2},
    {0x4003A018, 1},
    {0x4003A020, 13},
#endif

#if defined(AM_UTIL_REGDUMP_IOM)
    //! Group 20: IOM registers
    {0x40050000, 1},
    {0x40050100, 12},
    {0x40050200, 19},
    {0x40050280, 1},
    {0x400502C0, 2},
    {0x40050388, 1},
    {0x40051000, 1},
    {0x40051100, 12},
    {0x40051200, 19},
    {0x40051280, 1},
    {0x400512C0, 2},
    {0x40051388, 1},
#endif

#if defined(AM_UTIL_REGDUMP_I3C)
    //! Group 21: I3C registers
    {0x40059000, 6},
    {0x40059020, 9},
    {0x4005904C, 1},
    {0x40059058, 1},
    {0x40059200, 2},
    {0x40059300, 4},
    {0x40059500, 7},
    {0x40059520, 4},
    {0x40059550, 5},
    {0x40059580, 3},
    {0x40059590, 16},
    {0x400597A0, 4},
    {0x400597B4, 1},
    {0x400597BC, 6},
#endif

#if defined(AM_UTIL_REGDUMP_MSPI)
    //! Group 22: MSPI registers
    {0x40060000, 9},
    {0x40060030, 1},
    {0x40060044, 3},
    {0x40060080, 11},
    {0x40060100, 7},
    {0x40060200, 4},
    {0x400602A0, 1},
    {0x400602A8, 5},
    {0x400602C0, 2},
    {0x40061000, 9},
    {0x40061030, 1},
    {0x40061044, 3},
    {0x40061080, 11},
    {0x40061100, 7},
    {0x40061200, 4},
    {0x400612A0, 1},
    {0x400612A8, 5},
    {0x400612C0, 2},
#endif

#if defined(AM_UTIL_REGDUMP_SDIO)
    //! Group 23: SDIO registers
    {0x40070000, 29},
    {0x40070078, 1},
    {0x400700FC, 2},
    {0x40071000, 29},
    {0x40071078, 1},
    {0x400710FC, 2},
#endif

#if defined(AM_UTIL_REGDUMP_GPU)
    //! Group 24: GPU registers
    {0x40090000, 3},
    {0x40090010, 7},
    {0x40090030, 3},
    {0x40090090, 4},
    {0x400900B0, 1},
    {0x400900C0, 4},
    {0x400900E8, 9},
    {0x40090110, 11},
    {0x40090140, 3},
    {0x40090150, 3},
    {0x40090160, 15},
    {0x400901A0, 12},
    {0x400901EC, 2},
    {0x40090200, 4},
    {0x40090250, 1},
    {0x40090FF0, 1},
#endif

#if defined(AM_UTIL_REGDUMP_DC)
    //! Group 25: DC registers
    {0x400A0000, 18},
    {0x400A00E8, 3},
    {0x400A00F8, 4},
    {0x400A0114, 1},
    {0x400A0130, 5},
    {0x400A0180, 2},
    {0x400A01A0, 4},
    {0x400A1000, 1},
#endif

#if defined(AM_UTIL_REGDUMP_DSI)
    //! Group 26: DSI registers
    {0x400A8000, 9},
    {0x400A8028, 24},
    {0x400A8098, 3},
#endif

#if defined(AM_UTIL_REGDUMP_USB)
    //! Group 27: USB registers
    {0x400B0000, 14},
    {0x400B006C, 1},
    {0x400B0078, 1},
    {0x400B0080, 2},
    {0x400B2000, 2},
    {0x400B2014, 16},
    {0x400B2060, 8},
    {0x400B2100, 10},
    {0x400B2200, 10},
    {0x400B2300, 10},
    {0x400B2400, 10},
#endif

#if defined(AM_UTIL_REGDUMP_USBPHY)
    //! Group 28: USBPHY registers
    {0x400B4000, 34},
#endif

#if defined(AM_UTIL_REGDUMP_CRYPTO)
    //! Group 29: CRYPTO registers
    {0x400C0000, 47},
    {0x400C00C4, 1},
    {0x400C00D0, 6},
    {0x400C00F0, 1},
    {0x400C00F8, 1},
    {0x400C0100, 15},
    {0x400C0140, 1},
    {0x400C01B4, 8},
    {0x400C01D8, 2},
    {0x400C0380, 27},
    {0x400C0400, 29},
    {0x400C0478, 2},
    {0x400C04B4, 1},
    {0x400C04BC, 2},
    {0x400C04C8, 1},
    {0x400C04D8, 1},
    {0x400C04F0, 1},
    {0x400C04F8, 1},
    {0x400C0524, 1},
    {0x400C0640, 9},
    {0x400C0684, 2},
    {0x400C0694, 1},
    {0x400C06A4, 1},
    {0x400C07B0, 1},
    {0x400C07C0, 5},
    {0x400C07DC, 1},
    {0x400C07E4, 2},
    {0x400C0810, 1},
    {0x400C0818, 4},
    {0x400C0858, 1},
    {0x400C0900, 1},
    {0x400C0910, 1},
    {0x400C091C, 1},
    {0x400C0930, 1},
    {0x400C0960, 10},
    {0x400C0A00, 4},
    {0x400C0A24, 2},
    {0x400C0A38, 1},
    {0x400C0A78, 5},
    {0x400C0B00, 4},
    {0x400C0C00, 1},
    {0x400C0C20, 1},
    {0x400C0C28, 6},
    {0x400C0C48, 1},
    {0x400C0C50, 1},
    {0x400C0C58, 1},
    {0x400C0D20, 1},
    {0x400C0D28, 6},
    {0x400C0D44, 1},
    {0x400C0D50, 1},
    {0x400C0F00, 3},
    {0x400C0FD0, 1},
    {0x400C0FE0, 8},
    {0x400C1E00, 17},
    {0x400C1F04, 11},
#endif

#if defined(AM_UTIL_REGDUMP_OTPINFOC)
    //! Group 30: OTPINFOC registers
    {0x400C2044, 17},
    {0x400C2090, 8},
    {0x400C2200, 17},
    {0x400C224C, 93},
    {0x400C23FC, 1},
#endif

#if defined(AM_UTIL_REGDUMP_PDM)
    //! Group 31: PDM registers
    {0x40201000, 8},
    {0x40201100, 4},
    {0x40201140, 3},
    {0x40201150, 3},
    {0x40201160, 3},
    // {0x40202000, 8},
    // {0x40202100, 4},
    // {0x40202140, 3},
    // {0x40202150, 3},
    // {0x40202160, 3},
#endif

#if defined(AM_UTIL_REGDUMP_I2S)
    //! Group 32: I2S registers
    {0x40208000, 5},
    {0x40208020, 5},
    {0x40208040, 6},
    {0x40208060, 2},
    {0x40208100, 1},
    {0x40208200, 13},
    {0x40208300, 4},
    {0x40209000, 5},
    {0x40209020, 5},
    {0x40209040, 6},
    {0x40209060, 2},
    {0x40209100, 1},
    {0x40209200, 13},
    {0x40209300, 4},
#endif

#if defined(AM_UTIL_REGDUMP_MRAMINFO0)
    //! Group 33: MRAMINFO0 registers
    {0x42000000, 6},
    {0x42000028, 9},
    {0x42000054, 2},
    {0x42000060, 17},
#endif

#if defined(AM_UTIL_REGDUMP_MRAMINFO1)
    //! Group 34: MRAMINFO1 registers
    {0x42003200, 2},
    {0x42003210, 1},
    {0x42003240, 5},
    {0x42003258, 3},
    {0x42003300, 9},
    {0x42003328, 2},
    {0x42003330, 1},
    {0x42003370, 4},
    {0x42003400, 16},
    {0x42003450, 11},
#endif

#if defined(AM_UTIL_REGDUMP_OTPINFO0)
    //! Group 35: OTPINFO0 registers
    {0x42004000, 6},
    {0x42004028, 9},
    {0x42004054, 2},
    {0x42004060, 17},
#endif

#if defined(AM_UTIL_REGDUMP_OTPINFO1)
    //! Group 36: OTPINFO1 registers (per apollo510L_register_map.csv)
    {0x42006800, 2},
    {0x42006810, 1},
    {0x42006818, 1},
    {0x42006820, 13},
    {0x42006858, 7},
    {0x42006900, 9},
    {0x42006928, 2},
    {0x42006930, 1},
    {0x42006970, 4},
    {0x42006A50, 11},
#endif

#if defined(AM_UTIL_REGDUMP_ITM)
    //! Group 37: ITM registers
    {0xE0000000, 32},
    {0xE0000E00, 1},
    {0xE0000E40, 1},
    {0xE0000E80, 1},
    {0xE0000FB0, 2},
    {0xE0000FD0, 12},
#endif

#if defined(AM_UTIL_REGDUMP_SYSCTRL)
    //! Group 38: SYSCTRL registers
    {0xE000E004, 2},
    {0xE000ED04, 11},
    {0xE000ED34, 2},
    {0xE000ED88, 1},
    {0xE000EDFC, 1},
    {0xE000EF00, 1},
    {0xE000EF34, 3},
#endif

#if defined(AM_UTIL_REGDUMP_SYSTICK)
    //! Group 39: SYSTICK registers
    {0xE000E010, 4},
#endif

#if defined(AM_UTIL_REGDUMP_NVIC)
    //! Group 40: NVIC registers
    {0xE000E100, 1},
    {0xE000E180, 1},
    {0xE000E200, 1},
    {0xE000E280, 1},
    {0xE000E300, 1},
    {0xE000E400, 8},
#endif

#if defined(AM_UTIL_REGDUMP_TPIU)
    //! Group 41: TPIU registers - COMMENTED OUT for A0/A1 silicon.
    //! @todo Uncomment when B0 silicon with TPIU fixes is available.
    //! {0xE0040000, 2},
    //! {0xE0040010, 1},
    //! {0xE00400F0, 1},
    //! {0xE0040304, 1},
    //! {0xE0040F00, 1},
    //! {0xE0040FC8, 1}
#endif
};

//*****************************************************************************
//
//! @brief Total number of register groups (calculated dynamically)
//!
//! This constant is calculated at compile time using sizeof() to determine
//! the actual number of elements in the am_util_register_groups array.
//
//*****************************************************************************
const uint32_t am_util_register_group_count = sizeof(am_util_register_groups) / sizeof(am_util_register_group_t);

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
void
am_util_get_all_registers(void)
{
    uint32_t total_registers_read = 0;
    uint32_t total_groups_processed = 0;
    const uint32_t BATCH_SIZE = 10;  //!< Process 10 register groups at a time

    //! Enable UART printf for register dump output.
    am_bsp_uart_printf_enable();

    am_util_stdio_printf("\n\n========================================\n");
    am_util_stdio_printf("Starting AP510L register dump...\n");


    //! Read power status registers to determine which peripherals are powered.
    uint32_t devpwrstatus = *((volatile uint32_t*)AM_UTIL_REGDUMP_DEVPWRSTATUS_ADDR);
    uint32_t audsspwrstatus = *((volatile uint32_t*)AM_UTIL_REGDUMP_AUDSSPWRSTATUS_ADDR);
    uint32_t cm4pwrstate = *((volatile uint32_t*)AM_UTIL_REGDUMP_CM4PWRSTATE_ADDR);
    am_util_stdio_printf("DEVPWRSTATUS: 0x%08X\n", devpwrstatus);
    am_util_stdio_printf("AUDSSPWRSTATUS: 0x%08X\n", audsspwrstatus);
    am_util_stdio_printf("CM4PWRSTATE: 0x%08X\n", cm4pwrstate);
    am_util_stdio_printf("Reading all %d register groups...\n", am_util_register_group_count);
    am_util_stdio_printf("\n");

    //! Process register groups in batches to avoid buffer overflow.
    for (uint32_t batch_start = 0; batch_start < am_util_register_group_count; batch_start += BATCH_SIZE)
    {
        uint32_t batch_end = (batch_start + BATCH_SIZE < am_util_register_group_count) ?
                             (batch_start + BATCH_SIZE) : am_util_register_group_count;

        //! Process each group in the current batch.
        for (uint32_t group_idx = batch_start; group_idx < batch_end; group_idx++)
        {

            const am_util_register_group_t* group = &am_util_register_groups[group_idx];

            //! Check if this peripheral is powered before reading registers.
            if (!is_accessible(group->start_address, devpwrstatus, audsspwrstatus, cm4pwrstate))
            {
                am_util_stdio_printf("--- Group %d: 0x%08X (%d registers) --- SKIPPED (not powered)\n",
                                    group_idx, group->start_address, group->count);
                continue;
            }

            //! Show all registers in this group.
            am_util_stdio_printf("--- Group %d: 0x%08X (%d registers) ---\n",
                                group_idx, group->start_address,                 group->count);

            //! Skip register ranges that may hang when read (IOSLAVEFD0/1, ADC).
            if ((group->start_address >= 0x40035100 && group->start_address < 0x40035300) ||
                (group->start_address >= 0x40036100 && group->start_address < 0x40036300) ||
                (group->start_address >= 0x40038000 && group->start_address < 0x40039000))
            {
                if ((group->start_address >= 0x40035100 && group->start_address < 0x40035300) ||
                    (group->start_address >= 0x40036100 && group->start_address < 0x40036300))
                {
                    am_util_stdio_printf("--- IOSLAVEFD registers skipped (may cause system hang) ---\n");
                }
                else if (group->start_address >= 0x40038000 && group->start_address < 0x40039000)
                {
                    am_util_stdio_printf("--- ADC registers skipped (may cause system hang) ---\n");
                }
            }
            else
            {
                //! Read all registers in this group using the utility function.
                am_util_get_n_registers(group->start_address, group->count);
                total_registers_read += group->count;
            }
            am_util_stdio_printf("\n");
            total_groups_processed++;
        }

        //! Flush output between batches to ensure data is sent.
        am_util_stdio_printf("--- Batch %d-%d complete ---\n\n", batch_start, batch_end - 1);
    }

    am_util_stdio_printf("\n");
    am_util_stdio_printf("Register dump complete!\n");
    am_util_stdio_printf("Total groups processed: %u\n", total_groups_processed);
    am_util_stdio_printf("Total registers read: %u\n", total_registers_read);
    am_util_stdio_printf("========================================\n\n\n\n\n");

    //! Disable UART to minimize impact to the system.
    am_bsp_uart_printf_disable();
}

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
void
am_util_get_registers(uint32_t ui32StartAddr, uint32_t ui32EndAddr)
{
    uint32_t ui32Addr;
    uint32_t ui32Value;
    for (ui32Addr = ui32StartAddr; ui32Addr <= ui32EndAddr; ui32Addr += 4)
    {
        ui32Value = AM_REGVAL(ui32Addr);
        am_util_stdio_printf("0x%08X: 0x%08X\n", ui32Addr, ui32Value);
    }
}

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
void
am_util_get_n_registers(uint32_t ui32StartAddr, uint32_t ui32Count)
{
    uint32_t ui32Addr;
    uint32_t ui32Value;
    uint32_t i;

    for (i = 0; i < ui32Count; i++)
    {
        ui32Addr = ui32StartAddr + (i * 4);
        ui32Value = AM_REGVAL(ui32Addr);
        am_util_stdio_printf("0x%08X: 0x%08X\n", ui32Addr, ui32Value);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
