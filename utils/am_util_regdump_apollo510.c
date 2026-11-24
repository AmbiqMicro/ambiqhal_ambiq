//*****************************************************************************
//
//! @file am_util_regdump_apollo510.c
//!
//! @brief Functions to aid register dumping and debugging (AP510 specific)
//!
//! @addtogroup regdump_ap510_utils Register Dump Functionality - Apollo510
//! @ingroup utils
//! @{
//!
//! Purpose: This module provides register dump utilities for Ambiq Micro
//!          AP510 devices. It enables comprehensive register reading and
//!          debugging for embedded applications requiring detailed system
//!          state analysis. The utilities support efficient bulk reading
//!          of all contiguous register groups found in the AP510
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

#include "am_util_regdump_apollo510.h"

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
#define AM_UTIL_REGDUMP_DEVPWRSTATUS_ADDR    0x40021008
#define AM_UTIL_REGDUMP_AUDSSPWRSTATUS_ADDR  0x40021010
#define AM_UTIL_REGDUMP_ADCSTATUS_ADDR       0x40021194

//*****************************************************************************
//
//! @brief Check if a register group should be read based on power status
//!
//! @param group_start_address - Starting address of the register group
//! @param devpwrstatus - Device power status register value
//! @param audsspwrstatus - Audio subsystem power status register value
//! @param adcstatus - ADC status register value
//!
//! @returns true if the register group is accessible and should be read
//
//*****************************************************************************
static bool
is_accessible(uint32_t group_start_address, uint32_t devpwrstatus, uint32_t audsspwrstatus, uint32_t adcstatus)
{
    //! Map register group addresses to power status bits (AP510 register map).
    //! System and always-on registers (0x40000000-0x4002FFFF) - always accessible.
    if (group_start_address >= 0x40000000 && group_start_address < 0x40030000)
    {
        return true;
    }
    //! IOS0 registers (0x40030000-0x4003FFFF) - check PWRSTIOS0 bit.
    if (group_start_address >= 0x40030000 && group_start_address < 0x40040000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTIOS0) != 0;
    }

    //! IOM registers (0x40050000-0x4005FFFF) - check any IOM power bit (shared domain).
    if (group_start_address >= 0x40050000 && group_start_address < 0x40060000)
    {
        return (devpwrstatus & (AM_UTIL_REGDUMP_PWRSTIOM0 | AM_UTIL_REGDUMP_PWRSTIOM1 |
                               AM_UTIL_REGDUMP_PWRSTIOM2 | AM_UTIL_REGDUMP_PWRSTIOM3 |
                               AM_UTIL_REGDUMP_PWRSTIOM4 | AM_UTIL_REGDUMP_PWRSTIOM5 |
                               AM_UTIL_REGDUMP_PWRSTIOM6 | AM_UTIL_REGDUMP_PWRSTIOM7)) != 0;
    }

    //! MSPI registers (0x40060000-0x40063FFF) - check any MSPI power bit (shared domain).
    if (group_start_address >= 0x40060000 && group_start_address < 0x40064000)
    {
        return (devpwrstatus & (AM_UTIL_REGDUMP_PWRSTMSPI0 | AM_UTIL_REGDUMP_PWRSTMSPI1 |
                               AM_UTIL_REGDUMP_PWRSTMSPI2 | AM_UTIL_REGDUMP_PWRSTMSPI3)) != 0;
    }

    //! SDIO registers (0x40070000-0x40071FFF) - check any SDIO power bit (shared domain).
    if (group_start_address >= 0x40070000 && group_start_address < 0x40072000)
    {
        return (devpwrstatus & (AM_UTIL_REGDUMP_PWRSTSDIO0 | AM_UTIL_REGDUMP_PWRSTSDIO1)) != 0;
    }

    //! GPU registers (0x40090000-0x40090FFF) - check PWRSTGFX bit.
    if (group_start_address >= 0x40090000 && group_start_address < 0x40091000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTGFX) != 0;
    }

    //! Display Controller registers (0x400A0000-0x400A1FFF) - check PWRSTDISP bit.
    if (group_start_address >= 0x400A0000 && group_start_address < 0x400A2000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDISP) != 0;
    }

    //! DSI registers (0x400A8000-0x400A8FFF) - check PWRSTDISPPHY bit.
    if (group_start_address >= 0x400A8000 && group_start_address < 0x400A9000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTDISPPHY) != 0;
    }

    //! USB registers (0x400B0000-0x400B4FFF) - check PWRSTUSB bit.
    if (group_start_address >= 0x400B0000 && group_start_address < 0x400B5000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTUSB) != 0;
    }

    //! CRYPTO registers (0x400C0000-0x400C1FFF) - check PWRSTCRYPTO bit.
    if (group_start_address >= 0x400C0000 && group_start_address < 0x400C2000)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTCRYPTO) != 0;
    }

    //! OTP registers (0x400C2000-0x420069E0) - check PWRSTOTP bit.
    if (group_start_address >= 0x400C2000 && group_start_address <= 0x420069E0)
    {
        return (devpwrstatus & AM_UTIL_REGDUMP_PWRSTOTP) != 0;
    }

    //! Audio subsystem registers (0x40200000-0x4020FFFF) - check audio power status.
    if (group_start_address >= 0x40200000 && group_start_address < 0x40210000)
    {
        if (group_start_address >= 0x40201000 && group_start_address < 0x40205000)
        {
            return (audsspwrstatus & AM_UTIL_REGDUMP_PWRSTPDM0) != 0;  //!< PDM - PWRSTPDM0
        }
        else if (group_start_address >= 0x40208000 && group_start_address < 0x4020A000)
        {
            return (audsspwrstatus & (AM_UTIL_REGDUMP_PWRSTI2S0 | AM_UTIL_REGDUMP_PWRSTI2S1)) != 0;  //!< I2S
        }
        else
        {
            return (audsspwrstatus & (AM_UTIL_REGDUMP_PWRSTPDM0 | AM_UTIL_REGDUMP_PWRSTI2S0 |
                                      AM_UTIL_REGDUMP_PWRSTI2S1 | AM_UTIL_REGDUMP_PWRSTAUDADC)) != 0;
        }
    }

    //! Audio ADC registers (0x40210000-0x40210FFF) - check PWRSTAUDADC bit.
    if (group_start_address >= 0x40210000 && group_start_address < 0x40211000)
    {
        return (audsspwrstatus & AM_UTIL_REGDUMP_PWRSTAUDADC) != 0;
    }

    //! Cortex-M4 system registers (0xE0000000-0xE004FFFF) - always accessible.
    if (group_start_address >= 0xE0000000 && group_start_address <= 0xE004FFFF)
    {
        return true;
    }
    //! Registers not covered by power status (e.g. system) - always readable.
    return true;
}

//*****************************************************************************
//
//! @brief Register group definitions (AP510 register map)
//
//*****************************************************************************

const am_util_register_group_t am_util_register_groups[] = {
    //! Group 1: CFG registers

#if defined(AM_UTIL_REGDUMP_RSTGEN)

    {0x40000000, 3},

    //! Group 2: SIMOBODM registers
    {0x40000014, 2},

    //! Group 3: INTEN registers
    {0x40000200, 4},

    //! Group 4: OCTRL registers

#endif

#if defined(AM_UTIL_REGDUMP_CLKGEN)

    {0x4000400C, 2},

    //! Group 5: HFADJ registers
    {0x40004020, 1},

    //! Group 6: CLOCKENSTAT registers
    {0x40004030, 3},

    //! Group 7: MISC registers
    {0x40004044, 5},

    //! Group 8: LFRCCTRL registers
    {0x40004078, 1},

    //! Group 9: DISPCLKCTRL registers
    {0x40004084, 3},

    //! Group 10: MSPIIOCLKCTRL registers
    {0x40004110, 1},

    //! Group 11: CLKCTRL registers
    {0x40004120, 1},

    //! Group 12: RTCCTL registers

#endif

#if defined(AM_UTIL_REGDUMP_RTC)

    {0x40004800, 2},

    //! Group 13: CTRLOW registers
    {0x40004820, 2},

    //! Group 14: ALMLOW registers
    {0x40004830, 2},

    //! Group 15: INTEN registers
    {0x40004A00, 4},

    //! Group 16: CTRL registers

#endif

#if defined(AM_UTIL_REGDUMP_RSTGEN)

    {0x40008000, 2},

    //! Group 17: GLOBEN registers
    {0x40008010, 1},

    //! Group 18: INTEN registers
    {0x40008060, 4},

    //! Group 19: OUTCFG0 registers
    {0x40008080, 56},

    //! Group 20: CTRL0 registers
    {0x40008200, 6},

    //! Group 21: CTRL1 registers
    {0x40008220, 6},

    //! Group 22: CTRL2 registers
    {0x40008240, 6},

    //! Group 23: CTRL3 registers
    {0x40008260, 6},

    //! Group 24: CTRL4 registers
    {0x40008280, 6},

    //! Group 25: CTRL5 registers
    {0x400082A0, 6},

    //! Group 26: CTRL6 registers
    {0x400082C0, 6},

    //! Group 27: CTRL7 registers
    {0x400082E0, 6},

    //! Group 28: CTRL8 registers
    {0x40008300, 6},

    //! Group 29: CTRL9 registers
    {0x40008320, 6},

    //! Group 30: CTRL10 registers
    {0x40008340, 6},

    //! Group 31: CTRL11 registers
    {0x40008360, 6},

    //! Group 32: CTRL12 registers
    {0x40008380, 6},

    //! Group 33: CTRL13 registers
    {0x400083A0, 6},

    //! Group 34: CTRL14 registers
    {0x400083C0, 6},

    //! Group 35: CTRL15 registers
    {0x400083E0, 7},

    //! Group 36: STCFG registers

#endif

#if defined(AM_UTIL_REGDUMP_STIMER)

    {0x40008800, 2},

    //! Group 37: SCAPCTRL0 registers
    {0x40008810, 20},

    //! Group 38: STMINTEN registers

    {0x40008900, 4},

    //! Group 39: CFG registers


#endif

#if defined(AM_UTIL_REGDUMP_VCOMP)

    {0x4000C000, 3},

    //! Group 40: INTEN registers
    {0x4000C200, 4},

    //! Group 41: PINCFG0 registers


#endif

#if defined(AM_UTIL_REGDUMP_GPIO)

    {0x40010000, 224},

    //! Group 42: PADKEY registers
    {0x40010400, 75},

    //! Group 43: MCUN0INT0EN registers
    {0x40010530, 56},

    //! Group 44: RD0 registers


#endif

#if defined(AM_UTIL_REGDUMP_FPIO)

    {0x40011000, 49},

    //! Group 45: RNG registers


#endif

#if defined(AM_UTIL_REGDUMP_OTP)

    {0x40014AA0, 1},

    //! Group 46: INTERRUPT registers
    {0x40014AB0, 1},

    //! Group 47: PTMSTAT registers
    {0x40014AC4, 1},

    //! Group 48: CHIPPN registers


#endif

#if defined(AM_UTIL_REGDUMP_MCUCTRL)

    {0x40020000, 7},

    //! Group 49: DEBUGGER registers
    {0x40020020, 1},

    //! Group 50: ACRG registers
    {0x40020028, 1},

    //! Group 51: VREFGEN2 registers
    {0x40020044, 5},

    //! Group 52: VRCTRL registers
    {0x40020060, 1},

    //! Group 53: LDOREG1 registers
    {0x40020080, 1},

    //! Group 54: LDOREG2 registers
    {0x40020088, 1},

    //! Group 55: LFRC registers
    {0x400200E0, 1},

    //! Group 56: BODCTRL registers
    {0x40020100, 1},

    //! Group 57: ADCPWRCTRL registers
    {0x40020108, 3},

    //! Group 58: XTALCTRL registers
    {0x40020120, 4},

    //! Group 59: BGTLPCTRL registers
    {0x40020134, 1},

    //! Group 60: MRAMCRYPTOPWRCTRL registers
    {0x40020180, 1},

    //! Group 61: BODISABLE registers
    {0x400201AC, 2},

    //! Group 62: BOOTLOADER registers
    {0x400201B8, 4},

    //! Group 63: DBGR1 registers
    {0x40020200, 2},

    //! Group 64: WICCONTROL registers
    {0x4002021C, 1},

    //! Group 65: DBGCTRL registers
    {0x40020250, 1},

    //! Group 66: OTAPOINTER registers
    {0x40020264, 1},

    //! Group 67: APBDMACTRL registers
    {0x40020280, 2},

    //! Group 68: KEXTCLKSEL registers
    {0x40020338, 19},

    //! Group 69: USBRSTCTRL registers
    {0x40020388, 1},

    //! Group 70: FLASHWPROT0 registers
    {0x400203A8, 14},

    //! Group 71: SRAMRPROT0 registers
    {0x400203F0, 6},

    //! Group 72: AUDADCPWRCTRL registers
    {0x40020430, 2},

    //! Group 73: PGAADCIFCTRL registers
    {0x4002043C, 4},

    //! Group 74: SDIO0CTRL registers
    {0x40020454, 3},

    //! Group 75: DSIBIST registers
    {0x400204A0, 12},

    //! Group 76: PLLCTL0 registers
    {0x400204D8, 5},

    //! Group 77: MCUPERFREQ registers


#endif

#if defined(AM_UTIL_REGDUMP_PWRCTRL)

    {0x40021000, 14},

    //! Group 78: MMSOVERRIDE registers
    {0x40021040, 1},

    //! Group 79: CPUPWRCTRL registers
    {0x40021050, 3},

    //! Group 80: PWRACKOVR registers
    {0x40021084, 5},

    //! Group 81: VRCTRL registers
    {0x40021100, 3},

    //! Group 82: SRAMCTRL registers
    {0x40021190, 3},

    //! Group 83: TONCNTRCTRL registers
    {0x400211A0, 4},

    //! Group 84: LPOVRTHRESHVDDC registers
    {0x400211B4, 1},

    //! Group 85: LPOVRTHRESHVDDCLV registers
    {0x400211BC, 1},

    //! Group 86: LPOVRSTAT registers
    {0x400211C4, 2},

    //! Group 87: EMONCTRL registers
    {0x40021200, 9},

    //! Group 88: EMONCOUNT0 registers
    {0x40021228, 8},

    //! Group 89: EMONSTATUS registers
    {0x4002124C, 1},

    //! Group 90: CFG registers


#endif

#if defined(AM_UTIL_REGDUMP_WDT)

    {0x40024000, 4},

    //! Group 91: WDTIEREN registers
    {0x40024200, 4},

    //! Group 92: SRLOCKS registers


#endif

#if defined(AM_UTIL_REGDUMP_SSC)

    {0x40025000, 2},

    //! Group 93: CTRL registers


#endif

#if defined(AM_UTIL_REGDUMP_SECURITY)

    {0x40030000, 1},

    //! Group 94: SRCADDR registers
    {0x40030010, 1},

    //! Group 95: LEN registers
    {0x40030020, 1},

    //! Group 96: RESULT registers
    {0x40030030, 1},

    //! Group 97: LOCKCTRL registers
    {0x40030078, 6},

    //! Group 98: FIFOPTR registers


#endif

#if defined(AM_UTIL_REGDUMP_IOSLAVE)

    {0x40034100, 11},

    //! Group 99: DMACFG registers
    {0x40034130, 4},

    //! Group 100: INTEN registers
    {0x40034200, 8},

    //! Group 101: FIFOPTR registers


#endif

#if defined(AM_UTIL_REGDUMP_IOSLAVEFD)

    {0x40035100, 11},

    //! Group 102: DMACFG registers
    {0x40035130, 4},

    //! Group 103: INTEN registers
    {0x40035200, 8},

    //! Group 104: FIFOPTR registers
    {0x40036100, 11},

    //! Group 105: DMACFG registers
    {0x40036130, 4},

    //! Group 106: INTEN registers
    {0x40036200, 8},

    //! Group 107: CFG registers


#endif

#if defined(AM_UTIL_REGDUMP_ADC)

    {0x40038000, 17},

    //! Group 108: INTEN registers
    {0x40038200, 4},

    //! Group 109: DMATRIGEN registers
    {0x40038240, 2},

    //! Group 110: DMACFG registers
    {0x40038280, 1},

    //! Group 111: DMATOTCOUNT registers
    {0x40038288, 6},

    //! Group 112: DR registers


#endif

#if defined(AM_UTIL_REGDUMP_UART)

    {0x40039000, 2},

    //! Group 113: FR registers
    {0x40039018, 1},

    //! Group 114: ILPR registers
    {0x40039020, 13},

    //! Group 115: DR registers
    {0x4003A000, 2},

    //! Group 116: FR registers
    {0x4003A018, 1},

    //! Group 117: ILPR registers
    {0x4003A020, 13},

    //! Group 118: DR registers
    {0x4003B000, 2},

    //! Group 119: FR registers
    {0x4003B018, 1},

    //! Group 120: ILPR registers
    {0x4003B020, 13},

    //! Group 121: DR registers
    {0x4003C000, 2},

    //! Group 122: FR registers
    {0x4003C018, 1},

    //! Group 123: ILPR registers
    {0x4003C020, 13},

    //! Group 124: FIFO registers


#endif

#if defined(AM_UTIL_REGDUMP_IOM)

    {0x40050000, 1},

    //! Group 125: FIFOPTR registers
    {0x40050100, 12},

    //! Group 126: INTEN registers
    {0x40050200, 19},

    //! Group 127: MSPICFG registers
    {0x40050280, 1},

    //! Group 128: MI2CCFG registers
    {0x400502C0, 2},

    //! Group 129: IOMDBG registers
    {0x40050388, 1},

    //! Group 130: FIFO registers
    {0x40051000, 1},

    //! Group 131: FIFOPTR registers
    {0x40051100, 12},

    //! Group 132: INTEN registers
    {0x40051200, 19},

    //! Group 133: MSPICFG registers
    {0x40051280, 1},

    //! Group 134: MI2CCFG registers
    {0x400512C0, 2},

    //! Group 135: IOMDBG registers
    {0x40051388, 1},

    //! Group 136: FIFO registers
    {0x40052000, 1},

    //! Group 137: FIFOPTR registers
    {0x40052100, 12},

    //! Group 138: INTEN registers
    {0x40052200, 19},

    //! Group 139: MSPICFG registers
    {0x40052280, 1},

    //! Group 140: MI2CCFG registers
    {0x400522C0, 2},

    //! Group 141: IOMDBG registers
    {0x40052388, 1},

    //! Group 142: FIFO registers
    {0x40053000, 1},

    //! Group 143: FIFOPTR registers
    {0x40053100, 12},

    //! Group 144: INTEN registers
    {0x40053200, 19},

    //! Group 145: MSPICFG registers
    {0x40053280, 1},

    //! Group 146: MI2CCFG registers
    {0x400532C0, 2},

    //! Group 147: IOMDBG registers
    {0x40053388, 1},

    //! Group 148: FIFO registers
    {0x40054000, 1},

    //! Group 149: FIFOPTR registers
    {0x40054100, 12},

    //! Group 150: INTEN registers
    {0x40054200, 19},

    //! Group 151: MSPICFG registers
    {0x40054280, 1},

    //! Group 152: MI2CCFG registers
    {0x400542C0, 2},

    //! Group 153: IOMDBG registers
    {0x40054388, 1},

    //! Group 154: FIFO registers
    {0x40055000, 1},

    //! Group 155: FIFOPTR registers
    {0x40055100, 12},

    //! Group 156: INTEN registers
    {0x40055200, 19},

    //! Group 157: MSPICFG registers
    {0x40055280, 1},

    //! Group 158: MI2CCFG registers
    {0x400552C0, 2},

    //! Group 159: IOMDBG registers
    {0x40055388, 1},

    //! Group 160: FIFO registers
    {0x40056000, 1},

    //! Group 161: FIFOPTR registers
    {0x40056100, 12},

    //! Group 162: INTEN registers
    {0x40056200, 19},

    //! Group 163: MSPICFG registers
    {0x40056280, 1},

    //! Group 164: MI2CCFG registers
    {0x400562C0, 2},

    //! Group 165: IOMDBG registers
    {0x40056388, 1},

    //! Group 166: FIFO registers
    {0x40057000, 1},

    //! Group 167: FIFOPTR registers
    {0x40057100, 12},

    //! Group 168: INTEN registers
    {0x40057200, 19},

    //! Group 169: MSPICFG registers
    {0x40057280, 1},

    //! Group 170: MI2CCFG registers
    {0x400572C0, 2},

    //! Group 171: IOMDBG registers
    {0x40057388, 1},

    //! Group 172: HCIVERSION registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058000, 6},

    //! Group 173: INTRSTATUS registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058020, 9},

    //! Group 174: IBINOTIFYCTRL registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058058, 1},

    //! Group 175: DAT0 registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058200, 2},

    //! Group 176: DCT0 registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058300, 4},

    //! Group 177: RHSCONTROL registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058400, 5},

    //! Group 178: COMMANDQUEUEPORT registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058500, 7},

    //! Group 179: PIOINTRSTATUS registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058520, 4},

    //! Group 180: EXTCAPHEADER registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058700, 4},

    //! Group 181: MASTERCONFIG registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058714, 1},

    //! Group 182: CRSETUP registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058800, 3},

    //! Group 183: RHINTRSTATUS registers - DISABLED (I3C non-functional on Apollo510)
    // {0x40058810, 16},

    //! Group 184: CTRL registers


#endif

#if defined(AM_UTIL_REGDUMP_MSPI)

    {0x40060000, 9},

    //! Group 185: MSPICFG registers
    {0x40060030, 1},

    //! Group 186: PADOUTEN registers
    {0x40060044, 3},

    //! Group 187: DEV0AXI registers
    {0x40060080, 11},

    //! Group 188: DMACFG registers
    {0x40060100, 7},

    //! Group 189: INTEN registers
    {0x40060200, 4},

    //! Group 190: CQCFG registers
    {0x400602A0, 1},

    //! Group 191: CQADDR registers
    {0x400602A8, 5},

    //! Group 192: CQCURIDX registers
    {0x400602C0, 2},

    //! Group 193: CTRL registers
    {0x40061000, 9},

    //! Group 194: MSPICFG registers
    {0x40061030, 1},

    //! Group 195: PADOUTEN registers
    {0x40061044, 3},

    //! Group 196: DEV0AXI registers
    {0x40061080, 11},

    //! Group 197: DMACFG registers
    {0x40061100, 7},

    //! Group 198: INTEN registers
    {0x40061200, 4},

    //! Group 199: CQCFG registers
    {0x400612A0, 1},

    //! Group 200: CQADDR registers
    {0x400612A8, 5},

    //! Group 201: CQCURIDX registers
    {0x400612C0, 2},

    //! Group 202: CTRL registers
    {0x40062000, 9},

    //! Group 203: MSPICFG registers
    {0x40062030, 1},

    //! Group 204: PADOUTEN registers
    {0x40062044, 3},

    //! Group 205: DEV0AXI registers
    {0x40062080, 11},

    //! Group 206: DMACFG registers
    {0x40062100, 7},

    //! Group 207: INTEN registers
    {0x40062200, 4},

    //! Group 208: CQCFG registers
    {0x400622A0, 1},

    //! Group 209: CQADDR registers
    {0x400622A8, 5},

    //! Group 210: CQCURIDX registers
    {0x400622C0, 2},

    //! Group 211: CTRL registers
    {0x40063000, 9},

    //! Group 212: MSPICFG registers
    {0x40063030, 1},

    //! Group 213: PADOUTEN registers
    {0x40063044, 3},

    //! Group 214: DEV0AXI registers
    {0x40063080, 11},

    //! Group 215: DMACFG registers
    {0x40063100, 7},

    //! Group 216: INTEN registers
    {0x40063200, 4},

    //! Group 217: CQCFG registers
    {0x400632A0, 1},

    //! Group 218: CQADDR registers
    {0x400632A8, 5},

    //! Group 219: CQCURIDX registers
    {0x400632C0, 2},

    //! Group 220: SDMA registers


#endif

#if defined(AM_UTIL_REGDUMP_SDIO)

    {0x40070000, 29},

    //! Group 221: VENDOR registers
    {0x40070078, 1},

    //! Group 222: SLOTSTAT registers
    {0x400700FC, 2},

    //! Group 223: SDMA registers
    {0x40071000, 29},

    //! Group 224: VENDOR registers
    {0x40071078, 1},

    //! Group 225: SLOTSTAT registers
    {0x400710FC, 2},

    //! Group 226: TEX0BASE registers


#endif

#if defined(AM_UTIL_REGDUMP_GPU)

    {0x40090000, 3},

    //! Group 227: TEX1BASE registers
    {0x40090010, 7},

    //! Group 228: TEX3BASE registers
    {0x40090030, 3},

    //! Group 229: CGCMD registers
    {0x40090090, 4},

    //! Group 230: STATUS registers
    {0x400900B0, 1},

    //! Group 231: BUSCTRL registers
    {0x400900C0, 4},

    //! Group 232: CMDLISTSTATUS registers
    {0x400900E8, 9},

    //! Group 233: CLIPMIN registers
    {0x40090110, 11},

    //! Group 234: DRAWPT2X registers
    {0x40090140, 3},

    //! Group 235: DRAWPT3X registers
    {0x40090150, 3},

    //! Group 236: MM00 registers
    {0x40090160, 15},

    //! Group 237: REDX registers
    {0x400901A0, 12},

    //! Group 238: IDREG registers
    {0x400901EC, 2},

    //! Group 239: C0REG registers
    {0x40090200, 4},

    //! Group 240: ACTIVE registers
    {0x40090250, 1},

    //! Group 241: IRQID registers
    {0x40090FF0, 1},

    //! Group 242: MODE registers


#endif

#if defined(AM_UTIL_REGDUMP_DC)

    {0x400A0000, 18},

    //! Group 243: LAYER1MODE registers
    {0x400A0050, 6},

    //! Group 244: LAYER2MODE registers
    {0x400A0070, 6},

    //! Group 245: LAYER3MODE registers
    {0x400A0090, 6},

    //! Group 246: DBICMD registers
    {0x400A00E8, 8},

    //! Group 247: CRC registers
    {0x400A0184, 1},

    //! Group 248: GLLUT registers
    {0x400A0400, 1},

    //! Group 249: L0LUT registers
    {0x400A1000, 1},

    //! Group 250: L1LUT registers
    {0x400A1400, 1},

    //! Group 251: L2LUT0 registers
    {0x400A1800, 1},

    //! Group 252: L3LUT registers
    {0x400A1C00, 1},

    //! Group 253: DEVICEREADY registers


#endif

#if defined(AM_UTIL_REGDUMP_DSI)

    {0x400A8000, 9},

    //! Group 254: HSYNCCNT registers
    {0x400A8028, 24},

    //! Group 255: ERRORAUTORCOV registers
    {0x400A8098, 3},

    //! Group 256: CFG0 registers


#endif

#if defined(AM_UTIL_REGDUMP_USB)

    {0x400B0000, 14},

    //! Group 257: HWVERS registers
    {0x400B006C, 1},

    //! Group 258: INFO registers
    {0x400B0078, 1},

    //! Group 259: TIMEOUT1 registers
    {0x400B0080, 2},

    //! Group 260: CLKCTRL registers
    {0x400B2000, 2},

    //! Group 261: UTMISTICKYSTATUS registers
    {0x400B2014, 16},

    //! Group 262: ADMACMPINTEN registers
    {0x400B2060, 8},

    //! Group 263: ADMATOTCOUNT0 registers
    {0x400B2100, 10},

    //! Group 264: ADMATARGADDR0 registers
    {0x400B2200, 10},

    //! Group 265: ADMAEP0 registers
    {0x400B2300, 10},

    //! Group 266: ADMAREQSIZE0 registers
    {0x400B2400, 10},

    //! Group 267: REG00 registers


#endif

#if defined(AM_UTIL_REGDUMP_USBPHY)

    {0x400B4000, 34},

    //! Group 268: MEMORYMAP0 registers


#endif

#if defined(AM_UTIL_REGDUMP_CRYPTO)

    {0x400C0000, 47},

    //! Group 269: PKAVERSION registers
    {0x400C00C4, 1},

    //! Group 270: PKAMONREAD registers
    {0x400C00D0, 6},

    //! Group 271: PKAWORDACCESS registers
    {0x400C00F0, 1},

    //! Group 272: PKABUFFADDR registers
    {0x400C00F8, 1},

    //! Group 273: RNGIMR registers
    {0x400C0100, 15},

    //! Group 274: RNGSWRESET registers
    {0x400C0140, 1},

    //! Group 275: RNGDEBUGENINPUT registers
    {0x400C01B4, 8},

    //! Group 276: RNGWATCHDOGVAL registers
    {0x400C01D8, 2},

    //! Group 277: CHACHACONTROLREG registers
    {0x400C0380, 27},

    //! Group 278: AESKEY00 registers
    {0x400C0400, 29},

    //! Group 279: AESSK registers
    {0x400C0478, 2},

    //! Group 280: AESSK1 registers
    {0x400C04B4, 1},

    //! Group 281: AESREMAININGBYTES registers
    {0x400C04BC, 2},

    //! Group 282: AESHWFLAGS registers
    {0x400C04C8, 1},

    //! Group 283: AESCTRNOINCREMENT registers
    {0x400C04D8, 1},

    //! Group 284: AESDFAISON registers
    {0x400C04F0, 1},

    //! Group 285: AESDFAERRSTATUS registers
    {0x400C04F8, 1},

    //! Group 286: AESCMACSIZE0KICK registers
    {0x400C0524, 1},

    //! Group 287: HASHH0 registers
    {0x400C0640, 9},

    //! Group 288: AUTOHWPADDING registers
    {0x400C0684, 2},

    //! Group 289: LOADINITSTATE registers
    {0x400C0694, 1},

    //! Group 290: HASHSELAESMAC registers
    {0x400C06A4, 1},

    //! Group 291: HASHVERSION registers
    {0x400C07B0, 1},

    //! Group 292: HASHCONTROL registers
    {0x400C07C0, 5},

    //! Group 293: HASHPARAM registers
    {0x400C07DC, 1},

    //! Group 294: HASHAESSWRESET registers
    {0x400C07E4, 2},

    //! Group 295: AESCLKENABLE registers
    {0x400C0810, 1},

    //! Group 296: HASHCLKENABLE registers
    {0x400C0818, 4},

    //! Group 297: CHACHACLKENABLE registers
    {0x400C0858, 1},

    //! Group 298: CRYPTOCTL registers
    {0x400C0900, 1},

    //! Group 299: CRYPTOBUSY registers
    {0x400C0910, 1},

    //! Group 300: HASHBUSY registers
    {0x400C091C, 1},

    //! Group 301: CONTEXTID registers
    {0x400C0930, 1},

    //! Group 302: GHASHSUBKEY00 registers
    {0x400C0960, 10},

    //! Group 303: HOSTRGFIRR registers
    {0x400C0A00, 4},

    //! Group 304: HOSTRGFSIGNATURE registers
    {0x400C0A24, 2},

    //! Group 305: HOSTCRYPTOKEYSEL registers
    {0x400C0A38, 1},

    //! Group 306: HOSTCORECLKGATINGENABLE registers
    {0x400C0A78, 5},

    //! Group 307: AHBMSINGLES registers
    {0x400C0B00, 4},

    //! Group 308: DINBUFFER registers
    {0x400C0C00, 1},

    //! Group 309: DINMEMDMABUSY registers
    {0x400C0C20, 1},

    //! Group 310: SRCLLIWORD0 registers
    {0x400C0C28, 6},

    //! Group 311: DINCPUDATASIZE registers
    {0x400C0C48, 1},

    //! Group 312: FIFOINEMPTY registers
    {0x400C0C50, 1},

    //! Group 313: DINFIFORSTPNTR registers
    {0x400C0C58, 1},

    //! Group 314: DOUTMEMDMABUSY registers
    {0x400C0D20, 1},

    //! Group 315: DSTLLIWORD0 registers
    {0x400C0D28, 6},

    //! Group 316: READALIGNLAST registers
    {0x400C0D44, 1},

    //! Group 317: DOUTFIFOEMPTY registers
    {0x400C0D50, 1},

    //! Group 318: SRAMDATA registers
    {0x400C0F00, 3},

    //! Group 319: PERIPHERALID4 registers
    {0x400C0FD0, 1},

    //! Group 320: PERIPHERALID0 registers
    {0x400C0FE0, 8},

    //! Group 321: HOSTDCUEN0 registers
    {0x400C1E00, 17},

    //! Group 322: AIBFUSEPROGCOMPLETED registers
    {0x400C1F04, 11},

    //! Group 323: OTP registers


#endif

#if defined(AM_UTIL_REGDUMP_OTPINFOC)

    {0x400C2044, 17},
    {0x400C2090, 8},
    {0x400C2200, 17},
    {0x400C224C, 93},
    {0x400C23FC, 1},

    //! Group 328: CTRL registers


#endif

#if defined(AM_UTIL_REGDUMP_PDM)

    {0x40201000, 8},

    //! Group 329: INTEN registers
    {0x40201100, 4},

    //! Group 330: DMATRIGEN registers
    {0x40201140, 3},

    //! Group 331: DMATARGADDR registers
    {0x40201154, 2},

    //! Group 332: DMATARGADDRNEXT registers
    {0x40201160, 3},

    //! Group 333: DMATOTCOUNT registers
    {0x40201250, 1},

    //! Group 334: CTRL registers
    {0x40202000, 8},

    //! Group 335: INTEN registers
    {0x40202100, 4},

    //! Group 336: DMATRIGEN registers
    {0x40202140, 3},

    //! Group 337: DMATARGADDR registers
    {0x40202154, 2},

    //! Group 338: DMATARGADDRNEXT registers
    {0x40202160, 3},

    //! Group 339: DMATOTCOUNT registers
    {0x40202250, 1},

    //! Group 340: CTRL registers
    {0x40203000, 8},

    //! Group 341: INTEN registers
    {0x40203100, 4},

    //! Group 342: DMATRIGEN registers
    {0x40203140, 3},

    //! Group 343: DMATARGADDR registers
    {0x40203154, 2},

    //! Group 344: DMATARGADDRNEXT registers
    {0x40203160, 3},

    //! Group 345: DMATOTCOUNT registers
    {0x40203250, 1},

    //! Group 346: CTRL registers
    {0x40204000, 8},

    //! Group 347: INTEN registers
    {0x40204100, 4},

    //! Group 348: DMATRIGEN registers
    {0x40204140, 3},

    //! Group 349: DMATARGADDR registers
    {0x40204154, 2},

    //! Group 350: DMATARGADDRNEXT registers
    {0x40204160, 3},

    //! Group 351: DMATOTCOUNT registers
    {0x40204250, 1},

    //! Group 352: RXDATA registers


#endif

#if defined(AM_UTIL_REGDUMP_I2S)

    {0x40208000, 5},

    //! Group 353: TXDATA registers
    {0x40208020, 5},

    //! Group 354: I2SDATACFG registers
    {0x40208040, 6},

    //! Group 355: INTDIV registers
    {0x40208060, 2},

    //! Group 356: CLKCFG registers
    {0x40208100, 1},

    //! Group 357: DMACFG registers
    {0x40208200, 13},

    //! Group 358: INTEN registers
    {0x40208300, 4},

    //! Group 359: RXDATA registers
    {0x40209000, 5},

    //! Group 360: TXDATA registers
    {0x40209020, 5},

    //! Group 361: I2SDATACFG registers
    {0x40209040, 6},

    //! Group 362: INTDIV registers
    {0x40209060, 2},

    //! Group 363: CLKCFG registers
    {0x40209100, 1},

    //! Group 364: DMACFG registers
    {0x40209200, 13},

    //! Group 365: INTEN registers
    {0x40209300, 4},

    //! Group 366: CFG registers


#endif

#if defined(AM_UTIL_REGDUMP_AUDADC)

    {0x40210000, 19},

    //! Group 367: ZXCFG registers
    {0x40210060, 4},

    //! Group 368: SATCFG registers
    {0x402100A4, 4},

    //! Group 369: INTEN registers
    {0x40210200, 4},

    //! Group 370: DMATRIGEN registers
    {0x40210240, 2},

    //! Group 371: DMACFG registers
    {0x40210280, 1},

    //! Group 372: DMATOTCOUNT registers
    {0x40210288, 6},

    //! Group 373: INFO0 registers


#endif

#if defined(AM_UTIL_REGDUMP_MRAMINFO0)

    {0x42000000, 6},
    {0x42000028, 9},
    {0x42000054, 2},
    {0x42000060, 17},

    //! Group 377: INFO1 registers


#endif

#if defined(AM_UTIL_REGDUMP_MRAMINFO1)

    {0x42003200, 2},
    {0x42003210, 1},
    {0x42003240, 5},
    {0x42003258, 3},
    {0x42003300, 9},
    {0x42003328, 3},
    {0x42003340, 41},

    //! Group 384: OTP registers


#endif

#if defined(AM_UTIL_REGDUMP_OTPINFO0)

    {0x42004000, 6},
    {0x42004028, 9},
    {0x42004054, 2},
    {0x42004060, 17},


#endif

#if defined(AM_UTIL_REGDUMP_OTPINFO1)

    {0x42006800, 2},
    {0x42006810, 1},
    {0x42006818, 1},
    {0x42006820, 13},
    {0x42006858, 3},
    {0x42006900, 9},
    {0x42006928, 3},
    {0x42006940, 41},

    //! Group 396: STIM0 registers


#endif

#if defined(AM_UTIL_REGDUMP_ITM)

    {0xE0000000, 32},

    //! Group 397: TER registers
    {0xE0000E00, 1},

    //! Group 398: TPR registers
    {0xE0000E40, 1},

    //! Group 399: TCR registers
    {0xE0000E80, 1},

    //! Group 400: LOCKAREG registers
    {0xE0000FB0, 2},

    //! Group 401: PID4 registers
    {0xE0000FD0, 12},

    //! Group 402: ICTR registers


#endif

#if defined(AM_UTIL_REGDUMP_SYSCTRL)

    {0xE000E004, 2},

    //! Group 403: SYSTCSR registers

#endif

#if defined(AM_UTIL_REGDUMP_SYSTICK)

    {0xE000E010, 4},

    //! Group 404: ISER0 registers

#endif

#if defined(AM_UTIL_REGDUMP_SYSCTRL)

    {0xE000E100, 1},

    //! Group 405: ICER0 registers
    {0xE000E180, 1},

    //! Group 406: ISPR0 registers
    {0xE000E200, 1},

    //! Group 407: ICPR0 registers
    {0xE000E280, 1},

    //! Group 408: IABR0 registers
    {0xE000E300, 1},

    //! Group 409: IPR0 registers
    {0xE000E400, 8},

    //! Group 410: ICSR registers
    {0xE000ED04, 11},

    //! Group 411: MMFAR registers
    {0xE000ED34, 2},

    //! Group 412: CPACR registers
    {0xE000ED88, 1},

    //! Group 413: DEMCR registers
    {0xE000EDFC, 1},

    //! Group 414: STIR registers
    {0xE000EF00, 1},

    //! Group 415: FPCCR registers
    {0xE000EF34, 3},

    //! Group 416: SSPSR registers - DISABLED (TPIU disabled for now)
    // {0xE0040000, 2},

    //! Group 417: ACPR registers - DISABLED (TPIU disabled for now)
    // {0xE0040010, 1},

    //! Group 418: SPPR registers - DISABLED (TPIU disabled for now)
    // {0xE00400F0, 1},

    //! Group 419: FFCR registers - DISABLED (TPIU disabled for now)
    // {0xE0040304, 1},

    //! Group 420: ITCTRL registers - DISABLED (TPIU disabled for now)
    // {0xE0040F00, 1},
    // {0xE0040FC8, 1}


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
    am_util_stdio_printf("Starting AP510 register dump...\n");


    //! Read power status registers to determine which peripherals are powered.
    uint32_t devpwrstatus = *((volatile uint32_t*)AM_UTIL_REGDUMP_DEVPWRSTATUS_ADDR);
    uint32_t audsspwrstatus = *((volatile uint32_t*)AM_UTIL_REGDUMP_AUDSSPWRSTATUS_ADDR);
    uint32_t adcstatus = *((volatile uint32_t*)AM_UTIL_REGDUMP_ADCSTATUS_ADDR);
    am_util_stdio_printf("DEVPWRSTATUS: 0x%08X\n", devpwrstatus);
    am_util_stdio_printf("AUDSSPWRSTATUS: 0x%08X\n", audsspwrstatus);
    am_util_stdio_printf("ADCSTATUS: 0x%08X\n", adcstatus);
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
            if (!is_accessible(group->start_address, devpwrstatus, audsspwrstatus, adcstatus))
            {
                am_util_stdio_printf("--- Group %d: 0x%08X (%d registers) --- SKIPPED (not powered)\n",
                                    group_idx, group->start_address, group->count);
                continue;
            }

            //! Show all registers in this group.
            am_util_stdio_printf("--- Group %d: 0x%08X (%d registers) ---\n",
                                group_idx, group->start_address,                 group->count);

            //! Read all registers in this group using the utility function.
            am_util_get_n_registers(group->start_address, group->count);
            total_registers_read += group->count;
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
