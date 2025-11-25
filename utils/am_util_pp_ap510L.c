//*****************************************************************************
//
//! @file am_util_pp_ap510L.c
//!
//! @brief Functions to aid power profiling and debugging (AP510L specific)
//!
//! @addtogroup ppf_ap510L_utils Power Profiling Functionality
//!
//! @ingroup utils
//! @{
//! Purpose: This module provides pin programming and configuration utilities
//!          for Ambiq Micro devices. It enables GPIO setup, pad configuration,
//!          and pin function management for embedded applications requiring
//!          flexible I/O control. The utilities support various pin modes,
//!          drive strengths, and peripheral routing options.
//!
//! @section utils_pp_features Key Features
//!
//! 1. @b Pin @b Configuration: Flexible pin setup options.
//! 2. @b Pad @b Control: Advanced pad configuration.
//! 3. @b Function @b Selection: Multiple pin function support.
//! 4. @b Drive @b Strength: Configurable output drive.
//! 5. @b Protection: Pin protection mechanisms.
//!
//! @section utils_pp_functionality Functionality
//!
//! - Configure pin functions
//! - Set pad characteristics
//! - Manage drive strengths
//! - Handle pin protection
//! - Control pin states
//!
//! @section utils_pp_usage Usage
//!
//! 1. Initialize pin configuration
//! 2. Set pad parameters
//! 3. Configure pin functions
//! 4. Manage pin states
//!
//! @section utils_pp_configuration Configuration
//!
//! - Set pin function modes
//! - Configure drive strengths
//! - Define protection options
//! - Set pad characteristics
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
// This is part of revision release_sdk5_2_a_2-228a2539a of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
//#if defined(AM_PART_APOLLO330P_510L)
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"
#include "am_util_pp_ap510L.h"


#if USE_DMIC_PDM
#define AM_HAL_MAGIC_PDM                0xF956E2
#define AM_HAL_PDM_HANDLE_VALID(h)                                            \
    ((h) &&                                                                   \
    (((am_hal_handle_prefix_t *)(h))->s.bInit) &&                             \
    (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_PDM))
#endif

#define AM_VOS_SNAPSHOP_NUMS_STAGES     5

//
//This is the data structure we need to fill up
//
am_util_pp_b1_t am_util_pp1[AM_VOS_SNAPSHOP_NUMS_STAGES];
am_util_pp_b2_t am_util_pp2[AM_VOS_SNAPSHOP_NUMS_STAGES];
am_util_pp_b3_t am_util_pp3[AM_VOS_SNAPSHOP_NUMS_STAGES];
am_util_pp_b4_t am_util_pp4[AM_VOS_SNAPSHOP_NUMS_STAGES];
am_util_pp_b5_t am_util_pp5[AM_VOS_SNAPSHOP_NUMS_STAGES];

static bool bCaptured[5] = {0, };


//*****************************************************************************
//
// Prints the filled up JSON to serial port.
//
// This function will print filled JSON fields to serial port.
//
//*****************************************************************************
void
am_util_print_JSON(uint8_t uNumber)
{
    //
    // Print the filled JSON file out
    //
    char *pwrStr1 = "\n{\"PWRCTRL\":    \
    {                                   \
        \"Singleshot\": %u,             \
        \"SnapN\": %u,                  \
        \"MCUPERFREQ\": %u,          \
        \"DEVPWREN\": %u,            \
        \"DEVPWRSTATUS\": %u,        \
        \"AUDSSPWREN\": %u,          \
        \"AUDSSPWRSTATUS\": %u,      \
        \"MEMPWREN\": %u,            \
        \"MEMPWRSTATUS\": %u,        \
        \"MEMRETCFG\": %u,           \
        \"SYSPWRSTATUS\": %u,        \
        \"SSRAMPWREN\": %u,          \
        \"SSRAMPWRST\": %u,          \
        \"SSRAMRETCFG\": %u,         \
        \"DEVPWREVENTEN\": %u,       \
        \"MEMPWREVENTEN\": %u,       \
        \"MMSOVERRIDE\": %u,         \
    ";

    char *pwrStr2 = "                \
        \"CPUPWRCTRL\": %u,          \
        \"CPUPWRSTATUS\": %u,        \
        \"PWRACKOVR\": %u,           \
        \"PWRCNTDEFVAL\": %u,        \
        \"EPURETCFG\": %u,           \
        \"CM4PWRCTRL\": %u,          \
        \"CM4PWRSTATE\": %u,         \
        \"VRCTRL\": %u,              \
        \"LEGACYVRLPOVR\": %u,       \
        \"VRSTATUS\": %u,            \
    ";

    char *pwrStr3 = "                \
        \"SRAMCTRL\": %u,            \
        \"ADCSTATUS\": %u,           \
        \"MRAMEXTCTRL\": %u,         \
        \"I3CISOCTRL\": %u,          \
        \"EMONCTRL\": %u,            \
        \"EMONCFG0\": %u,            \
        \"EMONCFG1\": %u,            \
        \"EMONCFG2\": %u,            \
        \"EMONCFG3\": %u,            \
        \"EMONCFG4\": %u,            \
        \"EMONCFG5\": %u,            \
        \"EMONCFG6\": %u,            \
        \"EMONCFG7\": %u,            \
        \"EMONCOUNT0\": %u,          \
        \"EMONCOUNT1\": %u,          \
        \"EMONCOUNT2\": %u,          \
        \"EMONCOUNT3\": %u,          \
        \"EMONCOUNT4\": %u,          \
        \"EMONCOUNT5\": %u,          \
        \"EMONCOUNT6\": %u,          \
        \"EMONCOUNT7\": %u,          \
        \"EMONSTATUS\": %u,          \
        \"FPIOEN0\": %u,             \
        \"FPIOEN1\": %u,             \
        \"FPIOEN2\": %u              \
    },";

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
    // Ignore the "string length is great than the length" warning
    // when -Wpedantic option is active.
    // The mcuCtrlStr overflows the 4095 char max at the FLASHRPROT6.
#pragma GCC diagnostic ignored "-Woverlength-strings"
#endif
    char *mcuCtrlStr = " \"MCUCTRL\":   \
    {                                   \
        \"CHIPPN\": %u,              \
        \"CHIPID0\": %u,             \
        \"CHIPID1\": %u,             \
        \"CHIPREV\": %u,             \
        \"VENDORID\": %u,            \
        \"SKU\": %u,                 \
        \"SKUOVERRIDE\": %u,         \
        \"DEBUGGER\": %u,            \
        \"ACRG\": %u,                \
        \"VREFGEN\": %u,             \
        \"VREFGEN2\": %u,            \
        \"VREFGEN3\": %u,            \
        \"VREFGEN4\": %u,            \
        \"VREFGEN5\": %u,            \
        \"VREFBUF\": %u,             \
        \"VRCTRL\": %u,              \
        \"LDOREG1\": %u,             \
        \"LDOREG2\": %u,             \
        \"HFRC\": %u,                \
        \"LFRC\": %u,                \
        \"BODCTRL\": %u,             \
        \"ADCPWRCTRL\": %u,          \
        \"ADCCAL\": %u,              \
        \"ADCBATTLOAD\": %u,         \
        \"XTALCTRL\": %u,            \
        \"XTALGENCTRL\": %u,         \
        \"XTALHSCTRL\": %u,          \
        \"MRAMCRYPTOPWRCTRL\": %u,   \
        \"D2ASPARE\": %u,            \
        \"BODISABLE\": %u,           \
        \"BOOTLOADER\": %u,          \
        \"SHADOWVALID\": %u,         \
        \"SCRATCH0\": %u,            \
        \"SCRATCH1\": %u,            \
        \"DBGR1\": %u,               \
        \"DBGR2\": %u,               \
        \"WICCONTROL\": %u,          \
        \"DBGCTRL\": %u,             \
        \"OTAPOINTER\": %u,          \
        \"APBDMACTRL\": %u,          \
        \"KEXTCLKSEL\": %u,          \
        \"SIMOBUCK0\": %u,           \
        \"SIMOBUCK1\": %u,           \
        \"SIMOBUCK2\": %u,           \
        \"SIMOBUCK3\": %u,           \
        \"SIMOBUCK4\": %u,           \
        \"SIMOBUCK6\": %u,           \
        \"SIMOBUCK7\": %u,           \
        \"SIMOBUCK8\": %u,           \
        \"SIMOBUCK9\": %u,           \
        \"SIMOBUCK10\": %u,          \
        \"SIMOBUCK11\": %u,          \
        \"D2ASPARE2\": %u,           \
        \"I3CPHYCTRL\": %u,          \
        \"PWRSW0\": %u,              \
        \"PWRSW1\": %u,              \
        \"USBRSTCTRL\": %u,          \
        \"FLASHWPROT0\": %u,         \
        \"FLASHWPROT1\": %u,         \
        \"FLASHWPROT2\": %u,         \
        \"FLASHWPROT3\": %u,         \
        \"FLASHRPROT0\": %u,         \
        \"FLASHRPROT1\": %u,         \
        \"FLASHRPROT2\": %u,         \
        \"FLASHRPROT3\": %u,         \
        \"SRAMWPROT0\": %u,          \
        \"SRAMWPROT1\": %u,          \
        \"SRAMWPROT2\": %u,          \
        \"SRAMWPROT3\": %u,          \
        \"SRAMRPROT0\": %u,          \
        \"SRAMRPROT1\": %u,          \
        \"SRAMRPROT2\": %u,          \
        \"SRAMRPROT3\": %u,          \
        \"SDIO0CTRL\": %u,           \
        \"SDIO1CTRL\": %u,           \
        \"PDMCTRL\": %u,             \
        \"SSRAMMISCCTRL\": %u,       \
        \"DISPSTATUS\": %u,          \
        \"CPUCFG\": %u,              \
        \"PLLCTL0\": %u,             \
        \"PLLDIV0\": %u,             \
        \"PLLDIV1\": %u,             \
        \"PLLSTAT\": %u,             \
        \"PLLMUXCTL\": %u,           \
        \"CM4CODEBASE\": %u,         \
        \"RADIOFINECNT\": %u,        \
        \"RADIOCLKNCNT\": %u         \
    },";

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

    char *systemctrlStr = " \"SYSCTRL\":     \
    {                                \
        \"ICTR\": %u,                \
        \"ACTLR\": %u,               \
        \"ICSR\": %u,                \
        \"VTOR\": %u,                \
        \"AIRCR\": %u,               \
        \"SCR\": %u,                 \
        \"CCR\": %u,                 \
        \"SHPR1\": %u,               \
        \"SHPR2\": %u,               \
        \"SHPR3\": %u,               \
        \"SHCSR\": %u,               \
        \"CFSR\": %u,                \
        \"HFSR\": %u,                \
        \"MMFAR\": %u,               \
        \"BFAR\": %u,                \
        \"CPACR\": %u,               \
        \"DEMCR\": %u,               \
        \"STIR\": %u,                \
        \"FPCCR\": %u,               \
        \"FPCAR\": %u,               \
        \"FPDSCR\": %u               \
    },";

    char *clkStr = " \"CLK\":        \
    {                                \
        \"OCTRL\": %u,               \
        \"CLKOUT\": %u,              \
        \"HFADJ\": %u,               \
        \"CLOCKENSTAT\": %u,         \
        \"CLOCKEN2STAT\": %u,        \
        \"CLOCKEN3STAT\": %u,        \
        \"MISC\": %u,                \
        \"LFRCCTRL\": %u,            \
        \"CLKGENSPARES\": %u,        \
        \"HFRCIDLECOUNTERS\": %u,    \
        \"MSPIIOCLKCTRL\": %u,       \
        \"CLKCTRL\": %u              \
    },";

    char *stimerStr = " \"STIMER\":    \
    {                                \
        \"STCFG\": %u,               \
        \"STTMR\": %u,               \
        \"SCAPCTRL0\": %u,           \
        \"SCAPCTRL1\": %u,           \
        \"SCAPCTRL2\": %u,           \
        \"SCAPCTRL3\": %u,           \
        \"SCMPR0\": %u,              \
        \"SCMPR1\": %u,              \
        \"SCMPR2\": %u,              \
        \"SCMPR3\": %u,              \
        \"SCMPR4\": %u,              \
        \"SCMPR5\": %u,              \
        \"SCMPR6\": %u,              \
        \"SCMPR7\": %u,              \
        \"SCAPT0\": %u,              \
        \"SCAPT1\": %u,              \
        \"SCAPT2\": %u,              \
        \"SCAPT3\": %u,              \
        \"SNVR0\": %u,               \
        \"SNVR1\": %u,               \
        \"HALSTATES\": %u,           \
        \"STMINTEN\": %u,            \
        \"STMINTSTAT\": %u,          \
        \"STMINTCLR\": %u,           \
        \"STMINTSET\": %u            \
    },";

    char *timerStr = " \"TIMER\":    \
    {                                \
        \"CTRL\": %u,                \
        \"STATUS\": %u,              \
        \"GLOBEN\": %u,              \
        \"INTEN\": %u,               \
        \"INTSTAT\": %u,             \
        \"INTCLR\": %u,              \
        \"INTSET\": %u,              \
        \"CTRL0\": %u,               \
        \"TIMER0\": %u,              \
        \"TMR0CMP0\": %u,            \
        \"TMR0CMP1\": %u,            \
        \"MODE0\": %u,               \
        \"TMR0LMTVAL\": %u,          \
        \"CTRL1\": %u,               \
        \"TIMER1\": %u,              \
        \"TMR1CMP0\": %u,            \
        \"TMR1CMP1\": %u,            \
        \"MODE1\": %u,               \
        \"TMR1LMTVAL\": %u,          \
        \"CTRL2\": %u,               \
        \"TIMER2\": %u,              \
        \"TMR2CMP0\": %u,            \
        \"TMR2CMP1\": %u,            \
        \"MODE2\": %u,               \
        \"TMR2LMTVAL\": %u,          \
        \"CTRL3\": %u,               \
        \"TIMER3\": %u,              \
        \"TMR3CMP0\": %u,            \
        \"TMR3CMP1\": %u,            \
        \"MODE3\": %u,               \
        \"TMR3LMTVAL\": %u,          \
        \"CTRL4\": %u,               \
        \"TIMER4\": %u,              \
        \"TMR4CMP0\": %u,            \
        \"TMR4CMP1\": %u,            \
        \"MODE4\": %u,               \
        \"TMR4LMTVAL\": %u,          \
        \"CTRL5\": %u,               \
        \"TIMER5\": %u,              \
        \"TMR5CMP0\": %u,            \
        \"TMR5CMP1\": %u,            \
        \"MODE5\": %u,               \
        \"TMR5LMTVAL\": %u,          \
        \"CTRL6\": %u,               \
        \"TIMER6\": %u,              \
        \"TMR6CMP0\": %u,            \
        \"TMR6CMP1\": %u,            \
        \"MODE6\": %u,               \
        \"TMR6LMTVAL\": %u,          \
        \"CTRL7\": %u,               \
        \"TIMER7\": %u,              \
        \"TMR7CMP0\": %u,            \
        \"TMR7CMP1\": %u,            \
        \"MODE7\": %u,               \
        \"TMR7LMTVAL\": %u,          \
        \"CTRL8\": %u,               \
        \"TIMER8\": %u,              \
        \"TMR8CMP0\": %u,            \
        \"TMR8CMP1\": %u,            \
        \"MODE8\": %u,               \
        \"TMR8LMTVAL\": %u,          \
        \"CTRL9\": %u,               \
        \"TIMER9\": %u,              \
        \"TMR9CMP0\": %u,            \
        \"TMR9CMP1\": %u,            \
        \"MODE9\": %u,               \
        \"TMR9LMTVAL\": %u,          \
        \"CTRL10\": %u,              \
        \"TIMER10\": %u,             \
        \"TMR10CMP0\": %u,           \
        \"TMR10CMP1\": %u,           \
        \"MODE10\": %u,              \
        \"TMR10LMTVAL\": %u,         \
        \"CTRL11\": %u,              \
        \"TIMER11\": %u,             \
        \"TMR11CMP0\": %u,           \
        \"TMR11CMP1\": %u,           \
        \"MODE11\": %u,              \
        \"TMR11LMTVAL\": %u,         \
        \"CTRL12\": %u,              \
        \"TIMER12\": %u,             \
        \"TMR12CMP0\": %u,           \
        \"TMR12CMP1\": %u,           \
        \"MODE12\": %u,              \
        \"TMR12LMTVAL\": %u,         \
        \"CTRL13\": %u,              \
        \"TIMER13\": %u,             \
        \"TMR13CMP0\": %u,           \
        \"TMR13CMP1\": %u,           \
        \"MODE13\": %u,              \
        \"TMR13LMTVAL\": %u,         \
        \"CTRL14\": %u,              \
        \"TIMER14\": %u,             \
        \"TMR14CMP0\": %u,           \
        \"TMR14CMP1\": %u,           \
        \"MODE14\": %u,              \
        \"TMR14LMTVAL\": %u,         \
        \"CTRL15\": %u,              \
        \"TIMER15\": %u,             \
        \"TMR15CMP0\": %u,           \
        \"TMR15CMP1\": %u,           \
        \"MODE15\": %u,              \
        \"TMR15LMTVAL\": %u,         \
        \"TIMERSPARES\": %u          \
    },";

    char *pdmStr = " \"PDM\":           \
    {                                   \
        \"CTRL\": %u,                \
        \"CORECFG0\": %u,            \
        \"CORECFG1\": %u,            \
        \"CORECTRL\": %u,            \
        \"FIFOCNT\": %u,             \
        \"FIFOREAD\": %u,            \
        \"FIFOFLUSH\": %u,           \
        \"FIFOTHR\": %u,             \
        \"INTEN\": %u,               \
        \"INTSTAT\": %u,             \
        \"INTCLR\": %u,              \
        \"INTSET\": %u,              \
        \"DMATRIGEN\": %u,           \
        \"DMATRIGSTAT\": %u,         \
        \"DMACFG\": %u,              \
        \"DMATARGADDR\": %u,         \
        \"DMASTAT\": %u,             \
        \"DMATARGADDRNEXT\": %u,     \
        \"DMATOTCOUNTNEXT\": %u,     \
        \"DMAENNEXTCTRL\": %u,       \
        \"DMATOTCOUNT\": %u          \
    }}\n";

    //
    //To provide the single snapshot, we need enable the printf via UART
    //
    am_bsp_uart_printf_enable();

    am_util_stdio_printf(pwrStr1, am_util_pp1[uNumber].bSingle,
                            am_util_pp1[uNumber].uSnapShot,             \
                            am_util_pp1[uNumber].P_MCUPERFREQ,          \
                            am_util_pp1[uNumber].P_DEVPWREN,            \
                            am_util_pp1[uNumber].P_DEVPWRSTATUS,        \
                            am_util_pp1[uNumber].P_AUDSSPWREN,          \
                            am_util_pp1[uNumber].P_AUDSSPWRSTATUS,      \
                            am_util_pp1[uNumber].P_MEMPWREN,            \
                            am_util_pp1[uNumber].P_MEMPWRSTATUS,        \
                            am_util_pp1[uNumber].P_MEMRETCFG,           \
                            am_util_pp1[uNumber].P_SYSPWRSTATUS,        \
                            am_util_pp1[uNumber].P_SSRAMPWREN,          \
                            am_util_pp1[uNumber].P_SSRAMPWRST,          \
                            am_util_pp1[uNumber].P_SSRAMRETCFG,         \
                            am_util_pp1[uNumber].P_DEVPWREVENTEN,       \
                            am_util_pp1[uNumber].P_MEMPWREVENTEN,       \
                            am_util_pp1[uNumber].P_MMSOVERRIDE);        \

    am_util_stdio_printf(pwrStr2, am_util_pp1[uNumber].P_CPUPWRCTRL,    \
                            am_util_pp1[uNumber].P_CPUPWRSTATUS,        \
                            am_util_pp1[uNumber].P_PWRACKOVR,           \
                            am_util_pp1[uNumber].P_PWRCNTDEFVAL,        \
                            am_util_pp1[uNumber].P_EPURETCFG,           \
                            am_util_pp1[uNumber].P_CM4PWRCTRL,          \
                            am_util_pp1[uNumber].P_CM4PWRSTATE,         \
                            am_util_pp1[uNumber].P_VRCTRL,              \
                            am_util_pp1[uNumber].P_LEGACYVRLPOVR,       \
                            am_util_pp1[uNumber].P_VRSTATUS);           \

    am_util_stdio_printf(pwrStr3, am_util_pp1[uNumber].P_SRAMCTRL,      \
                            am_util_pp1[uNumber].P_ADCSTATUS,           \
                            am_util_pp1[uNumber].P_MRAMEXTCTRL,         \
                            am_util_pp1[uNumber].P_I3CISOCTRL,          \
                            am_util_pp1[uNumber].P_EMONCTRL,            \
                            am_util_pp1[uNumber].P_EMONCFG0,            \
                            am_util_pp1[uNumber].P_EMONCFG1,            \
                            am_util_pp1[uNumber].P_EMONCFG2,            \
                            am_util_pp1[uNumber].P_EMONCFG3,            \
                            am_util_pp1[uNumber].P_EMONCFG4,            \
                            am_util_pp1[uNumber].P_EMONCFG5,            \
                            am_util_pp1[uNumber].P_EMONCFG6,            \
                            am_util_pp1[uNumber].P_EMONCFG7,            \
                            am_util_pp1[uNumber].P_EMONCOUNT0,          \
                            am_util_pp1[uNumber].P_EMONCOUNT1,          \
                            am_util_pp1[uNumber].P_EMONCOUNT2,          \
                            am_util_pp1[uNumber].P_EMONCOUNT3,          \
                            am_util_pp1[uNumber].P_EMONCOUNT4,          \
                            am_util_pp1[uNumber].P_EMONCOUNT5,          \
                            am_util_pp1[uNumber].P_EMONCOUNT6,          \
                            am_util_pp1[uNumber].P_EMONCOUNT7,          \
                            am_util_pp1[uNumber].P_EMONSTATUS,          \
                            am_util_pp1[uNumber].P_FPIOEN0,             \
                            am_util_pp1[uNumber].P_FPIOEN1,             \
                            am_util_pp1[uNumber].P_FPIOEN2);

    am_util_stdio_printf(mcuCtrlStr, am_util_pp2[uNumber].M_CHIPPN,     \
                            am_util_pp2[uNumber].M_CHIPID0,             \
                            am_util_pp2[uNumber].M_CHIPID1,             \
                            am_util_pp2[uNumber].M_CHIPREV,             \
                            am_util_pp2[uNumber].M_VENDORID,            \
                            am_util_pp2[uNumber].M_SKU,                 \
                            am_util_pp2[uNumber].M_SKUOVERRIDE,         \
                            am_util_pp2[uNumber].M_DEBUGGER,            \
                            am_util_pp2[uNumber].M_ACRG,                \
                            am_util_pp2[uNumber].M_VREFGEN,             \
                            am_util_pp2[uNumber].M_VREFGEN2,            \
                            am_util_pp2[uNumber].M_VREFGEN3,            \
                            am_util_pp2[uNumber].M_VREFGEN4,            \
                            am_util_pp2[uNumber].M_VREFGEN5,            \
                            am_util_pp2[uNumber].M_VREFBUF,             \
                            am_util_pp2[uNumber].M_VRCTRL,              \
                            am_util_pp2[uNumber].M_LDOREG1,             \
                            am_util_pp2[uNumber].M_LDOREG2,             \
                            am_util_pp2[uNumber].M_HFRC,                \
                            am_util_pp2[uNumber].M_LFRC,                \
                            am_util_pp2[uNumber].M_BODCTRL,             \
                            am_util_pp2[uNumber].M_ADCPWRCTRL,          \
                            am_util_pp2[uNumber].M_ADCCAL,              \
                            am_util_pp2[uNumber].M_ADCBATTLOAD,         \
                            am_util_pp2[uNumber].M_XTALCTRL,            \
                            am_util_pp2[uNumber].M_XTALGENCTRL,         \
                            am_util_pp2[uNumber].M_XTALHSCTRL,          \
                            am_util_pp2[uNumber].M_MRAMCRYPTOPWRCTRL,   \
                            am_util_pp2[uNumber].M_D2ASPARE,            \
                            am_util_pp2[uNumber].M_BODISABLE,           \
                            am_util_pp2[uNumber].M_BOOTLOADER,          \
                            am_util_pp2[uNumber].M_SHADOWVALID,         \
                            am_util_pp2[uNumber].M_SCRATCH0,            \
                            am_util_pp2[uNumber].M_SCRATCH1,            \
                            am_util_pp2[uNumber].M_DBGR1,               \
                            am_util_pp2[uNumber].M_DBGR2,               \
                            am_util_pp2[uNumber].M_WICCONTROL,          \
                            am_util_pp2[uNumber].M_DBGCTRL,             \
                            am_util_pp2[uNumber].M_OTAPOINTER,          \
                            am_util_pp2[uNumber].M_APBDMACTRL,          \
                            am_util_pp2[uNumber].M_KEXTCLKSEL,          \
                            am_util_pp2[uNumber].M_SIMOBUCK0,           \
                            am_util_pp2[uNumber].M_SIMOBUCK1,           \
                            am_util_pp2[uNumber].M_SIMOBUCK2,           \
                            am_util_pp2[uNumber].M_SIMOBUCK3,           \
                            am_util_pp2[uNumber].M_SIMOBUCK4,           \
                            am_util_pp2[uNumber].M_SIMOBUCK6,           \
                            am_util_pp2[uNumber].M_SIMOBUCK7,           \
                            am_util_pp2[uNumber].M_SIMOBUCK8,           \
                            am_util_pp2[uNumber].M_SIMOBUCK9,           \
                            am_util_pp2[uNumber].M_SIMOBUCK10,          \
                            am_util_pp2[uNumber].M_SIMOBUCK11,          \
                            am_util_pp2[uNumber].M_D2ASPARE2,           \
                            am_util_pp2[uNumber].M_I3CPHYCTRL,          \
                            am_util_pp2[uNumber].M_PWRSW0,              \
                            am_util_pp2[uNumber].M_PWRSW1,              \
                            am_util_pp2[uNumber].M_USBRSTCTRL,          \
                            am_util_pp2[uNumber].M_FLASHWPROT0,         \
                            am_util_pp2[uNumber].M_FLASHWPROT1,         \
                            am_util_pp2[uNumber].M_FLASHWPROT2,         \
                            am_util_pp2[uNumber].M_FLASHWPROT3,         \
                            am_util_pp2[uNumber].M_FLASHRPROT0,         \
                            am_util_pp2[uNumber].M_FLASHRPROT1,         \
                            am_util_pp2[uNumber].M_FLASHRPROT2,         \
                            am_util_pp2[uNumber].M_FLASHRPROT3,         \
                            am_util_pp2[uNumber].M_SRAMWPROT0,          \
                            am_util_pp2[uNumber].M_SRAMWPROT1,          \
                            am_util_pp2[uNumber].M_SRAMWPROT2,          \
                            am_util_pp2[uNumber].M_SRAMWPROT3,          \
                            am_util_pp2[uNumber].M_SRAMRPROT0,          \
                            am_util_pp2[uNumber].M_SRAMRPROT1,          \
                            am_util_pp2[uNumber].M_SRAMRPROT2,          \
                            am_util_pp2[uNumber].M_SRAMRPROT3,          \
                            am_util_pp2[uNumber].M_SDIO0CTRL,           \
                            am_util_pp2[uNumber].M_SDIO1CTRL,           \
                            am_util_pp2[uNumber].M_PDMCTRL,             \
                            am_util_pp2[uNumber].M_SSRAMMISCCTRL,       \
                            am_util_pp2[uNumber].M_DISPSTATUS,          \
                            am_util_pp2[uNumber].M_CPUCFG,              \
                            am_util_pp2[uNumber].M_PLLCTL0,             \
                            am_util_pp2[uNumber].M_PLLDIV0,             \
                            am_util_pp2[uNumber].M_PLLDIV1,             \
                            am_util_pp2[uNumber].M_PLLSTAT,             \
                            am_util_pp2[uNumber].M_PLLMUXCTL,           \
                            am_util_pp2[uNumber].M_CM4CODEBASE,         \
                            am_util_pp2[uNumber].M_RADIOFINECNT,        \
                            am_util_pp2[uNumber].M_RADIOCLKNCNT);

    am_util_stdio_printf(clkStr, am_util_pp3[uNumber].C_OCTRL,          \
                            am_util_pp3[uNumber].C_CLKOUT,              \
                            am_util_pp3[uNumber].C_HFADJ,               \
                            am_util_pp3[uNumber].C_CLOCKENSTAT,         \
                            am_util_pp3[uNumber].C_CLOCKEN2STAT,        \
                            am_util_pp3[uNumber].C_CLOCKEN3STAT,        \
                            am_util_pp3[uNumber].C_MISC,                \
                            am_util_pp3[uNumber].C_LFRCCTRL,            \
                            am_util_pp3[uNumber].C_CLKGENSPARES,        \
                            am_util_pp3[uNumber].C_HFRCIDLECOUNTERS,    \
                            am_util_pp3[uNumber].C_MSPIIOCLKCTRL,       \
                            am_util_pp3[uNumber].C_CLKCTRL);

   am_util_stdio_printf(stimerStr, am_util_pp3[uNumber].ST_STCFG,        \
                            am_util_pp3[uNumber].ST_STTMR,               \
                            am_util_pp3[uNumber].ST_SCAPCTRL0,           \
                            am_util_pp3[uNumber].ST_SCAPCTRL1,           \
                            am_util_pp3[uNumber].ST_SCAPCTRL2,           \
                            am_util_pp3[uNumber].ST_SCAPCTRL3,           \
                            am_util_pp3[uNumber].ST_SCMPR0,              \
                            am_util_pp3[uNumber].ST_SCMPR1,              \
                            am_util_pp3[uNumber].ST_SCMPR2,              \
                            am_util_pp3[uNumber].ST_SCMPR3,              \
                            am_util_pp3[uNumber].ST_SCMPR4,              \
                            am_util_pp3[uNumber].ST_SCMPR5,              \
                            am_util_pp3[uNumber].ST_SCMPR6,              \
                            am_util_pp3[uNumber].ST_SCMPR7,              \
                            am_util_pp3[uNumber].ST_SCAPT0,              \
                            am_util_pp3[uNumber].ST_SCAPT1,              \
                            am_util_pp3[uNumber].ST_SCAPT2,              \
                            am_util_pp3[uNumber].ST_SCAPT3,              \
                            am_util_pp3[uNumber].ST_SNVR0,               \
                            am_util_pp3[uNumber].ST_SNVR1,               \
                            am_util_pp3[uNumber].ST_HALSTATES,           \
                            am_util_pp3[uNumber].ST_STMINTEN,            \
                            am_util_pp3[uNumber].ST_STMINTSTAT,          \
                            am_util_pp3[uNumber].ST_STMINTCLR,           \
                            am_util_pp3[uNumber].ST_STMINTSET);

    am_util_stdio_printf(timerStr, am_util_pp3[uNumber].T_CTRL,         \
                            am_util_pp3[uNumber].T_STATUS,              \
                            am_util_pp3[uNumber].T_GLOBEN,              \
                            am_util_pp3[uNumber].T_INTEN,               \
                            am_util_pp3[uNumber].T_INTSTAT,             \
                            am_util_pp3[uNumber].T_INTCLR,              \
                            am_util_pp3[uNumber].T_INTSET,              \
                            am_util_pp3[uNumber].T_CTRL0,               \
                            am_util_pp3[uNumber].T_TIMER0,              \
                            am_util_pp3[uNumber].T_TMR0CMP0,            \
                            am_util_pp3[uNumber].T_TMR0CMP1,            \
                            am_util_pp3[uNumber].T_MODE0,               \
                            am_util_pp3[uNumber].T_TMR0LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL1,               \
                            am_util_pp3[uNumber].T_TIMER1,              \
                            am_util_pp3[uNumber].T_TMR1CMP0,            \
                            am_util_pp3[uNumber].T_TMR1CMP1,            \
                            am_util_pp3[uNumber].T_MODE1,               \
                            am_util_pp3[uNumber].T_TMR1LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL2,               \
                            am_util_pp3[uNumber].T_TIMER2,              \
                            am_util_pp3[uNumber].T_TMR2CMP0,            \
                            am_util_pp3[uNumber].T_TMR2CMP1,            \
                            am_util_pp3[uNumber].T_MODE2,               \
                            am_util_pp3[uNumber].T_TMR2LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL3,               \
                            am_util_pp3[uNumber].T_TIMER3,              \
                            am_util_pp3[uNumber].T_TMR3CMP0,            \
                            am_util_pp3[uNumber].T_TMR3CMP1,            \
                            am_util_pp3[uNumber].T_MODE3,               \
                            am_util_pp3[uNumber].T_TMR3LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL4,               \
                            am_util_pp3[uNumber].T_TIMER4,              \
                            am_util_pp3[uNumber].T_TMR4CMP0,            \
                            am_util_pp3[uNumber].T_TMR4CMP1,            \
                            am_util_pp3[uNumber].T_MODE4,               \
                            am_util_pp3[uNumber].T_TMR4LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL5,               \
                            am_util_pp3[uNumber].T_TIMER5,              \
                            am_util_pp3[uNumber].T_TMR5CMP0,            \
                            am_util_pp3[uNumber].T_TMR5CMP1,            \
                            am_util_pp3[uNumber].T_MODE5,               \
                            am_util_pp3[uNumber].T_TMR5LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL6,               \
                            am_util_pp3[uNumber].T_TIMER6,              \
                            am_util_pp3[uNumber].T_TMR6CMP0,            \
                            am_util_pp3[uNumber].T_TMR6CMP1,            \
                            am_util_pp3[uNumber].T_MODE6,               \
                            am_util_pp3[uNumber].T_TMR6LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL7,               \
                            am_util_pp3[uNumber].T_TIMER7,              \
                            am_util_pp3[uNumber].T_TMR7CMP0,            \
                            am_util_pp3[uNumber].T_TMR7CMP1,            \
                            am_util_pp3[uNumber].T_MODE7,               \
                            am_util_pp3[uNumber].T_TMR7LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL8,               \
                            am_util_pp3[uNumber].T_TIMER8,              \
                            am_util_pp3[uNumber].T_TMR8CMP0,            \
                            am_util_pp3[uNumber].T_TMR8CMP1,            \
                            am_util_pp3[uNumber].T_MODE8,               \
                            am_util_pp3[uNumber].T_TMR8LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL9,               \
                            am_util_pp3[uNumber].T_TIMER9,              \
                            am_util_pp3[uNumber].T_TMR9CMP0,            \
                            am_util_pp3[uNumber].T_TMR9CMP1,            \
                            am_util_pp3[uNumber].T_MODE9,               \
                            am_util_pp3[uNumber].T_TMR9LMTVAL,          \
                            am_util_pp3[uNumber].T_CTRL10,              \
                            am_util_pp3[uNumber].T_TIMER10,             \
                            am_util_pp3[uNumber].T_TMR10CMP0,           \
                            am_util_pp3[uNumber].T_TMR10CMP1,           \
                            am_util_pp3[uNumber].T_MODE10,              \
                            am_util_pp3[uNumber].T_TMR10LMTVAL,         \
                            am_util_pp3[uNumber].T_CTRL11,              \
                            am_util_pp3[uNumber].T_TIMER11,             \
                            am_util_pp3[uNumber].T_TMR11CMP0,           \
                            am_util_pp3[uNumber].T_TMR11CMP1,           \
                            am_util_pp3[uNumber].T_MODE11,              \
                            am_util_pp3[uNumber].T_TMR11LMTVAL,         \
                            am_util_pp3[uNumber].T_CTRL12,              \
                            am_util_pp3[uNumber].T_TIMER12,             \
                            am_util_pp3[uNumber].T_TMR12CMP0,           \
                            am_util_pp3[uNumber].T_TMR12CMP1,           \
                            am_util_pp3[uNumber].T_MODE12,              \
                            am_util_pp3[uNumber].T_TMR12LMTVAL,         \
                            am_util_pp3[uNumber].T_CTRL13,              \
                            am_util_pp3[uNumber].T_TIMER13,             \
                            am_util_pp3[uNumber].T_TMR13CMP0,           \
                            am_util_pp3[uNumber].T_TMR13CMP1,           \
                            am_util_pp3[uNumber].T_MODE13,              \
                            am_util_pp3[uNumber].T_TMR13LMTVAL,         \
                            am_util_pp3[uNumber].T_CTRL14,              \
                            am_util_pp3[uNumber].T_TIMER14,             \
                            am_util_pp3[uNumber].T_TMR14CMP0,           \
                            am_util_pp3[uNumber].T_TMR14CMP1,           \
                            am_util_pp3[uNumber].T_MODE14,              \
                            am_util_pp3[uNumber].T_TMR14LMTVAL,         \
                            am_util_pp3[uNumber].T_CTRL15,              \
                            am_util_pp3[uNumber].T_TIMER15,             \
                            am_util_pp3[uNumber].T_TMR15CMP0,           \
                            am_util_pp3[uNumber].T_TMR15CMP1,           \
                            am_util_pp3[uNumber].T_MODE15,              \
                            am_util_pp3[uNumber].T_TMR15LMTVAL,         \
                            am_util_pp3[uNumber].T_TIMERSPARES);

    am_util_stdio_printf(systemctrlStr, am_util_pp4[uNumber].SC_ICTR,    \
                            am_util_pp4[uNumber].SC_ACTLR,               \
                            am_util_pp4[uNumber].SC_ICSR,                \
                            am_util_pp4[uNumber].SC_VTOR,                \
                            am_util_pp4[uNumber].SC_AIRCR,               \
                            am_util_pp4[uNumber].SC_SCR,                 \
                            am_util_pp4[uNumber].SC_CCR,                 \
                            am_util_pp4[uNumber].SC_SHPR1,               \
                            am_util_pp4[uNumber].SC_SHPR2,               \
                            am_util_pp4[uNumber].SC_SHPR3,               \
                            am_util_pp4[uNumber].SC_SHCSR,               \
                            am_util_pp4[uNumber].SC_CFSR,                \
                            am_util_pp4[uNumber].SC_HFSR,                \
                            am_util_pp4[uNumber].SC_MMFAR,               \
                            am_util_pp4[uNumber].SC_BFAR,                \
                            am_util_pp4[uNumber].SC_CPACR,               \
                            am_util_pp4[uNumber].SC_DEMCR,               \
                            am_util_pp4[uNumber].SC_STIR,                \
                            am_util_pp4[uNumber].SC_FPCCR,               \
                            am_util_pp4[uNumber].SC_FPCAR,               \
                            am_util_pp4[uNumber].SC_FPDSCR);

    am_util_stdio_printf(pdmStr, am_util_pp5[uNumber].PDM_CTRL,           \
                            am_util_pp5[uNumber].PDM_CORECFG0,            \
                            am_util_pp5[uNumber].PDM_CORECFG1,            \
                            am_util_pp5[uNumber].PDM_CORECTRL,            \
                            am_util_pp5[uNumber].PDM_FIFOCNT,             \
                            am_util_pp5[uNumber].PDM_FIFOREAD,            \
                            am_util_pp5[uNumber].PDM_FIFOFLUSH,           \
                            am_util_pp5[uNumber].PDM_FIFOTHR,             \
                            am_util_pp5[uNumber].PDM_INTEN,               \
                            am_util_pp5[uNumber].PDM_INTSTAT,             \
                            am_util_pp5[uNumber].PDM_INTCLR,              \
                            am_util_pp5[uNumber].PDM_INTSET,              \
                            am_util_pp5[uNumber].PDM_DMATRIGEN,           \
                            am_util_pp5[uNumber].PDM_DMATRIGSTAT,         \
                            am_util_pp5[uNumber].PDM_DMACFG,              \
                            am_util_pp5[uNumber].PDM_DMATARGADDR,         \
                            am_util_pp5[uNumber].PDM_DMASTAT,             \
                            am_util_pp5[uNumber].PDM_DMATARGADDRNEXT,     \
                            am_util_pp5[uNumber].PDM_DMATOTCOUNTNEXT,     \
                            am_util_pp5[uNumber].PDM_DMAENNEXTCTRL,       \
                            am_util_pp5[uNumber].PDM_DMATOTCOUNT);

    //
    //Now, we can disable the UART to provide minimized impact to the system
    //
    am_bsp_uart_printf_disable();

}

//*****************************************************************************
//
// This is the customer facing API function calls to invoke the snapshot
// bUseMemory is reserved for future application
//
//*****************************************************************************
void
am_util_pp_snapshot(bool bSingleShot, uint32_t uNumber, bool bStreamNow)
{

    if (bStreamNow)
    {
        if (bCaptured[uNumber])
        {
            am_util_stdio_printf("Outputting the captured power snapshot!\n");

            //
            //Step 4: Stream this to the UART for Python to capture
            //
            am_util_print_JSON(uNumber);
            bCaptured[uNumber] = false;
        }
        else
        {
            am_util_stdio_printf("No snapshot captured or repeated snapshot requested while in single shot mode!\n");
        }

        return;
    }
    else
    {
        //
        //Step 1: gather the system information
        //Function block 1: PWRCTRL
        //
        am_util_pp1[uNumber].bSingle = bSingleShot;
        if (bSingleShot && uNumber == am_util_pp1[uNumber].uSnapShot)
        {
            bCaptured[uNumber] = false;
            return;
        }

        am_util_pp1[uNumber].uSnapShot = uNumber;
        am_util_pp1[uNumber].P_MCUPERFREQ          = PWRCTRL->MCUPERFREQ;
        am_util_pp1[uNumber].P_DEVPWREN            = PWRCTRL->DEVPWREN;
        am_util_pp1[uNumber].P_DEVPWRSTATUS        = PWRCTRL->DEVPWRSTATUS;
        am_util_pp1[uNumber].P_AUDSSPWREN          = PWRCTRL->AUDSSPWREN;
        am_util_pp1[uNumber].P_AUDSSPWRSTATUS      = PWRCTRL->AUDSSPWRSTATUS;
        am_util_pp1[uNumber].P_MEMPWREN            = PWRCTRL->MEMPWREN;
        am_util_pp1[uNumber].P_MEMPWRSTATUS        = PWRCTRL->MEMPWRSTATUS;
        am_util_pp1[uNumber].P_MEMRETCFG           = PWRCTRL->MEMRETCFG;
        am_util_pp1[uNumber].P_SYSPWRSTATUS        = PWRCTRL->SYSPWRSTATUS;
        am_util_pp1[uNumber].P_SSRAMPWREN          = PWRCTRL->SSRAMPWREN;
        am_util_pp1[uNumber].P_SSRAMPWRST          = PWRCTRL->SSRAMPWRST;
        am_util_pp1[uNumber].P_SSRAMRETCFG         = PWRCTRL->SSRAMRETCFG;
        am_util_pp1[uNumber].P_DEVPWREVENTEN       = PWRCTRL->DEVPWREVENTEN;
        am_util_pp1[uNumber].P_MEMPWREVENTEN       = PWRCTRL->MEMPWREVENTEN;
        am_util_pp1[uNumber].P_MMSOVERRIDE         = PWRCTRL->MMSOVERRIDE;
        am_util_pp1[uNumber].P_CPUPWRCTRL          = PWRCTRL->CPUPWRCTRL;
        am_util_pp1[uNumber].P_CPUPWRSTATUS        = PWRCTRL->CPUPWRSTATUS;
        am_util_pp1[uNumber].P_PWRACKOVR           = PWRCTRL->PWRACKOVR;
        am_util_pp1[uNumber].P_PWRCNTDEFVAL        = PWRCTRL->PWRCNTDEFVAL;
        am_util_pp1[uNumber].P_EPURETCFG           = PWRCTRL->EPURETCFG;
        am_util_pp1[uNumber].P_CM4PWRCTRL          = PWRCTRL->CM4PWRCTRL;
        am_util_pp1[uNumber].P_CM4PWRSTATE         = PWRCTRL->CM4PWRSTATE;
        am_util_pp1[uNumber].P_VRCTRL              = PWRCTRL->VRCTRL;
        am_util_pp1[uNumber].P_LEGACYVRLPOVR       = PWRCTRL->LEGACYVRLPOVR;
        am_util_pp1[uNumber].P_VRSTATUS            = PWRCTRL->VRSTATUS;
        am_util_pp1[uNumber].P_SRAMCTRL            = PWRCTRL->SRAMCTRL;
        am_util_pp1[uNumber].P_ADCSTATUS           = PWRCTRL->ADCSTATUS;
        am_util_pp1[uNumber].P_MRAMEXTCTRL         = PWRCTRL->MRAMEXTCTRL;
        am_util_pp1[uNumber].P_I3CISOCTRL          = PWRCTRL->I3CISOCTRL;
        am_util_pp1[uNumber].P_EMONCTRL            = PWRCTRL->EMONCTRL;
        am_util_pp1[uNumber].P_EMONCFG0            = PWRCTRL->EMONCFG0;
        am_util_pp1[uNumber].P_EMONCFG1            = PWRCTRL->EMONCFG1;
        am_util_pp1[uNumber].P_EMONCFG2            = PWRCTRL->EMONCFG2;
        am_util_pp1[uNumber].P_EMONCFG3            = PWRCTRL->EMONCFG3;
        am_util_pp1[uNumber].P_EMONCFG4            = PWRCTRL->EMONCFG4;
        am_util_pp1[uNumber].P_EMONCFG5            = PWRCTRL->EMONCFG5;
        am_util_pp1[uNumber].P_EMONCFG6            = PWRCTRL->EMONCFG6;
        am_util_pp1[uNumber].P_EMONCFG7            = PWRCTRL->EMONCFG7;
        am_util_pp1[uNumber].P_EMONCOUNT0          = PWRCTRL->EMONCOUNT0;
        am_util_pp1[uNumber].P_EMONCOUNT1          = PWRCTRL->EMONCOUNT1;
        am_util_pp1[uNumber].P_EMONCOUNT2          = PWRCTRL->EMONCOUNT2;
        am_util_pp1[uNumber].P_EMONCOUNT3          = PWRCTRL->EMONCOUNT3;
        am_util_pp1[uNumber].P_EMONCOUNT4          = PWRCTRL->EMONCOUNT4;
        am_util_pp1[uNumber].P_EMONCOUNT5          = PWRCTRL->EMONCOUNT5;
        am_util_pp1[uNumber].P_EMONCOUNT6          = PWRCTRL->EMONCOUNT6;
        am_util_pp1[uNumber].P_EMONCOUNT7          = PWRCTRL->EMONCOUNT7;
        am_util_pp1[uNumber].P_EMONSTATUS          = PWRCTRL->EMONSTATUS;


        //
        //append the FPIO here
        //
        am_util_pp1[uNumber].P_FPIOEN0             = FPIO-> EN0;
        am_util_pp1[uNumber].P_FPIOEN1             = FPIO-> EN1;
        am_util_pp1[uNumber].P_FPIOEN2             = FPIO-> EN2;


        //
        // Function block 2: MCU_CTRL
        //
        am_util_pp2[uNumber].M_CHIPPN              = MCUCTRL->CHIPPN;
        am_util_pp2[uNumber].M_CHIPID0             = MCUCTRL->CHIPID0;
        am_util_pp2[uNumber].M_CHIPID1             = MCUCTRL->CHIPID1;
        am_util_pp2[uNumber].M_CHIPREV             = MCUCTRL->CHIPREV;
        am_util_pp2[uNumber].M_VENDORID            = MCUCTRL->VENDORID;
        am_util_pp2[uNumber].M_SKU                 = MCUCTRL->SKU;
        am_util_pp2[uNumber].M_SKUOVERRIDE         = MCUCTRL->SKUOVERRIDE;
        am_util_pp2[uNumber].M_DEBUGGER            = MCUCTRL->DEBUGGER;
        am_util_pp2[uNumber].M_ACRG                = MCUCTRL->ACRG;
        am_util_pp2[uNumber].M_VREFGEN             = MCUCTRL_VREFGEN;
        am_util_pp2[uNumber].M_VREFGEN2            = MCUCTRL->VREFGEN2;
        //am_util_pp2[uNumber].M_VREFGEN3            = MCUCTRL_VREFGEN3;
        am_util_pp2[uNumber].M_VREFGEN3            = MCUCTRL->VREFGEN3;
        am_util_pp2[uNumber].M_VREFGEN4            = MCUCTRL->VREFGEN4;
        //am_util_pp2[uNumber].M_VREFGEN5            = MCUCTRL_VREFGEN5;
        am_util_pp2[uNumber].M_VREFGEN5            = MCUCTRL->VREFGEN5;
        am_util_pp2[uNumber].M_VREFBUF             = MCUCTRL->VREFBUF;
        am_util_pp2[uNumber].M_VRCTRL              = MCUCTRL->VRCTRL;
        am_util_pp2[uNumber].M_LDOREG1             = MCUCTRL->LDOREG1;
        am_util_pp2[uNumber].M_LDOREG2             = MCUCTRL->LDOREG2;
        //am_util_pp2[uNumber].M_HFRC                = MCUCTRL_HFRC;
        am_util_pp2[uNumber].M_HFRC                = MCUCTRL->HFRC;
        am_util_pp2[uNumber].M_LFRC                = MCUCTRL->LFRC;
        am_util_pp2[uNumber].M_BODCTRL             = MCUCTRL->BODCTRL;
        am_util_pp2[uNumber].M_ADCPWRCTRL          = MCUCTRL->ADCPWRCTRL;
        am_util_pp2[uNumber].M_ADCCAL              = MCUCTRL->ADCCAL;
        am_util_pp2[uNumber].M_ADCBATTLOAD         = MCUCTRL->ADCBATTLOAD;
        am_util_pp2[uNumber].M_XTALCTRL            = MCUCTRL->XTALCTRL;
        am_util_pp2[uNumber].M_XTALGENCTRL         = MCUCTRL->XTALGENCTRL;
        am_util_pp2[uNumber].M_XTALHSCTRL          = MCUCTRL->XTALHSCTRL;
        am_util_pp2[uNumber].M_MRAMCRYPTOPWRCTRL   = MCUCTRL->MRAMCRYPTOPWRCTRL;
        //am_util_pp2[uNumber].M_MISCPWRCTRL         = MCUCTRL_MISCPWRCTRL;
        am_util_pp2[uNumber].M_D2ASPARE            = MCUCTRL->D2ASPARE;
        am_util_pp2[uNumber].M_BODISABLE           = MCUCTRL->BODISABLE;
        am_util_pp2[uNumber].M_BOOTLOADER          = MCUCTRL->BOOTLOADER;
        am_util_pp2[uNumber].M_SHADOWVALID         = MCUCTRL->SHADOWVALID;
        am_util_pp2[uNumber].M_SCRATCH0            = MCUCTRL->SCRATCH0;
        am_util_pp2[uNumber].M_SCRATCH1            = MCUCTRL->SCRATCH1;
        am_util_pp2[uNumber].M_DBGR1               = MCUCTRL->DBGR1;
        am_util_pp2[uNumber].M_DBGR2               = MCUCTRL->DBGR2;
        am_util_pp2[uNumber].M_WICCONTROL          = MCUCTRL->WICCONTROL;
        am_util_pp2[uNumber].M_DBGCTRL             = MCUCTRL->DBGCTRL;
        am_util_pp2[uNumber].M_OTAPOINTER          = MCUCTRL->OTAPOINTER;
        am_util_pp2[uNumber].M_APBDMACTRL          = MCUCTRL->APBDMACTRL;
        //am_util_pp2[uNumber].M_SRAMTRIMHP          = MCUCTRL_SRAMTRIMHP;
        //am_util_pp2[uNumber].M_SRAMTRIM            = MCUCTRL_SRAMTRIM;
        am_util_pp2[uNumber].M_KEXTCLKSEL          = MCUCTRL->KEXTCLKSEL;
        am_util_pp2[uNumber].M_SIMOBUCK0           = MCUCTRL->SIMOBUCK0;
        am_util_pp2[uNumber].M_SIMOBUCK1           = MCUCTRL->SIMOBUCK1;
        am_util_pp2[uNumber].M_SIMOBUCK2           = MCUCTRL->SIMOBUCK2;
        am_util_pp2[uNumber].M_SIMOBUCK3           = MCUCTRL->SIMOBUCK3;
        am_util_pp2[uNumber].M_SIMOBUCK4           = MCUCTRL->SIMOBUCK4;
        am_util_pp2[uNumber].M_SIMOBUCK6           = MCUCTRL->SIMOBUCK6;
        am_util_pp2[uNumber].M_SIMOBUCK7           = MCUCTRL->SIMOBUCK7;
        am_util_pp2[uNumber].M_SIMOBUCK8           = MCUCTRL->SIMOBUCK8;
        am_util_pp2[uNumber].M_SIMOBUCK9           = MCUCTRL->SIMOBUCK9;
        am_util_pp2[uNumber].M_SIMOBUCK10          = MCUCTRL->SIMOBUCK10;
        am_util_pp2[uNumber].M_SIMOBUCK11          = MCUCTRL->SIMOBUCK11;
        am_util_pp2[uNumber].M_D2ASPARE2           = MCUCTRL->D2ASPARE2;
        am_util_pp2[uNumber].M_I3CPHYCTRL          = MCUCTRL->I3CPHYCTRL;
        am_util_pp2[uNumber].M_PWRSW0              = MCUCTRL->PWRSW0;
        am_util_pp2[uNumber].M_PWRSW1              = MCUCTRL->PWRSW1;
        am_util_pp2[uNumber].M_USBRSTCTRL          = MCUCTRL->USBRSTCTRL;
        am_util_pp2[uNumber].M_FLASHWPROT0         = MCUCTRL->FLASHWPROT0;
        am_util_pp2[uNumber].M_FLASHWPROT1         = MCUCTRL->FLASHWPROT1;
        am_util_pp2[uNumber].M_FLASHWPROT2         = MCUCTRL->FLASHWPROT2;
        am_util_pp2[uNumber].M_FLASHWPROT3         = MCUCTRL->FLASHWPROT3;
        am_util_pp2[uNumber].M_FLASHRPROT0         = MCUCTRL->FLASHRPROT0;
        am_util_pp2[uNumber].M_FLASHRPROT1         = MCUCTRL->FLASHRPROT1;
        am_util_pp2[uNumber].M_FLASHRPROT2         = MCUCTRL->FLASHRPROT2;
        am_util_pp2[uNumber].M_FLASHRPROT3         = MCUCTRL->FLASHRPROT3;
        am_util_pp2[uNumber].M_SRAMWPROT0          = MCUCTRL->SRAMWPROT0;
        am_util_pp2[uNumber].M_SRAMWPROT1          = MCUCTRL->SRAMWPROT1;
        am_util_pp2[uNumber].M_SRAMWPROT2          = MCUCTRL->SRAMWPROT2;
        am_util_pp2[uNumber].M_SRAMWPROT3          = MCUCTRL->SRAMWPROT3;
        am_util_pp2[uNumber].M_SRAMRPROT0          = MCUCTRL->SRAMRPROT0;
        am_util_pp2[uNumber].M_SRAMRPROT1          = MCUCTRL->SRAMRPROT1;
        am_util_pp2[uNumber].M_SRAMRPROT2          = MCUCTRL->SRAMRPROT2;
        am_util_pp2[uNumber].M_SRAMRPROT3          = MCUCTRL->SRAMRPROT3;
        //am_util_pp2[uNumber].M_CPUICACHETRIM       = MCUCTRL_CPUICACHETRIM;
        //am_util_pp2[uNumber].M_CPUDCACHETRIM       = MCUCTRL_CPUDCACHETRIM;
        //am_util_pp2[uNumber].M_SSRAMTRIM           = MCUCTRL_SSRAMTRIM;
        am_util_pp2[uNumber].M_SDIO0CTRL           = MCUCTRL->SDIO0CTRL;
        am_util_pp2[uNumber].M_SDIO1CTRL           = MCUCTRL->SDIO1CTRL;
        am_util_pp2[uNumber].M_PDMCTRL             = MCUCTRL->PDMCTRL;
        am_util_pp2[uNumber].M_SSRAMMISCCTRL       = MCUCTRL->SSRAMMISCCTRL;
        am_util_pp2[uNumber].M_DISPSTATUS          = MCUCTRL->DISPSTATUS;
        am_util_pp2[uNumber].M_CPUCFG              = MCUCTRL->CPUCFG;
        am_util_pp2[uNumber].M_PLLCTL0             = MCUCTRL->PLLCTL0;
        am_util_pp2[uNumber].M_PLLDIV0             = MCUCTRL->PLLDIV0;
        am_util_pp2[uNumber].M_PLLDIV1             = MCUCTRL->PLLDIV1;
        am_util_pp2[uNumber].M_PLLSTAT             = MCUCTRL->PLLSTAT;
        am_util_pp2[uNumber].M_PLLMUXCTL           = MCUCTRL->PLLMUXCTL;
        am_util_pp2[uNumber].M_CM4CODEBASE         = MCUCTRL->CM4CODEBASE;
        am_util_pp2[uNumber].M_RADIOFINECNT        = MCUCTRL->RADIOFINECNT;
        am_util_pp2[uNumber].M_RADIOCLKNCNT        = MCUCTRL->RADIOCLKNCNT;


        //
        //Function Block 3: CLKGEN
        //
        am_util_pp3[uNumber].C_OCTRL               = CLKGEN->OCTRL;
        am_util_pp3[uNumber].C_CLKOUT              = CLKGEN->CLKOUT;
        am_util_pp3[uNumber].C_HFADJ               = CLKGEN->HFADJ;
        am_util_pp3[uNumber].C_CLOCKENSTAT         = CLKGEN->CLOCKENSTAT;
        am_util_pp3[uNumber].C_CLOCKEN2STAT        = CLKGEN->CLOCKEN2STAT;
        am_util_pp3[uNumber].C_CLOCKEN3STAT        = CLKGEN->CLOCKEN3STAT;
        am_util_pp3[uNumber].C_MISC                = CLKGEN->MISC;
        am_util_pp3[uNumber].C_LFRCCTRL            = CLKGEN->LFRCCTRL;
        am_util_pp3[uNumber].C_CLKGENSPARES        = CLKGEN->CLKGENSPARES;
        am_util_pp3[uNumber].C_HFRCIDLECOUNTERS    = CLKGEN->HFRCIDLECOUNTERS;
        am_util_pp3[uNumber].C_MSPIIOCLKCTRL       = CLKGEN->MSPIIOCLKCTRL;
        am_util_pp3[uNumber].C_CLKCTRL             = CLKGEN->CLKCTRL;


        //
        //Function Block 4: System timer
        //
        am_util_pp3[uNumber].ST_STCFG               = STIMER->STCFG;
        am_util_pp3[uNumber].ST_STTMR               = STIMER->STTMR;
        am_util_pp3[uNumber].ST_SCAPCTRL0           = STIMER->SCAPCTRL0;
        am_util_pp3[uNumber].ST_SCAPCTRL1           = STIMER->SCAPCTRL1;
        am_util_pp3[uNumber].ST_SCAPCTRL2           = STIMER->SCAPCTRL2;
        am_util_pp3[uNumber].ST_SCAPCTRL3           = STIMER->SCAPCTRL3;
        am_util_pp3[uNumber].ST_SCMPR0              = STIMER->SCMPR0;
        am_util_pp3[uNumber].ST_SCMPR1              = STIMER->SCMPR1;
        am_util_pp3[uNumber].ST_SCMPR2              = STIMER->SCMPR2;
        am_util_pp3[uNumber].ST_SCMPR3              = STIMER->SCMPR3;
        am_util_pp3[uNumber].ST_SCMPR4              = STIMER->SCMPR4;
        am_util_pp3[uNumber].ST_SCMPR5              = STIMER->SCMPR5;
        am_util_pp3[uNumber].ST_SCMPR6              = STIMER->SCMPR6;
        am_util_pp3[uNumber].ST_SCMPR7              = STIMER->SCMPR7;
        am_util_pp3[uNumber].ST_SCAPT0              = STIMER->SCAPT0;
        am_util_pp3[uNumber].ST_SCAPT1              = STIMER->SCAPT1;
        am_util_pp3[uNumber].ST_SCAPT2              = STIMER->SCAPT2;
        am_util_pp3[uNumber].ST_SCAPT3              = STIMER->SCAPT3;
        am_util_pp3[uNumber].ST_SNVR0               = STIMER->SNVR0;
        am_util_pp3[uNumber].ST_SNVR1               = STIMER->SNVR1;
        am_util_pp3[uNumber].ST_HALSTATES           = STIMER->HALSTATES;
        am_util_pp3[uNumber].ST_STMINTEN            = STIMER->STMINTEN;
        am_util_pp3[uNumber].ST_STMINTSTAT          = STIMER->STMINTSTAT;
        am_util_pp3[uNumber].ST_STMINTCLR           = STIMER->STMINTCLR;
        am_util_pp3[uNumber].ST_STMINTSET           = STIMER->STMINTSET;


        //
        //Function Block 5: Timer
        //
        am_util_pp3[uNumber].T_CTRL                = TIMER->CTRL;
        am_util_pp3[uNumber].T_STATUS              = TIMER->STATUS;
        am_util_pp3[uNumber].T_GLOBEN              = TIMER->GLOBEN;
        am_util_pp3[uNumber].T_INTEN               = TIMER->INTEN;
        am_util_pp3[uNumber].T_INTSTAT             = TIMER->INTSTAT;
        am_util_pp3[uNumber].T_INTCLR              = TIMER->INTCLR;
        am_util_pp3[uNumber].T_INTSET              = TIMER->INTSET;
        // am_util_pp3[uNumber].T_OUTCFG0             = TIMER->OUTCFG0;
        // am_util_pp3[uNumber].T_OUTCFG1             = TIMER->OUTCFG1;
        // am_util_pp3[uNumber].T_OUTCFG2             = TIMER->OUTCFG2;
        // am_util_pp3[uNumber].T_OUTCFG3             = TIMER->OUTCFG3;
        // am_util_pp3[uNumber].T_OUTCFG4             = TIMER->OUTCFG4;
        // am_util_pp3[uNumber].T_OUTCFG5             = TIMER->OUTCFG5;
        // am_util_pp3[uNumber].T_OUTCFG6             = TIMER->OUTCFG6;
        // am_util_pp3[uNumber].T_OUTCFG7             = TIMER->OUTCFG7;
        // am_util_pp3[uNumber].T_OUTCFG8             = TIMER->OUTCFG8;
        // am_util_pp3[uNumber].T_OUTCFG9             = TIMER->OUTCFG9;
        // am_util_pp3[uNumber].T_OUTCFG10            = TIMER->OUTCFG10;
        // am_util_pp3[uNumber].T_OUTCFG11            = TIMER->OUTCFG11;
        // am_util_pp3[uNumber].T_OUTCFG12            = TIMER->OUTCFG12;
        // am_util_pp3[uNumber].T_OUTCFG13            = TIMER->OUTCFG13;
        // am_util_pp3[uNumber].T_OUTCFG14            = TIMER->OUTCFG14;
        // am_util_pp3[uNumber].T_OUTCFG15            = TIMER->OUTCFG15;
        // am_util_pp3[uNumber].T_OUTCFG16            = TIMER->OUTCFG16;
        // am_util_pp3[uNumber].T_OUTCFG17            = TIMER->OUTCFG17;
        // am_util_pp3[uNumber].T_OUTCFG18            = TIMER->OUTCFG18;
        // am_util_pp3[uNumber].T_OUTCFG19            = TIMER->OUTCFG19;
        // am_util_pp3[uNumber].T_OUTCFG20            = TIMER->OUTCFG20;
        // am_util_pp3[uNumber].T_OUTCFG21            = TIMER->OUTCFG21;
        // am_util_pp3[uNumber].T_OUTCFG22            = TIMER->OUTCFG22;
        // am_util_pp3[uNumber].T_OUTCFG23            = TIMER->OUTCFG23;
        // am_util_pp3[uNumber].T_OUTCFG24            = TIMER->OUTCFG24;
        // am_util_pp3[uNumber].T_OUTCFG25            = TIMER->OUTCFG25;
        // am_util_pp3[uNumber].T_OUTCFG26            = TIMER->OUTCFG26;
        // am_util_pp3[uNumber].T_OUTCFG27            = TIMER->OUTCFG27;
        // am_util_pp3[uNumber].T_OUTCFG28            = TIMER->OUTCFG28;
        // am_util_pp3[uNumber].T_OUTCFG29            = TIMER->OUTCFG29;
        am_util_pp3[uNumber].T_CTRL0               = TIMER->CTRL0;
        am_util_pp3[uNumber].T_TIMER0              = TIMER->TIMER0;
        am_util_pp3[uNumber].T_TMR0CMP0            = TIMER->TMR0CMP0;
        am_util_pp3[uNumber].T_TMR0CMP1            = TIMER->TMR0CMP1;
        am_util_pp3[uNumber].T_MODE0               = TIMER->MODE0;
        am_util_pp3[uNumber].T_TMR0LMTVAL          = TIMER->TMR0LMTVAL;
        am_util_pp3[uNumber].T_CTRL1               = TIMER->CTRL1;
        am_util_pp3[uNumber].T_TIMER1              = TIMER->TIMER1;
        am_util_pp3[uNumber].T_TMR1CMP0            = TIMER->TMR1CMP0;
        am_util_pp3[uNumber].T_TMR1CMP1            = TIMER->TMR1CMP1;
        am_util_pp3[uNumber].T_MODE1               = TIMER->MODE1;
        am_util_pp3[uNumber].T_TMR1LMTVAL          = TIMER->TMR1LMTVAL;
        am_util_pp3[uNumber].T_CTRL2               = TIMER->CTRL2;
        am_util_pp3[uNumber].T_TIMER2              = TIMER->TIMER2;
        am_util_pp3[uNumber].T_TMR2CMP0            = TIMER->TMR2CMP0;
        am_util_pp3[uNumber].T_TMR2CMP1            = TIMER->TMR2CMP1;
        am_util_pp3[uNumber].T_MODE2               = TIMER->MODE2;
        am_util_pp3[uNumber].T_TMR2LMTVAL          = TIMER->TMR2LMTVAL;
        am_util_pp3[uNumber].T_CTRL3               = TIMER->CTRL3;
        am_util_pp3[uNumber].T_TIMER3              = TIMER->TIMER3;
        am_util_pp3[uNumber].T_TMR3CMP0            = TIMER->TMR3CMP0;
        am_util_pp3[uNumber].T_TMR3CMP1            = TIMER->TMR3CMP1;
        am_util_pp3[uNumber].T_MODE3               = TIMER->MODE3;
        am_util_pp3[uNumber].T_TMR3LMTVAL          = TIMER->TMR3LMTVAL;
        am_util_pp3[uNumber].T_CTRL4               = TIMER->CTRL4;
        am_util_pp3[uNumber].T_TIMER4              = TIMER->TIMER4;
        am_util_pp3[uNumber].T_TMR4CMP0            = TIMER->TMR4CMP0;
        am_util_pp3[uNumber].T_TMR4CMP1            = TIMER->TMR4CMP1;
        am_util_pp3[uNumber].T_MODE4               = TIMER->MODE4;
        am_util_pp3[uNumber].T_TMR4LMTVAL          = TIMER->TMR4LMTVAL;
        am_util_pp3[uNumber].T_CTRL5               = TIMER->CTRL5;
        am_util_pp3[uNumber].T_TIMER5              = TIMER->TIMER5;
        am_util_pp3[uNumber].T_TMR5CMP0            = TIMER->TMR5CMP0;
        am_util_pp3[uNumber].T_TMR5CMP1            = TIMER->TMR5CMP1;
        am_util_pp3[uNumber].T_MODE5               = TIMER->MODE5;
        am_util_pp3[uNumber].T_TMR5LMTVAL          = TIMER->TMR5LMTVAL;
        am_util_pp3[uNumber].T_CTRL6               = TIMER->CTRL6;
        am_util_pp3[uNumber].T_TIMER6              = TIMER->TIMER6;
        am_util_pp3[uNumber].T_TMR6CMP0            = TIMER->TMR6CMP0;
        am_util_pp3[uNumber].T_TMR6CMP1            = TIMER->TMR6CMP1;
        am_util_pp3[uNumber].T_MODE6               = TIMER->MODE6;
        am_util_pp3[uNumber].T_TMR6LMTVAL          = TIMER->TMR6LMTVAL;
        am_util_pp3[uNumber].T_CTRL7               = TIMER->CTRL7;
        am_util_pp3[uNumber].T_TIMER7              = TIMER->TIMER7;
        am_util_pp3[uNumber].T_TMR7CMP0            = TIMER->TMR7CMP0;
        am_util_pp3[uNumber].T_TMR7CMP1            = TIMER->TMR7CMP1;
        am_util_pp3[uNumber].T_MODE7               = TIMER->MODE7;
        am_util_pp3[uNumber].T_TMR7LMTVAL          = TIMER->TMR7LMTVAL;
        am_util_pp3[uNumber].T_CTRL8               = TIMER->CTRL8;
        am_util_pp3[uNumber].T_TIMER8              = TIMER->TIMER8;
        am_util_pp3[uNumber].T_TMR8CMP0            = TIMER->TMR8CMP0;
        am_util_pp3[uNumber].T_TMR8CMP1            = TIMER->TMR8CMP1;
        am_util_pp3[uNumber].T_MODE8               = TIMER->MODE8;
        am_util_pp3[uNumber].T_TMR8LMTVAL          = TIMER->TMR8LMTVAL;
        am_util_pp3[uNumber].T_CTRL9               = TIMER->CTRL9;
        am_util_pp3[uNumber].T_TIMER9              = TIMER->TIMER9;
        am_util_pp3[uNumber].T_TMR9CMP0            = TIMER->TMR9CMP0;
        am_util_pp3[uNumber].T_TMR9CMP1            = TIMER->TMR9CMP1;
        am_util_pp3[uNumber].T_MODE9               = TIMER->MODE9;
        am_util_pp3[uNumber].T_TMR9LMTVAL          = TIMER->TMR9LMTVAL;
        am_util_pp3[uNumber].T_CTRL10              = TIMER->CTRL10;
        am_util_pp3[uNumber].T_TIMER10             = TIMER->TIMER10;
        am_util_pp3[uNumber].T_TMR10CMP0           = TIMER->TMR10CMP0;
        am_util_pp3[uNumber].T_TMR10CMP1           = TIMER->TMR10CMP1;
        am_util_pp3[uNumber].T_MODE10              = TIMER->MODE10;
        am_util_pp3[uNumber].T_TMR10LMTVAL         = TIMER->TMR10LMTVAL;
        am_util_pp3[uNumber].T_CTRL11              = TIMER->CTRL11;
        am_util_pp3[uNumber].T_TIMER11             = TIMER->TIMER11;
        am_util_pp3[uNumber].T_TMR11CMP0           = TIMER->TMR11CMP0;
        am_util_pp3[uNumber].T_TMR11CMP1           = TIMER->TMR11CMP1;
        am_util_pp3[uNumber].T_MODE11              = TIMER->MODE11;
        am_util_pp3[uNumber].T_TMR11LMTVAL         = TIMER->TMR11LMTVAL;
        am_util_pp3[uNumber].T_CTRL12              = TIMER->CTRL12;
        am_util_pp3[uNumber].T_TIMER12             = TIMER->TIMER12;
        am_util_pp3[uNumber].T_TMR12CMP0           = TIMER->TMR12CMP0;
        am_util_pp3[uNumber].T_TMR12CMP1           = TIMER->TMR12CMP1;
        am_util_pp3[uNumber].T_MODE12              = TIMER->MODE12;
        am_util_pp3[uNumber].T_TMR12LMTVAL         = TIMER->TMR12LMTVAL;
        am_util_pp3[uNumber].T_CTRL13              = TIMER->CTRL13;
        am_util_pp3[uNumber].T_TIMER13             = TIMER->TIMER13;
        am_util_pp3[uNumber].T_TMR13CMP0           = TIMER->TMR13CMP0;
        am_util_pp3[uNumber].T_TMR13CMP1           = TIMER->TMR13CMP1;
        am_util_pp3[uNumber].T_MODE13              = TIMER->MODE13;
        am_util_pp3[uNumber].T_TMR13LMTVAL         = TIMER->TMR13LMTVAL;
        am_util_pp3[uNumber].T_CTRL14              = TIMER->CTRL14;
        am_util_pp3[uNumber].T_TIMER14             = TIMER->TIMER14;
        am_util_pp3[uNumber].T_TMR14CMP0           = TIMER->TMR14CMP0;
        am_util_pp3[uNumber].T_TMR14CMP1           = TIMER->TMR14CMP1;
        am_util_pp3[uNumber].T_MODE14              = TIMER->MODE14;
        am_util_pp3[uNumber].T_TMR14LMTVAL         = TIMER->TMR14LMTVAL;
        am_util_pp3[uNumber].T_CTRL15              = TIMER->CTRL15;
        am_util_pp3[uNumber].T_TIMER15             = TIMER->TIMER15;
        am_util_pp3[uNumber].T_TMR15CMP0           = TIMER->TMR15CMP0;
        am_util_pp3[uNumber].T_TMR15CMP1           = TIMER->TMR15CMP1;
        am_util_pp3[uNumber].T_MODE15              = TIMER->MODE15;
        am_util_pp3[uNumber].T_TMR15LMTVAL         = TIMER->TMR15LMTVAL;
        am_util_pp3[uNumber].T_TIMERSPARES         = TIMER->TIMERSPARES;


        //
        //Function Block 6: SYSTEM CONTROL
        //
        am_util_pp4[uNumber].SC_ICTR                = SYSCTRL_ICTR;
        am_util_pp4[uNumber].SC_ACTLR               = SYSCTRL_ACTLR;
        am_util_pp4[uNumber].SC_ICSR                = SYSCTRL_ICSR;
        am_util_pp4[uNumber].SC_VTOR                = SYSCTRL_VTOR;
        am_util_pp4[uNumber].SC_AIRCR               = SYSCTRL_AIRCR;
        am_util_pp4[uNumber].SC_SCR                 = SYSCTRL_SCR;
        am_util_pp4[uNumber].SC_CCR                 = SYSCTRL_CCR;
        am_util_pp4[uNumber].SC_SHPR1               = SYSCTRL_SHPR1;
        am_util_pp4[uNumber].SC_SHPR2               = SYSCTRL_SHPR2;
        am_util_pp4[uNumber].SC_SHPR3               = SYSCTRL_SHPR3;
        am_util_pp4[uNumber].SC_SHCSR               = SYSCTRL_SHCSR;
        am_util_pp4[uNumber].SC_CFSR                = SYSCTRL_CFSR;
        am_util_pp4[uNumber].SC_HFSR                = SYSCTRL_HFSR;
        am_util_pp4[uNumber].SC_MMFAR               = SYSCTRL_MMFAR;
        am_util_pp4[uNumber].SC_BFAR                = SYSCTRL_BFAR;
        am_util_pp4[uNumber].SC_CPACR               = SYSCTRL_CPACR;
        am_util_pp4[uNumber].SC_DEMCR               = SYSCTRL_DEMCR;
        am_util_pp4[uNumber].SC_STIR                = SYSCTRL_STIR;
        am_util_pp4[uNumber].SC_FPCCR               = SYSCTRL_FPCCR;
        am_util_pp4[uNumber].SC_FPCAR               = SYSCTRL_FPCAR;
        am_util_pp4[uNumber].SC_FPDSCR              = SYSCTRL_FPDSCR;

#if USE_DMIC_PDM
        //
        //Function Block 7: PDM
        //
        if ( g_sVosBrd.pvPDMHandle && (AM_HAL_PDM_HANDLE_VALID(g_sVosBrd.pvPDMHandle)) )
        {
            am_util_pp5[uNumber].PDM_CTRL                = PDMn(0)->CTRL;
            am_util_pp5[uNumber].PDM_CORECFG0            = PDMn(0)->CORECFG0;
            am_util_pp5[uNumber].PDM_CORECFG1            = PDMn(0)->CORECFG1;
            am_util_pp5[uNumber].PDM_CORECTRL            = PDMn(0)->CORECTRL;
            am_util_pp5[uNumber].PDM_FIFOCNT             = PDMn(0)->FIFOCNT;
            am_util_pp5[uNumber].PDM_FIFOREAD            = PDMn(0)->FIFOREAD;
            am_util_pp5[uNumber].PDM_FIFOFLUSH           = PDMn(0)->FIFOFLUSH;
            am_util_pp5[uNumber].PDM_FIFOTHR             = PDMn(0)->FIFOTHR;
            am_util_pp5[uNumber].PDM_INTEN               = PDMn(0)->INTEN;
            am_util_pp5[uNumber].PDM_INTSTAT             = PDMn(0)->INTSTAT;
            am_util_pp5[uNumber].PDM_INTCLR              = PDMn(0)->INTCLR;
            am_util_pp5[uNumber].PDM_INTSET              = PDMn(0)->INTSET;
            am_util_pp5[uNumber].PDM_DMATRIGEN           = PDMn(0)->DMATRIGEN;
            am_util_pp5[uNumber].PDM_DMATRIGSTAT         = PDMn(0)->DMATRIGSTAT;
            am_util_pp5[uNumber].PDM_DMACFG              = PDMn(0)->DMACFG;
            am_util_pp5[uNumber].PDM_DMATARGADDR         = PDMn(0)->DMATARGADDR;
            am_util_pp5[uNumber].PDM_DMASTAT             = PDMn(0)->DMASTAT;
            am_util_pp5[uNumber].PDM_DMATARGADDRNEXT     = PDMn(0)->DMATARGADDRNEXT;
            am_util_pp5[uNumber].PDM_DMATOTCOUNTNEXT     = PDMn(0)->DMATOTCOUNTNEXT;
            am_util_pp5[uNumber].PDM_DMAENNEXTCTRL       = PDMn(0)->DMAENNEXTCTRL;
            am_util_pp5[uNumber].PDM_DMATOTCOUNT         = PDMn(0)->DMATOTCOUNT;
        }
#endif

       bCaptured[uNumber] = true;  //now the snapshot is in memory
    }
} // am_util_pp_snapshot()
//#endif
