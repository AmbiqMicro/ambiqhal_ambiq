//*****************************************************************************
//
//! @file am_util_pp.c
//!
//! @brief Functions to aid power profiling and debugging
//!
//! @addtogroup ppf Power Profiling Functionality
//! @ingroup utils
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
// This is part of revision release_sdk5p0p0-5f68a8286b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_util_pp.h"
#include "am_bsp.h"

#if USE_AMIC_AUDADC
#define AM_HAL_MAGIC_AUDADC                0xAFAFAF
#define AM_HAL_AUDADC_CHK_HANDLE(h)        ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_AUDADC))
#endif

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
        \"MCUPERFREQ\": %u,             \
        \"DEVPWREN\": %u,               \
        \"DEVPWRSTATUS\": %u,           \
        \"AUDSSPWREN\": %u,             \
        \"AUDSSPWRSTATUS\": %u,         \
        \"MEMPWREN\": %u,               \
        \"MEMPWRSTATUS\": %u,           \
        \"MEMRETCFG\": %u,              \
        \"SYSPWRSTATUS\": %u,           \
        \"SSRAMPWREN\": %u,             \
        \"SSRAMPWRST\": %u,             \
        \"SSRAMRETCFG\": %u,            \
        \"DEVPWREVENTEN\": %u,          \
        \"MEMPWREVENTEN\": %u,          \
        \"MMSOVERRIDE\": %u, \
    ";

    char *pwrStr2 = "                   \
        \"CPUPWRCTRL\": %u,             \
        \"PWRCTRLMODESTATUS\": %u,      \
        \"CPUPWRSTATUS\": %u,           \
        \"PWRACKOVR\": %u,              \
        \"PWRCNTDEFVAL\": %u,           \
        \"GFXPERFREQ\": %u,             \
        \"GFXPWRSWSEL\": %u,            \
        \"EPURETCFG\": %u,              \
        \"VRCTRL\": %u,                 \
        \"LEGACYVRLPOVR\": %u,          \
        \"VRSTATUS\": %u,               \
    ";

    char *pwrStr3 = "                   \
        \"SRAMCTRL\": %u,               \
        \"ADCSTATUS\": %u,              \
        \"AUDADCSTATUS\": %u,           \
        \"TONCNTRCTRL\": %u,            \
        \"LPOVRTHRESHVDDS\": %u,        \
        \"LPOVRHYSTCNT\": %u,           \
        \"LPOVRTHRESHVDDF\": %u,        \
        \"LPOVRTHRESHVDDC\": %u,        \
        \"LPOVRTHRESHVDDCLV\": %u,      \
        \"LPOVRSTAT\": %u,              \
        \"MRAMEXTCTRL\": %u,            \
        \"EMONCTRL\": %u,               \
        \"EMONCFG0\": %u,               \
        \"EMONCFG1\": %u,               \
        \"EMONCFG2\": %u,               \
        \"EMONCFG3\": %u,               \
        \"EMONCFG4\": %u,               \
        \"EMONCFG5\": %u,               \
        \"EMONCFG6\": %u,               \
        \"EMONCFG7\": %u,               \
        \"EMONCOUNT0\": %u,             \
        \"EMONCOUNT1\": %u,             \
        \"EMONCOUNT2\": %u,             \
        \"EMONCOUNT3\": %u,             \
        \"EMONCOUNT4\": %u,             \
        \"EMONCOUNT5\": %u,             \
        \"EMONCOUNT6\": %u,             \
        \"EMONCOUNT7\": %u,             \
        \"EMONSTATUS\": %u,             \
        \"FPIOEN0\": %u,                \
        \"FPIOEN1\": %u,                \
        \"FPIOEN2\": %u                 \
    },";

    char *mcuCtrlStr = " \"MCUCTRL\":   \
    {                                   \
        \"CHIPPN\": %u,                 \
        \"CHIPID0\": %u,                \
        \"CHIPID1\": %u,                \
        \"CHIPREV\": %u,                \
        \"VENDORID\": %u,               \
        \"SKU\": %u,                    \
        \"SKUOVERRIDE\": %u,            \
        \"DEBUGGER\": %u,               \
        \"ACRG\": %u,                   \
        \"VREFGEN\": %u,                \
        \"VREFGEN2\": %u,               \
        \"VREFGEN3\": %u,               \
        \"VREFGEN4\": %u,               \
        \"VREFGEN5\": %u,               \
        \"VREFBUF\": %u,                \
        \"VRCTRL\": %u,                 \
        \"LDOREG1\": %u,                \
        \"LDOREG2\": %u,                \
        \"HFRC\": %u,                   \
        \"HFRC2\": %u,                  \
        \"LFRC\": %u,                   \
        \"BODCTRL\": %u,                \
        \"ADCPWRCTRL\": %u,             \
        \"ADCCAL\": %u,                 \
        \"ADCBATTLOAD\": %u,            \
        \"XTALCTRL\": %u,               \
        \"XTALGENCTRL\": %u,            \
        \"XTALHSTRIMS\": %u,            \
        \"XTALHSCTRL\": %u,             \
        \"BGTLPCTRL\": %u,              \
        \"MRAMCRYPTOPWRCTRL\": %u,      \
        \"MISCPWRCTRL\": %u,            \
        \"BODISABLE\": %u,              \
        \"D2ASPARE\": %u,               \
        \"BOOTLOADER\": %u,             \
        \"SHADOWVALID\": %u,            \
        \"SCRATCH0\": %u,               \
        \"SCRATCH1\": %u,               \
        \"DBGR1\": %u,                  \
        \"DBGR2\": %u,                  \
        \"WICCONTROL\": %u,             \
        \"DBGCTRL\": %u,                \
        \"OTAPOINTER\": %u,             \
        \"APBDMACTRL\": %u,             \
        \"FORCEAXICLKEN\": %u,          \
        \"SRAMTRIMHP\": %u,             \
        \"SRAMTRIM\": %u,               \
        \"KEXTCLKSEL\": %u,             \
        \"SIMOBUCK0\": %u,              \
        \"SIMOBUCK1\": %u,              \
        \"SIMOBUCK2\": %u,              \
        \"SIMOBUCK3\": %u,              \
        \"SIMOBUCK4\": %u,              \
        \"SIMOBUCK5\": %u,              \
        \"SIMOBUCK6\": %u,              \
        \"SIMOBUCK7\": %u,              \
        \"SIMOBUCK8\": %u,              \
        \"SIMOBUCK9\": %u,              \
        \"SIMOBUCK10\": %u,             \
        \"SIMOBUCK11\": %u,             \
        \"SIMOBUCK12\": %u,             \
        \"SIMOBUCK13\": %u,             \
        \"SIMOBUCK14\": %u,             \
        \"SIMOBUCK15\": %u,             \
        \"PWRSW0\": %u,                 \
        \"PWRSW1\": %u,                 \
        \"USBRSTCTRL\": %u,             \
        \"FLASHWPROT0\": %u,            \
        \"FLASHWPROT1\": %u,            \
        \"FLASHWPROT2\": %u,            \
        \"FLASHWPROT3\": %u,            \
        \"FLASHRPROT0\": %u,            \
        \"FLASHRPROT1\": %u,            \
        \"FLASHRPROT2\": %u,            \
        \"FLASHRPROT3\": %u,            \
        \"SRAMWPROT0\": %u,             \
        \"SRAMWPROT1\": %u,             \
        \"SRAMWPROT2\": %u,             \
        \"SRAMWPROT3\": %u,             \
        \"SRAMWPROT4\": %u,             \
        \"SRAMWPROT5\": %u,             \
        \"SRAMRPROT0\": %u,             \
        \"SRAMRPROT1\": %u,             \
        \"SRAMRPROT2\": %u,             \
        \"SRAMRPROT3\": %u,             \
        \"SRAMRPROT4\": %u,             \
        \"SRAMRPROT5\": %u,             \
        \"CPUICACHETRIM\": %u,          \
        \"CPUDCACHETRIM\": %u,          \
        \"SSRAMTRIM\": %u,              \
        \"AUDADCPWRCTRL\": %u,          \
        \"AUDIO1\": %u,                 \
        \"PGAADCIFCTRL\": %u,           \
        \"PGACTRL1\": %u,               \
        \"PGACTRL2\": %u,               \
        \"AUDADCPWRDLY\": %u,           \
        \"SDIO0CTRL\": %u,              \
        \"SDIO1CTRL\": %u,              \
        \"PDMCTRL\": %u,                \
        \"MMSMISCCTRL\": %u,            \
        \"FLASHWPROT4\": %u,            \
        \"FLASHWPROT5\": %u,            \
        \"FLASHWPROT6\": %u,            \
        \"FLASHWPROT7\": %u,            \
        \"FLASHRPROT4\": %u,            \
        \"FLASHRPROT5\": %u,            \
        \"FLASHRPROT6\": %u,            \
        \"FLASHRPROT7\": %u,            \
        \"PWRSW2\": %u,                 \
        \"CPUCFG\": %u,                 \
        \"PLLCTL0\": %u,                \
        \"PLLDIV0\": %u,                \
        \"PLLDIV1\": %u,                \
        \"PLLSTAT\": %u,                \
        \"PLLMUXCTL\": %u               \
    },";

    char *audadcStr = " \"AUDADC\":     \
    {                                   \
        \"CFG\": %u,                    \
        \"STAT\": %u,                   \
        \"SWT\": %u,                    \
        \"SL0CFG\": %u,                 \
        \"SL1CFG\": %u,                 \
        \"SL2CFG\": %u,                 \
        \"SL3CFG\": %u,                 \
        \"SL4CFG\": %u,                 \
        \"SL5CFG\": %u,                 \
        \"SL6CFG\": %u,                 \
        \"SL7CFG\": %u,                 \
        \"WULIM\": %u,                  \
        \"WLLIM\": %u,                  \
        \"SCWLIM\": %u,                 \
        \"FIFO\": %u,                   \
        \"FIFOPR\": %u,                 \
        \"FIFOSTAT\": %u,               \
        \"DATAOFFSET\": %u,             \
        \"ZXCFG\": %u,                  \
        \"ZXLIM\": %u,                  \
        \"GAINCFG\": %u,                \
        \"GAIN\": %u,                   \
        \"SATCFG\": %u,                 \
        \"SATLIM\": %u,                 \
        \"SATMAX\": %u,                 \
        \"SATCLR\": %u,                 \
        \"IEREN\": %u,                  \
        \"IERSTAT\": %u,                \
        \"IERCLR\": %u,                 \
        \"IERSET\": %u,                 \
        \"DMATRIGEN\": %u,              \
        \"DMATRIGSTAT\": %u,            \
        \"DMACFG\": %u,                 \
        \"DMATOTCOUNT\": %u,            \
        \"DMATARGADDR\": %u,            \
        \"DMASTAT\": %u                 \
    },";

    char *clkStr = " \"CLK\":           \
    {                                   \
        \"OCTRL\": %u,                  \
        \"CLKOUT\": %u,                 \
        \"HFADJ\": %u,                  \
        \"CLOCKENSTAT\": %u,            \
        \"CLOCKEN2STAT\": %u,           \
        \"CLOCKEN3STAT\": %u,           \
        \"MISC\": %u,                   \
        \"HF2ADJ0\": %u,                \
        \"HF2ADJ1\": %u,                \
        \"HF2ADJ2\": %u,                \
        \"HF2VAL\": %u,                 \
        \"LFRCCTRL\": %u,               \
        \"DISPCLKCTRL\": %u,            \
        \"CLKGENSPARES\": %u,           \
        \"HFRCIDLECOUNTERS\": %u,       \
        \"MSPIIOCLKCTRL\": %u,          \
        \"CLKCTRL\": %u                 \
    },";

    char *timerStr = " \"Timers\":      \
    {                                   \
        \"STCFG\": %u,                  \
        \"STMINTSTAT\": %u,             \
        \"CTRL\": %u,                   \
        \"STATUS\": %u,                 \
        \"GLOBEN\": %u,                 \
        \"INTSTAT\": %u,                \
        \"CTRL0\": %u,                  \
        \"CTRL1\": %u,                  \
        \"CTRL2\": %u,                  \
        \"CTRL3\": %u,                  \
        \"CTRL4\": %u,                  \
        \"CTRL5\": %u,                  \
        \"CTRL6\": %u,                  \
        \"CTRL7\": %u,                  \
        \"CTRL8\": %u,                  \
        \"CTRL9\": %u,                  \
        \"CTRL10\": %u,                 \
        \"CTRL11\": %u,                 \
        \"CTRL12\": %u,                 \
        \"CTRL13\": %u,                 \
        \"CTRL14\": %u,                 \
        \"CTRL15\": %u                  \
    },";

    char *pdmStr = " \"PDM\":           \
    {                                   \
        \"CTRL\": %u,                   \
        \"CORECFG0\": %u,               \
        \"CORECFG1\": %u,               \
        \"CORECTRL\": %u,               \
        \"FIFOCNT\": %u,                \
        \"FIFOREAD\": %u,               \
        \"FIFOFLUSH\": %u,              \
        \"FIFOTHR\": %u,                \
        \"INTEN\": %u,                  \
        \"INTSTAT\": %u,                \
        \"INTCLR\": %u,                 \
        \"INTSET\": %u,                 \
        \"DMATRIGEN\": %u,              \
        \"DMATRIGSTAT\": %u,            \
        \"DMACFG\": %u,                 \
        \"DMATARGADDR\": %u,            \
        \"DMASTAT\": %u,                \
        \"DMATARGADDRNEXT\": %u,        \
        \"DMATOTCOUNTNEXT\": %u,        \
        \"DMAENNEXTCTRL\": %u,          \
        \"DMATOTCOUNT\": %u             \
    }}\n";

    //
    //To provide the single snapshot, we need enable the printf via UART
    //
    am_bsp_uart_printf_enable();

    am_util_stdio_printf(pwrStr1, am_util_pp1[uNumber].bSingle,
                          am_util_pp1[uNumber].uSnapShot,           \
                          am_util_pp1[uNumber].P_MCUPERFREQ,        \
                          am_util_pp1[uNumber].P_DEVPWREN,          \
                          am_util_pp1[uNumber].P_DEVPWRSTATUS,      \
                          am_util_pp1[uNumber].P_AUDSSPWREN,        \
                          am_util_pp1[uNumber].P_AUDSSPWRSTATUS,    \
                          am_util_pp1[uNumber].P_MEMPWREN,          \
                          am_util_pp1[uNumber].P_MEMPWRSTATUS,      \
                          am_util_pp1[uNumber].P_MEMRETCFG,         \
                          am_util_pp1[uNumber].P_SYSPWRSTATUS,      \
                          am_util_pp1[uNumber].P_SSRAMPWREN,        \
                          am_util_pp1[uNumber].P_SSRAMPWRST,        \
                          am_util_pp1[uNumber].P_SSRAMRETCFG,       \
                          am_util_pp1[uNumber].P_DEVPWREVENTEN,     \
                          am_util_pp1[uNumber].P_MEMPWREVENTEN,     \
                          am_util_pp1[uNumber].P_MMSOVERRIDE);

    am_util_stdio_printf(pwrStr2, am_util_pp1[uNumber].P_CPUPWRCTRL, \
                          am_util_pp1[uNumber].P_PWRCTRLMODESTATUS, \
                          am_util_pp1[uNumber].P_CPUPWRSTATUS,      \
                          am_util_pp1[uNumber].P_PWRACKOVR,         \
                          am_util_pp1[uNumber].P_PWRCNTDEFVAL,      \
                          am_util_pp1[uNumber].P_GFXPERFREQ,        \
                          am_util_pp1[uNumber].P_GFXPWRSWSEL,       \
                          am_util_pp1[uNumber].P_EPURETCFG,         \
                          am_util_pp1[uNumber].P_VRCTRL,            \
                          am_util_pp1[uNumber].P_LEGACYVRLPOVR,     \
                          am_util_pp1[uNumber].P_VRSTATUS);

    am_util_stdio_printf(pwrStr3, am_util_pp1[uNumber].P_SRAMCTRL,  \
                          am_util_pp1[uNumber].P_ADCSTATUS,         \
                          am_util_pp1[uNumber].P_AUDADCSTATUS,      \
                          am_util_pp1[uNumber].P_TONCNTRCTRL,       \
                          am_util_pp1[uNumber].P_LPOVRTHRESHVDDS,   \
                          am_util_pp1[uNumber].P_LPOVRHYSTCNT,      \
                          am_util_pp1[uNumber].P_LPOVRTHRESHVDDF,   \
                          am_util_pp1[uNumber].P_LPOVRTHRESHVDDC,   \
                          am_util_pp1[uNumber].P_LPOVRTHRESHVDDCLV, \
                          am_util_pp1[uNumber].P_LPOVRSTAT,         \
                          am_util_pp1[uNumber].P_MRAMEXTCTRL,       \
                          am_util_pp1[uNumber].P_EMONCTRL,          \
                          am_util_pp1[uNumber].P_EMONCFG0,          \
                          am_util_pp1[uNumber].P_EMONCFG1,          \
                          am_util_pp1[uNumber].P_EMONCFG2,          \
                          am_util_pp1[uNumber].P_EMONCFG3,          \
                          am_util_pp1[uNumber].P_EMONCFG4,          \
                          am_util_pp1[uNumber].P_EMONCFG5,          \
                          am_util_pp1[uNumber].P_EMONCFG6,          \
                          am_util_pp1[uNumber].P_EMONCFG7,          \
                          am_util_pp1[uNumber].P_EMONCOUNT0,        \
                          am_util_pp1[uNumber].P_EMONCOUNT1,        \
                          am_util_pp1[uNumber].P_EMONCOUNT2,        \
                          am_util_pp1[uNumber].P_EMONCOUNT3,        \
                          am_util_pp1[uNumber].P_EMONCOUNT4,        \
                          am_util_pp1[uNumber].P_EMONCOUNT5,        \
                          am_util_pp1[uNumber].P_EMONCOUNT6,        \
                          am_util_pp1[uNumber].P_EMONCOUNT7,        \
                          am_util_pp1[uNumber].P_EMONSTATUS,        \
                          am_util_pp1[uNumber].P_FPIOEN0,           \
                          am_util_pp1[uNumber].P_FPIOEN1,           \
                          am_util_pp1[uNumber].P_FPIOEN2);

    am_util_stdio_printf(mcuCtrlStr, am_util_pp2[uNumber].M_CHIPPN, \
                          am_util_pp2[uNumber].M_CHIPID0,           \
                          am_util_pp2[uNumber].M_CHIPID1,           \
                          am_util_pp2[uNumber].M_CHIPREV,           \
                          am_util_pp2[uNumber].M_VENDORID,          \
                          am_util_pp2[uNumber].M_SKU,               \
                          am_util_pp2[uNumber].M_SKUOVERRIDE,       \
                          am_util_pp2[uNumber].M_DEBUGGER,          \
                          am_util_pp2[uNumber].M_ACRG,              \
                          am_util_pp2[uNumber].M_VREFGEN,           \
                          am_util_pp2[uNumber].M_VREFGEN2,          \
                          am_util_pp2[uNumber].M_VREFGEN3,          \
                          am_util_pp2[uNumber].M_VREFGEN4,          \
                          am_util_pp2[uNumber].M_VREFGEN5,          \
                          am_util_pp2[uNumber].M_VREFBUF,           \
                          am_util_pp2[uNumber].M_VRCTRL,            \
                          am_util_pp2[uNumber].M_LDOREG1,           \
                          am_util_pp2[uNumber].M_LDOREG2,           \
                          am_util_pp2[uNumber].M_HFRC,              \
                          am_util_pp2[uNumber].M_HFRC2,             \
                          am_util_pp2[uNumber].M_LFRC,              \
                          am_util_pp2[uNumber].M_BODCTRL,           \
                          am_util_pp2[uNumber].M_ADCPWRCTRL,        \
                          am_util_pp2[uNumber].M_ADCCAL,            \
                          am_util_pp2[uNumber].M_ADCBATTLOAD,       \
                          am_util_pp2[uNumber].M_XTALCTRL,          \
                          am_util_pp2[uNumber].M_XTALGENCTRL,       \
                          am_util_pp2[uNumber].M_XTALHSTRIMS,       \
                          am_util_pp2[uNumber].M_XTALHSCTRL,        \
                          am_util_pp2[uNumber].M_BGTLPCTRL,         \
                          am_util_pp2[uNumber].M_MRAMCRYPTOPWRCTRL, \
                          am_util_pp2[uNumber].M_MISCPWRCTRL,       \
                          am_util_pp2[uNumber].M_BODISABLE,         \
                          am_util_pp2[uNumber].M_D2ASPARE,          \
                          am_util_pp2[uNumber].M_BOOTLOADER,        \
                          am_util_pp2[uNumber].M_SHADOWVALID,       \
                          am_util_pp2[uNumber].M_SCRATCH0,          \
                          am_util_pp2[uNumber].M_SCRATCH1,          \
                          am_util_pp2[uNumber].M_DBGR1,             \
                          am_util_pp2[uNumber].M_DBGR2,             \
                          am_util_pp2[uNumber].M_WICCONTROL,        \
                          am_util_pp2[uNumber].M_DBGCTRL,           \
                          am_util_pp2[uNumber].M_OTAPOINTER,        \
                          am_util_pp2[uNumber].M_APBDMACTRL,        \
                          am_util_pp2[uNumber].M_FORCEAXICLKEN,     \
                          am_util_pp2[uNumber].M_SRAMTRIMHP,        \
                          am_util_pp2[uNumber].M_SRAMTRIM,          \
                          am_util_pp2[uNumber].M_KEXTCLKSEL,        \
                          am_util_pp2[uNumber].M_SIMOBUCK0,         \
                          am_util_pp2[uNumber].M_SIMOBUCK1,         \
                          am_util_pp2[uNumber].M_SIMOBUCK2,         \
                          am_util_pp2[uNumber].M_SIMOBUCK3,         \
                          am_util_pp2[uNumber].M_SIMOBUCK4,         \
                          am_util_pp2[uNumber].M_SIMOBUCK5,         \
                          am_util_pp2[uNumber].M_SIMOBUCK6,         \
                          am_util_pp2[uNumber].M_SIMOBUCK7,         \
                          am_util_pp2[uNumber].M_SIMOBUCK8,         \
                          am_util_pp2[uNumber].M_SIMOBUCK9,         \
                          am_util_pp2[uNumber].M_SIMOBUCK10,        \
                          am_util_pp2[uNumber].M_SIMOBUCK11,        \
                          am_util_pp2[uNumber].M_SIMOBUCK12,        \
                          am_util_pp2[uNumber].M_SIMOBUCK13,         \
                          am_util_pp2[uNumber].M_SIMOBUCK14,        \
                          am_util_pp2[uNumber].M_SIMOBUCK15,        \
                          am_util_pp2[uNumber].M_PWRSW0,            \
                          am_util_pp2[uNumber].M_PWRSW1,            \
                          am_util_pp2[uNumber].M_USBRSTCTRL,        \
                          am_util_pp2[uNumber].M_FLASHWPROT0,       \
                          am_util_pp2[uNumber].M_FLASHWPROT1,       \
                          am_util_pp2[uNumber].M_FLASHWPROT2,       \
                          am_util_pp2[uNumber].M_FLASHWPROT3,       \
                          am_util_pp2[uNumber].M_FLASHRPROT0,       \
                          am_util_pp2[uNumber].M_FLASHRPROT1,       \
                          am_util_pp2[uNumber].M_FLASHRPROT2,       \
                          am_util_pp2[uNumber].M_FLASHRPROT3,       \
                          am_util_pp2[uNumber].M_SRAMWPROT0,        \
                          am_util_pp2[uNumber].M_SRAMWPROT1,        \
                          am_util_pp2[uNumber].M_SRAMWPROT2,        \
                          am_util_pp2[uNumber].M_SRAMWPROT3,        \
                          am_util_pp2[uNumber].M_SRAMWPROT4,        \
                          am_util_pp2[uNumber].M_SRAMWPROT5,        \
                          am_util_pp2[uNumber].M_SRAMRPROT0,        \
                          am_util_pp2[uNumber].M_SRAMRPROT1,        \
                          am_util_pp2[uNumber].M_SRAMRPROT2,        \
                          am_util_pp2[uNumber].M_SRAMRPROT3,        \
                          am_util_pp2[uNumber].M_SRAMRPROT4,        \
                          am_util_pp2[uNumber].M_SRAMRPROT5,        \
                          am_util_pp2[uNumber].M_CPUICACHETRIM,     \
                          am_util_pp2[uNumber].M_CPUDCACHETRIM,     \
                          am_util_pp2[uNumber].M_SSRAMTRIM,         \
                          am_util_pp2[uNumber].M_AUDADCPWRCTRL,     \
                          am_util_pp2[uNumber].M_AUDIO1,            \
                          am_util_pp2[uNumber].M_PGAADCIFCTRL,      \
                          am_util_pp2[uNumber].M_PGACTRL1,          \
                          am_util_pp2[uNumber].M_PGACTRL2,          \
                          am_util_pp2[uNumber].M_AUDADCPWRDLY,      \
                          am_util_pp2[uNumber].M_SDIO0CTRL,         \
                          am_util_pp2[uNumber].M_SDIO1CTRL,         \
                          am_util_pp2[uNumber].M_PDMCTRL,           \
                          am_util_pp2[uNumber].M_MMSMISCCTRL,       \
                          am_util_pp2[uNumber].M_FLASHWPROT4,       \
                          am_util_pp2[uNumber].M_FLASHWPROT5,       \
                          am_util_pp2[uNumber].M_FLASHWPROT6,       \
                          am_util_pp2[uNumber].M_FLASHWPROT7,       \
                          am_util_pp2[uNumber].M_FLASHRPROT4,       \
                          am_util_pp2[uNumber].M_FLASHRPROT5,       \
                          am_util_pp2[uNumber].M_FLASHRPROT6,       \
                          am_util_pp2[uNumber].M_FLASHRPROT7,       \
                          am_util_pp2[uNumber].M_PWRSW2,            \
                          am_util_pp2[uNumber].M_CPUCFG,            \
                          am_util_pp2[uNumber].M_PLLCTL0,           \
                          am_util_pp2[uNumber].M_PLLDIV0,           \
                          am_util_pp2[uNumber].M_PLLDIV1,           \
                          am_util_pp2[uNumber].M_PLLSTAT,           \
                          am_util_pp2[uNumber].M_PLLMUXCTL);

    am_util_stdio_printf(audadcStr, am_util_pp4[uNumber].AU_CFG,    \
                          am_util_pp4[uNumber].AU_STAT,             \
                          am_util_pp4[uNumber].AU_SWT,              \
                          am_util_pp4[uNumber].AU_SL0CFG,           \
                          am_util_pp4[uNumber].AU_SL1CFG,           \
                          am_util_pp4[uNumber].AU_SL2CFG,           \
                          am_util_pp4[uNumber].AU_SL3CFG,           \
                          am_util_pp4[uNumber].AU_SL4CFG,           \
                          am_util_pp4[uNumber].AU_SL5CFG,           \
                          am_util_pp4[uNumber].AU_SL6CFG,           \
                          am_util_pp4[uNumber].AU_SL7CFG,           \
                          am_util_pp4[uNumber].AU_WULIM,            \
                          am_util_pp4[uNumber].AU_WLLIM,            \
                          am_util_pp4[uNumber].AU_SCWLIM,           \
                          am_util_pp4[uNumber].AU_FIFO,             \
                          am_util_pp4[uNumber].AU_FIFOPR,           \
                          am_util_pp4[uNumber].AU_FIFOSTAT,         \
                          am_util_pp4[uNumber].AU_DATAOFFSET,       \
                          am_util_pp4[uNumber].AU_ZXCFG,            \
                          am_util_pp4[uNumber].AU_ZXLIM,            \
                          am_util_pp4[uNumber].AU_GAINCFG,          \
                          am_util_pp4[uNumber].AU_GAIN,             \
                          am_util_pp4[uNumber].AU_SATCFG,           \
                          am_util_pp4[uNumber].AU_SATLIM,           \
                          am_util_pp4[uNumber].AU_SATMAX,           \
                          am_util_pp4[uNumber].AU_SATCLR,           \
                          am_util_pp4[uNumber].AU_IEREN,            \
                          am_util_pp4[uNumber].AU_IERSTAT,          \
                          am_util_pp4[uNumber].AU_IERCLR,           \
                          am_util_pp4[uNumber].AU_IERSET,           \
                          am_util_pp4[uNumber].AU_DMATRIGEN,        \
                          am_util_pp4[uNumber].AU_DMATRIGSTAT,      \
                          am_util_pp4[uNumber].AU_DMACFG,           \
                          am_util_pp4[uNumber].AU_DMATOTCOUNT,      \
                          am_util_pp4[uNumber].AU_DMATARGADDR,      \
                          am_util_pp4[uNumber].AU_DMASTAT);

    am_util_stdio_printf(clkStr, am_util_pp3[uNumber].C_OCTRL,      \
                          am_util_pp3[uNumber].C_CLKOUT,            \
                          am_util_pp3[uNumber].C_HFADJ,             \
                          am_util_pp3[uNumber].C_CLOCKENSTAT,       \
                          am_util_pp3[uNumber].C_CLOCKEN2STAT,      \
                          am_util_pp3[uNumber].C_CLOCKEN3STAT,      \
                          am_util_pp3[uNumber].C_MISC,              \
                          am_util_pp3[uNumber].C_HF2ADJ0,           \
                          am_util_pp3[uNumber].C_HF2ADJ1,           \
                          am_util_pp3[uNumber].C_HF2ADJ2,           \
                          am_util_pp3[uNumber].C_HF2VAL,            \
                          am_util_pp3[uNumber].C_LFRCCTRL,          \
                          am_util_pp3[uNumber].C_DISPCLKCTRL,       \
                          am_util_pp3[uNumber].C_CLKGENSPARES,      \
                          am_util_pp3[uNumber].C_HFRCIDLECOUNTERS,  \
                          am_util_pp3[uNumber].C_MSPIIOCLKCTRL,     \
                          am_util_pp3[uNumber].C_CLKCTRL);

   am_util_stdio_printf(timerStr, am_util_pp3[uNumber].ST_STCFG,    \
                          am_util_pp3[uNumber].ST_STMINTSTAT,       \
                          am_util_pp3[uNumber].T_CTRL,              \
                          am_util_pp3[uNumber].T_STATUS,            \
                          am_util_pp3[uNumber].T_GLOBEN,            \
                          am_util_pp3[uNumber].T_INTSTAT,           \
                          am_util_pp3[uNumber].T_CTRL0,             \
                          am_util_pp3[uNumber].T_CTRL1,             \
                          am_util_pp3[uNumber].T_CTRL2,             \
                          am_util_pp3[uNumber].T_CTRL3,             \
                          am_util_pp3[uNumber].T_CTRL4,             \
                          am_util_pp3[uNumber].T_CTRL5,             \
                          am_util_pp3[uNumber].T_CTRL6,             \
                          am_util_pp3[uNumber].T_CTRL7,             \
                          am_util_pp3[uNumber].T_CTRL8,             \
                          am_util_pp3[uNumber].T_CTRL9,             \
                          am_util_pp3[uNumber].T_CTRL10,            \
                          am_util_pp3[uNumber].T_CTRL11,            \
                          am_util_pp3[uNumber].T_CTRL12,            \
                          am_util_pp3[uNumber].T_CTRL13,            \
                          am_util_pp3[uNumber].T_CTRL14,            \
                          am_util_pp3[uNumber].T_CTRL15 );

   am_util_stdio_printf(pdmStr, am_util_pp5[uNumber].PDM_CTRL,      \
                          am_util_pp5[uNumber].PDM_CORECFG0,        \
                          am_util_pp5[uNumber].PDM_CORECFG1,        \
                          am_util_pp5[uNumber].PDM_CORECTRL,        \
                          am_util_pp5[uNumber].PDM_FIFOCNT,         \
                          am_util_pp5[uNumber].PDM_FIFOREAD,        \
                          am_util_pp5[uNumber].PDM_FIFOFLUSH,       \
                          am_util_pp5[uNumber].PDM_FIFOTHR,         \
                          am_util_pp5[uNumber].PDM_INTEN,           \
                          am_util_pp5[uNumber].PDM_INTSTAT,         \
                          am_util_pp5[uNumber].PDM_INTCLR,          \
                          am_util_pp5[uNumber].PDM_INTSET,          \
                          am_util_pp5[uNumber].PDM_DMATRIGEN,       \
                          am_util_pp5[uNumber].PDM_DMATRIGSTAT,     \
                          am_util_pp5[uNumber].PDM_DMACFG,          \
                          am_util_pp5[uNumber].PDM_DMATARGADDR,     \
                          am_util_pp5[uNumber].PDM_DMASTAT,         \
                          am_util_pp5[uNumber].PDM_DMATARGADDRNEXT, \
                          am_util_pp5[uNumber].PDM_DMATOTCOUNTNEXT, \
                          am_util_pp5[uNumber].PDM_DMAENNEXTCTRL,   \
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
        am_util_pp1[uNumber].P_MCUPERFREQ       = PWRCTRL-> MCUPERFREQ;
        am_util_pp1[uNumber].P_DEVPWREN         = PWRCTRL->DEVPWREN;
        am_util_pp1[uNumber].P_DEVPWRSTATUS     = PWRCTRL->DEVPWRSTATUS;
        am_util_pp1[uNumber].P_AUDSSPWREN       = PWRCTRL->AUDSSPWREN;
        am_util_pp1[uNumber].P_AUDSSPWRSTATUS   = PWRCTRL->AUDSSPWRSTATUS;
        am_util_pp1[uNumber].P_MEMPWREN         = PWRCTRL->MEMPWREN;
        am_util_pp1[uNumber].P_MEMPWRSTATUS     = PWRCTRL->MEMPWRSTATUS;
        am_util_pp1[uNumber].P_MEMRETCFG        = PWRCTRL->MEMRETCFG;
        am_util_pp1[uNumber].P_SYSPWRSTATUS     = PWRCTRL->SYSPWRSTATUS;
        am_util_pp1[uNumber].P_SSRAMPWREN       = PWRCTRL->SSRAMPWREN;
        am_util_pp1[uNumber].P_SSRAMPWRST       = PWRCTRL->SSRAMPWRST;
        am_util_pp1[uNumber].P_SSRAMRETCFG      = PWRCTRL->SSRAMRETCFG;
        am_util_pp1[uNumber].P_DEVPWREVENTEN    = PWRCTRL->DEVPWREVENTEN;
        am_util_pp1[uNumber].P_MEMPWREVENTEN    = PWRCTRL->MEMPWREVENTEN;
        am_util_pp1[uNumber].P_MMSOVERRIDE      = PWRCTRL->MMSOVERRIDE;
        am_util_pp1[uNumber].P_CPUPWRCTRL       = PWRCTRL->CPUPWRCTRL;
        am_util_pp1[uNumber].P_PWRCTRLMODESTATUS = PWRCTRL->PWRCTRLMODESTATUS;
        am_util_pp1[uNumber].P_CPUPWRSTATUS     = PWRCTRL->CPUPWRSTATUS;
        am_util_pp1[uNumber].P_PWRACKOVR        = PWRCTRL->PWRACKOVR;
        am_util_pp1[uNumber].P_PWRCNTDEFVAL     = PWRCTRL->PWRCNTDEFVAL;
        am_util_pp1[uNumber].P_GFXPERFREQ       = PWRCTRL->GFXPERFREQ;
        am_util_pp1[uNumber].P_GFXPWRSWSEL      = PWRCTRL->GFXPWRSWSEL;
        am_util_pp1[uNumber].P_EPURETCFG        = PWRCTRL->EPURETCFG;
        am_util_pp1[uNumber].P_VRCTRL           = PWRCTRL->VRCTRL;
        am_util_pp1[uNumber].P_LEGACYVRLPOVR    = PWRCTRL->LEGACYVRLPOVR;
        am_util_pp1[uNumber].P_VRSTATUS         = PWRCTRL->VRSTATUS;
        am_util_pp1[uNumber].P_SRAMCTRL         = PWRCTRL->SRAMCTRL;
        am_util_pp1[uNumber].P_ADCSTATUS        = PWRCTRL->ADCSTATUS;
        am_util_pp1[uNumber].P_AUDADCSTATUS     = PWRCTRL->AUDADCSTATUS;
        am_util_pp1[uNumber].P_TONCNTRCTRL      = PWRCTRL->TONCNTRCTRL;
        am_util_pp1[uNumber].P_LPOVRTHRESHVDDS  = PWRCTRL->LPOVRTHRESHVDDS;
        am_util_pp1[uNumber].P_LPOVRHYSTCNT     = PWRCTRL->LPOVRHYSTCNT;
        am_util_pp1[uNumber].P_LPOVRTHRESHVDDF  = PWRCTRL->LPOVRTHRESHVDDF;
        am_util_pp1[uNumber].P_LPOVRTHRESHVDDC  = PWRCTRL->LPOVRTHRESHVDDC;
        am_util_pp1[uNumber].P_LPOVRTHRESHVDDCLV = PWRCTRL->LPOVRTHRESHVDDCLV;
        am_util_pp1[uNumber].P_LPOVRSTAT        = PWRCTRL->LPOVRSTAT;
        am_util_pp1[uNumber].P_MRAMEXTCTRL      = PWRCTRL->MRAMEXTCTRL;
        am_util_pp1[uNumber].P_EMONCTRL         = PWRCTRL->EMONCTRL;
        am_util_pp1[uNumber].P_EMONCFG0         = PWRCTRL->EMONCFG0;
        am_util_pp1[uNumber].P_EMONCFG1         = PWRCTRL->EMONCFG1;
        am_util_pp1[uNumber].P_EMONCFG2         = PWRCTRL->EMONCFG2;
        am_util_pp1[uNumber].P_EMONCFG3         = PWRCTRL->EMONCFG3;
        am_util_pp1[uNumber].P_EMONCFG4         = PWRCTRL->EMONCFG4;
        am_util_pp1[uNumber].P_EMONCFG5         = PWRCTRL->EMONCFG5;
        am_util_pp1[uNumber].P_EMONCFG6         = PWRCTRL->EMONCFG6;
        am_util_pp1[uNumber].P_EMONCFG7         = PWRCTRL->EMONCFG7;
        am_util_pp1[uNumber].P_EMONCOUNT0       = PWRCTRL->EMONCOUNT0;
        am_util_pp1[uNumber].P_EMONCOUNT1       = PWRCTRL->EMONCOUNT1;
        am_util_pp1[uNumber].P_EMONCOUNT2       = PWRCTRL->EMONCOUNT2;
        am_util_pp1[uNumber].P_EMONCOUNT3       = PWRCTRL->EMONCOUNT3;
        am_util_pp1[uNumber].P_EMONCOUNT4       = PWRCTRL->EMONCOUNT4;
        am_util_pp1[uNumber].P_EMONCOUNT5       = PWRCTRL->EMONCOUNT5;
        am_util_pp1[uNumber].P_EMONCOUNT6       = PWRCTRL->EMONCOUNT6;
        am_util_pp1[uNumber].P_EMONCOUNT7       = PWRCTRL->EMONCOUNT7;
        am_util_pp1[uNumber].P_EMONSTATUS       = PWRCTRL->EMONSTATUS;

        //
        //append the FPIO here
        //
        am_util_pp1[uNumber].P_FPIOEN0          = FPIO-> EN0 ;
        am_util_pp1[uNumber].P_FPIOEN1          = FPIO-> EN1;
        am_util_pp1[uNumber].P_FPIOEN2          = FPIO-> EN2;

        //
        // Function block 2: MCU_CTRL
        //
        am_util_pp2[uNumber].M_CHIPPN           = MCUCTRL->CHIPPN;
        am_util_pp2[uNumber].M_CHIPID0          = MCUCTRL->CHIPID0;
        am_util_pp2[uNumber].M_CHIPID1          = MCUCTRL->CHIPID1;
        am_util_pp2[uNumber].M_CHIPREV          = MCUCTRL->CHIPREV;
        am_util_pp2[uNumber].M_VENDORID         = MCUCTRL->VENDORID;
        am_util_pp2[uNumber].M_SKU              = MCUCTRL->SKU;
        am_util_pp2[uNumber].M_SKUOVERRIDE      = MCUCTRL->SKUOVERRIDE;
        am_util_pp2[uNumber].M_DEBUGGER         = MCUCTRL->DEBUGGER;
        am_util_pp2[uNumber].M_ACRG             = MCUCTRL->ACRG;
        am_util_pp2[uNumber].M_VREFGEN          = MCUCTRL_VREFGEN;
        am_util_pp2[uNumber].M_VREFGEN2         = MCUCTRL->VREFGEN2;
        am_util_pp2[uNumber].M_VREFGEN3         = MCUCTRL_VREFGEN3;
        am_util_pp2[uNumber].M_VREFGEN4         = MCUCTRL->VREFGEN4;
        am_util_pp2[uNumber].M_VREFGEN5         = MCUCTRL_VREFGEN5;
        am_util_pp2[uNumber].M_VREFBUF          = MCUCTRL->VREFBUF;
        am_util_pp2[uNumber].M_VRCTRL           = MCUCTRL->VRCTRL;
        am_util_pp2[uNumber].M_LDOREG1          = MCUCTRL->LDOREG1;
        am_util_pp2[uNumber].M_LDOREG2          = MCUCTRL->LDOREG2;
        am_util_pp2[uNumber].M_HFRC             = MCUCTRL_HFRC;
        am_util_pp2[uNumber].M_HFRC2            = MCUCTRL_HFRC2;
        am_util_pp2[uNumber].M_LFRC             = MCUCTRL->LFRC;
        am_util_pp2[uNumber].M_BODCTRL          = MCUCTRL->BODCTRL;
        am_util_pp2[uNumber].M_ADCPWRCTRL       = MCUCTRL->ADCPWRCTRL;
        am_util_pp2[uNumber].M_ADCCAL           = MCUCTRL->ADCCAL;
        am_util_pp2[uNumber].M_ADCBATTLOAD      = MCUCTRL->ADCBATTLOAD;
        am_util_pp2[uNumber].M_XTALCTRL         = MCUCTRL->XTALCTRL;
        am_util_pp2[uNumber].M_XTALGENCTRL      = MCUCTRL->XTALGENCTRL;
        am_util_pp2[uNumber].M_XTALHSTRIMS      = MCUCTRL->XTALHSTRIMS;
        am_util_pp2[uNumber].M_XTALHSCTRL       = MCUCTRL->XTALHSCTRL;
        am_util_pp2[uNumber].M_BGTLPCTRL        = MCUCTRL->BGTLPCTRL;
        am_util_pp2[uNumber].M_MRAMCRYPTOPWRCTRL = MCUCTRL->MRAMCRYPTOPWRCTRL;
        am_util_pp2[uNumber].M_MISCPWRCTRL      = MCUCTRL_MISCPWRCTRL;
        am_util_pp2[uNumber].M_BODISABLE        = MCUCTRL->BODISABLE;
        am_util_pp2[uNumber].M_D2ASPARE         = MCUCTRL->D2ASPARE;
        am_util_pp2[uNumber].M_BOOTLOADER       = MCUCTRL->BOOTLOADER;
        am_util_pp2[uNumber].M_SHADOWVALID      = MCUCTRL->SHADOWVALID;
        am_util_pp2[uNumber].M_SCRATCH0         = MCUCTRL->SCRATCH0;
        am_util_pp2[uNumber].M_SCRATCH1         = MCUCTRL->SCRATCH1;
        am_util_pp2[uNumber].M_DBGR1            = MCUCTRL->DBGR1;
        am_util_pp2[uNumber].M_DBGR2            = MCUCTRL->DBGR2;
        am_util_pp2[uNumber].M_WICCONTROL       = MCUCTRL->WICCONTROL;
        am_util_pp2[uNumber].M_DBGCTRL          = MCUCTRL->DBGCTRL;
        am_util_pp2[uNumber].M_OTAPOINTER       = MCUCTRL->OTAPOINTER;
        am_util_pp2[uNumber].M_APBDMACTRL       = MCUCTRL->APBDMACTRL;
        am_util_pp2[uNumber].M_FORCEAXICLKEN    = MCUCTRL->FORCEAXICLKEN;
        am_util_pp2[uNumber].M_SRAMTRIMHP       = MCUCTRL_SRAMTRIMHP;
        am_util_pp2[uNumber].M_SRAMTRIM         = MCUCTRL_SRAMTRIM;
        am_util_pp2[uNumber].M_KEXTCLKSEL       = MCUCTRL->KEXTCLKSEL;
        am_util_pp2[uNumber].M_SIMOBUCK0        = MCUCTRL->SIMOBUCK0;
        am_util_pp2[uNumber].M_SIMOBUCK1        = MCUCTRL->SIMOBUCK1;
        am_util_pp2[uNumber].M_SIMOBUCK2        = MCUCTRL->SIMOBUCK2;
        am_util_pp2[uNumber].M_SIMOBUCK3        = MCUCTRL->SIMOBUCK3;
        am_util_pp2[uNumber].M_SIMOBUCK4        = MCUCTRL->SIMOBUCK4;
        am_util_pp2[uNumber].M_SIMOBUCK5        = MCUCTRL->SIMOBUCK5;
        am_util_pp2[uNumber].M_SIMOBUCK6        = MCUCTRL->SIMOBUCK6;
        am_util_pp2[uNumber].M_SIMOBUCK7        = MCUCTRL->SIMOBUCK7;
        am_util_pp2[uNumber].M_SIMOBUCK8        = MCUCTRL->SIMOBUCK8;
        am_util_pp2[uNumber].M_SIMOBUCK9        = MCUCTRL->SIMOBUCK9;
        am_util_pp2[uNumber].M_SIMOBUCK10       = MCUCTRL->SIMOBUCK10;
        am_util_pp2[uNumber].M_SIMOBUCK11       = MCUCTRL->SIMOBUCK11;
        am_util_pp2[uNumber].M_SIMOBUCK12       = MCUCTRL->SIMOBUCK12;
        am_util_pp2[uNumber].M_SIMOBUCK13       = MCUCTRL->SIMOBUCK13;
        am_util_pp2[uNumber].M_SIMOBUCK14       = MCUCTRL->SIMOBUCK14;
        am_util_pp2[uNumber].M_SIMOBUCK15       = MCUCTRL->SIMOBUCK15;
        am_util_pp2[uNumber].M_PWRSW0           = MCUCTRL->PWRSW0;
        am_util_pp2[uNumber].M_PWRSW1           = MCUCTRL->PWRSW1;
        am_util_pp2[uNumber].M_USBRSTCTRL       = MCUCTRL->USBRSTCTRL;
        am_util_pp2[uNumber].M_FLASHWPROT0      = MCUCTRL->FLASHWPROT0;
        am_util_pp2[uNumber].M_FLASHWPROT1      = MCUCTRL->FLASHWPROT1;
        am_util_pp2[uNumber].M_FLASHWPROT2      = MCUCTRL->FLASHWPROT2;
        am_util_pp2[uNumber].M_FLASHWPROT3      = MCUCTRL->FLASHWPROT3;
        am_util_pp2[uNumber].M_FLASHRPROT0      = MCUCTRL->FLASHRPROT0;
        am_util_pp2[uNumber].M_FLASHRPROT1      = MCUCTRL->FLASHRPROT1;
        am_util_pp2[uNumber].M_FLASHRPROT2      = MCUCTRL->FLASHRPROT2;
        am_util_pp2[uNumber].M_FLASHRPROT3      = MCUCTRL->FLASHRPROT3;
        am_util_pp2[uNumber].M_SRAMWPROT0       = MCUCTRL->SRAMWPROT0;
        am_util_pp2[uNumber].M_SRAMWPROT1       = MCUCTRL->SRAMWPROT1;
        am_util_pp2[uNumber].M_SRAMWPROT2       = MCUCTRL->SRAMWPROT2;
        am_util_pp2[uNumber].M_SRAMWPROT3       = MCUCTRL->SRAMWPROT3;
        am_util_pp2[uNumber].M_SRAMWPROT4       = MCUCTRL->SRAMWPROT4;
        am_util_pp2[uNumber].M_SRAMWPROT5       = MCUCTRL->SRAMWPROT5;
        am_util_pp2[uNumber].M_SRAMRPROT0       = MCUCTRL->SRAMRPROT0;
        am_util_pp2[uNumber].M_SRAMRPROT1       = MCUCTRL->SRAMRPROT1;
        am_util_pp2[uNumber].M_SRAMRPROT2       = MCUCTRL->SRAMRPROT2;
        am_util_pp2[uNumber].M_SRAMRPROT3       = MCUCTRL->SRAMRPROT3;
        am_util_pp2[uNumber].M_SRAMRPROT4       = MCUCTRL->SRAMRPROT4;
        am_util_pp2[uNumber].M_SRAMRPROT5       = MCUCTRL->SRAMRPROT5;
        am_util_pp2[uNumber].M_CPUICACHETRIM    = MCUCTRL_CPUICACHETRIM;
        am_util_pp2[uNumber].M_CPUDCACHETRIM    = MCUCTRL_CPUDCACHETRIM;
        am_util_pp2[uNumber].M_SSRAMTRIM        = MCUCTRL_SSRAMTRIM;
        am_util_pp2[uNumber].M_AUDADCPWRCTRL    = MCUCTRL->AUDADCPWRCTRL;
        am_util_pp2[uNumber].M_AUDIO1           = MCUCTRL->AUDIO1;
        am_util_pp2[uNumber].M_PGAADCIFCTRL     = MCUCTRL->PGAADCIFCTRL;
        am_util_pp2[uNumber].M_PGACTRL1         = MCUCTRL->PGACTRL1;
        am_util_pp2[uNumber].M_PGACTRL2         = MCUCTRL->PGACTRL2;
        am_util_pp2[uNumber].M_AUDADCPWRDLY     = MCUCTRL->AUDADCPWRDLY;
        am_util_pp2[uNumber].M_SDIO0CTRL        = MCUCTRL->SDIO0CTRL;
        am_util_pp2[uNumber].M_SDIO1CTRL        = MCUCTRL->SDIO1CTRL;
        am_util_pp2[uNumber].M_PDMCTRL          = MCUCTRL->PDMCTRL;
        am_util_pp2[uNumber].M_MMSMISCCTRL      = MCUCTRL->MMSMISCCTRL;
        am_util_pp2[uNumber].M_FLASHWPROT4      = MCUCTRL->FLASHWPROT4;
        am_util_pp2[uNumber].M_FLASHWPROT5      = MCUCTRL->FLASHWPROT5;
        am_util_pp2[uNumber].M_FLASHWPROT6      = MCUCTRL->FLASHWPROT6;
        am_util_pp2[uNumber].M_FLASHWPROT7      = MCUCTRL->FLASHWPROT7;
        am_util_pp2[uNumber].M_FLASHRPROT4      = MCUCTRL->FLASHRPROT4;
        am_util_pp2[uNumber].M_FLASHRPROT5      = MCUCTRL->FLASHRPROT5;
        am_util_pp2[uNumber].M_FLASHRPROT6      = MCUCTRL->FLASHRPROT6;
        am_util_pp2[uNumber].M_FLASHRPROT7      = MCUCTRL->FLASHRPROT7;
        am_util_pp2[uNumber].M_PWRSW2           = MCUCTRL->PWRSW2;
        am_util_pp2[uNumber].M_CPUCFG           = MCUCTRL->CPUCFG;
        am_util_pp2[uNumber].M_PLLCTL0          = MCUCTRL->PLLCTL0;
        am_util_pp2[uNumber].M_PLLDIV0          = MCUCTRL->PLLDIV0;
        am_util_pp2[uNumber].M_PLLDIV1          = MCUCTRL->PLLDIV1;
        am_util_pp2[uNumber].M_PLLSTAT          = MCUCTRL->PLLSTAT;
        am_util_pp2[uNumber].M_PLLMUXCTL        = MCUCTRL->PLLMUXCTL;

        //
        //Function Block 3: CLKGEN
        //
        am_util_pp3[uNumber].C_OCTRL            = CLKGEN->OCTRL;
        am_util_pp3[uNumber].C_CLKOUT           = CLKGEN->CLKOUT;
        am_util_pp3[uNumber].C_HFADJ            = CLKGEN->HFADJ;
        am_util_pp3[uNumber].C_CLOCKENSTAT      = CLKGEN->CLOCKENSTAT;
        am_util_pp3[uNumber].C_CLOCKEN2STAT     = CLKGEN->CLOCKEN2STAT;
        am_util_pp3[uNumber].C_CLOCKEN3STAT     = CLKGEN->CLOCKEN3STAT;
        am_util_pp3[uNumber].C_MISC             = CLKGEN->MISC;
        am_util_pp3[uNumber].C_HF2ADJ0          = CLKGEN->HF2ADJ0;
        am_util_pp3[uNumber].C_HF2ADJ1          = CLKGEN->HF2ADJ1;
        am_util_pp3[uNumber].C_HF2ADJ2          = CLKGEN->HF2ADJ2;
        am_util_pp3[uNumber].C_HF2VAL           = CLKGEN->HF2VAL;
        am_util_pp3[uNumber].C_LFRCCTRL         = CLKGEN->LFRCCTRL;
        am_util_pp3[uNumber].C_DISPCLKCTRL      = CLKGEN->DISPCLKCTRL;
        am_util_pp3[uNumber].C_CLKGENSPARES     = CLKGEN->CLKGENSPARES;
        am_util_pp3[uNumber].C_HFRCIDLECOUNTERS = CLKGEN->HFRCIDLECOUNTERS;
        am_util_pp3[uNumber].C_MSPIIOCLKCTRL    = CLKGEN->MSPIIOCLKCTRL;
        am_util_pp3[uNumber].C_CLKCTRL          = CLKGEN->CLKCTRL;

        //
        //Function Block 4: System timer
        //
        am_util_pp3[uNumber].ST_STCFG = STIMER->STCFG;
        am_util_pp3[uNumber].ST_STMINTSTAT = STIMER->STMINTSTAT;

        //
        //Function Block 5: Timer
        //
        am_util_pp3[uNumber].T_CTRL     = TIMER->CTRL;
        am_util_pp3[uNumber].T_STATUS   = TIMER->STATUS;
        am_util_pp3[uNumber].T_GLOBEN   = TIMER->GLOBEN;
        am_util_pp3[uNumber].T_INTSTAT  = TIMER->INTSTAT;
        am_util_pp3[uNumber].T_CTRL0    = TIMER->CTRL0;
        am_util_pp3[uNumber].T_CTRL1    = TIMER->CTRL1;
        am_util_pp3[uNumber].T_CTRL2    = TIMER->CTRL2;
        am_util_pp3[uNumber].T_CTRL3    = TIMER->CTRL3;
        am_util_pp3[uNumber].T_CTRL4    = TIMER->CTRL4;
        am_util_pp3[uNumber].T_CTRL5    = TIMER->CTRL5;
        am_util_pp3[uNumber].T_CTRL6    = TIMER->CTRL6;
        am_util_pp3[uNumber].T_CTRL7    = TIMER->CTRL7;
        am_util_pp3[uNumber].T_CTRL8    = TIMER->CTRL8;
        am_util_pp3[uNumber].T_CTRL9    = TIMER->CTRL9;
        am_util_pp3[uNumber].T_CTRL10   = TIMER->CTRL10;
        am_util_pp3[uNumber].T_CTRL11   = TIMER->CTRL11;
        am_util_pp3[uNumber].T_CTRL12   = TIMER->CTRL12;
        am_util_pp3[uNumber].T_CTRL13   = TIMER->CTRL13;
        am_util_pp3[uNumber].T_CTRL14   = TIMER->CTRL14;
        am_util_pp3[uNumber].T_CTRL15   = TIMER->CTRL15;

#if USE_AMIC_AUDADC
        //
        //Function Block 6: AUDADC
        //
        if (g_sVosBrd.pvAUDADCHandle && (AM_HAL_AUDADC_CHK_HANDLE(g_sVosBrd.pvAUDADCHandle)))
        {
            am_util_pp4[uNumber].AU_CFG         = AUDADC->CFG;
            am_util_pp4[uNumber].AU_STAT        = AUDADC->STAT;
            am_util_pp4[uNumber].AU_SWT         = AUDADC->SWT;
            am_util_pp4[uNumber].AU_SL0CFG      = AUDADC->SL0CFG;
            am_util_pp4[uNumber].AU_SL1CFG      = AUDADC->SL1CFG;
            am_util_pp4[uNumber].AU_SL2CFG      = AUDADC->SL2CFG;
            am_util_pp4[uNumber].AU_SL3CFG      = AUDADC->SL3CFG;
            am_util_pp4[uNumber].AU_SL4CFG      = AUDADC->SL4CFG;
            am_util_pp4[uNumber].AU_SL5CFG      = AUDADC->SL5CFG;
            am_util_pp4[uNumber].AU_SL6CFG      = AUDADC->SL6CFG;
            am_util_pp4[uNumber].AU_SL7CFG      = AUDADC->SL7CFG;
            am_util_pp4[uNumber].AU_WULIM       = AUDADC->WULIM;
            am_util_pp4[uNumber].AU_WLLIM       = AUDADC->WLLIM;
            am_util_pp4[uNumber].AU_SCWLIM      = AUDADC->SCWLIM;
            am_util_pp4[uNumber].AU_FIFO        = AUDADC->FIFO;
            am_util_pp4[uNumber].AU_FIFOPR      = AUDADC->FIFOPR;
            am_util_pp4[uNumber].AU_FIFOSTAT    = AUDADC->FIFOSTAT;
            am_util_pp4[uNumber].AU_DATAOFFSET  = AUDADC->DATAOFFSET;
            am_util_pp4[uNumber].AU_ZXCFG       = AUDADC->ZXCFG;
            am_util_pp4[uNumber].AU_ZXLIM       = AUDADC->ZXLIM;
            am_util_pp4[uNumber].AU_GAINCFG     = AUDADC->GAINCFG;
            am_util_pp4[uNumber].AU_GAIN        = AUDADC->GAIN;
            am_util_pp4[uNumber].AU_SATCFG      = AUDADC->SATCFG;

            am_util_pp4[uNumber].AU_SATLIM      = AUDADC->SATLIM;
            am_util_pp4[uNumber].AU_SATMAX      = AUDADC->SATMAX;
            am_util_pp4[uNumber].AU_SATCLR      = AUDADC->SATCLR;
            am_util_pp4[uNumber].AU_IEREN       = AUDADC->INTEN;
            am_util_pp4[uNumber].AU_IERSTAT     = AUDADC->INTSTAT;
            am_util_pp4[uNumber].AU_IERCLR      = AUDADC->INTCLR;
            am_util_pp4[uNumber].AU_IERSET      = AUDADC->INTSET;
            am_util_pp4[uNumber].AU_DMATRIGEN   = AUDADC->DMATRIGEN;
            am_util_pp4[uNumber].AU_DMATRIGSTAT = AUDADC->DMATRIGSTAT;
            am_util_pp4[uNumber].AU_DMACFG      = AUDADC->DMACFG;
            am_util_pp4[uNumber].AU_DMATOTCOUNT = AUDADC->DMATOTCOUNT;
            am_util_pp4[uNumber].AU_DMATARGADDR = AUDADC->DMATARGADDR;
            am_util_pp4[uNumber].AU_DMASTAT     = AUDADC->DMASTAT;
        }
#endif

#if USE_DMIC_PDM
        if ( g_sVosBrd.pvPDMHandle && (AM_HAL_PDM_HANDLE_VALID(g_sVosBrd.pvPDMHandle)) )
        {
            am_util_pp5[uNumber].PDM_CTRL       = PDMn(0)->CTRL;
            am_util_pp5[uNumber].PDM_CORECFG0   = PDMn(0)->CORECFG0;
            am_util_pp5[uNumber].PDM_CORECFG1   = PDMn(0)->CORECFG1;
            am_util_pp5[uNumber].PDM_CORECTRL   = PDMn(0)->CORECTRL;
            am_util_pp5[uNumber].PDM_FIFOCNT    = PDMn(0)->FIFOCNT;
            am_util_pp5[uNumber].PDM_FIFOREAD   = PDMn(0)->FIFOREAD;
            am_util_pp5[uNumber].PDM_FIFOFLUSH  = PDMn(0)->FIFOFLUSH;
            am_util_pp5[uNumber].PDM_FIFOTHR    = PDMn(0)->FIFOTHR;
            am_util_pp5[uNumber].PDM_INTEN      = PDMn(0)->INTEN;
            am_util_pp5[uNumber].PDM_INTSTAT    = PDMn(0)->INTSTAT;
            am_util_pp5[uNumber].PDM_INTCLR     = PDMn(0)->INTCLR;
            am_util_pp5[uNumber].PDM_INTSET     = PDMn(0)->INTSET;
            am_util_pp5[uNumber].PDM_DMATRIGEN      = PDMn(0)->DMATRIGEN;
            am_util_pp5[uNumber].PDM_DMATRIGSTAT    = PDMn(0)->DMATRIGSTAT;
            am_util_pp5[uNumber].PDM_DMACFG         = PDMn(0)->DMACFG;
            am_util_pp5[uNumber].PDM_DMATARGADDR    = PDMn(0)->DMATARGADDR;
            am_util_pp5[uNumber].PDM_DMASTAT        = PDMn(0)->DMASTAT;
            am_util_pp5[uNumber].PDM_DMATARGADDRNEXT = PDMn(0)->DMATARGADDRNEXT;
            am_util_pp5[uNumber].PDM_DMATOTCOUNTNEXT = PDMn(0)->DMATOTCOUNTNEXT;
            am_util_pp5[uNumber].PDM_DMAENNEXTCTRL  = PDMn(0)->DMAENNEXTCTRL;
            am_util_pp5[uNumber].PDM_DMATOTCOUNT    = PDMn(0)->DMATOTCOUNT;
        }
#endif

       bCaptured[uNumber] = true;  //now the snapshot is in memory
    }
} // am_util_pp_snapshot()

