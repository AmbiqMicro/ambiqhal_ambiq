// ****************************************************************************
//
//! @file am_hal_spotmgr_pcm2_2.c
//!
//! @brief SPOT manager functions that manage power states for PCM2.2 parts
//!
//! @addtogroup spotmgr5b SPOTMGR - SPOT Manager
//! @ingroup apollo510_hal
//! @{
//
// ****************************************************************************

// ****************************************************************************
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
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"

#if !AM_HAL_SPOTMGR_PCM2_2_DISABLE

//
//! Enum for power states descriptions, only bit[0:11] are used to determine power state.
//
typedef enum
{
    AM_HAL_SPOTMGR_POWER_STATE_DESC_0  = 0x000300, // CPULP, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_1  = 0x000200, // CPULP, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_2  = 0x000100, // CPULP, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_3  = 0x000000, // CPULP, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_4  = 0x000310, // CPULP + P, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_5  = 0x000210, // CPULP + P, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_6  = 0x000110, // CPULP + P, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_7  = 0x000010, // CPULP + P, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_8  = 0x000301, // CPUHP, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_9  = 0x000201, // CPUHP, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_10 = 0x000101, // CPUHP, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_11 = 0x000001, // CPUHP, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_12 = 0x000311, // CPUHP + P, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_13 = 0x000211, // CPUHP + P, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_14 = 0x000111, // CPUHP + P, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_15 = 0x000011, // CPUHP + P, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_16 = 0x100310, // CPULP + P + GPU/SDIO, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_17 = 0x100210, // CPULP + P + GPU/SDIO, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_18 = 0x100110, // CPULP + P + GPU/SDIO, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_19 = 0x100010, // CPULP + P + GPU/SDIO, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_20 = 0x100311, // CPUHP + P + GPU/SDIO, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_21 = 0x100211, // CPUHP + P + GPU/SDIO, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_22 = 0x100111, // CPUHP + P + GPU/SDIO, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_23 = 0x100011  // CPUHP + P + GPU/SDIO, Temp 0
} am_hal_spotmgr_power_state_desc_e;

//
//! Enum for Ton states descriptions, only bit[0:3] and bit[12:19] are used to determine Ton state.
//
typedef enum
{
    AM_HAL_SPOTMGR_TON_STATE_DESC_0  = 0x00000, // CPULP
    AM_HAL_SPOTMGR_TON_STATE_DESC_1  = 0x00001, // CPUHP
    AM_HAL_SPOTMGR_TON_STATE_DESC_2  = 0x01000, // CPULP + GPU
    AM_HAL_SPOTMGR_TON_STATE_DESC_3  = 0x11000, // CPULP + GPU + Periph
    AM_HAL_SPOTMGR_TON_STATE_DESC_4  = 0x02000, // CPULP + GPUHP
    AM_HAL_SPOTMGR_TON_STATE_DESC_5  = 0x12000, // CPULP + GPUHP + Periph
    AM_HAL_SPOTMGR_TON_STATE_DESC_6  = 0x01001, // CPUHP + GPU
    AM_HAL_SPOTMGR_TON_STATE_DESC_7  = 0x11001, // CPUHP + GPU + Periph
    AM_HAL_SPOTMGR_TON_STATE_DESC_8  = 0x02001, // CPUHP + GPUHP
    AM_HAL_SPOTMGR_TON_STATE_DESC_9  = 0x12001, // CPUHP + GPUHP + Periph
    AM_HAL_SPOTMGR_TON_STATE_DESC_10 = 0x10000, // CPULP + Periph
    AM_HAL_SPOTMGR_TON_STATE_DESC_11 = 0x10001, // CPUHP + Periph
} am_hal_spotmgr_ton_state_desc_e;

//
//! Enum for power states transition sequences.
//
typedef enum
{
    AM_HAL_SPOTMGR_TRANS_SEQ_0  , // From "PCM2.1 LP" to "PCM2.0 LP peripheral on and GPU&SDIO off" or "PCM2.0 LP GPU/SDIO on"
    AM_HAL_SPOTMGR_TRANS_SEQ_1  , // From "PCM2.1 LP" to "PCM2.1 HP"
    AM_HAL_SPOTMGR_TRANS_SEQ_2  , // From "PCM2.0 LP peripheral on and GPU&SDIO off" or "PCM2.0 LP GPU/SDIO on" to "PCM2.1 LP"
    AM_HAL_SPOTMGR_TRANS_SEQ_3  , // From "PCM2.0 LP peripheral on and GPU&SDIO off" to "PCM2.0 HP peripheral on" or "PCM2.0 LP GPU/SDIO on"
    AM_HAL_SPOTMGR_TRANS_SEQ_4  , // From "PCM2.1 HP" to "PCM2.1 LP"
    AM_HAL_SPOTMGR_TRANS_SEQ_5  , // From "PCM2.1 HP" to "PCM2.0 HP peripheral on"
    AM_HAL_SPOTMGR_TRANS_SEQ_6  , // From "PCM2.0 HP peripheral on" to "PCM2.0 LP peripheral on and GPU&SDIO off"
    AM_HAL_SPOTMGR_TRANS_SEQ_7  , // From "PCM2.0 HP peripheral on" to "PCM2.1 HP"
    AM_HAL_SPOTMGR_TRANS_SEQ_8  , // From "PCM2.0 HP peripheral on" to "PCM2.0 LP GPU/SDIO on"
    AM_HAL_SPOTMGR_TRANS_SEQ_9  , // From "PCM2.0 LP GPU/SDIO on" to "PCM2.0 LP peripheral on and GPU&SDIO off"
    AM_HAL_SPOTMGR_TRANS_SEQ_10 , // From "PCM2.0 LP GPU/SDIO on" to "PCM2.0 HP peripheral on"
    AM_HAL_SPOTMGR_TRANS_SEQ_11 , // Temperature transitions to > 50C and Power state 9 to 8 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_12 , // Temperature transitions to > 50C and Power state 1 to 0 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_13 , // Temperature transitions to < 50C and Power state 8 to 9 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_14 , // Temperature transitions to < 50C and Power state 0 to 1 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_15 , // Temperature transitions to > 0C and Power state 2 to 1 or 10 to 9 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_16 , // Temperature transitions to > 0C and not Power state 2 to 1 or 10 to 9 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_17 , // Temperature transitions to < 0C and Power state 9 to 10 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_18 , // Temperature transitions to < 0C and Power state 1 to 2 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_19 , // Temperature transitions to < 0C and not Power state 9 to 10 or 1 to 2 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_20 , // Temperature transitions: Power state 11 to 10 or 10 to 11 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_21 , // Power state 0 to 8 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_22 , // Power state 8 to 0 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_23 , // Power state 8 to 12 or 12 to 8 transition
    AM_HAL_SPOTMGR_TRANS_SEQ_24 , // Other temperature transitions besides SEQ_11 to SEQ_20
    AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS, // The first entry of Temperature transitions
    AM_HAL_SPOTMGR_TRANS_SEQ_INVALID, // Invalid state transitions
} am_hal_spotmgr_transition_sequence_e;

//
//! Bitfield definitions for power states descriptions
//
typedef union
{
    am_hal_spotmgr_power_state_desc_e ePwrStateDesc;

    am_hal_spotmgr_ton_state_desc_e   eTonStateDesc;

    struct
    {
        //! CPU performance mode
        uint32_t CPUMODE        : 4;
        //! Peripheral power state
        uint32_t PERIPHMODE1  : 4;
        //! Temperature range
        uint32_t TEMPRANGE      : 4;
        //! GPU power state
        uint32_t GPUMODE        : 4;
        //! Peripheral power state
        uint32_t PERIPHMODE     : 4;
        //! GPU or SDIO power state
        uint32_t GPUSDIOMODE       : 4;
        //! Reserved
        uint32_t                : 8;
    } PWRSTATEDESC_b;
} am_hal_spotmgr_power_state_desc_t;

//
//! Function pointer of power state transition sequence.
//
typedef void (*TransitionSequencePtr)(uint32_t, uint32_t, uint32_t, uint32_t);

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//! Only bit[0:11] and bit[20:23] are used to determine power state.
#define PWR_STATE_DESC_MASK 0x00F00FFF

//! Only bit[0:3] and bit[12:19] are used to determine ton state.
#define TON_STATE_DESC_MASK 0x000FF00F

//! Maximum delay for waiting for timer AM_HAL_INTERNAL_TIMER_NUM_A is disabled
#define TIMER_A_DISABLE_WAIT_IN_US   2500

//! Use the blocking delay instead of the timer
#define USE_DELAY_INSTEAD_OF_TIMER false

//! VDDCLVACTLOWTONTRIM trim code for all other power states except power state 1 (and 5).
#define VDDCLVACTLOWTONTRIM_DEFAULT 6

//! Mask of power state group
#define STATE_GROUP_MASK 0xFFFFFFFC

//! Mask of temperange range of power state
#define STATE_TEMP_RANGE_MASK 0x00000003

//! Get the power state group index
#define StateGroup(m) ((m) >> 2)

//! Check if 2 states in the same group
#define IsInSameStateGroup(m, n) (StateGroup(m) == StateGroup(n))

//! Check if 2 states are not in the same temperature range
#define IsNotInSameTempRange(m, n) (((m) & STATE_TEMP_RANGE_MASK) != ((n) & STATE_TEMP_RANGE_MASK))

//! Check if the state is in the temperature range > 50C
#define IsStateGT50C(m) (((m & STATE_TEMP_RANGE_MASK) == 0) && (StateGroup(m) <= 4))

//! Check if the state is in the temperature range <= 50C
#define IsStateLE50C(m) (((m & STATE_TEMP_RANGE_MASK) > 0) && (StateGroup(m) <= 4))

//! Check if the state is in the temperature range > 0C
#define IsStateGT0C(m) (((m & STATE_TEMP_RANGE_MASK) <= 1) && (StateGroup(m) <= 4))

//! Check if the state is in the temperature range <= 0C
#define IsStateLE0C(m) (((m & STATE_TEMP_RANGE_MASK) >= 2) && (StateGroup(m) <= 4))

//! Timer compare value in micro second
#define MAX_WAIT_TIME_FOR_SPOTMGR_TIMER_EXPIRE_IN_US 60

//! Common header of state transition function.
#define COMMON_HEADER                                                                       \
    uint8_t vddclv[4];                                                                      \
    bool bEnableICache = false;                                                             \
    am_hal_spotmgr_trim_settings_t *pTrimSettings =                                         \
                    &g_sSpotMgrINFO1regs.sPowerStateArray[ui32PwrState];                    \
    am_hal_spotmgr_trim_settings_t *pCurTrimSettings =                                      \
                    &g_sSpotMgrINFO1regs.sPowerStateArray[ui32CurPwrState];                 \
    am_hal_spotmgr_vddclv_trim_settings_t *pVddclvSettings =                                \
                    &g_sSpotMgrINFO1regs.sVddclvActTrimAdj;                                 \
    vddclv[0] = pVddclvSettings->VDDCLVACTTRIMADJ_b.TVRGCLVACTTRIM0;                        \
    vddclv[1] = pVddclvSettings->VDDCLVACTTRIMADJ_b.TVRGCLVACTTRIM1;                        \
    vddclv[2] = pVddclvSettings->VDDCLVACTTRIMADJ_b.TVRGCLVACTTRIM2;                        \
    vddclv[3] = pVddclvSettings->VDDCLVACTTRIMADJ_b.TVRGCLVACTTRIM3;                        \
    uint32_t ui32CurVddfTrim = pCurTrimSettings->PWRSTATE_b.TVRGFACTTRIM;                   \
    uint32_t ui32CurVddcTrim = pCurTrimSettings->PWRSTATE_b.TVRGCACTTRIM;                   \
    uint32_t ui32NewVddcTrim = pTrimSettings->PWRSTATE_b.TVRGCACTTRIM;                      \
    uint32_t ui32NewVddfTrim = pTrimSettings->PWRSTATE_b.TVRGFACTTRIM;                      \
    uint32_t ui32CurVddclvTrim = vddclv[ui32CurPwrState % 4];                               \
    uint32_t ui32NewVddclvTrim = vddclv[ui32PwrState % 4];

//! Update globals
#define UPDATE_GLOBALS                                                                      \
    g_ui32TargetTonState = ui32TonState;                                                    \
    g_ui32TargetPowerState = ui32PwrState;                                                  \
    g_ui32NewCoreldoActTrim = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;                     \
    g_ui32NewCoreldoTempcoTrim = pTrimSettings->PWRSTATE_b.CORELDOTEMPCOTRIM;               \
    g_ui32NewVddcTrim = ui32NewVddcTrim;                                                    \
    g_ui32NewVddfTrim = ui32NewVddfTrim;

//! Wait for spotmgr timer expiration, then call the ISR manually
#define WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR()                                             \
    do                                                                                      \
    {                                                                                       \
        if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)                            \
        {                                                                                   \
            for (uint32_t i = 0; i < MAX_WAIT_TIME_FOR_SPOTMGR_TIMER_EXPIRE_IN_US; i++)     \
            {                                                                               \
                if (TIMER->INTSTAT & AM_HAL_TIMER_MASK(AM_HAL_INTERNAL_TIMER_NUM_A, AM_HAL_TIMER_COMPARE0)) \
                {                                                                           \
                    break;                                                                  \
                }                                                                           \
                else                                                                        \
                {                                                                           \
                    am_hal_delay_us(1);                                                     \
                }                                                                           \
            }                                                                               \
            am_hal_spotmgr_pcm2_2_boost_timer_interrupt_service();                          \
        }                                                                                   \
    } while (0)

//! Adjust Ton trims
#define ADJUST_TON_IF_REQUIRED()                                                            \
    do                                                                                      \
    {                                                                                       \
        spotmgr_power_ton_adjust(ui32TonState, ui32PwrState);                               \
    } while (0)

//! Trim coreldo
#define LOAD_CORELDO_TRIM()                                                                 \
    do                                                                                      \
    {                                                                                       \
        MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM = pTrimSettings->PWRSTATE_b.CORELDOTEMPCOTRIM; \
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;    \
    } while (0)

//! Trim VDDC, TVRGCVREFSEL is no longer used, just trim TVRGCVREFTRIM here.
#define LOAD_VDDC_TRIM()                                                                    \
    do                                                                                      \
    {                                                                                       \
        MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = ui32NewVddcTrim;                                \
    } while (0)

//! Trim VDDF
#define LOAD_VDDF_TRIM()                                                                    \
    do                                                                                      \
    {                                                                                       \
        MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = ui32NewVddfTrim;                                \
    } while (0)

//! Trim VDDC_LV
#define LOAD_VDDC_LV_TRIM()                                                                 \
    do                                                                                      \
    {                                                                                       \
        MCUCTRL->VREFGEN3_b.TVRGCLVVREFTRIM = ui32NewVddclvTrim;                            \
    } while (0)

//! Boost VDDF for VDDC and VDDF separation
#define BOOST_VDDF_FOR_SEPARATION(diff)                                                                 \
    do                                                                                      \
    {                                                                                       \
        if ((ui32NewVddfTrim + (diff)) > MAX_VDDF_TRIM)                                     \
        {                                                                                   \
            MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = MAX_VDDF_TRIM;                              \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = ui32NewVddfTrim + (diff);                   \
        }                                                                                   \
    } while (0)

//! Double boost VDDF
#define DOUBLE_BOOST_VDDF()                                                                 \
    do                                                                                      \
    {                                                                                       \
        int32_t i32Diff = ui32NewVddfTrim - ui32CurVddfTrim;                                \
        int32_t i32DblBstDiff = (i32Diff > 0) ? (2 * i32Diff) : 0;                          \
        if ((ui32CurVddfTrim + i32DblBstDiff) > MAX_VDDF_TRIM)                              \
        {                                                                                   \
            MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = MAX_VDDF_TRIM;                              \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = ui32CurVddfTrim + i32DblBstDiff;            \
        }                                                                                   \
    } while (0)

//! Double boost VDDC
#define DOUBLE_BOOST_VDDC()                                                                 \
    do                                                                                      \
    {                                                                                       \
        int32_t i32Diff = ui32NewVddcTrim - ui32CurVddcTrim;                                \
        int32_t i32DblBstDiff = (i32Diff > 0) ? (2 * i32Diff) : 0;                          \
        if ((ui32CurVddcTrim + i32DblBstDiff) > MAX_VDDC_TRIM)                              \
        {                                                                                   \
            MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = MAX_VDDC_TRIM;                            \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = ui32CurVddcTrim + i32DblBstDiff;            \
        }                                                                                   \
    } while (0)

//! Double boost VDDC_LV
#define DOUBLE_BOOST_VDDC_LV()                                                              \
    do                                                                                      \
    {                                                                                       \
        int32_t i32Diff = ui32NewVddclvTrim - ui32CurVddclvTrim;                            \
        int32_t i32DblBstDiff = (i32Diff > 0) ? (2 * i32Diff) : 0;                          \
        if ((ui32CurVddclvTrim + i32DblBstDiff) > MAX_VDDC_LV_TRIM)                         \
        {                                                                                   \
            MCUCTRL->VREFGEN3_b.TVRGCLVVREFTRIM = MAX_VDDC_LV_TRIM;                        \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            MCUCTRL->VREFGEN3_b.TVRGCLVVREFTRIM = (ui32CurVddclvTrim + i32DblBstDiff);      \
        }                                                                                   \
    } while (0)

//! Disable ICache
#define DISABLE_ICACHE()                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (SCB->CCR & SCB_CCR_IC_Msk)                                                      \
        {                                                                                   \
            am_hal_cachectrl_icache_disable();                                              \
            bEnableICache = true;                                                           \
        }                                                                                   \
    } while (0)

//! Enable ICache
#define ENABLE_ICACHE()                                                                     \
    do                                                                                      \
    {                                                                                       \
        if (bEnableICache)                                                                  \
        {                                                                                   \
            am_hal_cachectrl_icache_enable();                                               \
        }                                                                                   \
    } while (0)

//! Switch CPU to LP mode as a workaround
#define SWITCH_TO_LP()                                                                      \
    do                                                                                      \
    {                                                                                       \
        PWRCTRL->MCUPERFREQ_b.MCUPERFREQ = PWRCTRL_MCUPERFREQ_MCUPERFREQ_LP;                \
        for ( uint32_t i = 0; i < AM_HAL_PWRCTRL_PERF_SWITCH_WAIT_US; i++ )                 \
        {                                                                                   \
            if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )                                     \
            {                                                                               \
                break;                                                                      \
            }                                                                               \
            am_hal_delay_us(1);                                                             \
        }                                                                                   \
    } while (0)

//! Switch CPU to HP mode as a workaround
#define SWITCH_TO_HP()                                                                      \
    do                                                                                      \
    {                                                                                       \
        PWRCTRL->MCUPERFREQ_b.MCUPERFREQ = PWRCTRL_MCUPERFREQ_MCUPERFREQ_HP;                \
        for ( uint32_t i = 0; i < AM_HAL_PWRCTRL_PERF_SWITCH_WAIT_US; i++ )                 \
        {                                                                                   \
            if ( PWRCTRL->MCUPERFREQ_b.MCUPERFACK > 0 )                                     \
            {                                                                               \
                break;                                                                      \
            }                                                                               \
            am_hal_delay_us(1);                                                             \
        }                                                                                   \
    } while (0)

//! Wait for power stable
#if AM_HAL_SPOTMGR_REPLACE_DELAY_WITH_WFI
#define WAIT_FOR_POWER_STABLE(m) am_hal_spotmgr_wfi_wait(m)
#else
#define WAIT_FOR_POWER_STABLE(m) am_hal_delay_us(m)
#endif

//! Remove VDDC double boost
#define REMOVE_VDDC_DOUBLE_BOOST()      LOAD_VDDC_TRIM()

//! Remove VDDF double boost
#define REMOVE_VDDF_DOUBLE_BOOST()      LOAD_VDDF_TRIM()

//! Remove VDDC_LV double boost
#define REMOVE_VDDC_LV_DOUBLE_BOOST()   LOAD_VDDC_LV_TRIM()

//! Assume g_sSpotMgrINFO1regs.sPowerStateArray[1].PWRSTATE_b.TVRGFACTTRIM is set to TVRGFACTTRIM0
#define TVRGFACTTRIM0 (g_sSpotMgrINFO1regs.sPowerStateArray[1].PWRSTATE_b.TVRGFACTTRIM)

//! Assume g_sSpotMgrINFO1regs.sPowerStateArray[19].PWRSTATE_b.TVRGFACTTRIM is set to TVRGFACTTRIM2
#define TVRGFACTTRIM2 (g_sSpotMgrINFO1regs.sPowerStateArray[19].PWRSTATE_b.TVRGFACTTRIM)

//! Assume g_sSpotMgrINFO1regs.sPowerStateArray[0].PWRSTATE_b.TVRGFACTTRIM is set to TVRGFACTTRIM3
#define TVRGFACTTRIM3 (g_sSpotMgrINFO1regs.sPowerStateArray[0].PWRSTATE_b.TVRGFACTTRIM)

//*****************************************************************************
//
//! Local function declaration.
//
//*****************************************************************************
static void spotmgr_power_ton_adjust(uint32_t ui32TonState, uint32_t ui32PwrState);

//*****************************************************************************
//
//! Static Variable
//
//*****************************************************************************

//! Whether to run the second part of sequence #21
static bool g_bContinueSeq21b = false;

//! The default power state on PCM2.2 is 7
static uint32_t g_ui32CurPowerStateStatic = 7;

//! Power state before starting timer
static uint32_t g_ui32LastStateBfStartTimer = 0xFF;

//! New VDDC TVRGCVREFTRIM trim value
static uint32_t g_ui32NewVddcTrim = 0;

//! New VDDF TVRGFVREFTRIM trim value
static uint32_t g_ui32NewVddfTrim = 0;

//! New coreldo active trim value
static uint32_t g_ui32NewCoreldoActTrim = 0;

//! New  coreldo tempco trim value
static uint32_t g_ui32NewCoreldoTempcoTrim = 0;

//! Target TON state
static volatile uint32_t g_ui32TargetTonState = 0;

//! Target power state
static volatile uint32_t g_ui32TargetPowerState = 0;

//! Ongoing sequence
static am_hal_spotmgr_transition_sequence_e eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;

#if AM_HAL_SPOTMGR_REPLACE_DELAY_WITH_WFI
//*****************************************************************************
//
// Define a simple function that will go to sleep.
//
//*****************************************************************************
//
// Prototype the assembly function.
//
typedef void (*WFIfunc_t)(void);

#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__align(32)
#define WA_ATTRIB
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
#warning This attribute is not yet tested on ARM6.
#define WA_ATTRIB   __attribute__ ((aligned (32)))
#elif defined(__GNUC_STDC_INLINE__)
#define WA_ATTRIB   __attribute__ ((aligned (32)))
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 32
#define WA_ATTRIB
#else
#error Unknown compiler.
#endif

static
uint16_t WFIRAM[16] WA_ATTRIB =
{
    0xF3BF, 0x8F4F,     // DSB
    0xBF30,             // WFI
    0xF3BF, 0x8F6F,     // ISB
    0x4770,             // bx lr
    0xBF00,             // nop
};
#define DELAY_TIME_COMPENSATION 10

//
// Prototype the assembly function.
//
WFIfunc_t WFIfuncRAM = (WFIfunc_t)((uint8_t *)WFIRAM + 1);

static uint32_t nvic_en[16];
static uint32_t origBasePri, origTimerPri;
static bool g_bSTIntEnabled = false;

//*****************************************************************************
//
// Mask off all interrupts
//
//*****************************************************************************
static inline void
am_critical_mask_all_begin(void)
{
    uint32_t basePrioGrouping = ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));

    //
    // In this HAL, am_critical_mask_all_begin is called from CRITICAL SECTION
    //
    // orgPriMask = am_hal_interrupt_master_disable();
    origBasePri = __get_BASEPRI();
    __set_BASEPRI((basePrioGrouping >= (8 - __NVIC_PRIO_BITS))?(1 << (basePrioGrouping + 1)):(1 << (8 - __NVIC_PRIO_BITS)));
    for (uint32_t i = 0; i < 16; i++)
    {
        nvic_en[i] = NVIC->ISER[i];
        NVIC->ICER[i] = nvic_en[i];
    }
    //
    // Disable SysTick Int if it was enabled.
    //
    if (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)
    {
        g_bSTIntEnabled = true;
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }

} // am_critical_mask_all_begin()

//*****************************************************************************
//
//  Restore the interrupts
//
//*****************************************************************************
static inline void
am_critical_mask_all_end(void)
{
    uint32_t ui32STVal = 0;
    bool bSetSTPend = false;

    for (uint32_t i = 0; i < 16; i++ )
    {
        NVIC->ISER[i] = nvic_en[i];
    }
    __set_BASEPRI(origBasePri);

    if (g_bSTIntEnabled)
    {
        //
        // Address potential race condition where Systick interrupt could have
        // been fired - while we disabled it
        //
        // Current snapshot for Systick
        //
        ui32STVal = SysTick->VAL;

        //
        // Need to read COUNTFLAG before writing to CTRL, as it would get reset
        //
        bSetSTPend = SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;

        //
        // Enable SysTick Int
        //
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

        //
        // Read Systick again to see if it wrapped
        //
        if (SysTick->VAL > ui32STVal)
        {
              bSetSTPend = true;
        }

        //
        // Set PENDSTSET
        //
        if (bSetSTPend)
        {
            SCB->ICSR |= SCB_ICSR_PENDSTSET_Msk;
        }
    }
    //
    // In this HAL, am_critical_mask_all_begin is called from CRITICAL SECTION
    //
    // am_hal_interrupt_master_set(orgPriMask);
} // am_critical_mask_all_end()

//*****************************************************************************
//
// am_hal_spotmgr_wfi_wait
//
//*****************************************************************************
static void
am_hal_spotmgr_wfi_wait(uint32_t ui32Delayus)
{
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sActCpdlpConfig;
    uint32_t ui32CpdlpConfig = 0;

    //
    // Mask off all interrupts
    //
    am_critical_mask_all_begin();

    //
    // This interrupt needs to be set as the highest priority (0)
    //
    origTimerPri = NVIC->IPR[TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A];
    NVIC->IPR[TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A] = (uint8_t)((0 << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);

    //
    // Start the timer.
    //
    am_hal_spotmgr_timer_start(ui32Delayus - DELAY_TIME_COMPENSATION);

    //
    // CPDLPSTATE may affect DBG power status, remove it before we understand this.
    //
    // Get the current CPDLPSTATE configuration in active mode
    //
    am_hal_pwrctrl_pwrmodctl_cpdlp_get(&sActCpdlpConfig);

    //
    // Prepare the data for restoring CPDLPSTATE configuration after waking up
    //
    ui32CpdlpConfig |= (sActCpdlpConfig.eRlpConfig << PWRMODCTL_CPDLPSTATE_RLPSTATE_Pos);
    ui32CpdlpConfig |= (sActCpdlpConfig.eElpConfig << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos);
    ui32CpdlpConfig |= (sActCpdlpConfig.eClpConfig << PWRMODCTL_CPDLPSTATE_CLPSTATE_Pos);

    //
    // Set the CPDLPSTATE configuration in deepsleep mode
    //
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sDSCpdlpConfig =
    {
        .eRlpConfig = sActCpdlpConfig.eRlpConfig,
        .eElpConfig = AM_HAL_PWRCTRL_ELP_RET,
        .eClpConfig = AM_HAL_PWRCTRL_CLP_RET
    };

    //
    // If ELP is OFF in active state, keep it OFF.
    //
    if (sActCpdlpConfig.eElpConfig == AM_HAL_PWRCTRL_ELP_OFF)
    {
        sDSCpdlpConfig.eElpConfig = AM_HAL_PWRCTRL_ELP_OFF;
    }

    if ( PWRCTRL->DEVPWRSTATUS_b.PWRSTDBG )
    {
        sDSCpdlpConfig.eClpConfig = AM_HAL_PWRCTRL_CLP_ON_CLK_OFF;
    }
    am_hal_pwrctrl_pwrmodctl_cpdlp_config(sDSCpdlpConfig);

    //
    // Set for deep sleep before calling WFIfunc()
    //
    SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);

#if !AM_HAL_STALL_CPU_HPWAKE
        //
        // If in HP mode, we need to wait till HFRC2 is ready and CPU is fully back in HP mode, before attempting deepsleep
        //
        if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
        {
            while ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
            {
                am_hal_delay_us(1);
            }
        }
#endif // !AM_HAL_STALL_CPU_HPWAKE

    //
    // Before executing WFI, flush APB writes.
    //
    am_hal_sysctrl_sysbus_write_flush();

    //
    // Call the function to go to sleep.
    //
    WFIfuncRAM();

#if AM_HAL_STALL_CPU_HPWAKE
    //
    // If in HP mode, we need to wait till HFRC2 is ready and CPU is fully back in HP mode
    //
    if ((bSleepDeep == AM_HAL_SYSCTRL_SLEEP_DEEP) && (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE))
    {
        while ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE )
        {
            am_hal_delay_us(1);
        }
    }
#endif // AM_HAL_STALL_CPU_HPWAKE

    //
    // Stop/Disable the timer
    //
    am_hal_spotmgr_timer_stop();

    //
    // Restore the original timer priority
    //
    NVIC->IPR[TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A] = origTimerPri;

    //
    // Restore the CPDLPSTATE
    //
    PWRMODCTL->CPDLPSTATE = ui32CpdlpConfig;

    //
    // Restore all interrupts
    //
    am_critical_mask_all_end();
} // am_hal_spotmgr_wfi_wait()
#endif

//*****************************************************************************
//
//! Inline function to convert temperature in float to PCM2.2 temperature range
//
//*****************************************************************************
static inline am_hal_spotmgr_tempco_range_e
spotmgr_temp_to_range(float fTemp)
{
    if ((fTemp < VDDC_VDDF_TEMPCO_THRESHOLD_LOW) && (fTemp >= LOW_LIMIT))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_VERY_LOW;
    }
    else if ((fTemp >= VDDC_VDDF_TEMPCO_THRESHOLD_LOW) && (fTemp < VDDC_VDDF_TEMPCO_THRESHOLD_MID))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW;
    }
    else if ((fTemp >= VDDC_VDDF_TEMPCO_THRESHOLD_MID) && (fTemp < VDDC_VDDF_TEMPCO_THRESHOLD_HIGH))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_MID;
    }
    else if ((fTemp >= VDDC_VDDF_TEMPCO_THRESHOLD_HIGH) && (fTemp < HIGH_LIMIT))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH;
    }
    return AM_HAL_SPOTMGR_TEMPCO_OUT_OF_RANGE;
}

//*****************************************************************************
//
//! Power state transition sequence.
//
//*****************************************************************************

//*****************************************************************************
// Temporarily suppress unused variable warnings
//*****************************************************************************
DIAG_SUPPRESS_UNUSED_VAR

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.1 LP" to
//! "PCM2.0 LP peripheral on and GPU&SDIO off" or "PCM2.0 LP GPU/SDIO on".
//
//*****************************************************************************
static inline void
transition_sequence_0(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
    {
        if (ui32PwrState == g_ui32LastStateBfStartTimer)
        {
            ADJUST_TON_IF_REQUIRED();
            LOAD_VDDC_TRIM();
            am_hal_spotmgr_timer_stop();
            eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;
            return;
        }
        else
        {
            WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
        }
    }
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    DOUBLE_BOOST_VDDF();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDF_DOUBLE_BOOST();
    DISABLE_ICACHE();
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 1;
    am_hal_delay_us(20);
    ENABLE_ICACHE();
    LOAD_CORELDO_TRIM();
    LOAD_VDDC_TRIM();
    // Enable peripheral
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.1 LP" to "PCM2.1 HP".
//
//*****************************************************************************
static inline void
transition_sequence_1(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    DOUBLE_BOOST_VDDF();
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    REMOVE_VDDF_DOUBLE_BOOST();
    DISABLE_ICACHE();
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
    am_hal_delay_us(20);
    ENABLE_ICACHE();
    //
    // Switch to HP mode after excuting the sequences above.
    //
}

#if USE_DELAY_INSTEAD_OF_TIMER
//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP peripheral on and GPU&SDIO off"
//! or "PCM2.0 LP GPU/SDIO on" to "PCM2.1 LP".
//
//*****************************************************************************
static inline void
transition_sequence_2(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Turn off peripheral, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    //
    // Boost VDDF for separation
    //
    MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = TVRGFACTTRIM2;
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    LOAD_VDDF_TRIM();
}
#else
//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP peripheral on and GPU&SDIO off"
//! or "PCM2.0 LP GPU/SDIO on" to "PCM2.1 LP".
//
//*****************************************************************************
static inline void
transition_sequence_2(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Turn off peripheral, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    //
    // Boost VDDF for separation
    //
    MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = TVRGFACTTRIM2;
    DOUBLE_BOOST_VDDC();
    g_ui32LastStateBfStartTimer = ui32PwrState;
    am_hal_spotmgr_timer_start(50);
    eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_2;
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP peripheral on and GPU&SDIO off"
//! or "PCM2.0 LP GPU/SDIO on" to "PCM2.1 LP".
//! Call this from ISR.
//
//*****************************************************************************
static inline void
transition_sequence_2b()
{
    //
    // After 50us timer expires, execute the sequences below.
    //
    MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
    MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM = g_ui32NewCoreldoTempcoTrim;
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM  = g_ui32NewCoreldoActTrim;
    am_hal_delay_us(5);
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
    //
    // Set to the default value.
    //
    eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;
}
#endif

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP peripheral on and GPU&SDIO off"
//! to "PCM2.0 HP peripheral on" or "PCM2.0 LP GPU/SDIO on".
//
//*****************************************************************************
static inline void
transition_sequence_3(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    DOUBLE_BOOST_VDDF();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDF_DOUBLE_BOOST();
    //
    // Switch to CPU HP mode or Enable GPU and/or SDIO after executing the sequences above.
    //
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.1 HP" to "PCM2.1 LP".
//
//*****************************************************************************
static inline void
transition_sequence_4(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Switch to CPU LP mode, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_CORELDO_TRIM();
    LOAD_VDDC_TRIM();
    LOAD_VDDF_TRIM();
    ADJUST_TON_IF_REQUIRED();
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 0;
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.1 HP" to "PCM2.0 HP peripheral on".
//
//*****************************************************************************
static inline void
transition_sequence_5(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
    {
        if (ui32PwrState == g_ui32LastStateBfStartTimer)
        {
            ADJUST_TON_IF_REQUIRED();
            LOAD_VDDC_TRIM();
            am_hal_spotmgr_timer_stop();
            eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;
            return;
        }
        else
        {
            WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
        }
    }
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    SWITCH_TO_LP();
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 1;
    SWITCH_TO_HP();
    LOAD_CORELDO_TRIM();
    LOAD_VDDC_TRIM();
    //
    // Turn on peripheral after executing the sequences above.
    //
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 HP peripheral on" to
//! "PCM2.0 LP peripheral on and GPU&SDIO off".
//
//*****************************************************************************
static inline void
transition_sequence_6(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Switch to LP mode, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_VDDF_TRIM();
    ADJUST_TON_IF_REQUIRED();
}

#if USE_DELAY_INSTEAD_OF_TIMER
//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 HP peripheral on" to "PCM2.1 HP".
//
//*****************************************************************************
static inline void
transition_sequence_7(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Turn off peripheral(s), then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    uint32_t ui32Diff = (ui32NewVddfTrim - TVRGFACTTRIM0) * 0.9f;
    BOOST_VDDF_FOR_SEPARATION(ui32Diff);
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    LOAD_VDDF_TRIM();
    SWITCH_TO_LP();
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    SWITCH_TO_HP();
}
#else
//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 HP peripheral on" to "PCM2.1 HP".
//
//*****************************************************************************
static inline void
transition_sequence_7(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Turn off peripheral(s), then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    volatile uint32_t ui32Diff = (ui32NewVddfTrim - TVRGFACTTRIM0) * 0.9f;
    BOOST_VDDF_FOR_SEPARATION(ui32Diff);
    DOUBLE_BOOST_VDDC();
    g_ui32LastStateBfStartTimer = ui32PwrState;
    am_hal_spotmgr_timer_start(50);
    eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_7;
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 HP peripheral on" to "PCM2.1 HP".
//! Call this from ISR.
//
//*****************************************************************************
static inline void
transition_sequence_7b()
{
    bool bSwitchBackToHp = true;
    //
    // After 50us timer expires, execute the sequences below.
    //
    MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
    MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM = g_ui32NewCoreldoTempcoTrim;
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM  = g_ui32NewCoreldoActTrim;
    am_hal_delay_us(5);
    MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
    //
    // If currently CPU is in HP mode, switch it to LP mode for switching power domains,
    // then switch back to HP.
    //
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
    {
        SWITCH_TO_LP();
        bSwitchBackToHp = true;
    }
    else
    {
        bSwitchBackToHp = false;
    }
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    if (bSwitchBackToHp)
    {
        SWITCH_TO_HP();
    }
    //
    // Set to the default value.
    //
    eOngoingSeq = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;
}
#endif

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 HP peripheral on" to
//! "PCM2.0 LP GPU/SDIO on".
//
//*****************************************************************************
static inline void
transition_sequence_8(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Switch to LP mode, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP GPU/SDIO on" to
//! "PCM2.0 LP peripheral on and GPU&SDIO off".
//
//*****************************************************************************
static inline void
transition_sequence_9(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Disable GPU or SDIO, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_VDDF_TRIM();
    ADJUST_TON_IF_REQUIRED();
}

//*****************************************************************************
//
//! Sequences for the transition from "PCM2.0 LP GPU/SDIO on" to
//! "PCM2.0 HP peripheral on".
//
//*****************************************************************************
static inline void
transition_sequence_10(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    //
    // Switch to HP mode after executing the sequences above.
    //
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 9 to 8 (transitions to > 50C).
//
//*****************************************************************************
static inline void
transition_sequence_11(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    SWITCH_TO_LP();
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 1;
    SWITCH_TO_HP();
    LOAD_CORELDO_TRIM();
    LOAD_VDDC_TRIM();
    DOUBLE_BOOST_VDDF();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDF_DOUBLE_BOOST();
    ADJUST_TON_IF_REQUIRED();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 1 to 0 (transitions to > 50C).
//
//*****************************************************************************
static inline void
transition_sequence_12(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    DOUBLE_BOOST_VDDF();
    LOAD_CORELDO_TRIM();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDF_DOUBLE_BOOST();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 8 to 9 (transitions to < 50C).
//
//*****************************************************************************
static inline void
transition_sequence_13(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
    volatile uint32_t ui32Diff = (ui32NewVddfTrim - TVRGFACTTRIM0) * 0.9f;
    BOOST_VDDF_FOR_SEPARATION(ui32Diff);
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    LOAD_VDDF_TRIM();
    SWITCH_TO_LP();
    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    SWITCH_TO_HP();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 0 to 1 (transitions to < 50C).
//
//*****************************************************************************
static inline void
transition_sequence_14(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_VDDF_TRIM();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 2 to 1 or 10 to 9 (transitions to > 0C).
//
//*****************************************************************************
static inline void
transition_sequence_15(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_VDDC_LV_TRIM();
    LOAD_CORELDO_TRIM();
    LOAD_VDDC_TRIM();
}

//*****************************************************************************
//
//! Sequences for the transitions to > 0C,
//! but not Power state 2 to 1 or 10 to 9 transition.
//
//*****************************************************************************
static inline void
transition_sequence_16(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_VDDC_LV_TRIM();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 9 to 10 (transitions to < 0C).
//
//*****************************************************************************
static inline void
transition_sequence_17(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    DOUBLE_BOOST_VDDC_LV();
    LOAD_CORELDO_TRIM();
    REMOVE_VDDC_LV_DOUBLE_BOOST();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 1 to 2 (transitions to < 0C).
//
//*****************************************************************************
static inline void
transition_sequence_18(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    DOUBLE_BOOST_VDDC_LV();
    uint32_t ui32Diff = (TVRGFACTTRIM3 - ui32NewVddfTrim) * 2;
    BOOST_VDDF_FOR_SEPARATION(ui32Diff);
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    LOAD_VDDF_TRIM();
    REMOVE_VDDC_LV_DOUBLE_BOOST();
}

//*****************************************************************************
//
//! Sequences for the transitions to < 0C,
//! but not Power state 9 to 10 or 1 to 2 transition.
//
//*****************************************************************************
static inline void
transition_sequence_19(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    DOUBLE_BOOST_VDDC_LV();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_LV_DOUBLE_BOOST();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 11 to 10 or 10 to 11 (temperature transition).
//
//*****************************************************************************
static inline void
transition_sequence_20(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 0 to 8.
//
//*****************************************************************************
static inline void
transition_sequence_21(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    g_bContinueSeq21b = true;
    ADJUST_TON_IF_REQUIRED();
    DOUBLE_BOOST_VDDF();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDF_DOUBLE_BOOST();
    DISABLE_ICACHE();
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 1;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 1;
    am_hal_delay_us(20);
    ENABLE_ICACHE();
    //
    // Switch to HP mode after executing the sequences above.
    //
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 0 to 8.
//! Call it after CPU LP to HP switch.
//
//*****************************************************************************
static inline void
transition_sequence_21b()
{
    MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[g_ui32CurPowerStateStatic].PWRSTATE_b.CORELDOTEMPCOTRIM;
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = g_sSpotMgrINFO1regs.sPowerStateArray[g_ui32CurPowerStateStatic].PWRSTATE_b.CORELDOACTTRIM;
    MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[g_ui32CurPowerStateStatic].PWRSTATE_b.TVRGCACTTRIM;
    g_bContinueSeq21b = false;
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 8 to 0.
//
//*****************************************************************************
static inline void
transition_sequence_22(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Switch to LP mode, then execute the sequences below.
    //
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    DOUBLE_BOOST_VDDC();
    WAIT_FOR_POWER_STABLE(50);
    REMOVE_VDDC_DOUBLE_BOOST();
    LOAD_CORELDO_TRIM();
    am_hal_delay_us(5);
    MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = 0;
    MCUCTRL->PWRSW0_b.PWRSWVDDMLSTATSEL = 0;
    LOAD_VDDF_TRIM();
    ADJUST_TON_IF_REQUIRED();
}

//*****************************************************************************
//
//! Sequences for the transition from Power state 8 to 12 or 12 to 8.
//
//*****************************************************************************
static inline void
transition_sequence_23(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    COMMON_HEADER
    WAIT_FOR_TIMER_EXPIRE_AND_EXECUTE_ISR();
    UPDATE_GLOBALS
    ADJUST_TON_IF_REQUIRED();
}

//*****************************************************************************
//
//! Sequences for other temperature transitions besides SEQ_11 to SEQ_20
//
//*****************************************************************************
static inline void
transition_sequence_24(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // No operation
    //
}

//*****************************************************************************
//
//! Not used
//
//*****************************************************************************
static inline void
transition_sequence_temp(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Not used currently.
    //
}

//*****************************************************************************
//
//! Not used
//
//*****************************************************************************
static inline void
transition_sequence_invalid(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    //
    // Not used currently.
    //
}


//*****************************************************************************
// Restore unused variable warnings to default
//*****************************************************************************
DIAG_SUPPRESS_UNUSED_VAR_DEFAULT


//*****************************************************************************
//
//! Put all sequences in a function pointer array.
//
//*****************************************************************************
TransitionSequencePtr powerStateTransitionSeq[] =
{
    //
    // Transition sequences between groups.
    //
    transition_sequence_0,  transition_sequence_1,  transition_sequence_2, transition_sequence_3,
    transition_sequence_4,  transition_sequence_5,  transition_sequence_6,  transition_sequence_7,
    transition_sequence_8,  transition_sequence_9,  transition_sequence_10,
    //
    // Transition sequences in the same group (temperature transitions).
    //
    transition_sequence_11, transition_sequence_12, transition_sequence_13, transition_sequence_14,
    transition_sequence_15, transition_sequence_16, transition_sequence_17, transition_sequence_18,
    transition_sequence_19, transition_sequence_20,
    //
    // Special handling for power state 8<->0 and 8<->12 transitions.
    //
    transition_sequence_21, transition_sequence_22,
    transition_sequence_23,
    //
    // Other temperature transitions besides SEQ_11 to SEQ_20.
    //
    transition_sequence_24,
    //
    // Not used currently.
    //
    transition_sequence_temp, transition_sequence_invalid
};

//*****************************************************************************
//
//! Power settings after CPU LP to HP transition
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_pcm2_2_post_lptohp_handle(void)
{
    if (g_bContinueSeq21b)
    {
        transition_sequence_21b();
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Timer interrupt service for spotmgr
//
//*****************************************************************************
void
am_hal_spotmgr_pcm2_2_boost_timer_interrupt_service(void)
{
#if !USE_DELAY_INSTEAD_OF_TIMER
    AM_CRITICAL_BEGIN

    if (eOngoingSeq == AM_HAL_SPOTMGR_TRANS_SEQ_2)
    {
        transition_sequence_2b();
    }
    else if (eOngoingSeq == AM_HAL_SPOTMGR_TRANS_SEQ_7)
    {
        transition_sequence_7b();
    }

    //
    // Clear interrupt status for this timer.
    // Disable timer.
    //
    am_hal_spotmgr_timer_stop();

    AM_CRITICAL_END
#endif
}

//*****************************************************************************
//
//! @brief Determine the buck state in deepsleep by setting g_bFrcBuckAct
//!
//! @param psPwrStatus    - Pointer of current power status.
//!
//! @return None.
//
//*****************************************************************************
static void
spotmgr_buck_deepsleep_state_determine(am_hal_spotmgr_power_status_t * psPwrStatus)
{
    //
    // Check temperature range and peripherals power status, if there is any
    // peripheral enabled in deepsleep or temperature range is HIGH, the
    // simobuck must be forced to stay in active mode in deepsleep.
    //
    if ((psPwrStatus->eTempRange == AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH) ||
        (psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK)    ||
        (psPwrStatus->ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK))
    {
        g_bFrcBuckAct = true;
        return;
    }
    else
    {
        //
        // Check stimer status and clock source, if it is using either the HFRC,
        // HFRC2 or a GPIO external clock input as the clock source in
        // deepsleep, the simobuck must be forced to stay in active mode in
        // deepsleep.
        //
        if (am_hal_stimer_is_running()                                &&
            (STIMER->STCFG_b.CLKSEL >= STIMER_STCFG_CLKSEL_HFRC_6MHZ) &&
            (STIMER->STCFG_b.CLKSEL <= STIMER_STCFG_CLKSEL_HFRC_375KHZ))
        {
            g_bFrcBuckAct = true;
            return;
        }
        else
        {
            //
            // Check timer status and clock source, if any timer instance is using
            // either the HFRC, HFRC2 or a GPIO external clock input as the clock
            // source in deepsleep, the simobuck must be forced to stay in active
            // mode in deepsleep.
            //
            for (uint32_t ui32TimerNumber = 0; ui32TimerNumber < AM_REG_NUM_TIMERS; ui32TimerNumber++)
            {
                if ((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN == TIMER_CTRL0_TMR0EN_EN) &&
                    (TIMER->GLOBEN & (TIMER_GLOBEN_ENB0_EN << ui32TimerNumber))        &&
                    (((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK >= AM_HAL_TIMER_CLOCK_HFRC_DIV4)            &&
                      (TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK <= AM_HAL_TIMER_CLOCK_HFRC_DIV4K))          ||
                     ((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK >= AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV8)    &&
                      (TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK <= AM_HAL_TIMER_CLOCK_HFRC2_125MHz_DIV256)) ||
                     ((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK >= AM_HAL_TIMER_CLOCK_GPIO0)                &&
                      (TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK <= AM_HAL_TIMER_CLOCK_GPIO223))))
                {
                    g_bFrcBuckAct = true;
                    return;
                }
            }
            //
            // Set g_bFrcBuckAct to false if all the conditions above are not met.
            //
            g_bFrcBuckAct = false;
        }
    }
}

//*****************************************************************************
//
//! @brief Internal power domain settings
//!
//! @param eCpuState     - Requested CPU modes.
//! @param eLastCpuState - Last/Current CPU modes.
//!
//! @return None.
//
//*****************************************************************************
static void
spotmgr_internal_power_domain_set(am_hal_spotmgr_cpu_state_e eCpuState, am_hal_spotmgr_cpu_state_e eLastCpuState)
{
    //
    // If entering deepsleep in CPU HP mode, g_bHpToDeepSleep must be set
    //
    if ((eLastCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP) && (eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
    {
        //
        // Set this bit to enable Vddcaor Vddcpu Override clear and set when switching between HP and deepsleep.
        //
        g_bHpToDeepSleep = true;
    }
}

//*****************************************************************************
//
//! @brief Ton adjust
//!        The Simobuck Ton values for VDDC and VDDF must be adjusted accordingly
//!        when the status is changed.
//!
//! @param ui32TonState    - New Ton state.
//! @param ui32TarPwrState - New power state.
//!
//! @return None.
//
//*****************************************************************************
static void
spotmgr_power_ton_adjust(uint32_t ui32TonState, uint32_t ui32PwrState)
{
    uint32_t ui32VDDCACTLOWTONTRIM, ui32VDDFACTLOWTONTRIM;

    //
    // If it is power state 8, force change the Ton state to 7.
    //
    if (ui32PwrState == 8)
    {
        ui32TonState = 7;
    }
    //
    // Apply different Ton trims to different modes
    //
    switch (ui32TonState)
    {
        case 0:
            //
            // If CPU in LP mode, GPU off and all peripherals off, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sStmTon.STMTON_b.STMCPULPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sStmTon.STMTON_b.STMCPULPVDDFTON;
            break;

        case 1:
            //
            // If CPU in HP mode, GPU off and all peripherals off, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sStmTon.STMTON_b.STMCPUHPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sStmTon.STMTON_b.STMCPUHPVDDFTON;
            break;

        case 2:
            //
            // If CPU in LP mode and GPU in LP mode, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.GPULPCPULPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON_b.GPULPCPULPVDDFTON;
            break;

        case 3:
            //
            // If CPU in LP mode and GPU in HP mode, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.GPUHPCPULPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON_b.GPUHPCPULPVDDFTON;
            break;

        case 4:
            //
            // If CPU in HP mode and GPU in LP mode, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.GPULPCPUHPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON_b.GPULPCPUHPVDDFTON;
            break;

        case 5:
            //
            // If CPU in HP mode and GPU in HP mode, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.GPUHPCPUHPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON_b.GPUHPCPUHPVDDFTON;
            break;

        case 6:
            //
            // If CPU in LP mode, GPU turned off, any peripherals on, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sDefaultTon.DEFAULTTON_b.VDDCACTLOWTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sDefaultTon.DEFAULTTON_b.VDDFACTLOWTON;
            break;

        case 7:
            //
            // If CPU in HP mode, GPU turned off, any peripheral on, execute the following lines of code to change the Tons
            //
            ui32VDDCACTLOWTONTRIM = MCUCTRL->SIMOBUCK2_b.VDDCACTHIGHTONTRIM;
            ui32VDDFACTLOWTONTRIM = MCUCTRL->SIMOBUCK6_b.VDDFACTHIGHTONTRIM;
            break;

        default:
            //
            // Should never get here, assign ui32VDDCACTLOWTONTRIM and ui32VDDFACTLOWTONTRIM for avoiding warning.
            //
            ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.GPUHPCPUHPVDDCTON;
            ui32VDDFACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON_b.GPUHPCPUHPVDDFTON;
            break;
    }

    MCUCTRL->SIMOBUCK2_b.VDDCACTLOWTONTRIM = ui32VDDCACTLOWTONTRIM;
    MCUCTRL->SIMOBUCK7_b.VDDFACTLOWTONTRIM = ui32VDDFACTLOWTONTRIM;
}

//*****************************************************************************
//
//! @brief Determine sequences of power state transition
//!
//! @param ui32TarPwrState - Target power state.
//! @param ui32CurPwrState - Current/Last power state.
//!
//! @return SUCCESS or FAIL.
//
//*****************************************************************************
static uint32_t
spotmgr_state_transition_sequence_determine(uint32_t ui32TarPwrState, uint32_t ui32CurPwrState, am_hal_spotmgr_transition_sequence_e * peSeqNum)
{
    am_hal_spotmgr_transition_sequence_e eTransitionSeqTable[5][5] =
    {
        {AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS, AM_HAL_SPOTMGR_TRANS_SEQ_0,          AM_HAL_SPOTMGR_TRANS_SEQ_1,          AM_HAL_SPOTMGR_TRANS_SEQ_INVALID,    AM_HAL_SPOTMGR_TRANS_SEQ_0         },
        {AM_HAL_SPOTMGR_TRANS_SEQ_2,          AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS, AM_HAL_SPOTMGR_TRANS_SEQ_INVALID,    AM_HAL_SPOTMGR_TRANS_SEQ_3,          AM_HAL_SPOTMGR_TRANS_SEQ_3         },
        {AM_HAL_SPOTMGR_TRANS_SEQ_4,          AM_HAL_SPOTMGR_TRANS_SEQ_INVALID,    AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS, AM_HAL_SPOTMGR_TRANS_SEQ_5,          AM_HAL_SPOTMGR_TRANS_SEQ_INVALID   },
        {AM_HAL_SPOTMGR_TRANS_SEQ_INVALID,    AM_HAL_SPOTMGR_TRANS_SEQ_6,          AM_HAL_SPOTMGR_TRANS_SEQ_7,          AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS, AM_HAL_SPOTMGR_TRANS_SEQ_8         },
        {AM_HAL_SPOTMGR_TRANS_SEQ_2,          AM_HAL_SPOTMGR_TRANS_SEQ_9,          AM_HAL_SPOTMGR_TRANS_SEQ_INVALID,    AM_HAL_SPOTMGR_TRANS_SEQ_10,         AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS}
    };

    //
    // Get the sequence index from the table
    //
    *peSeqNum = eTransitionSeqTable[StateGroup(ui32CurPwrState)][StateGroup(ui32TarPwrState)];
    if (*peSeqNum == AM_HAL_SPOTMGR_TRANS_SEQ_INVALID)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    //
    // Special handling for power state 0<->8 transitions
    //
    if ((ui32CurPwrState == 0) && (ui32TarPwrState == 8))
    {
        *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_21;
    }
    else if ((ui32CurPwrState == 8) && (ui32TarPwrState == 0))
    {
        *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_22;
    }
    //
    // Special handling for power state 12<->8 transitions
    //
    if (((ui32CurPwrState == 8) && (ui32TarPwrState == 12)) ||
        ((ui32CurPwrState == 12) && (ui32TarPwrState == 8)))
    {
        *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_23;
    }
    //
    // If it is tempperature transition, execute the code below.
    //
    if (*peSeqNum == AM_HAL_SPOTMGR_TRANS_SEQ_TEMP_TRANS)
    {
        //
        // Tansitions to > 50C
        //
        if (IsStateLE50C(ui32CurPwrState) && IsStateGT50C(ui32TarPwrState))
        {
            if ((ui32CurPwrState == 9) &&
                (ui32TarPwrState == 8))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_11;
            }
            else if ((ui32CurPwrState == 1) &&
                     (ui32TarPwrState == 0))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_12;
            }
            else
            {
                //
                // The temperature transition does not need any trim update
                //
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_24;
            }
        }
        //
        // Tansitions to < 50C
        //
        else if (IsStateGT50C(ui32CurPwrState) && IsStateLE50C(ui32TarPwrState))
        {
            if ((ui32CurPwrState == 8) &&
                (ui32TarPwrState == 9))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_13;
            }
            else if ((ui32CurPwrState == 0) &&
                     (ui32TarPwrState == 1))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_14;
            }
            else
            {
                //
                // The temperature transition does not need any trim update
                //
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_24;
            }
        }
        //
        // Tansitions to > 0C
        //
        else if (IsStateLE0C(ui32CurPwrState) && IsStateGT0C(ui32TarPwrState))
        {
            if (((ui32CurPwrState == 2) && (ui32TarPwrState == 1)) ||
                ((ui32CurPwrState == 10) && (ui32TarPwrState == 9)))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_15;
            }
            else
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_16;
            }
        }
        //
        // Tansitions to < 0C
        //
        else if (IsStateGT0C(ui32CurPwrState) && IsStateLE0C(ui32TarPwrState))
        {
            if ((ui32CurPwrState == 9) && (ui32TarPwrState == 10))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_17;
            }
            else if ((ui32CurPwrState == 1) && (ui32TarPwrState == 2))
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_18;
            }
            else
            {
                *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_19;
            }
        }
        //
        // Special handling for 10<->11.
        //
        else if (((ui32CurPwrState == 10) && (ui32TarPwrState == 11)) ||
                 ((ui32CurPwrState == 11) && (ui32TarPwrState == 10)))
        {
            *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_20;
        }
        else
        {
            //
            // The temperature transition does not need any trim update
            //
            *peSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_24;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Separate temperature transition into step by step, does not skip any
//!        temperature range.
//!
//! @param ui32PwrState    - Target power state.
//! @param ui32CurPwrState - Current/Last power state.
//! @param ui32TonState    - Target Ton state.
//! @param ui32CurTonState - Current/Last Ton state.
//!
//! @return None.
//
//*****************************************************************************
static void
spotmgr_temperature_transition_separate(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    uint32_t ui32StartingState = 0, ui32EndState = 0;
    am_hal_spotmgr_transition_sequence_e eSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;

    if (ui32PwrState > ui32CurPwrState)
    {
        for (ui32StartingState = ui32CurPwrState; ui32StartingState < ui32PwrState; ui32StartingState++)
        {
            ui32EndState = ui32StartingState + 1;
            if (AM_HAL_STATUS_SUCCESS == spotmgr_state_transition_sequence_determine(ui32EndState, ui32StartingState, &eSeqNum))
            {
                //
                // Call the corresponding transition sequence
                //
                powerStateTransitionSeq[eSeqNum](ui32PwrState, ui32CurPwrState, ui32TonState, ui32CurTonState);
            }
        }
    }
    else
    {
        for (ui32StartingState = ui32CurPwrState; ui32StartingState > ui32PwrState; ui32StartingState--)
        {
            ui32EndState = ui32StartingState - 1;
            if (AM_HAL_STATUS_SUCCESS == spotmgr_state_transition_sequence_determine(ui32EndState, ui32StartingState, &eSeqNum))
            {
                //
                // Call the corresponding transition sequence
                //
                powerStateTransitionSeq[eSeqNum](ui32PwrState, ui32CurPwrState, ui32TonState, ui32CurTonState);
            }
        }
    }
}

//*****************************************************************************
//
//! @brief Power trims update
//!        Update power trims and Ton trims accodrding to the power state and
//!        Ton state.
//!
//! @param ui32PwrState    - Target power state.
//! @param ui32CurPwrState - Current/Last power state.
//! @param ui32TonState    - Target Ton state.
//! @param ui32CurTonState - Current/Last Ton state.
//!
//! @return None.
//
//*****************************************************************************
static void
spotmgr_power_trims_update(uint32_t ui32PwrState, uint32_t ui32CurPwrState, uint32_t ui32TonState, uint32_t ui32CurTonState)
{
    am_hal_spotmgr_transition_sequence_e eSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;

    //
    // If PwrState does not change, check the TonState changes and update Ton trims
    //
    if (ui32PwrState == ui32CurPwrState)
    {
        //
        // Adjust VDDC and VDDF Simobuck Tons if Ton state is changing
        //
        if (ui32TonState != ui32CurTonState)
        {
            spotmgr_power_ton_adjust(ui32TonState, ui32PwrState);
        }
    }
    else
    {
        if (IsInSameStateGroup(ui32PwrState, ui32CurPwrState))
        {
            //
            // If it is only a simple temprature transition, execute the transition sequences range by range.
            //
            spotmgr_temperature_transition_separate(ui32PwrState, ui32CurPwrState, ui32TonState, ui32CurTonState);
        }
        else
        {
            if (IsNotInSameTempRange(ui32PwrState, ui32CurPwrState))
            {
                //
                // If it is mixed temprature transition and transion between different state group,
                // split it into a temprature transition and a state group transition.
                //
                uint32_t ui32MidPwrState = (ui32CurPwrState & STATE_GROUP_MASK) | (ui32PwrState & STATE_TEMP_RANGE_MASK);
                spotmgr_temperature_transition_separate(ui32MidPwrState, ui32CurPwrState, ui32TonState, ui32CurTonState);
                ui32CurPwrState = ui32MidPwrState;
            }
            //
            // State group transition
            //
            if (AM_HAL_STATUS_SUCCESS == spotmgr_state_transition_sequence_determine(ui32PwrState, ui32CurPwrState, &eSeqNum))
            {
                //
                // Call the corresponding transition sequence
                //
                powerStateTransitionSeq[eSeqNum](ui32PwrState, ui32CurPwrState, ui32TonState, ui32CurTonState);
            }
        }
    }
}

//*****************************************************************************
//
//! @brief Power states determine
//!        Determine the next power state according to temperature, CPU status,
//!        GPU status and other peripheral status.
//!
//! @param psPwrStatus - Pointer of am_hal_spotmgr_power_status_t struct, is used
//!                      to transfer temperature, CPU status, GPU status and
//!                      other peripheral status.
//! @param pui32PwrState  - Pointer of an uint32_t variable, is used to return the
//!                      power state index.
//! @param pui32TonState  - Pointer of an uint32_t variable, is used to return the
//!                      Ton state index.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
static int32_t
spotmgr_power_state_determine(am_hal_spotmgr_power_status_t * psPwrStatus, uint32_t * pui32PwrState, uint32_t * pui32TonState)
{
    am_hal_spotmgr_power_state_desc_t sPwrStatDesc =
    {
        .ePwrStateDesc = (am_hal_spotmgr_power_state_desc_e)0
    };

    //
    // Update sPwrStatDesc according to temperature, power status and perfomance modes.
    //
    sPwrStatDesc.PWRSTATEDESC_b.TEMPRANGE = (uint32_t) (psPwrStatus->eTempRange);

    if (psPwrStatus->eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP)
    {
        sPwrStatDesc.PWRSTATEDESC_b.CPUMODE = 1;
    }
    else if (psPwrStatus->eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP)
    {
        sPwrStatDesc.PWRSTATEDESC_b.CPUMODE = 0;
    }
    else // for deep sleep and normal sleep
    {
        //
        // When reporting deep sleep state, keep the current MCU performance mode for power state and ton state determination.
        //
        if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
        {
            sPwrStatDesc.PWRSTATEDESC_b.CPUMODE = 1;
        }
        else
        {
            sPwrStatDesc.PWRSTATEDESC_b.CPUMODE = 0;
        }
    }

    if ((psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_LP) ||
        (psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_HP) ||
        (psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK)    ||
        (psPwrStatus->ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK))
    {
        sPwrStatDesc.PWRSTATEDESC_b.PERIPHMODE1 = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.PERIPHMODE1 = 0;
    }

    if (psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_HP)
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUMODE = 2;
    }
    else if (psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_LP)
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUMODE = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUMODE = 0;
    }

    if ((psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK)  ||
        (psPwrStatus->ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK))
    {
        sPwrStatDesc.PWRSTATEDESC_b.PERIPHMODE = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.PERIPHMODE = 0;
    }

    if ((psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_LP)     ||
        (psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_HP)     ||
        (psPwrStatus->ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTSDIO0_Msk) ||
        (psPwrStatus->ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTSDIO1_Msk))
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUSDIOMODE = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUSDIOMODE = 0;
    }

    //
    // Determine the power state.
    //
    switch (sPwrStatDesc.ePwrStateDesc & PWR_STATE_DESC_MASK)
    {
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_0: // CPULP, Temp 3
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 4;
            }
            else
            {
                *pui32PwrState = 0;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_1: // CPULP, Temp 2
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 5;
            }
            else
            {
                *pui32PwrState = 1;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_2: // CPULP, Temp 1
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 6;
            }
            else
            {
                *pui32PwrState = 2;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_3: // CPULP, Temp 0
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 7;
            }
            else
            {
                *pui32PwrState = 3;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_4: // CPULP + P, Temp 3
            *pui32PwrState = 4;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_5: // CPULP + P, Temp 2
            *pui32PwrState = 5;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_6: // CPULP + P, Temp 1
            *pui32PwrState = 6;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_7: // CPULP + P, Temp 0
            *pui32PwrState = 7;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_8: // CPUHP, Temp 3
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 12;
            }
            else
            {
                *pui32PwrState = 8;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_9: // CPUHP, Temp 2
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 13;
            }
            else
            {
                *pui32PwrState = 9;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_10: // CPUHP, Temp 1
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 14;
            }
            else
            {
                *pui32PwrState = 10;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_11: // CPUHP, Temp 0
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32PwrState = 15;
            }
            else
            {
                *pui32PwrState = 11;
            }
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_12: // CPUHP + P, Temp 3
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_20: // CPUHP + P + GPU/SDIO, Temp 3
            *pui32PwrState = 12;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_13: // CPUHP + P, Temp 2
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_21: // CPUHP + P + GPU/SDIO, Temp 2
            *pui32PwrState = 13;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_14: // CPUHP + P, Temp 1
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_22: // CPUHP + P + GPU/SDIO, Temp 1
            *pui32PwrState = 14;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_15: // CPUHP + P, Temp 0
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_23: // CPUHP + P + GPU/SDIO, Temp 0
            *pui32PwrState = 15;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_16: // CPULP + P + GPU/SDIO, Temp 3
            *pui32PwrState = 16;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_17: // CPULP + P + GPU/SDIO, Temp 2
            *pui32PwrState = 17;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_18: // CPULP + P + GPU/SDIO, Temp 1
            *pui32PwrState = 18;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_19: // CPULP + P + GPU/SDIO, Temp 0
            *pui32PwrState = 19;
            break;

        default:
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Determine the Ton state.
    //
    switch (sPwrStatDesc.eTonStateDesc & TON_STATE_DESC_MASK)
    {
        case AM_HAL_SPOTMGR_TON_STATE_DESC_0: // CPULP
            *pui32TonState = 0;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_1: // CPUHP
            if (IS_PROFILE_COLLAPSE_STM_AND_STMP)
            {
                *pui32TonState = 7;
            }
            else
            {
                *pui32TonState = 1;
            }
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_2: // CPULP + GPU
        case AM_HAL_SPOTMGR_TON_STATE_DESC_3: // CPULP + GPU + Periph
            *pui32TonState = 2;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_4: // CPULP + GPUHP
        case AM_HAL_SPOTMGR_TON_STATE_DESC_5: // CPULP + GPUHP + Periph
            *pui32TonState = 3;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_6: // CPUHP + GPU
        case AM_HAL_SPOTMGR_TON_STATE_DESC_7: // CPUHP + GPU + Periph
            *pui32TonState = 4;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_8: // CPUHP + GPUHP
        case AM_HAL_SPOTMGR_TON_STATE_DESC_9: // CPUHP + GPUHP + Periph
            *pui32TonState = 5;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_10: // CPULP + Periph
            *pui32TonState = 6;
            break;

        case AM_HAL_SPOTMGR_TON_STATE_DESC_11: // CPUHP + Periph
            *pui32TonState = 7;
            break;

        default:
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Power states update for PCM2.2
//!
//! @param eStimulus - Stimilus for power states transition. For GPU state/power
//!                    changes, please use AM_HAL_SPOTMGR_STIM_GPU_STATE but not
//!                    AM_HAL_SPOTMGR_STIM_DEVPWR.
//! @param bOn       - Only needs to be set to true/false when turning on/off
//!                    peripherals or memories included in DEVPWRSTATUS,
//!                    AUDSSPWRSTATUS, MEMPWRSTATUS and SSRAMPWRST. It is
//!                    ignored when updating temperature, CPU state and GPU state.
//! @param pArgs     - Pointer to arguments for power states update, assign it
//!                    to NULL if not needed.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_CPU_STATE,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_cpu_state_e enum.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_GPU_STATE,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_gpu_state_e enum.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_TEMP,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_tempco_param_t struct.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_DEVPWR,
//! bOn must be set to true when turning on a peripheral,
//! bOn must be set to false when turning off a peripheral,
//! and pArgs must point to the DEVPWRSTATUS_MASK of the peripheral to be opened
//! when turning on a peripheral, pArgs is ignored when turning off a peripheral.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_AUDSSPWR,
//! bOn must be set to true when turning on a peripheral,
//! bOn must be set to false when turning off a peripheral,
//! and pArgs must point to the AUDSSPWRSTATUS_MASK of the peripheral to be opened
//! when turning on a peripheral, pArgs is ignored when turning off a peripheral.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_MEMPWR,
//! bOn is ignored, and pArgs must point to the entire MEMPWRSTATUS.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_SSRAMPWR,
//! bOn must be set to true when turning on partial or entire SSRAM,
//! bOn must be set to false when turning off partial or entire SSRAM,
//! and pArgs must point to the expected SSRAMPWRST when turning on,
//! pArgs is ignored when turning off.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_pcm2_2_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32PowerState = 0, ui32TonState = 0;
    am_hal_spotmgr_power_status_t sPwrStatus;
    bool bSkipAllUpdates = false, bReqPwrOrTonStateChg = true;
#ifdef AM_HAL_SPOTMGR_PROFILING
    bool bLogSleepChangeEvt = false;
#endif
    //
    // Check if SIMOBUCK is enabled
    //
    if ( PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT )
    {
        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // Check if INFO1 regs and Original trims are valid
    //
    if (g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid != INFO1GLOBALVALID)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Static variables for storing the last/current status, initialise them to the default values after MCU powering up.
    //
    static uint32_t ui32CurTonStateStatic = 6; // The default Ton state is 6 - CPU LP, GPU off and any peripheral on.
    static am_hal_spotmgr_tempco_range_e eCurTempRangeStatic = AM_HAL_SPOTMGR_TEMPCO_RANGE_VERY_LOW;    // The default state is 7. The temp range is VERY_LOW in state 7.
    static am_hal_spotmgr_cpu_state_e eLastCpuStateStatic = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;          // The default CPU state after MCU powering up is LP.
    AM_CRITICAL_BEGIN
#if defined(AM_HAL_SPOTMGR_PROFILING) && defined(AM_HAL_SPOTMGR_PROFILING_VERBOSE)
    // Log a verbose entry alongside the normal entries, with markings on tonState for verbose entry indication
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = AM_HAL_SPOTMGR_PROFILING_PWRST_INVALID;
    changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_TONST_VERBOSE;
    changeLog.u.s.eStimulus = eStimulus;
    changeLog.u.s.bOn = bOn;
    changeLog.args = pArgs ? *((uint32_t *)pArgs) : 0xDEADBEEF;
    am_hal_spotmgr_log_change(&changeLog);
#endif

    //
    // Improve the sleep exit latency
    //
    if ((eStimulus == AM_HAL_SPOTMGR_STIM_CPU_STATE) && (pArgs != NULL))
    {
        sPwrStatus.eCpuState = *((am_hal_spotmgr_cpu_state_e *)pArgs);
        if (((eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_AMB)   ||
             (eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_ARM)   ||
             (eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP)) &&
            ((sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP)  ||
             (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP)))
        {
            bSkipAllUpdates = true;
            eLastCpuStateStatic = sPwrStatus.eCpuState;
#ifdef AM_HAL_SPOTMGR_PROFILING
            bLogSleepChangeEvt = true;
#endif
        }
    }

    if (!bSkipAllUpdates)
    {
        //
        // Get current status
        //
        sPwrStatus.ui32DevPwrSt = PWRCTRL->DEVPWRSTATUS;
        sPwrStatus.ui32AudSSPwrSt = PWRCTRL->AUDSSPWRSTATUS;
        sPwrStatus.ui32MemPwrSt = PWRCTRL->MEMPWRSTATUS;
        sPwrStatus.ui32SsramPwrSt = PWRCTRL->SSRAMPWRST;
        sPwrStatus.eTempRange = eCurTempRangeStatic;
        sPwrStatus.eGpuState = (sPwrStatus.ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk) ?
                               (g_eCurGpuPwrMode == AM_HAL_PWRCTRL_GPU_MODE_LOW_POWER ? AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_LP : AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_HP) :
                               AM_HAL_SPOTMGR_GPUSTATE_OFF;
        sPwrStatus.eCpuState = eLastCpuStateStatic;
        //
        // Get the requested/target status
        //
        switch (eStimulus)
        {
            case AM_HAL_SPOTMGR_STIM_DEVPWR:
                if (bOn)
                {
                    if (pArgs != NULL)
                    {
                        sPwrStatus.ui32DevPwrSt |= *((uint32_t *)pArgs);
                    }
                    else
                    {
                        ui32Status = AM_HAL_STATUS_INVALID_ARG;
                    }
                }
                break;

            case AM_HAL_SPOTMGR_STIM_AUDSSPWR:
                if (bOn)
                {
                    if (pArgs != NULL)
                    {
                        sPwrStatus.ui32AudSSPwrSt |= *((uint32_t *)pArgs);
                    }
                    else
                    {
                        ui32Status = AM_HAL_STATUS_INVALID_ARG;
                    }
                }
                break;

            case AM_HAL_SPOTMGR_STIM_MEMPWR:
                if (pArgs != NULL)
                {
                    sPwrStatus.ui32MemPwrSt = *((uint32_t *)pArgs);
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
                break;

            case AM_HAL_SPOTMGR_STIM_SSRAMPWR:
                if (bOn)
                {
                    if (pArgs != NULL)
                    {
                        sPwrStatus.ui32SsramPwrSt = *((uint32_t *)pArgs);
                    }
                    else
                    {
                        ui32Status = AM_HAL_STATUS_INVALID_ARG;
                    }
                }
                break;

            case AM_HAL_SPOTMGR_STIM_TEMP:
                if (pArgs != NULL)
                {
                    am_hal_spotmgr_tempco_param_t *psTempCo = (am_hal_spotmgr_tempco_param_t *)pArgs;
                    sPwrStatus.eTempRange = spotmgr_temp_to_range(psTempCo->fTemperature);
                    eCurTempRangeStatic = sPwrStatus.eTempRange;
                    g_bTempLessThan50C = (eCurTempRangeStatic <= AM_HAL_SPOTMGR_TEMPCO_RANGE_MID);

                    switch(eCurTempRangeStatic)
                    {
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_VERY_LOW:
                            psTempCo->fRangeLower = LOW_LIMIT;
                            psTempCo->fRangeHigher = VDDC_VDDF_TEMPCO_THRESHOLD_LOW;
                            break;
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW:
                            psTempCo->fRangeLower = VDDC_VDDF_TEMPCO_THRESHOLD_LOW - TEMP_HYSTERESIS;
                            psTempCo->fRangeHigher = VDDC_VDDF_TEMPCO_THRESHOLD_MID;
                            break;
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_MID:
                            psTempCo->fRangeLower = VDDC_VDDF_TEMPCO_THRESHOLD_MID - TEMP_HYSTERESIS;
                            psTempCo->fRangeHigher = VDDC_VDDF_TEMPCO_THRESHOLD_HIGH;
                            break;
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH:
                            psTempCo->fRangeLower = VDDC_VDDF_TEMPCO_THRESHOLD_HIGH - TEMP_HYSTERESIS;
                            psTempCo->fRangeHigher = HIGH_LIMIT;
                            break;
                        case AM_HAL_SPOTMGR_TEMPCO_OUT_OF_RANGE:
                            psTempCo->fRangeLower = 0.0f;
                            psTempCo->fRangeHigher = 0.0f;
                            ui32Status = AM_HAL_STATUS_INVALID_ARG;
                            break;
                    }
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
                break;

            case AM_HAL_SPOTMGR_STIM_CPU_STATE:
                if (pArgs != NULL)
                {
                    sPwrStatus.eCpuState = *((am_hal_spotmgr_cpu_state_e *)pArgs);
                    if (eLastCpuStateStatic != sPwrStatus.eCpuState)
                    {
                        #ifdef AM_HAL_SPOTMGR_PROFILING
                        if ((eLastCpuStateStatic >= AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP) ||
                            (sPwrStatus.eCpuState >= AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
                        {
                            bLogSleepChangeEvt = true;
                        }
                        #endif

                        //
                        // If CPU is going to deep sleep, need to determine the buck state
                        // in deep sleep.
                        //
                        if (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP)
                        {
                            spotmgr_buck_deepsleep_state_determine(&sPwrStatus);
                            //
                            // Make sure current power state is 9-11,
                            // and the power state transition to 9-11 is completed.
                            //
                            if ((g_ui32CurPowerStateStatic >= 9)  &&
                                (g_ui32CurPowerStateStatic <= 11) &&
                                (eOngoingSeq != AM_HAL_SPOTMGR_TRANS_SEQ_7))
                            {
                                g_bVddcaorVddcpuOverride = true;
                            }
                            else
                            {
                                g_bVddcaorVddcpuOverride = false;
                            }
                        }
                        //
                        // If CPU is switching from LP to HP, voltage level will be increased,
                        // set internal power domain after updating PwrState related trims.
                        //
                        if ((eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP) &&
                            (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP))
                        {
                            //
                            // Handle this case in timer isr, just update eLastCpuStateStatic here.
                            //
                            eLastCpuStateStatic = sPwrStatus.eCpuState;
                        }
                        //
                        // If CPU is switching from HP to LP, voltage level will be reduced,
                        // set internal power domain after updating PwrState related trims.
                        //
                        else if ((eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP) &&
                                 (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP))
                        {
                            eLastCpuStateStatic = sPwrStatus.eCpuState;
                        }
                        //
                        // If entering deepsleep in CPU LP mode, only updates the eLastCpuStateStatic.
                        //
                        else if ((eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP) &&
                                 (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
                        {
                            eLastCpuStateStatic = sPwrStatus.eCpuState;
                        }
                        //
                        // If eLastCpuStateStatic or sPwrStatus.eCpuState is sleep, PwrState does not change,
                        // only set internal power domain and clear bReqPwrOrTonStateChg here.
                        //
                        else
                        {
                            spotmgr_internal_power_domain_set(sPwrStatus.eCpuState, eLastCpuStateStatic);
                            bReqPwrOrTonStateChg = false;
                            eLastCpuStateStatic = sPwrStatus.eCpuState;
                        }
                    }
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
                break;

            case AM_HAL_SPOTMGR_STIM_GPU_STATE:
                //
                // GPU performance mode switching is only allowed when GPU is off, so we can only request new power state when powering on/off GPU.
                //
                if (pArgs != NULL)
                {
                    sPwrStatus.eGpuState = *((am_hal_spotmgr_gpu_state_e *)pArgs);
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
                break;

            default:
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
                break;
        }
        if ((ui32Status == AM_HAL_STATUS_SUCCESS) && bReqPwrOrTonStateChg)
        {
            //
            // Determine the requested/target power state
            //
            ui32Status = spotmgr_power_state_determine(&sPwrStatus, &ui32PowerState, &ui32TonState);
            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                //
                // If the power/ton state needs to be changed, call spotmgr_power_trims_update().
                //
                if ((ui32PowerState != g_ui32CurPowerStateStatic) || (ui32TonState != ui32CurTonStateStatic))
                {
#ifdef AM_HAL_SPOTMGR_PROFILING
                    am_hal_spotmgr_changelog_t changeLog;
                    changeLog.u.s.pwrState = ui32PowerState;
                    changeLog.u.s.tonState = ui32TonState;
                    changeLog.u.s.eStimulus = eStimulus;
                    changeLog.u.s.bOn = bOn;
                    changeLog.args = pArgs ? *((uint32_t *)pArgs) : 0xDEADBEEF;
                    am_hal_spotmgr_log_change(&changeLog);
#endif
                    //
                    // Update trims
                    //
                    spotmgr_power_trims_update(ui32PowerState, g_ui32CurPowerStateStatic, ui32TonState, ui32CurTonStateStatic);
                }
                //
                // Maintain a static variable with current trim settings.
                //
                g_ui32CurPowerStateStatic = ui32PowerState;
                ui32CurTonStateStatic   = ui32TonState;
            }
        }
    }

#ifdef AM_HAL_SPOTMGR_PROFILING
    if (bLogSleepChangeEvt)
    {
        // Getting in or out of sleep
        am_hal_spotmgr_changelog_t changeLog;
        changeLog.u.s.pwrState = AM_HAL_SPOTMGR_PROFILING_PWRST_INVALID;
        changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_TONST_INVALID;
        changeLog.u.s.eStimulus = eStimulus;
        changeLog.u.s.bOn = bOn;
        changeLog.args = sPwrStatus.eCpuState;
        am_hal_spotmgr_log_change(&changeLog);
    }
#endif
    AM_CRITICAL_END

    return ui32Status;
}

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just after enabling
//!        SIMOBUCK for PCM2.2
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_2_simobuck_init_aft_enable(void)
{
    if ( g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid == INFO1GLOBALVALID )
    {
        //
        // Reduce CORELDOACTIVETRIM
        //
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = g_sSpotMgrINFO1regs.sPowerStateArray[7].PWRSTATE_b.CORELDOACTTRIM;
        //
        // Change memldo trim to support switching memldo reference to tvrgf
        //
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.MEMLDOACTTRIM;
        MCUCTRL->D2ASPARE_b.MEMLDOREF = g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.MEMLDOD2ASPARE;
    }
    return AM_HAL_STATUS_SUCCESS;
}

#if NO_TEMPSENSE_IN_DEEPSLEEP
//*****************************************************************************
//
//! @brief Prepare SPOT manager for suspended tempco during deep sleep for
//!        PCM2.2
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_2_tempco_suspend(void)
{
    am_hal_spotmgr_tempco_param_t sTempCo;
    if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE)
    {
        //
        // MCU needs the highest voltage which corresponds to the highest temperature range for HP mode.
        // Fix the temperature range to the highest and update the SPOT Manager before going to deepsleep.
        //
        sTempCo.fTemperature = VDDC_VDDF_TEMPCO_THRESHOLD_HIGH + 1.0f;
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_TEMP, false, (void *) &sTempCo);
    }
    else
    {
        //
        // MCU needs the highest voltage which corresponds to the lowest temperature range for LP mode.
        // Fix the temperature range to the lowest and update the SPOT Manager before going to deepsleep.
        //
        sTempCo.fTemperature =  VDDC_VDDF_TEMPCO_THRESHOLD_LOW - 1.0f;
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_TEMP, false, (void *) &sTempCo);
    }
    return AM_HAL_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
//! @brief SPOT manager init for PCM2.2
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_pcm2_2_init(void)
{
    uint32_t ui32Status;
    uint32_t info1buf[5];

//
// Helper macros for INFO1 populate.
// CHK_OFFSET_DELTA: Helper macro to assure continguousness of registers.
// RD_INFO1: Macro to call am_hal_info1_read() and check return status.
//
#define CHK_OFFSET_DELTA(offh, offl, n)     STATIC_ASSERT((((offh - offl) / 4) + 1) != n)

#define RD_INFO1(infospace, wdoffset, numwds, pData)                    \
    ui32Status = am_hal_info1_read(infospace, wdoffset, numwds, pData); \
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )                          \
    {                                                                   \
        return ui32Status;                                              \
    }

    if ( (MCUCTRL->SHADOWVALID_b.INFO1SELOTP == MCUCTRL_SHADOWVALID_INFO1SELOTP_VALID) &&
         (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP   != PWRCTRL_DEVPWRSTATUS_PWRSTOTP_ON) )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_POWERSTATE19_O , AM_REG_OTP_INFO1_POWERSTATE0_O , 20);
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_POWERSTATE0_O  / 4), 20, (uint32_t *) &(g_sSpotMgrINFO1regs.sPowerStateArray[0]));

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_VDDCLVACTTRIMADJ_O, AM_REG_OTP_INFO1_GPUVDDCTON_O, 5 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_GPUVDDCTON_O  / 4), 5, &info1buf[0]);
    g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON    = info1buf[0];
    g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON    = info1buf[1];
    g_sSpotMgrINFO1regs.sStmTon.STMTON            = info1buf[2];
    g_sSpotMgrINFO1regs.sDefaultTon.DEFAULTTON    = info1buf[3];
    g_sSpotMgrINFO1regs.sVddclvActTrimAdj.VDDCLVACTTRIMADJ = info1buf[4];

    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_MEMLDOCONFIG_O  / 4), 1, &info1buf[0]);
    g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG   = info1buf[0];
    //
    // All done, mark the data as valid
    //
    g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid = INFO1GLOBALVALID;

    //
    // Initialise the timer for power boost
    //
    am_hal_spotmgr_timer_init();

#ifdef AM_HAL_SPOTMGR_PROFILING
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = 7;
    changeLog.u.s.tonState = 5;
    changeLog.u.s.eStimulus = AM_HAL_SPOTMGR_PROFILING_ESTIM_INVALID;
    changeLog.u.s.bOn = AM_HAL_SPOTMGR_PROFILING_BON_INVALID;
    changeLog.args = 0xDEADBEEF;
    am_hal_spotmgr_log_change(&changeLog);
#endif

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Reset power state to POR default for PCM2.2
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_2_default_reset(void)
{
    am_hal_pwrctrl_temp_thresh_t dummy;
    //
    // Turn on OTP if all peripherals are all off
    //
    if ((PWRCTRL->DEVPWRSTATUS == 0) &&
        (PWRCTRL->AUDSSPWRSTATUS == 0 ))
    {
        if (am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP) != AM_HAL_STATUS_SUCCESS)
        {
            return AM_HAL_STATUS_FAIL;
        }
    }
    else
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Report the lowest temperature to spotmgr
    //
    if (am_hal_pwrctrl_temp_update(-40.0f, &dummy) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Polling the timer AM_HAL_INTERNAL_TIMER_NUM_A status until it is disabled,
    // to guarantee all power state updating operations are completed.
    //
    if (am_hal_delay_us_status_change(TIMER_A_DISABLE_WAIT_IN_US,
                                    (uint32_t)&TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0,
                                    TIMER_CTRL0_TMR0EN_Msk,
                                    TIMER_CTRL0_TMR0EN_DIS)
        != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_TIMEOUT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
