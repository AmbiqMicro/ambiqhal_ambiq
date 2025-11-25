// ****************************************************************************
//
//! @file am_hal_clkmgr.c
//!
//! @brief Clock manager functions that manage system clocks and minimize
//!        power consumption by powering down clocks when possible.
//!
//! @addtogroup clkmgr_ap510L CLKMGR - Clock Manager
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides comprehensive clock management functions for
//! Apollo5 devices, handling system clocks including HFRC, HFRC2, SYSPLL,
//! XTAL_HS, XTAL_LS, and LFRC. It manages clock requests, releases,
//! configuration, and power management to minimize power consumption.
//!
//! @section hal_clkmgr_features Key Features
//!
//! 1. @b Multi-Clock @b Support: Manage HFRC, HFRC2, SYSPLL, XTAL_HS, XTAL_LS, LFRC.
//! 2. @b Power @b Management: Automatic clock gating and power-down for efficiency.
//! 3. @b User @b Tracking: Track multiple users requesting the same clock.
//! 4. @b Clock @b Stabilization: Handle clock startup and stabilization timing.
//! 5. @b Configuration @b Management: Dynamic clock configuration and frequency control.
//!
//! @section hal_clkmgr_functionality Functionality
//!
//! - Request and release system clocks with user tracking
//! - Configure clock frequencies and parameters
//! - Handle clock stabilization and startup timing
//! - Manage power states and automatic clock gating
//! - Support for board-specific clock configurations
//!
//! @section hal_clkmgr_usage Usage
//!
//! 1. Request clocks using am_hal_clkmgr_clock_request()
//! 2. Configure clock parameters as needed
//! 3. Monitor clock status and user counts
//! 4. Release clocks when no longer needed
//! 5. Use board info functions for custom configurations
//!
//! @section hal_clkmgr_configuration Configuration
//!
//! - @b Board @b Info: Configure board-specific clock parameters
//! - @b User @b IDs: Track clock usage by different modules
//! - @b Stabilization @b Timing: Set up clock startup delays
//! - @b Power @b States: Configure automatic power management
//****************************************************************************

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
// This is part of revision release_sdk5_2_a_2-228a2539a of the AmbiqSuite Development Package.
//
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_hal_clkmgr_private.h"
#include "mcu/am_hal_clkgen_private.h"

#define AM_HAL_CLKMGR_HFRC_ADJ_WAIT_TIME_US             (10000)
#define AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_TIME_US      (3000)
#define AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US         (10)

#define AM_HAL_CLKMGR_HFRC_ADJ_WAIT_LOOP_CNT            (AM_HAL_CLKMGR_HFRC_ADJ_WAIT_TIME_US / AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US)
#define AM_HAL_CLKMGR_XTAL_HS_REQ_WAIT_LOOP_CNT         (AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_TIME_US / AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US)

#define AM_HAL_CLKMGR_USERID_DWORD_CNT ((AM_HAL_CLKMGR_USER_ID_MAX + 31) >> 5)

#define AM_HAL_CLKMGR_CLK_ID_SYSPLL AM_HAL_CLKMGR_CLK_ID_MAX
#define AM_HAL_CLKMGR_CLK_ID_MAX_PRIV (AM_HAL_CLKMGR_CLK_ID_MAX + 1)

// Clock Manager Config
static am_hal_clkmgr_board_info_t g_sClkMgrBoardInfo =
{
    .sXtalHs.eXtalHsMode    = AM_HAL_CLKMGR_DEFAULT_XTAL_HS_MODE,
    .sXtalHs.ui32XtalHsFreq = AM_HAL_CLKMGR_DEFAULT_XTAL_HS_FREQ_HZ,
    .sXtalLs.eXtalLsMode    = AM_HAL_CLKMGR_DEFAULT_XTAL_LS_MODE,
    .sXtalLs.ui32XtalLsFreq = AM_HAL_CLKMGR_DEFAULT_XTAL_LS_FREQ_HZ,
    .ui32ExtRefClkFreq      = AM_HAL_CLKMGR_DEFAULT_EXTREF_CLK_FREQ_HZ,
};



// Bitmap array to store user request status for each clock
static uint32_t g_ui32ClkMgrClkSrcBm[AM_HAL_CLKMGR_CLK_ID_MAX_PRIV][AM_HAL_CLKMGR_USERID_DWORD_CNT] = {0};

// Variable clock config - valid flags
static bool g_bClkMgrClkCfgValid_HFRC   = true;
static bool g_bClkMgrClkCfgValid_RFXTAL = false;

// Variable clock config - configured clock rate
static uint32_t g_ui32ClkMgrClkCfgFreq_HFRC   = AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ;
static uint32_t g_ui32ClkMgrClkCfgFreq_PLLVCO = 0;
static uint32_t g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV = 0;

// Variable clock configurations
static am_hal_clkmgr_clkcfg_t g_sClkMgrClkConfig_HFRC   = {0};
static am_hal_clkmgr_clkcfg_t g_sClkMgrClkConfig_SYSPLL = {0};
static am_hal_clkmgr_rfxtal_config_t g_sClkMgrClkConfig_RFXTAL = {0};

// Instance handle for SysPLL
static void *g_pSyspllHandle = NULL;

// Flag for clock stabilizing status
static bool      g_bClkStabilizing_HFRC_ADJ             = false;
static bool      g_bClkStablized_HFRC_ADJ               = false;
static uint32_t *g_pui32ClkStabilizingCounter_HFRC_ADJ  = NULL;

// Flag for XTAL_HS request status waiting
static bool       g_bClkOnReqWaiting_XTAL_HS          = false;
static bool       g_bClkOnReqSuccess_XTAL_HS          = true;
static uint32_t  *g_pui32ClkOnReqWaitCounter_XTAL_HS  = NULL;
static bool       g_bClkOffReqWaiting_XTAL_HS         = false;
static bool       g_bClkOffReqSuccess_XTAL_HS         = true;
static uint32_t  *g_pui32ClkOffReqWaitCounter_XTAL_HS = NULL;
static bool       g_bClkConfigWaiting_XTAL_HS         = false;
static bool       g_bClkConfigSuccess_XTAL_HS         = true;

// Flags for clock start pending
static bool g_bSYSPLLStartPending = false;

// Flags for HFADJ behaviour on full released
static bool g_bDisableHfadjOnFullReleased = false;

// Flags to indicate that SYSPLL has been disabled for DeepSleep
static bool g_bSysPllDisabledForDeepSleep = false;
static bool g_bSysPllReleasedHFRCForDeepSleep = false;

// Syspll Fref Priority
static am_hal_clkmgr_syspll_fref_priority_t g_sFrefPriority = {.high = AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4,
                                                               .mid  = AM_HAL_SYSPLL_FREFSEL_XTAL48MHz,
                                                               .low  = AM_HAL_SYSPLL_FREFSEL_EXTREFCLK};

// Variable to record the clocks that are released as a result of prestart
// release control
static uint32_t g_ui32PrestartReleasedClk = 0;

//-----------------------------------------------------------------------------
// Static function prototypes
//-----------------------------------------------------------------------------
static uint32_t am_hal_clkmgr_request_EXTREF_CLK(am_hal_clkmgr_user_id_e eUserId);
static uint32_t am_hal_clkmgr_release_EXTREF_CLK(am_hal_clkmgr_user_id_e eUserId);
static uint32_t am_hal_clkmgr_request_XTAL_HS(am_hal_clkmgr_user_id_e eUserId);
static uint32_t am_hal_clkmgr_release_XTAL_HS(am_hal_clkmgr_user_id_e eUserId);

//*****************************************************************************
//
//! @brief Utiltiy function to calculate bit set in a uint32_t
//!
//! @param ui32Val - variable where bits set is to be counted
//!
//! @return numbers of bit set in ui32Val
//
//*****************************************************************************
static inline uint8_t calculate_bit_set(uint32_t val)
{
    uint32_t buf = val - ((val >> 1) & 0x55555555);
    buf = (buf & 0x33333333) + ((buf >> 2) & 0x33333333);
    return (uint8_t)((((buf + (buf >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24);
}

//*****************************************************************************
//
//! @brief Utiltiy function to get fref clock frequency
//!
//! @param eFrefsel - enum of type am_hal_syspll_frefsel_e for clock source to
//!                   be queried
//!
//! @return - Frequency of the clock. Value 0 means clock source unavailable
//
//*****************************************************************************
static inline uint32_t am_hal_clkmgr_SYSPLL_fref_freq_get(am_hal_syspll_frefsel_e eFrefSel)
{
    switch(eFrefSel)
    {
        case AM_HAL_SYSPLL_FREFSEL_XTAL48MHz:
            return g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq;

        case AM_HAL_SYSPLL_FREFSEL_EXTREFCLK:
            return g_sClkMgrBoardInfo.ui32ExtRefClkFreq;

        case AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4:
            return 48000000U;
    }
    return 0;
}

//*****************************************************************************
//
//! @brief Utiltiy function to decide fref clock to be used by SYSPLL
//!
//! @param peFrefSel - Pointer to enum variable of type am_hal_syspll_frefsel_e
//!                    to return clock source selection
//! @param pui32FrefFreq - Pointer to uint32_t to return clock source frequency
//
//*****************************************************************************
static inline void am_hal_clkmgr_SYSPLL_avail_fref_get(am_hal_syspll_frefsel_e* peFrefSel , uint32_t *pui32FrefFreq)
{
    uint32_t ui32Freq = 0;
    am_hal_syspll_frefsel_e eFref;

    // Find the highest priority fref clock source that is available
    for (uint32_t ui32Prio = 0; ui32Prio < AM_HAL_CLKMGR_SYSPLL_FREF_PRIORITY_COUNT; ui32Prio++)
    {
        eFref = g_sFrefPriority.prio[ui32Prio];
        ui32Freq = am_hal_clkmgr_SYSPLL_fref_freq_get(eFref);

        if (ui32Freq)
        {
            break;
        }
    }

    // Fallback to HFRC/4 if no available fref is found
    if (ui32Freq == 0)
    {
        eFref = AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4;
        ui32Freq = am_hal_clkmgr_SYSPLL_fref_freq_get(eFref);
    }

    *peFrefSel = eFref;
    *pui32FrefFreq = ui32Freq;
}

//*****************************************************************************
//
//! @brief Check wehther a clock ID is requested by any clock user
//!
//! @param eClockID - am_hal_clkmgr_clock_id_e value that indicates which clock
//!                   to check
//!
//! @return result - true: there is at least one active user. false: no user
//
//*****************************************************************************
static inline bool am_hal_clkmgr_is_requested(am_hal_clkmgr_clock_id_e eClockId)
{
    uint8_t idx;
    for (idx = 0; idx < AM_HAL_CLKMGR_USERID_DWORD_CNT; idx++)
    {
        if (g_ui32ClkMgrClkSrcBm[eClockId][idx] != 0)
        {
            return true;
        }
    }
    return false;
}

//*****************************************************************************
//
//! @brief Check wehther a clock ID is requested by the specified clock user
//!
//! @param eClockID - am_hal_clkmgr_clock_id_e value that indicates which clock
//!                   to check
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates which clock
//!                  user to be checked
//!
//! @return result - true: requested. false: not requested
//
//*****************************************************************************
static inline bool am_hal_clkmgr_is_requested_by_user(am_hal_clkmgr_clock_id_e eClockId, am_hal_clkmgr_user_id_e eUserId)
{
    uint8_t ui8DwordIdx = ((uint32_t)eUserId) >> 5;
    uint8_t ui8BitIdx   = ((uint32_t)eUserId) & 0x1F;
    return (g_ui32ClkMgrClkSrcBm[eClockId][ui8DwordIdx] & ((uint32_t)1 << ui8BitIdx)) != 0;
}

//*****************************************************************************
//
//! @brief Check for number of users of a clock
//!
//! @param eClockID - am_hal_clkmgr_clock_id_e value that indicates which clock
//!                   to check
//!
//! @return Numbers of users have requested for the clock
//*****************************************************************************
static inline uint8_t am_hal_clkmgr_user_count_get(am_hal_clkmgr_clock_id_e eClockId)
{
    //
    // Calculate numbers of users
    //
    uint8_t ui8DwordIdx;
    uint8_t ui8Count = 0;
     for (ui8DwordIdx = 0; ui8DwordIdx < AM_HAL_CLKMGR_USERID_DWORD_CNT; ui8DwordIdx++)
     {
        ui8Count += calculate_bit_set(g_ui32ClkMgrClkSrcBm[eClockId][ui8DwordIdx]);
     }
     return ui8Count;
}

//*****************************************************************************
//
//! @brief Update the user list for the specified clock ID by adding/removing
//!        the clock user specified
//!
//! @param eClockID - am_hal_clkmgr_clock_id_e value that indicates which clock
//!                   the user list is to be updated
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates which clock
//!                  user to be added/removed from the user list
//! @param bClockRequested - true: clock is being requested, false: clock is
//!                          being released.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static inline uint32_t am_hal_clkmgr_user_set(am_hal_clkmgr_clock_id_e eClockId, am_hal_clkmgr_user_id_e eUserId, bool bClockRequested)
{
    //
    // Check validity of input values
    //
    if ((eClockId >= AM_HAL_CLKMGR_CLK_ID_MAX_PRIV) || (eUserId >= AM_HAL_CLKMGR_USER_ID_MAX))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Calculate userId corresponding DWORD index and bit position
    //
    uint8_t ui8DwordIdx = ((uint32_t)eUserId) >> 5;
    uint8_t ui8BitIdx   = ((uint32_t)eUserId) & 0x1F;

    //
    // Set/Clear user bit for the clock specified
    //
    if (bClockRequested)
    {
        g_ui32ClkMgrClkSrcBm[eClockId][ui8DwordIdx] |= ((uint32_t)1 << ui8BitIdx);
    }
    else
    {
        g_ui32ClkMgrClkSrcBm[eClockId][ui8DwordIdx] &= ~((uint32_t)1 << ui8BitIdx);
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Wait for clock stabilizing counter to expire, or stabilizing flag to
//!        be cleared
//!
//! @param pbStabilizingFlag - Pointer to bool variable that holds stabilizing
//!                            status of the clock.
//! @param ui32Loop - Numbers of loop to wait for the stabilizing status of the
//!                   clock to be cleared.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static inline void am_hal_clkmgr_clock_stabilize_wait(bool *pbStabilizingFlag, uint32_t *pui32Loop)
{
    while (*pui32Loop)
    {
        if (!(*pbStabilizingFlag))
        {
            break;
        }
        am_hal_delay_us(AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US);
        (*pui32Loop)--;
    }
}

//*****************************************************************************
//
//! @brief Helper function to clear all status flags used for stabilizing
//
//*****************************************************************************
static inline void am_hal_clkmgr_clear_stablizing_status(void)
{
    g_bClkStabilizing_HFRC_ADJ = false;
    g_pui32ClkStabilizingCounter_HFRC_ADJ = NULL;
    g_bClkStablized_HFRC_ADJ = false;
}

//*****************************************************************************
//
//! @brief Handles clock config for HFRC clock soruce
//!
//! @param ui32RequestedClk - Frequency HFRC clock is to be configured to
//! @param psClockConfig - [Optional] clock config structure to be used for
//!                        HFRC. set to NULL if auto-generation is desired.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_config_HFRC(uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockCfg)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_clkmgr_clkcfg_t sGeneratedConfig =
    {
        .hfrc.HFRCAdj_b = AM_HAL_CLKGEN_DEFAULT_HFRC_ADJ_CONFIG
    };

    //
    // Check whether the HFRC frequency requested is supported
    //
    if ((ui32RequestedClk != AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ) &&
        (ui32RequestedClk != AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ))
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // If requested clock is Adjusted HFRC, check or generate config
    //
    if (ui32RequestedClk != AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ)
    {
        // Check whether XTAL_LS is available for the board
        if (g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq  == 0)
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }

        // Generate HFRC ADJ config
        if (psClockCfg == NULL)
        {
            uint32_t ui32AdjTarget;
            ui32Status = am_hal_clkgen_hfrcadj_target_calculate(g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq, ui32RequestedClk, &ui32AdjTarget);
            sGeneratedConfig.hfrc.HFRCAdj_b.ui32TargetVal = ui32AdjTarget;
            psClockCfg = &sGeneratedConfig;
        }
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        //
        // Check whether HFRC already has active user
        //
        if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_HFRC) != 0)
        {
            ui32Status = AM_HAL_STATUS_IN_USE;

            // If HFRC already has active user, Only allow switching between
            // HFRC-FreeRun and HFADJ-48MHz
            if ((ui32RequestedClk == AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ) ||
                (ui32RequestedClk == AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ))
            {
                if ((g_ui32ClkMgrClkCfgFreq_HFRC == AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ) ||
                    (g_ui32ClkMgrClkCfgFreq_HFRC == AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ))
                {
                    // If the switching is between HFRC-FreeRun and HFADJ-48MHZ
                    // Apply new config to HFADJ
                    if ( ui32RequestedClk == AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ )
                    {
                        ui32Status = am_hal_clkgen_private_hfadj_disable();
                    }
                    else
                    {
                        ui32Status = am_hal_clkgen_private_hfadj_apply(psClockCfg->hfrc.HFRCAdj);
                        if (ui32Status != AM_HAL_STATUS_SUCCESS)
                        {
                            // Rollback HFADJ configuration if HFADJ apply failed
                            am_hal_clkgen_private_hfadj_apply(g_sClkMgrClkConfig_HFRC.hfrc.HFRCAdj);
                        }
                    }
                }
            }
        }
        else if (ui32RequestedClk != g_ui32ClkMgrClkCfgFreq_HFRC)
        {
            // No active user and requested frequency is different from
            // current selected frequency, disable HFADJ.
            am_hal_clkgen_private_hfadj_disable();
            am_hal_clkmgr_clear_stablizing_status();
        }

        //
        // Save Configuration if configuration is successful
        //
        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (ui32RequestedClk != AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ)
            {
                memcpy(&g_sClkMgrClkConfig_HFRC, psClockCfg, sizeof(am_hal_clkmgr_clkcfg_t));
            }
            g_ui32ClkMgrClkCfgFreq_HFRC = ui32RequestedClk;
            g_bClkMgrClkCfgValid_HFRC = true;
        }
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock config for PLLVCO clock soruce from SYSPLL
//! @note  The PLLVCO frequency should always be configured before PLLPOSTDIV
//!        if only ui32RequestedClk is used for both clock config.
//!
//! @param ui32RequestedClk - Frequency PLLVCO clock is to be
//!                           configured to
//! @param psClockConfig - [Optional] clock config structure to be used for
//!                        SYSPLL. set to NULL if auto-generation is desired.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_config_PLLVCO(uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockCfg)
{
    bool bAutoGenMode = (psClockCfg == NULL);
    bool bInvalidateConfig = false;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_clkmgr_clkcfg_t sGeneratedConfig = {0};


    // Check whether this is a request to invalidate PLLVCO
    if (ui32RequestedClk == 0)
    {
        bInvalidateConfig = true;
    }
    // Check whether requested clock rate is within range
    else if ((ui32RequestedClk > AM_HAL_CLKMGR_MAX_PLLVCO_FREQ_HZ) || (ui32RequestedClk < AM_HAL_CLKMGR_MIN_PLLVCO_FREQ_HZ))
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if (bInvalidateConfig)
    {
        // Do nothing
    }
    else if (bAutoGenMode)
    {
        uint32_t ui32RefFreq = 0;

        am_hal_clkmgr_SYSPLL_avail_fref_get(&sGeneratedConfig.syspll.eFref, &ui32RefFreq);

        ui32Status = am_hal_syspll_config_generate(&sGeneratedConfig.syspll, (float)ui32RefFreq / 1000000, (float)ui32RequestedClk / 1000000);

        psClockCfg = &sGeneratedConfig;
    }
    else
    {
        if ((psClockCfg->syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL48MHz) &&
            (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq == 0))
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }

        if ((psClockCfg->syspll.eFref == AM_HAL_SYSPLL_FREFSEL_EXTREFCLK) &&
            (g_sClkMgrBoardInfo.ui32ExtRefClkFreq == 0))
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        //
        // Check whether SYSPLL already has active user
        //
        if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_SYSPLL) != 0)
        {
            ui32Status = AM_HAL_STATUS_IN_USE;
        }

        //
        // Save Configuration if configuration is successful
        //
        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (bInvalidateConfig)
            {
                g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable = false;
                g_ui32ClkMgrClkCfgFreq_PLLVCO = 0;
            }
            else
            {
                g_sClkMgrClkConfig_SYSPLL.syspll.eFref         = psClockCfg->syspll.eFref         ;
                g_sClkMgrClkConfig_SYSPLL.syspll.eVCOSel       = psClockCfg->syspll.eVCOSel       ;
                g_sClkMgrClkConfig_SYSPLL.syspll.eFractionMode = psClockCfg->syspll.eFractionMode ;
                g_sClkMgrClkConfig_SYSPLL.syspll.ui8RefDiv     = psClockCfg->syspll.ui8RefDiv     ;
                g_sClkMgrClkConfig_SYSPLL.syspll.ui16FBDivInt  = psClockCfg->syspll.ui16FBDivInt  ;
                g_sClkMgrClkConfig_SYSPLL.syspll.ui32FBDivFrac = psClockCfg->syspll.ui32FBDivFrac ;
                g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable = true;
                g_ui32ClkMgrClkCfgFreq_PLLVCO                  = ui32RequestedClk;


                // If config is automatically generated, Mark PLLVCO Valid and
                // PLLPOSTDIV Invalid.
                // Note: For auto generated configs, if PLLPOSTDIV clock is needed,
                //       make sure a config call for PLLVCO config call is followed
                //       by PLLPOSTDIV config call.
                if (bAutoGenMode)
                {
                    g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable = false;
                    g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV                  = 0;
                }
                // If config is passed in from caller, check whether PLLPOSTDIV
                // frequency expected is within its permissible range
                else
                {
                    uint32_t ui32PostDivFreq = ui32RequestedClk / ((psClockCfg->syspll.ui8PostDiv1) * (psClockCfg->syspll.ui8PostDiv2));
                    if (ui32PostDivFreq > AM_HAL_CLKMGR_MAX_PLLPOSTDIV_FREQ_HZ)
                    {
                        g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable = false;
                        g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV                  = 0;
                    }
                    else
                    {
                        g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv1       = psClockCfg->syspll.ui8PostDiv1;
                        g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv2       = psClockCfg->syspll.ui8PostDiv2;
                        g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable = true;
                        g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV                  = ui32PostDivFreq;
                    }
                }

            }
        }
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock config for PLLPOSTDIV clock soruce from SYSPLL
//! @note  Configuring PLLPOSTDIV with psClockCfg will override PLLVCO freq!!
//!
//! @param ui32RequestedClk - Frequency PLLPOSTDIV clock is to be
//!                           configured to
//! @param psClockConfig - [Optional] clock config structure to be used for
//!                        SYSPLL. set to NULL if auto-generation is desired.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_config_PLLPOSTDIV(uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockCfg)
{
    bool bAutoGenMode = (psClockCfg == NULL);
    bool bInvalidateConfig = false;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_clkmgr_clkcfg_t sGeneratedConfig = {0};
    bool bConfigVco = true;

    // Check whether this is a request to invalidate PLLVCO
    if (ui32RequestedClk == 0)
    {
        bInvalidateConfig = true;
    }
    // Check whether requested clock rate is within range
    else if (ui32RequestedClk > AM_HAL_CLKMGR_MAX_PLLPOSTDIV_FREQ_HZ)
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if (bInvalidateConfig)
    {
        // Do nothing
    }
    else if (bAutoGenMode)
    {
        if (g_ui32ClkMgrClkCfgFreq_PLLVCO == 0)
        {
            uint32_t ui32RefFreq = 0;

            am_hal_clkmgr_SYSPLL_avail_fref_get(&sGeneratedConfig.syspll.eFref, &ui32RefFreq);

            ui32Status = am_hal_syspll_config_generate_with_postdiv(&sGeneratedConfig.syspll, ui32RefFreq, ui32RequestedClk);
            psClockCfg = &sGeneratedConfig;
            bConfigVco = true;
        }
        else
        {
            //
            // Exact match only for now, PPM=0
            //
            ui32Status = am_hal_syspll_config_generate_postdiv(&sGeneratedConfig.syspll, g_ui32ClkMgrClkCfgFreq_PLLVCO, ui32RequestedClk, 0);
            psClockCfg = &sGeneratedConfig;
            bConfigVco = false;
        }
    }
    else
    {
        if ((psClockCfg->syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL48MHz) &&
            (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq == 0))
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }

        if ((psClockCfg->syspll.eFref == AM_HAL_SYSPLL_FREFSEL_EXTREFCLK) &&
            (g_sClkMgrBoardInfo.ui32ExtRefClkFreq == 0))
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN

        uint32_t ui32PllVcoFreq = g_ui32ClkMgrClkCfgFreq_PLLVCO;

        //
        // Check whether SYSPLL already has active user
        //
        if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_SYSPLL) != 0)
        {
            ui32Status = AM_HAL_STATUS_IN_USE;
        }

        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (bInvalidateConfig)
            {
                g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable = false;
                g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV = 0;
            }
            else
            {
                //
                // Save Configuration if configuration is successful
                //
                if (bConfigVco)
                {

                    memcpy(&g_sClkMgrClkConfig_SYSPLL, psClockCfg, sizeof(am_hal_clkmgr_clkcfg_t));
                    g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV = ui32RequestedClk;
                    ui32PllVcoFreq = ui32RequestedClk *
                                    g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv1 *
                                    g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv2 ;
                }
                else
                {
                    g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv1 = psClockCfg->syspll.ui8PostDiv1;
                    g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv2 = psClockCfg->syspll.ui8PostDiv2;
                    g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV = g_ui32ClkMgrClkCfgFreq_PLLVCO /
                                                        g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv1 /
                                                        g_sClkMgrClkConfig_SYSPLL.syspll.ui8PostDiv2;
                }

                g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable = true;

                if (!bAutoGenMode)
                {
                    if (ui32PllVcoFreq > AM_HAL_CLKMGR_MAX_PLLVCO_FREQ_HZ)
                    {
                        g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable = false;
                        g_ui32ClkMgrClkCfgFreq_PLLVCO = 0;
                    }
                    else
                    {
                        g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable = true;
                        g_ui32ClkMgrClkCfgFreq_PLLVCO = ui32PllVcoFreq;
                    }
                }
            }
        }
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock request for LFRC clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_LFRC(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if it has already been requested
    //
    if ( am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_LFRC, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Set User flag for LFRC clock
    //
    AM_CRITICAL_BEGIN
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_LFRC, eUserId, true);
    if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
    {
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_LFRC, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock release for LFRC clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_LFRC(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_LFRC, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Clear User flag for LFRC clock
    //
    AM_CRITICAL_BEGIN
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_LFRC, eUserId, false);
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock request for XTAL_LS clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_XTAL_LS(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Check whether XTAL_LS is configured for the device
    //
    if (g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Return success immediately if it has already been requested
    //
    if ( am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_LS, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN

    #ifdef AM_HAL_CLKMGR_MANAGE_XTAL_LS
    //
    // Get current status for XTAL_LS
    //
    am_hal_mcuctrl_ext32k_status_e eXtalLsStatus;
    am_hal_mcuctrl_extclk32k_status_get(&eXtalLsStatus);

    //
    // If XTAL_LS is not enabled by SW, enable it
    //
    if (eXtalLsStatus == AM_HAL_MCUCTRL_EXT32K_STATUS_OFF)
    {
        if ( g_sClkMgrBoardInfo.sXtalLs.eXtalLsMode == AM_HAL_CLKMGR_XTAL_LS_MODE_XTAL )
        {
            am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE, NULL);
        }
        else
        {
            bool bExt32KClk = true;
            am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE, &bExt32KClk);
        }
    }
    else if ((eXtalLsStatus == AM_HAL_MCUCTRL_EXT32K_STATUS_EXT_CLK) &&
             (g_sClkMgrBoardInfo.sXtalLs.eXtalLsMode == AM_HAL_CLKMGR_XTAL_LS_MODE_XTAL))
    {
        // Current XTAL_LS mode is external clock, but requested is XTAL. Mark clock busy.
        ui32Status = AM_HAL_STATUS_IN_USE;
    }
    else if ((eXtalLsStatus == AM_HAL_MCUCTRL_EXT32K_STATUS_XTAL) &&
             (g_sClkMgrBoardInfo.sXtalLs.eXtalLsMode == AM_HAL_CLKMGR_XTAL_LS_MODE_EXT))
    {
        // Current XTAL_HS mode is XTAL, but requested is external clock. Mark clock busy.
        ui32Status = AM_HAL_STATUS_IN_USE;
    }
    #endif

    if ( ui32Status == AM_HAL_STATUS_SUCCESS )
    {
        //
        // Set User Flag for XTAL_LS clock
        //
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_LS, eUserId, true);
        if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_LS, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
        }
    }
    AM_CRITICAL_END

    return ui32Status;
}


//*****************************************************************************
//
//! @brief Handles clock release for XTAL_LS clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_XTAL_LS(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_LS, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag for XTAL_LS clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_LS, eUserId, false);

    #ifdef AM_HAL_CLKMGR_MANAGE_XTAL_LS
    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_XTAL_LS))
    {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE, NULL);
    }
    #endif
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock request for EXTREF_CLK clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_EXTREF_CLK(am_hal_clkmgr_user_id_e eUserId)
{
    am_hal_gpio_pincfg_t sExtRefClkPinCfg = AM_HAL_GPIO_PINCFG_DISABLED;
    sExtRefClkPinCfg.GP.cfg_b.uFuncSel = AM_HAL_PIN_15_REFCLK_EXT;

    //
    // Check whether EXTREFCLK is configured for the device
    //
    if (g_sClkMgrBoardInfo.ui32ExtRefClkFreq == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Return success immediately if it has already been requested
    //
    if ( am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // If GPIO is not configured as EXTREF_CLK, configure it as EXTREF_CLK
    //
    am_hal_gpio_pinconfig(15, sExtRefClkPinCfg);

    //
    // Set User Flag for EXTREF_CLK clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK, eUserId, true);
    if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
    {
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock release for EXTREF_CLK clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_EXTREF_CLK(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag EXTREF_CLK clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK))
    {
        am_hal_gpio_pincfg_t sExtRefClkPinCfg = AM_HAL_GPIO_PINCFG_DISABLED;
        am_hal_gpio_pinconfig(15, sExtRefClkPinCfg);
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handler for mbox callback of XTAL_HS ON response callback
//
//*****************************************************************************
static void am_hal_clkmgr_mbox_rfxtal_enable_resp_handler(void *pArgs)
{
    g_bClkOnReqSuccess_XTAL_HS = true;
    g_bClkOnReqWaiting_XTAL_HS = false;
}

//*****************************************************************************
//
//! @brief Handler for mbox callback of XTAL_HS OFF response callback
//
//*****************************************************************************
static void am_hal_clkmgr_mbox_rfxtal_disable_resp_handler(void *pArgs)
{
    g_bClkOffReqSuccess_XTAL_HS = true;
    g_bClkOffReqWaiting_XTAL_HS = false;
}

//*****************************************************************************
//
//! @brief Handler for mbox callback of XTAL_HS configuration response callback
//
//*****************************************************************************
static void am_hal_clkmgr_mbox_rfxtal_config_resp_handler(void *pArgs)
{
    g_bClkConfigSuccess_XTAL_HS = true;
    g_bClkConfigWaiting_XTAL_HS = false;
}

//*****************************************************************************
//
//! @brief Wait for XTAL_HS requests with timeout
//!
//! @param  pui32StabilizingCounter - pointer to timeout counter for the clock.
//
//*****************************************************************************
static inline void am_hal_clkmgr_wait_XTAL_HS_response(am_hal_clkmgr_xtalhs_op_e eClkOp, uint32_t *pui32WaitCounter)
{
    bool *pbWaitFlag = NULL;
    bool *pbSuccessFlag = NULL;

    switch (eClkOp)
    {
        case AM_HAL_CLKMGR_XTAL_HS_OP_ENABLE:
            pbWaitFlag = &g_bClkOnReqWaiting_XTAL_HS;
            pbSuccessFlag = &g_bClkOnReqSuccess_XTAL_HS;
            break;
        case AM_HAL_CLKMGR_XTAL_HS_OP_DISABLE:
            pbWaitFlag = &g_bClkOffReqWaiting_XTAL_HS;
            pbSuccessFlag = &g_bClkOffReqSuccess_XTAL_HS;
            break;
        case AM_HAL_CLKMGR_XTAL_HS_OP_CONFIG:
            pbWaitFlag = &g_bClkConfigWaiting_XTAL_HS;
            pbSuccessFlag = &g_bClkConfigSuccess_XTAL_HS;
            break;
        default:
            break;
    }

    //
    // If XTAL_HS has just been started in XTAL mode. wait for XTAL clock to
    // stabilize.
    //
    if (*pbWaitFlag)
    {

        while (*pui32WaitCounter)
        {
            //
            // Check if any pending incoming mailbox message on RX FIFO and handle it.
            // The IN_PEND status field will be cleared automatically if no incoming
            // pending message on the RX FIFO.
            //
            if (AM_HAL_IPC_MBOX_STATUS_IN_PEND)
            {
                AM_CRITICAL_BEGIN
                am_hal_ipc_mbox_msg_handler();
                AM_CRITICAL_END
            }
            //
            // The waiting flag will be cleared in the corresponding response handler
            // of current RFXTAL request operation.
            //
            if (!(*pbWaitFlag))
            {
                break;
            }
            am_hal_delay_us(AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US);
            (*pui32WaitCounter)--;
        }
    }

    AM_CRITICAL_BEGIN
    //
    // Mark timeout failure if wait status is not cleared yet
    //
    if (*pbWaitFlag)
    {
        *pbSuccessFlag = false;
        *pbWaitFlag = false;
    }

    //
    // Clear counter pointers
    //
    switch (eClkOp)
    {
        case AM_HAL_CLKMGR_XTAL_HS_OP_ENABLE:
            g_pui32ClkOnReqWaitCounter_XTAL_HS = NULL;
            break;
        case AM_HAL_CLKMGR_XTAL_HS_OP_DISABLE:
            g_pui32ClkOffReqWaitCounter_XTAL_HS = NULL;
            break;
        default:
            break;
    }
    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief Request to enable XTAL_HS via MBOX HAL
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static inline uint32_t am_hal_clkmgr_mbox_request_XTAL_HS_ON(void)
{

    //
    // Execute the actual request via mailbox
    //
    uint32_t ui32RfxtalReq = AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_ON_REQ;
    return am_hal_ipc_mbox_data_write(&ui32RfxtalReq, 1);
}

//*****************************************************************************
//
//! @brief Request to disable XTAL_HS via MBOX HAL
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static inline uint32_t am_hal_clkmgr_mbox_request_XTAL_HS_OFF(void)
{

    //
    // Execute the actual request via mailbox
    //
    uint32_t ui32RfxtalReq = AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_OFF_REQ;
    return am_hal_ipc_mbox_data_write(&ui32RfxtalReq, 1);
}

//*****************************************************************************
//
//! @brief Handles clock request for XTAL_HS clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_XTAL_HS(am_hal_clkmgr_user_id_e eUserId)
{
    bool bXtalHsStartPending = false;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32ReqWaitCounter =  AM_HAL_CLKMGR_XTAL_HS_REQ_WAIT_LOOP_CNT;

    //
    // Check whether XTAL_HS is configured for the device
    //
    if (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Return success if it has already been request, and XTAL_HS request has
    // been acknowledged
    //
    if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId))
    {
        AM_CRITICAL_BEGIN
        if (g_bClkOnReqWaiting_XTAL_HS)
        {
            ui32ReqWaitCounter = *g_pui32ClkOnReqWaitCounter_XTAL_HS;
        }
        g_pui32ClkOnReqWaitCounter_XTAL_HS = &ui32ReqWaitCounter;
        AM_CRITICAL_END
        am_hal_clkmgr_wait_XTAL_HS_response(AM_HAL_CLKMGR_XTAL_HS_OP_ENABLE, &ui32ReqWaitCounter);
        return g_bClkOnReqSuccess_XTAL_HS ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Check whether RSS is powered up and ready for communication
    //
    if (am_hal_ipc_mbox_init_state_get() != AM_HAL_IPC_MBOX_INIT_STATE_INITIALIZED)
    {
        ui32Status = AM_HAL_STATUS_INVALID_OPERATION;
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        bXtalHsStartPending = (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_XTAL_HS) == 0);
        if (bXtalHsStartPending)
        {
            //
            // Request RFXTAL enable from RSS
            //
            ui32Status = am_hal_clkmgr_mbox_request_XTAL_HS_ON();
        }

        //
        // Set user flag for XTAL_HS clock
        //
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId, true);
        if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
        }

        //
        // Update clock request wait timer
        //
        if (g_bClkOnReqWaiting_XTAL_HS)
        {
            ui32ReqWaitCounter = *g_pui32ClkOnReqWaitCounter_XTAL_HS;
        }

        //
        // Update flag to indicate we are waiting for response if this is the
        // first user requesting for the clock
        //
        if (bXtalHsStartPending)
        {
            g_bClkOnReqWaiting_XTAL_HS = true;
            g_bClkOnReqSuccess_XTAL_HS = false;
        }

        //
        // Update timeout counter with timeout-chaining
        //
        if (g_bClkOnReqWaiting_XTAL_HS)
        {
            g_pui32ClkOnReqWaitCounter_XTAL_HS = &ui32ReqWaitCounter;
        }
        AM_CRITICAL_END
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        //
        // Wait for clock request response from RSS
        //
        am_hal_clkmgr_wait_XTAL_HS_response(AM_HAL_CLKMGR_XTAL_HS_OP_ENABLE, &ui32ReqWaitCounter);

        //
        // Revert user flag if clock startup timeout
        //
        AM_CRITICAL_BEGIN
        if ( !g_bClkOnReqSuccess_XTAL_HS )
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId, false);
        }
        AM_CRITICAL_END
        return g_bClkOnReqSuccess_XTAL_HS ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_TIMEOUT;
    }
    else
    {
        return ui32Status;
    }
}

//*****************************************************************************
//
//! @brief Handles clock release for XTAL_HS clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_XTAL_HS(am_hal_clkmgr_user_id_e eUserId)
{
    bool bXtalHsStopPending = false;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32ReqWaitCounter =  AM_HAL_CLKMGR_XTAL_HS_REQ_WAIT_LOOP_CNT;

    //
    // Return success if the user flag is already cleared, and XTAL_HS request
    // has been acknowledged
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId) )
    {
        AM_CRITICAL_BEGIN
        if (g_bClkOffReqWaiting_XTAL_HS)
        {
            ui32ReqWaitCounter = *g_pui32ClkOffReqWaitCounter_XTAL_HS;
        }
        g_pui32ClkOffReqWaitCounter_XTAL_HS = &ui32ReqWaitCounter;
        AM_CRITICAL_END

        am_hal_clkmgr_wait_XTAL_HS_response(AM_HAL_CLKMGR_XTAL_HS_OP_DISABLE, &ui32ReqWaitCounter);
        return g_bClkOffReqSuccess_XTAL_HS ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_TIMEOUT;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear user flag for XTAL_HS clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_XTAL_HS))
    {
        //
        // Check whether RSS is powered up and ready for communication. If
        // It is not, it also means that RFXTAL is not enabled. In that case,
        // return failure without sending RFXTAL disable request.
        //
        if (am_hal_ipc_mbox_init_state_get() == AM_HAL_IPC_MBOX_INIT_STATE_INITIALIZED)
        {
            bXtalHsStopPending = true;
        }

        if ( bXtalHsStopPending )
        {
            //
            // Request RFXTAL disable from RSS
            //
            ui32Status = am_hal_clkmgr_mbox_request_XTAL_HS_OFF();

            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                //
                // Update clock request wait counter with timeout-chaining
                //
                if (g_bClkOffReqWaiting_XTAL_HS)
                {
                    ui32ReqWaitCounter = *g_pui32ClkOffReqWaitCounter_XTAL_HS;
                }
                g_pui32ClkOffReqWaitCounter_XTAL_HS = &ui32ReqWaitCounter;

                //
                // Update flag to indicate we are waiting for response
                //
                g_bClkOffReqWaiting_XTAL_HS = true;
                g_bClkOffReqSuccess_XTAL_HS = false;
            }
        }
    }
    AM_CRITICAL_END

    if ( bXtalHsStopPending && (ui32Status == AM_HAL_STATUS_SUCCESS))
    {
        am_hal_clkmgr_wait_XTAL_HS_response(AM_HAL_CLKMGR_XTAL_HS_OP_DISABLE, &ui32ReqWaitCounter);
        return g_bClkOffReqSuccess_XTAL_HS ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_TIMEOUT;
    }
    else
    {
        return ui32Status;
    }
}

//*****************************************************************************
//
//! @brief Wait for HFRC_ADJ to be stabilized
//!
//! @param  pui32StabilizingCounter - pointer to timeout counter for the clock.
//
//*****************************************************************************
static inline void am_hal_clkmgr_wait_HFRC_ADJ_stabilize(uint32_t *pui32StabilizingCounter)
{
    //
    // If HFRC has just been started in ADJ mode, wait for HFRC frequency to
    // stabilize.
    //
    if (g_bClkStabilizing_HFRC_ADJ)
    {
        am_hal_clkmgr_clock_stabilize_wait(&g_bClkStabilizing_HFRC_ADJ, pui32StabilizingCounter);

        AM_CRITICAL_BEGIN
        g_bClkStablized_HFRC_ADJ = true;
        g_bClkStabilizing_HFRC_ADJ = false;
        g_pui32ClkStabilizingCounter_HFRC_ADJ = NULL;
        AM_CRITICAL_END
    }
}

//*****************************************************************************
//
//! @brief Handles clock request for HFRC clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_HFRC(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32StabilizingCounter = AM_HAL_CLKMGR_HFRC_ADJ_WAIT_LOOP_CNT;

    //
    // If HFRC has already been requested, wait for stabilizing count to
    // complete and return SUCCESS
    if ( am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_HFRC, eUserId) )
    {
        AM_CRITICAL_BEGIN
        if (g_bClkStabilizing_HFRC_ADJ)
        {
            ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_HFRC_ADJ;
        }
        g_pui32ClkStabilizingCounter_HFRC_ADJ = &ui32StabilizingCounter;
        AM_CRITICAL_END
        am_hal_clkmgr_wait_HFRC_ADJ_stabilize(&ui32StabilizingCounter);
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Update clock stabilizing counter
    //
    if (g_bClkStabilizing_HFRC_ADJ)
    {
        ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_HFRC_ADJ;
    }

    //
    // Check HFRC clock configuration is valid
    //
    if (!g_bClkMgrClkCfgValid_HFRC)
    {
        ui32Status = AM_HAL_STATUS_FAIL;
    }

    //
    // Configure HFRC clock if the clock wasn't use before this.
    // Then, Set User Flag for HFRC clock.
    //
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        uint8_t ui8CurUserCnt = am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_HFRC);
        if (ui8CurUserCnt == 0)
        {
            am_hal_clkgen_private_hfrc_force_on(true);
            if (g_ui32ClkMgrClkCfgFreq_HFRC != AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ)
            {
                ui32Status = am_hal_clkgen_private_hfadj_apply(g_sClkMgrClkConfig_HFRC.hfrc.HFRCAdj);

                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    if (!g_bClkStabilizing_HFRC_ADJ && !g_bClkStablized_HFRC_ADJ)
                    {
                        g_bClkStabilizing_HFRC_ADJ = true;
                    }
                }
                else
                {
                    am_hal_clkgen_private_hfrc_force_on(false);
                }
            }
        }

        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC, eUserId, true);
            if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
            {
                am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
            }
        }

        if (g_bClkStabilizing_HFRC_ADJ)
        {
            g_pui32ClkStabilizingCounter_HFRC_ADJ = &ui32StabilizingCounter;
        }
    }
    AM_CRITICAL_END
    am_hal_clkmgr_wait_HFRC_ADJ_stabilize(&ui32StabilizingCounter);
    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock release for HFRC clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_HFRC(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_HFRC, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag HFRC clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_HFRC))
    {
        if ( g_bDisableHfadjOnFullReleased )
        {
            am_hal_clkgen_private_hfadj_disable();
            am_hal_clkmgr_clear_stablizing_status();
        }
        am_hal_clkgen_private_hfrc_force_on(false);
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock request for SYSPLL clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_SYSPLL(am_hal_clkmgr_user_id_e eUserId)
{
    bool bWaitLock = false;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Return success immediately if it has already been requested
    //
    if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId))
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Check wehther requested SYSPLL sub-domain configuration is valid.
    //
    if ((eUserId == AM_HAL_CLKMGR_USER_ID_PLLVCO) && !g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable)
    {
        return AM_HAL_STATUS_FAIL;
    }
    if ((eUserId == AM_HAL_CLKMGR_USER_ID_PLLPOSTDIV) && !g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Set User Flag for SYSPLL clock first to avoid changing of clock config
    // when we are waiting for clock dependency to be ready.
    //
    AM_CRITICAL_BEGIN
    if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_SYSPLL) == 0)
    {
        g_bSYSPLLStartPending = true;
    }
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, true);
    AM_CRITICAL_END

    if ((g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable || g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable) && g_bSYSPLLStartPending)
    {
        switch(g_sClkMgrClkConfig_SYSPLL.syspll.eFref)
        {
            case AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4:
                ui32Status = am_hal_clkmgr_request_HFRC(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_EXTREFCLK:
                ui32Status = am_hal_clkmgr_request_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_XTAL48MHz:
                ui32Status = am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;
        }
    }

    //
    // If clock dependency request successful, continue to enable syspll
    // if it is not yet configured.
    //
    AM_CRITICAL_BEGIN
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        if (!g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable && !g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable)
        {
            ui32Status = AM_HAL_STATUS_FAIL;
        }

        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (g_bSYSPLLStartPending)
            {
                g_bSYSPLLStartPending = false;
                if (g_pSyspllHandle == NULL)
                {
                    ui32Status = am_hal_syspll_initialize(0, &g_pSyspllHandle);
                }

                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    ui32Status = am_hal_syspll_configure(g_pSyspllHandle, &g_sClkMgrClkConfig_SYSPLL.syspll);
                }

                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    ui32Status = am_hal_syspll_enable(g_pSyspllHandle);
                }

                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    bWaitLock = true;
                }
                else
                {
                    if (g_pSyspllHandle != NULL)
                    {
                        am_hal_syspll_deinitialize(g_pSyspllHandle);
                        g_pSyspllHandle = NULL;
                    }
                }
            }
        }

        // Clock request failed, clear User Flag from the clock
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, false);
        }
    }
    else
    {
        // Clock request failed, clear User Flag from the clock
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, false);
    }
    AM_CRITICAL_END

    //
    // Release clock if previous operation have failed.
    //
    if ((g_sClkMgrClkConfig_SYSPLL.syspll.bVCOOutEnable || g_sClkMgrClkConfig_SYSPLL.syspll.bPostDivOutEnable) && ui32Status)
    {
        switch(g_sClkMgrClkConfig_SYSPLL.syspll.eFref)
        {
            case AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4:
                am_hal_clkmgr_release_HFRC(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_EXTREFCLK:
                am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_XTAL48MHz:
                am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;
        }
    }

    //
    // Wait SysPLL lock if we have just enabled SYSPLL
    //
    if ( bWaitLock )
    {
        ui32Status = am_hal_syspll_lock_wait(g_pSyspllHandle);
    }

    //
    // Clear User Flag for SYSPLL clock if SYSPLL failed to lock
    //
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, false);
        am_hal_syspll_deinitialize(g_pSyspllHandle);
        g_pSyspllHandle = NULL;
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock release for SYSPLL clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_SYSPLL(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    bool bReleaseXtalHs = false;

    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_SYSPLL))
    {
        // Disable and Deinitialize SYSPLL
        g_bSYSPLLStartPending = false;
        am_hal_syspll_disable(g_pSyspllHandle);
        am_hal_syspll_deinitialize(g_pSyspllHandle);
        g_pSyspllHandle = NULL;

        // Swap SYSPLL Clock User ID at its clock source
        switch(g_sClkMgrClkConfig_SYSPLL.syspll.eFref)
        {
            case AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4:
                ui32Status = am_hal_clkmgr_release_HFRC(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_EXTREFCLK:
                ui32Status = am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                break;

            case AM_HAL_SYSPLL_FREFSEL_XTAL48MHz:
                ui32Status = am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL_REL);
                if (ui32Status == AM_HAL_STATUS_SUCCESS)
                {
                    ui32Status = am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
                    bReleaseXtalHs = true;
                }
                break;
        }
    }
    AM_CRITICAL_END

    if (bReleaseXtalHs)
    {
        am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL_REL);
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock request for SYSPLL VCO clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_PLLVCO(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status;

    //
    // Return success immediately if it has already been requested
    //
    if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_PLLVCO, eUserId))
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Set User Flag for SYSPLL clock first to avoid changing of clock config
    // when we are waiting for clock dependency to be ready.
    //
    AM_CRITICAL_BEGIN
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLVCO, eUserId, true);
    if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
    {
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLVCO, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
    }
    AM_CRITICAL_END

    ui32Status = am_hal_clkmgr_request_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLVCO);

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLVCO, eUserId, false);
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock release for SYSPLL VCO clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_PLLVCO(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_PLLVCO, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag for PLLVCO
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLVCO, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_PLLVCO))
    {
        am_hal_clkmgr_request_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLVCO_REL);
        am_hal_clkmgr_release_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLVCO);
    }
    AM_CRITICAL_END

    am_hal_clkmgr_release_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLVCO_REL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Handles clock request for SYSPLL PostDiv clocks
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_PLLPOSTDIV(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status;

    //
    // Return success immediately if it has already been requested
    //
    if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, eUserId))
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Set User Flag for SYSPLL clock first to avoid changing of clock config
    // when we are waiting for clock dependency to be ready.
    //
    AM_CRITICAL_BEGIN
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, eUserId, true);
    if ( eUserId != AM_HAL_CLKMGR_USER_ID_PRESTART )
    {
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, AM_HAL_CLKMGR_USER_ID_PRESTART, false);
    }
    AM_CRITICAL_END

    ui32Status = am_hal_clkmgr_request_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLPOSTDIV);

    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        AM_CRITICAL_BEGIN
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, eUserId, false);
        AM_CRITICAL_END
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock release for SYSPLL PostDiv clocks
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_PLLPOSTDIV(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag for PLLPOSTDIV
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV))
    {
        am_hal_clkmgr_request_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLPOSTDIV_REL);
        am_hal_clkmgr_release_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLPOSTDIV);
    }
    AM_CRITICAL_END

    am_hal_clkmgr_release_SYSPLL(AM_HAL_CLKMGR_USER_ID_PLLPOSTDIV_REL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize Clock Manager
//
//*****************************************************************************
uint32_t am_hal_clkmgr_initialize(void)
{
    //
    // Register XTAL_HS request response callback handler
    //
    am_hal_ipc_mbox_handler_register(AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_ON_RSP, am_hal_clkmgr_mbox_rfxtal_enable_resp_handler, NULL);
    am_hal_ipc_mbox_handler_register(AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_OFF_RSP, am_hal_clkmgr_mbox_rfxtal_disable_resp_handler, NULL);
    am_hal_ipc_mbox_handler_register(AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_CONFIG_RSP, am_hal_clkmgr_mbox_rfxtal_config_resp_handler, NULL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set Clock Configuration for a specific clock
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_config(am_hal_clkmgr_clock_id_e eClockId, uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockConfig)
{
    uint32_t ui32Status;

    switch(eClockId)
    {
        case AM_HAL_CLKMGR_CLK_ID_HFRC:
            ui32Status = am_hal_clkmgr_config_HFRC(ui32RequestedClk, psClockConfig);
            break;
        case AM_HAL_CLKMGR_CLK_ID_PLLVCO:
            ui32Status = am_hal_clkmgr_config_PLLVCO(ui32RequestedClk, psClockConfig);
            break;
        case AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV:
            ui32Status = am_hal_clkmgr_config_PLLPOSTDIV(ui32RequestedClk, psClockConfig);
            break;
        default:
            ui32Status = AM_HAL_STATUS_INVALID_OPERATION;
            break;
    }
    return ui32Status;
}


//*****************************************************************************
//
// Get Clock Configuration for a specific clock
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_config_get(am_hal_clkmgr_clock_id_e eClockId, uint32_t *pui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockConfig)
{
    am_hal_clkmgr_clkcfg_t *pCfg = NULL;

    //
    // Check Clock Config pointer validity
    //
    if (pui32RequestedClk == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check clockID and return clock configuration
    //
    switch (eClockId)
    {
        case AM_HAL_CLKMGR_CLK_ID_HFRC:
            *pui32RequestedClk = g_ui32ClkMgrClkCfgFreq_HFRC;
            pCfg = &g_sClkMgrClkConfig_HFRC;
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLVCO:
            *pui32RequestedClk = g_ui32ClkMgrClkCfgFreq_PLLVCO;
            pCfg = &g_sClkMgrClkConfig_SYSPLL;
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV:
            *pui32RequestedClk = g_ui32ClkMgrClkCfgFreq_PLLPOSTDIV;
            pCfg = &g_sClkMgrClkConfig_SYSPLL;
            break;

        default:
            break;
    }

    if (pCfg == NULL)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if (psClockConfig != NULL)
    {
        memcpy(psClockConfig, pCfg, sizeof(am_hal_clkmgr_clkcfg_t));
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Request for a clock
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_request(am_hal_clkmgr_clock_id_e eClockId, am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_INVALID_ARG;

    //
    //  check userID validity.
    //
    if (eUserId >= AM_HAL_CLKMGR_USER_ID_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // If pre-start user is requesting for a clock, and the clock is already
    // started, return success directly.
    //
    if ((eUserId == AM_HAL_CLKMGR_USER_ID_PRESTART) && (am_hal_clkmgr_is_requested(eClockId)))
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    switch(eClockId)
    {
        case AM_HAL_CLKMGR_CLK_ID_LFRC:
            ui32Status = am_hal_clkmgr_request_LFRC(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_XTAL_LS:
            ui32Status = am_hal_clkmgr_request_XTAL_LS(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_XTAL_HS:
            ui32Status = am_hal_clkmgr_request_XTAL_HS(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK:
            ui32Status = am_hal_clkmgr_request_EXTREF_CLK(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_HFRC:
            ui32Status = am_hal_clkmgr_request_HFRC(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLVCO:
            ui32Status = am_hal_clkmgr_request_PLLVCO(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV:
            ui32Status = am_hal_clkmgr_request_PLLPOSTDIV(eUserId);
            break;

        default:
            ui32Status = AM_HAL_STATUS_INVALID_ARG;
            break;
    }
    return ui32Status;
}

//*****************************************************************************
//
//  Release clock specified for the user specified
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_release(am_hal_clkmgr_clock_id_e eClockId, am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    //  check userID validity.
    //
    if (eUserId >= AM_HAL_CLKMGR_USER_ID_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch(eClockId)
    {
        case AM_HAL_CLKMGR_CLK_ID_LFRC:
            ui32Status = am_hal_clkmgr_release_LFRC(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_XTAL_LS:
            ui32Status = am_hal_clkmgr_release_XTAL_LS(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_XTAL_HS:
            ui32Status = am_hal_clkmgr_release_XTAL_HS(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK:
            ui32Status = am_hal_clkmgr_release_EXTREF_CLK(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_HFRC:
            ui32Status = am_hal_clkmgr_release_HFRC(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLVCO:
            ui32Status = am_hal_clkmgr_release_PLLVCO(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV:
            ui32Status = am_hal_clkmgr_release_PLLPOSTDIV(eUserId);
            break;

        default:
            ui32Status = AM_HAL_STATUS_INVALID_ARG;
            break;
    }
    return ui32Status;
}


//*****************************************************************************
//
// Release all clock requested for the user specified
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_release_all(am_hal_clkmgr_user_id_e eUserId)
{
    //
    //  check userID validity.
    //
    if (eUserId >= AM_HAL_CLKMGR_USER_ID_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Loop through Clock ID and check for clock that is reqested and free them
    //
    for (am_hal_clkmgr_clock_id_e eClockId = AM_HAL_CLKMGR_CLK_ID_LFRC; eClockId < AM_HAL_CLKMGR_CLK_ID_MAX; eClockId++)
    {
        if (am_hal_clkmgr_is_requested_by_user(eClockId, eUserId))
        {
            am_hal_clkmgr_clock_release(eClockId, eUserId);
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Get clock status
//
//*****************************************************************************
uint32_t am_hal_clkmgr_clock_status_get(am_hal_clkmgr_clock_id_e eClockId, uint32_t *pui32UserCount)
{
    //
    // Check eCLockId validity
    //
    if (eClockId >= AM_HAL_CLKMGR_CLK_ID_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check User Count pointer. User count pointer is madatory
    //
    if (pui32UserCount == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    *pui32UserCount = am_hal_clkmgr_user_count_get(eClockId);

    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Set Clock Manager Board Info
//
//*****************************************************************************
uint32_t am_hal_clkmgr_board_info_set(am_hal_clkmgr_board_info_t *psBoardInfo)
{
    if (psBoardInfo == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memcpy(&g_sClkMgrBoardInfo, psBoardInfo, sizeof(am_hal_clkmgr_board_info_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get Clock Manager Board Info
//
//*****************************************************************************
uint32_t am_hal_clkmgr_board_info_get(am_hal_clkmgr_board_info_t *psBoardInfo)
{
    if (psBoardInfo == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memcpy(psBoardInfo, &g_sClkMgrBoardInfo, sizeof(am_hal_clkmgr_board_info_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clock Manager controls
//
//*****************************************************************************
uint32_t am_hal_clkmgr_control(am_hal_clkmgr_control_e eControl, const void *pConfig)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters
    //
    if (eControl > AM_HAL_CLKMGR_CONTROL_MAX)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch ( eControl )
    {
        case AM_HAL_CLKMGR_DISABLE_HFADJ_ON_FULL_RELEASE:
        {
            //
            // Configuration to disable HFADJ on full release (No more active
            // users for the HFRC clock)
            //
            if (pConfig != NULL)
            {
                g_bDisableHfadjOnFullReleased = (*((bool *)pConfig) == true);
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            else
            {
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
            }
            break;
        }

        case AM_HAL_CLKMGR_CONFIGURE_RFXTAL:
        {
            bool bOtpTrimOkay = false;
            bool bOffOtpOnExit = false;
            bool bUseDefaultConfig = (pConfig == NULL);
            uint32_t ui32RfTrim[4];

            //
            // Ensure OTP is on before reading the RF trims.
            //
            if (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP == 0)
            {
                bOffOtpOnExit = true;
                am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
            }

            //
            // Word0: RF Trim Version; Word1: XOCONFIG; Word2: OSCTRIM; Word3: RADIOTUNE
            //
            ui32Status = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_OTP_INFO1, AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O / 4, 4, ui32RfTrim);
            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                //
                // Check the trim version to ensure there is available trim in the OTP info1
                //
                if (ui32RfTrim[0] != 0)
                {
                    bOtpTrimOkay = true;
                }
                else
                {
                    ui32Status = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_MRAM_INFO1, AM_REG_INFO1_RFTRIM_TRIM_VER_O / 4, 4, ui32RfTrim);
                    if (ui32Status == AM_HAL_STATUS_SUCCESS)
                    {
                        //
                        // Check the trim version to ensure there is available trim in the MRAM info1
                        //
                        if (ui32RfTrim[0] != 0)
                        {
                            bOtpTrimOkay = true;
                        }
                    }
                }
            }

            if (bUseDefaultConfig)
            {
                if (bOtpTrimOkay)
                {
                    //
                    // Configure the XOSCTRIM, XOCONFIG and RADIOTUNE
                    //
                    g_sClkMgrClkConfig_RFXTAL.ui32XOFreqTrim = ui32RfTrim[(AM_REG_OTP_INFO1_RFTRIM_OSCTRIM_O - AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O) / 4];
                    g_sClkMgrClkConfig_RFXTAL.ui32XOConfig = ui32RfTrim[(AM_REG_OTP_INFO1_RFTRIM_XOCONFIG_O - AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O) / 4];
                    g_sClkMgrClkConfig_RFXTAL.ui32RadioTune = ui32RfTrim[(AM_REG_OTP_INFO1_RFTRIM_RADIOTUNE_O - AM_REG_OTP_INFO1_RFTRIM_TRIM_VER_O) / 4];
                    g_bClkMgrClkCfgValid_RFXTAL = true;
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_OPERATION;
                }
            }
            else
            {
                //
                // Configure the user specific RFXTAL configurations
                //
                memcpy(&g_sClkMgrClkConfig_RFXTAL, pConfig, sizeof(am_hal_clkmgr_rfxtal_config_t));
                g_bClkMgrClkCfgValid_RFXTAL = true;
            }

            //
            // Recover the OTP power state
            //
            if (bOffOtpOnExit)
            {
                am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
            }

            //
            // Send the configuration if the radio subsystem is powered up and mailbox is ready
            //
            if (am_hal_ipc_mbox_init_state_get() == AM_HAL_IPC_MBOX_INIT_STATE_INITIALIZED)
            {
                ui32Status = am_hal_clkmgr_private_rfxtal_config_send();
            }
        }
            break;

        case AM_HAL_CLKMGR_SYPLL_FREF_PRIORITY_SET:
        {
            am_hal_clkmgr_syspll_fref_priority_t *prio;

            //
            // Validate configuration passed in
            //
            if (pConfig == NULL)
            {
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
                break;
            }

            prio = (am_hal_clkmgr_syspll_fref_priority_t *)pConfig;

            for (uint32_t ui32Idx = 0; ui32Idx < AM_HAL_CLKMGR_SYSPLL_FREF_PRIORITY_COUNT; ui32Idx++)
            {
                if (prio->prio[ui32Idx] > (am_hal_syspll_frefsel_e)MCUCTRL_PLLCTL0_FREFSEL_RFXTAL48MHZ)
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
            }

            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                memcpy(&g_sFrefPriority, prio, sizeof(am_hal_clkmgr_syspll_fref_priority_t));
            }
            break;
        }

        case AM_HAL_CLKMGR_RELEASE_PRESTART_CLK:
        {
            am_hal_clkmgr_clock_id_e eClkID;
            for (eClkID = AM_HAL_CLKMGR_CLK_ID_LFRC; eClkID < AM_HAL_CLKMGR_CLK_ID_MAX; eClkID++)
            {
                if (am_hal_clkmgr_is_requested_by_user(eClkID, AM_HAL_CLKMGR_USER_ID_PRESTART))
                {
                    g_ui32PrestartReleasedClk |= (1 << eClkID);
                    am_hal_clkmgr_clock_release(eClkID, AM_HAL_CLKMGR_USER_ID_PRESTART);
                }
            }
            break;
        }

        case AM_HAL_CLKMGR_RESUME_PRESTART_CLK:
        {
            while (g_ui32PrestartReleasedClk)
            {
                uint32_t ui32BitSetPos = __builtin_ctz(g_ui32PrestartReleasedClk);
                am_hal_clkmgr_clock_request((am_hal_clkmgr_clock_id_e)ui32BitSetPos, AM_HAL_CLKMGR_USER_ID_PRESTART);
                g_ui32PrestartReleasedClk &= ~(1U << ui32BitSetPos);
            }
            break;
        }

        case AM_HAL_CLKMGR_SYSPLL_EMPHASIS_SET:
        {
            //
            // Configuration to set SYSPLL configuration emphasis lock speed
            // or lowest current
            //
            if (pConfig != NULL)
            {
                am_hal_syspll_emphasis_set(*((am_hal_syspll_emphasis_mode_e *)pConfig));
                ui32Status = AM_HAL_STATUS_SUCCESS;
            }
            else
            {
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
            }
            break;
        }


        default:
            return AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    return ui32Status;
}

//! @cond CLKMGR_PRIVATE_FUNC
//*****************************************************************************
//
// Handle Call to Legacy CLKGEN AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE
//
//*****************************************************************************
uint32_t am_hal_clkmgr_private_clkgen_hfadj_apply(void* pArgs)
{
    //
    // If Board Information show LFRC is absent, return INVALID OPERATION
    //
    if (g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // In legacy clkgen API, HFADJ with pArgs==0 would request for HFADJ-48MHZ
    // otherwise, pArgs is the register value to fill into HFADJ register.
    // Only support HFADJ-48MHz for now.
    //
    if ( pArgs == NULL )
    {
        return am_hal_clkmgr_config_HFRC(AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ, NULL);
    }
    else
    {
        uint32_t ui32Ratio = _FLD2VAL(CLKGEN_HFADJ_HFXTADJ, *((uint32_t*)pArgs));
        uint32_t ui32FreqReq = ui32Ratio * g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq;

        // Check if this request is actually HFADJ_48MHZ. If so, set frequency
        // requested to exactly AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ so that
        // HFRC clock config API recognize it correctly.
        if ((ui32FreqReq < AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ) &&
            ((AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ - ui32FreqReq) < g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq))
        {
            ui32FreqReq = AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ;
        }
        else if ((ui32FreqReq > AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ) &&
                 ((ui32FreqReq - AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ) < g_sClkMgrBoardInfo.sXtalLs.ui32XtalLsFreq))
        {
            ui32FreqReq = AM_HAL_CLKMGR_HFRC_FREQ_ADJ_48MHZ;
        }

        return am_hal_clkmgr_config_HFRC(ui32FreqReq, pArgs);
    }
}

//*****************************************************************************
//
// Handle Call to Legacy CLKGEN AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE
//
//*****************************************************************************
uint32_t am_hal_clkmgr_private_clkgen_hfadj_disable()
{
    return am_hal_clkmgr_config_HFRC(AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ, NULL);
}

//*****************************************************************************
//
// Prepare System Clock for deep sleep
//
//*****************************************************************************
void am_hal_clkmgr_private_deepsleep_enter(void)
{
    // Disable and Power Off SYSPLL from SW register if CPU is the only user of
    // SYSPLL.
    // Note: HW will keep SYSPLL powered up until deepsleep is entered
    if ((am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_PLLVCO) == 1) &&
        (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV) == 0) &&
        am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_PLLVCO, AM_HAL_CLKMGR_USER_ID_CPU))
    {
        // Disable SYSPLL
        am_hal_syspll_disable(g_pSyspllHandle);

        // Power Off SYSPLL
        am_hal_pwrctrl_syspll_disable();

        // Set SYSPLL disabled for deep sleep flag
        g_bSysPllDisabledForDeepSleep = true;

        // Release HFRC if it is the FREF for SYSPLL, and let hardware manage
        // it if CPU is the only user.
        if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_SYSPLL))
        {
            am_hal_clkmgr_release_HFRC(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            g_bSysPllReleasedHFRCForDeepSleep = true;
        }
    }
}

//*****************************************************************************
//
// Recover System Clock after wakeup from deep sleep
//
//*****************************************************************************
void am_hal_clkmgr_private_deepsleep_exit(void)
{
    // Disable and Power Off SYSPLL from SW register if CPU is the only user of
    // SYSPLL.
    // Note: HW will keep SYSPLL powered up until deepsleep is entered
    if (g_bSysPllDisabledForDeepSleep)
    {
        // Request HFRC if it was disabled before deep sleep
        if (g_bSysPllReleasedHFRCForDeepSleep)
        {
            // Request HFRC as clock source for SYSPLL
            am_hal_clkmgr_request_HFRC(AM_HAL_CLKMGR_USER_ID_SYSPLL);

            // Clear HFRC released for deep sleep flag
            g_bSysPllReleasedHFRCForDeepSleep = false;
        }

        // Power Off SYSPLL
        am_hal_pwrctrl_syspll_enable();

        // Disable SYSPLL
        am_hal_syspll_enable(g_pSyspllHandle);

        // Clear SYSPLL disabled for deep sleep flag
        g_bSysPllDisabledForDeepSleep = false;
    }
}

//*****************************************************************************
//
// Send the RFXTAL configuration to radio subsystem via mailbox
//
//*****************************************************************************
uint32_t am_hal_clkmgr_private_rfxtal_config_send(void)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32Config[5] = {AM_HAL_IPC_MBOX_SIGNAL_MSG_RFXTAL_CONFIG_REQ};
    uint32_t ui32ReqWaitCounter = AM_HAL_CLKMGR_XTAL_HS_REQ_WAIT_LOOP_CNT;

    if (!g_bClkMgrClkCfgValid_RFXTAL)
    {
        //
        // Just return success and the RFXTAL will use default configurations
        // set in radio subsystem.
        //
        return ui32Status;
    }

    g_bClkConfigWaiting_XTAL_HS = true;
    g_bClkConfigSuccess_XTAL_HS = false;

    //
    // Set the command size
    //
    ui32Config[1] = sizeof(am_hal_clkmgr_rfxtal_config_t) / sizeof(uint32_t);

    //
    // Set the RFXTAL configurations
    //
    ui32Config[2] = g_sClkMgrClkConfig_RFXTAL.ui32XOFreqTrim;
    ui32Config[3] = g_sClkMgrClkConfig_RFXTAL.ui32XOConfig;
    ui32Config[4] = g_sClkMgrClkConfig_RFXTAL.ui32RadioTune;

    ui32Status = am_hal_ipc_mbox_data_write(&ui32Config[0], (ui32Config[1] + 2));
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        am_hal_clkmgr_wait_XTAL_HS_response(AM_HAL_CLKMGR_XTAL_HS_OP_CONFIG, &ui32ReqWaitCounter);
        if (!g_bClkConfigSuccess_XTAL_HS)
        {
            ui32Status = AM_HAL_STATUS_TIMEOUT;
        }
    }

    return ui32Status;
}
//! @endcond CLKMGR_PRIVATE_FUNC

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
