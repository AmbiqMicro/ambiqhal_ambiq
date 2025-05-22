// ****************************************************************************
//
//! @file am_hal_clkmgr.c
//!
//! @brief Clock manager functions that manage system clocks and minimize
//!        power consumption by powering down clocks when possible.
//!
//! @addtogroup clkmgr5b CLKMGR - Clock Manager
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
#include "mcu/am_hal_clkgen_private.h"

#define AM_HAL_CLKMGR_HFRC2_ADJ_WAIT_TIME_US            (500)
#define AM_HAL_CLKMGR_HFRC_ADJ_WAIT_TIME_US             (10000)
#define AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_TIME_US      (1500)
#define AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US         (10)
#define AM_HAL_CLKMGR_HFRC2_ADJ_WAIT_LOOP_CNT           (AM_HAL_CLKMGR_HFRC2_ADJ_WAIT_TIME_US / AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US)
#define AM_HAL_CLKMGR_HFRC_ADJ_WAIT_LOOP_CNT            (AM_HAL_CLKMGR_HFRC_ADJ_WAIT_TIME_US / AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US)
#define AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_LOOP_CNT     (AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_TIME_US / AM_HAL_CLKMGR_CLOCK_STABILIZING_LOOP_US)

#define AM_HAL_CLKMGR_USERID_DWORD_CNT ((AM_HAL_CLKMGR_USER_ID_MAX + 31) >> 5)

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
static uint32_t g_ui32ClkMgrClkSrcBm[AM_HAL_CLKMGR_CLK_ID_MAX][AM_HAL_CLKMGR_USERID_DWORD_CNT] = {0};

// Variable clock config - valid flags
// HFRC and HFRC2 boot-up with free-running HFRC frequencies as default
static bool g_bClkMgrClkCfgValid_HFRC   = true;
static bool g_bClkMgrClkCfgValid_HFRC2  = true;
static bool g_bClkMgrClkCfgValid_SYSPLL = false;

// Variable clock config - configured clock rate
static uint32_t g_ui32ClkMgrClkCfgFreq_HFRC   = AM_HAL_CLKMGR_HFRC_FREQ_FREE_RUN_APPROX_48MHZ;
static uint32_t g_ui32ClkMgrClkCfgFreq_HFRC2  = AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ;
static uint32_t g_ui32ClkMgrClkCfgFreq_SYSPLL = 0;

// Variable clock configurations
static am_hal_clkmgr_clkcfg_t g_sClkMgrClkConfig_HFRC   = {0};
static am_hal_clkmgr_clkcfg_t g_sClkMgrClkConfig_HFRC2  = {0};
static am_hal_clkmgr_clkcfg_t g_sClkMgrClkConfig_SYSPLL = {0};

// Instance handle for SysPLL
static void *g_pSyspllHandle = NULL;

// Flag for clock stabilizing status
static bool      g_bClkStabilizing_XTAL_HS              = false;
static bool      g_bClkStabilizing_HFRC_ADJ             = false;
static bool      g_bClkStabilizing_HFRC2_ADJ            = false;
static bool      g_bClkStablized_HFRC_ADJ               = false;
static uint32_t *g_pui32ClkStabilizingCounter_XTAL_HS   = NULL;
static uint32_t *g_pui32ClkStabilizingCounter_HFRC_ADJ  = NULL;
static uint32_t *g_pui32ClkStabilizingCounter_HFRC2_ADJ = NULL;

// Flags for clock start pending
static bool g_bHFRC2StartPending  = false;
static bool g_bSYSPLLStartPending = false;

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
    if ((eClockId >= AM_HAL_CLKMGR_CLK_ID_MAX) || (eUserId >= AM_HAL_CLKMGR_USER_ID_MAX))
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
            g_bClkStabilizing_HFRC_ADJ = false;
            g_pui32ClkStabilizingCounter_HFRC_ADJ = NULL;
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
//! @brief Handles clock config for HFRC2 clock soruce
//!
//! @param ui32RequestedClk - Frequency HFRC2 clock is to be configured to
//! @param psClockConfig - [Optional] clock config structure to be used for
//!                        HFRC2. set to NULL if auto-generation is desired.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_config_HFRC2(uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockCfg)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    bool bSwitchingAllowed = (((ui32RequestedClk == AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ) || (ui32RequestedClk == AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_250MHZ)) &&
                              ((g_ui32ClkMgrClkCfgFreq_HFRC2 == AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ) || (g_ui32ClkMgrClkCfgFreq_HFRC2 == AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_250MHZ)));
    am_hal_clkmgr_clkcfg_t sGeneratedConfig =
    {
        .hfrc2 = AM_HAL_CLKGEN_DEFAULT_HFRC2_ADJ_CONFIG
    };

    //
    // Check whether the HFRC2 frequency requested is supported
    //
    if ((ui32RequestedClk != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ) &&
        (ui32RequestedClk != AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ) &&
        (ui32RequestedClk != AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_250MHZ))
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // If requested clock is Adjusted HFRC2, check or generate config
    //
    if (ui32RequestedClk != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ)
    {
        if (psClockCfg == NULL)
        {
            uint32_t ui32RefFreq = 0;
            if (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq != 0)
            {
                sGeneratedConfig.hfrc2.eRefClkSel = AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS;
                ui32RefFreq = g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq;
            }
            else if (g_sClkMgrBoardInfo.ui32ExtRefClkFreq != 0)
            {
                sGeneratedConfig.hfrc2.eRefClkSel = AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_EXTREF_CLK;
                ui32RefFreq = g_sClkMgrBoardInfo.ui32ExtRefClkFreq;
            }
            else
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            ui32Status = am_hal_clkgen_hfrc2adj_ratio_calculate(ui32RefFreq, ui32RequestedClk, sGeneratedConfig.hfrc2.eRefClkDiv, &sGeneratedConfig.hfrc2.ui32AdjRatio);
            psClockCfg = &sGeneratedConfig;
        }
        else
        {
            if ((psClockCfg->hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS) &&
                (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq == 0))
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            if ((psClockCfg->hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_EXTREF_CLK) &&
                (g_sClkMgrBoardInfo.ui32ExtRefClkFreq == 0))
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }
        }
    }

    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        bool bClockRequested = false;
        AM_CRITICAL_BEGIN
        // Check current user count
        if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_HFRC2) != 0)
        {
            // Only allow switching between HFRC2-FreeRun and HF2ADJ-250MHz
            // when there is already active user
            if ( bSwitchingAllowed )
            {
                bClockRequested = true;
            }
            else
            {
                ui32Status = AM_HAL_STATUS_IN_USE;
            }
        }

        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            // Save configuration
            if (ui32RequestedClk != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ)
            {
                memcpy(&g_sClkMgrClkConfig_HFRC2, psClockCfg, sizeof(am_hal_clkmgr_clkcfg_t));
            }
            g_ui32ClkMgrClkCfgFreq_HFRC2 = ui32RequestedClk;
            g_bClkMgrClkCfgValid_HFRC2 = true;
        }
        AM_CRITICAL_END

        // If clock already been requested, check whether we should apply ADJ
        if (bClockRequested)
        {

            // Request new clock source
            if (g_sClkMgrClkConfig_HFRC2.hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS)
            {
                am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
            }
            else
            {
                am_hal_clkmgr_request_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
            }

            AM_CRITICAL_BEGIN
            if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_HFRC2) != 0)
            {
                if ( g_ui32ClkMgrClkCfgFreq_HFRC2 == AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ )
                {
                    ui32Status = am_hal_clkgen_private_hf2adj_disable();
                }
                else
                {
                    ui32Status = am_hal_clkgen_private_hf2adj_apply(&(g_sClkMgrClkConfig_HFRC2.hfrc2));

                    // Make sure unneeded reference source is released
                    if (ui32Status == AM_HAL_STATUS_SUCCESS)
                    {
                        if (g_sClkMgrClkConfig_HFRC2.hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS)
                        {
                            am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
                        }
                        else
                        {
                            am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
                        }
                    }
                }

                if ((ui32Status != AM_HAL_STATUS_SUCCESS) || (g_ui32ClkMgrClkCfgFreq_HFRC2 == AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ))
                {
                    am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
                    am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
                }
            }
            else
            {
                am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
                am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
            }
            AM_CRITICAL_END
        }
    }
    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock config for SYSPLL clock soruce
//!
//! @param ui32RequestedClk - Frequency SYSPLL FOUTPOSTDIV clock is to be
//!                           configured to
//! @param psClockConfig - [Optional] clock config structure to be used for
//!                        SYSPLL. set to NULL if auto-generation is desired.
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_config_SYSPLL(uint32_t ui32RequestedClk, am_hal_clkmgr_clkcfg_t *psClockCfg)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_clkmgr_clkcfg_t sGeneratedConfig = {0};

    //
    // Check or Generate Config
    //
    if (psClockCfg == NULL)
    {
        uint32_t ui32RefFreq = 0;
        if (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq != 0)
        {
            sGeneratedConfig.syspll.eFref = AM_HAL_SYSPLL_FREFSEL_XTAL32MHz;
            ui32RefFreq = g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq;
        }
        else if (g_sClkMgrBoardInfo.ui32ExtRefClkFreq != 0)
        {
            sGeneratedConfig.syspll.eFref = AM_HAL_SYSPLL_FREFSEL_EXTREFCLK;
            ui32RefFreq = g_sClkMgrBoardInfo.ui32ExtRefClkFreq;
        }
        else
        {
            return AM_HAL_STATUS_INVALID_OPERATION;
        }

        ui32Status = am_hal_syspll_config_generate_with_postdiv(&sGeneratedConfig.syspll, ui32RefFreq, ui32RequestedClk);
        psClockCfg = &sGeneratedConfig;
    }
    else
    {
        if ((psClockCfg->syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL32MHz) &&
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
            memcpy(&g_sClkMgrClkConfig_SYSPLL, psClockCfg, sizeof(am_hal_clkmgr_clkcfg_t));
            g_ui32ClkMgrClkCfgFreq_SYSPLL = ui32RequestedClk;
            g_bClkMgrClkCfgValid_SYSPLL = true;
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
//! @brief Wait for XTAL_HS clock to be stabilized
//!
//! @param  pui32StabilizingCounter - pointer to timeout counter for the clock.
//
//*****************************************************************************
static inline void am_hal_clkmgr_wait_XTAL_HS_stabilize(uint32_t *pui32StabilizingCounter)
{
    //
    // If XTAL_HS has just been started in XTAL mode. wait for XTAL clock to
    // stabilize.
    //
    if (g_bClkStabilizing_XTAL_HS)
    {
        am_hal_clkmgr_clock_stabilize_wait(&g_bClkStabilizing_XTAL_HS, pui32StabilizingCounter);

        AM_CRITICAL_BEGIN
        g_bClkStabilizing_XTAL_HS = false;
        g_pui32ClkStabilizingCounter_XTAL_HS = NULL;
        AM_CRITICAL_END
    }
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
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32StabilizingCounter = AM_HAL_CLKMGR_XTAL_HS_STARTUP_WAIT_LOOP_CNT;

    //
    // Check whether XTAL_HS is configured for the device
    //
    if (g_sClkMgrBoardInfo.sXtalHs.ui32XtalHsFreq == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Return success immediately if it has already been requested
    //
    if ( am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId) )
    {
        AM_CRITICAL_BEGIN
        if (g_bClkStabilizing_XTAL_HS)
        {
            ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_XTAL_HS;
        }
        g_pui32ClkStabilizingCounter_XTAL_HS = &ui32StabilizingCounter;
        AM_CRITICAL_END
        am_hal_clkmgr_wait_XTAL_HS_stabilize(&ui32StabilizingCounter);
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Get current XTAL HS enable status
    //
    am_hal_mcuctrl_ext32m_status_e eXtalHsStatus;
    am_hal_mcuctrl_extclk32m_status_get(&eXtalHsStatus);

    //
    // Update Clock Stabilizing counter
    //
    if (g_bClkStabilizing_XTAL_HS)
    {
        ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_XTAL_HS;
    }

    //
    // Check whether requested clock can be fulfilled
    //
    if (eXtalHsStatus == AM_HAL_MCUCTRL_EXT32M_STATUS_OFF)
    {
        // XTAL_HS not previously enabled. Enable XTAL_HS with mode requested
        if (g_sClkMgrBoardInfo.sXtalHs.eXtalHsMode == AM_HAL_CLKMGR_XTAL_HS_MODE_EXT)
        {
            bool bTrue = true;
            am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL, &bTrue);
        }
        else
        {
            am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, NULL);
            if (!g_bClkStabilizing_XTAL_HS)
            {
                g_bClkStabilizing_XTAL_HS = true;
            }
        }
    }
    else if ((eXtalHsStatus == AM_HAL_MCUCTRL_EXT32M_STATUS_EXT_CLK) &&
             (g_sClkMgrBoardInfo.sXtalHs.eXtalHsMode == AM_HAL_CLKMGR_XTAL_HS_MODE_XTAL))
    {
        // Current XTAL_HS mode is external clock, but requested is XTAL. Mark clock busy.
        ui32Status = AM_HAL_STATUS_IN_USE;
    }
    else if ((eXtalHsStatus == AM_HAL_MCUCTRL_EXT32M_STATUS_XTAL) &&
             (g_sClkMgrBoardInfo.sXtalHs.eXtalHsMode == AM_HAL_CLKMGR_XTAL_HS_MODE_EXT))
    {
        // Current XTAL_HS mode is XTAL, but requested is external clock. Mark clock busy.
        ui32Status = AM_HAL_STATUS_IN_USE;
    }

    //
    // If clock request can be fulfilled, set User Flag for XTAL_HS clock
    //
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId, true);
    }

    //
    // Check and update clock stabilizing variables
    //
    if (g_bClkStabilizing_XTAL_HS)
    {
        g_pui32ClkStabilizingCounter_XTAL_HS = &ui32StabilizingCounter;
    }
    AM_CRITICAL_END

    am_hal_clkmgr_wait_XTAL_HS_stabilize(&ui32StabilizingCounter);

    return ui32Status;
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
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag XTAL_HS clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_XTAL_HS))
    {
        bool bTrue = true;
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE, &bTrue);
        g_bClkStabilizing_XTAL_HS = false;
        g_pui32ClkStabilizingCounter_XTAL_HS = NULL;
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
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
    //
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
        am_hal_clkgen_private_hfrc_force_on(false);
    }
    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Wait for HFRC2_ADJ to be stabilized
//!
//! @param  pui32StabilizingCounter - pointer to timeout counter for the clock.
//
//*****************************************************************************
static inline void am_hal_clkmgr_wait_HFRC2_ADJ_stabilize(uint32_t *pui32StabilizingCounter)
{
    //
    // If HFRC2 has just been started in ADJ mode, wait for HFRC2 frequency to
    // stabilize.
    //
    if (g_bClkStabilizing_HFRC2_ADJ)
    {
        am_hal_clkmgr_clock_stabilize_wait(&g_bClkStabilizing_HFRC2_ADJ, pui32StabilizingCounter);

        AM_CRITICAL_BEGIN
        g_bClkStabilizing_HFRC2_ADJ = false;
        g_pui32ClkStabilizingCounter_HFRC2_ADJ = NULL;
        AM_CRITICAL_END
    }
}

//*****************************************************************************
//
//! @brief Handles clock request for HFRC2 clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user requesting the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_request_HFRC2(am_hal_clkmgr_user_id_e eUserId)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32StabilizingCounter = AM_HAL_CLKMGR_HFRC2_ADJ_WAIT_LOOP_CNT;

    //
    // Return success immediately if it has already been requested
    //
    if (am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId))
    {
        AM_CRITICAL_BEGIN
        if (g_bClkStabilizing_HFRC2_ADJ)
        {
            ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_HFRC2_ADJ;
        }
        g_pui32ClkStabilizingCounter_HFRC2_ADJ = &ui32StabilizingCounter;
        AM_CRITICAL_END
        am_hal_clkmgr_wait_HFRC2_ADJ_stabilize(&ui32StabilizingCounter);
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Set User Flag for HFRC2 clock first to avoid changing of clock config
    // when we are waiting for clock dependency to be ready.
    //
    AM_CRITICAL_BEGIN
    if (am_hal_clkmgr_user_count_get(AM_HAL_CLKMGR_CLK_ID_HFRC2) == 0)
    {
        g_bHFRC2StartPending = true;
    }
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId, true);
    AM_CRITICAL_END

    //
    // Check and request for clock dependency
    //
    if ((g_bClkMgrClkCfgValid_HFRC2) &&
        (g_ui32ClkMgrClkCfgFreq_HFRC2 != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ))
    {
        if (g_sClkMgrClkConfig_HFRC2.hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS)
        {
            ui32Status = am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
        }
        else
        {
            ui32Status = am_hal_clkmgr_request_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
        }

        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            // Clock request failed, clear User Flag from the clock
            AM_CRITICAL_BEGIN
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId, false);
            AM_CRITICAL_END
        }
    }

    //
    // If clock dependency request successful, continue to enable HFRC2
    // if it is not yet configured.
    //
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        AM_CRITICAL_BEGIN
        if (g_bClkStabilizing_HFRC2_ADJ)
        {
            ui32StabilizingCounter = *g_pui32ClkStabilizingCounter_HFRC2_ADJ;
        }

        if ( !g_bClkMgrClkCfgValid_HFRC2 )
        {
            ui32Status = AM_HAL_STATUS_FAIL;
        }

        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (g_bHFRC2StartPending)
            {
                g_bHFRC2StartPending = false;
                am_hal_clkgen_private_hfrc2_force_on(true);
                if (g_ui32ClkMgrClkCfgFreq_HFRC2 != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ)
                {
                    ui32Status = am_hal_clkgen_private_hf2adj_apply(&(g_sClkMgrClkConfig_HFRC2.hfrc2));

                    if (ui32Status == AM_HAL_STATUS_SUCCESS)
                    {
                        if (!g_bClkStabilizing_HFRC2_ADJ)
                        {
                            g_bClkStabilizing_HFRC2_ADJ = true;
                        }
                    }
                    else
                    {
                        am_hal_clkgen_private_hfrc2_force_on(false);
                    }
                }
            }
        }

        //
        // Clock request failed, clear User Flag from the clock
        //
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId, false);
        }

        //
        // Check and update clock stabilizing variables
        //
        if (g_bClkStabilizing_HFRC2_ADJ)
        {
            g_pui32ClkStabilizingCounter_HFRC2_ADJ = &ui32StabilizingCounter;
        }

        AM_CRITICAL_END
    }

    //
    // In case HFRC2 request failed, release reference clock for HFRC2 ADJ
    //
    if ((ui32Status != AM_HAL_STATUS_SUCCESS) &&
        (g_bClkMgrClkCfgValid_HFRC2) &&
        (g_ui32ClkMgrClkCfgFreq_HFRC2 != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ))
    {
        if (g_sClkMgrClkConfig_HFRC2.hfrc2.eRefClkSel == AM_HAL_CLKGEN_HFRC2ADJ_REFCLK_MUX_XTAL_HS)
        {
            ui32Status = am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
        }
        else
        {
            ui32Status = am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
        }
    }

    am_hal_clkmgr_wait_HFRC2_ADJ_stabilize(&ui32StabilizingCounter);

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Handles clock release for HFRC2 clock
//!
//! @param eUserID - am_hal_clkmgr_user_id_e value that indicates the clock
//!                  user releasing the clock
//!
//! @return status - Status for the operation, as defined in am_hal_status_e
//
//*****************************************************************************
static uint32_t am_hal_clkmgr_release_HFRC2(am_hal_clkmgr_user_id_e eUserId)
{
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag HFRC clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_HFRC2, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_HFRC2))
    {
        if (g_ui32ClkMgrClkCfgFreq_HFRC2 != AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ)
        {
            g_bHFRC2StartPending = false;
            am_hal_clkgen_private_hf2adj_disable();
            am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_HFRC2);
            am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_HFRC2);
            g_bClkStabilizing_HFRC2_ADJ = false;
            g_pui32ClkStabilizingCounter_HFRC2_ADJ = NULL;
        }
        am_hal_clkgen_private_hfrc2_force_on(false);
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

    //
    // Check and request for clock dependency
    // Note: For SYSPLL, we will request for both sources of reference clock to
    //       ensure successful switching of the clock mux.
    //
    if (g_bClkMgrClkCfgValid_SYSPLL)
    {
        if (g_sClkMgrClkConfig_SYSPLL.syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL32MHz)
        {
            ui32Status = am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            am_hal_clkmgr_request_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
        }
        else
        {
            ui32Status = am_hal_clkmgr_request_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            am_hal_clkmgr_request_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
        }
    }

    //
    // If clock dependency request successful, continue to enable syspll
    // if it is not yet configured.
    //
    AM_CRITICAL_BEGIN
    if (ui32Status == AM_HAL_STATUS_SUCCESS)
    {
        if (!g_bClkMgrClkCfgValid_SYSPLL)
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
    // Release clock that is no longer needed
    //
    if (g_bClkMgrClkCfgValid_SYSPLL)
    {
        // If HF2ADJ is activated successfully, release the non-selected clock
        // source
        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            if (g_sClkMgrClkConfig_SYSPLL.syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL32MHz)
            {
                am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            }
            else
            {
                am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            }
        }
        // If HF2ADJ is activation failed, release both clock sources
        else
        {
            am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
            am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
        }
    }

    //
    // Wait SysPLL lock if we have just enabled SYSPLL
    //
    if ( bWaitLock)
    {
        ui32Status = am_hal_syspll_lock_wait(g_pSyspllHandle);
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
    //
    // Return success immediately if the user flag is already cleared
    //
    if ( !am_hal_clkmgr_is_requested_by_user(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId) )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    AM_CRITICAL_BEGIN
    //
    // Clear User flag HFRC clock
    //
    am_hal_clkmgr_user_set(AM_HAL_CLKMGR_CLK_ID_SYSPLL, eUserId, false);

    //
    // Check whether the clock is still requested by any user, and turn off
    // the clock if there isn't any left.
    //
    if (!am_hal_clkmgr_is_requested(AM_HAL_CLKMGR_CLK_ID_SYSPLL))
    {
        g_bSYSPLLStartPending = false;
        am_hal_syspll_disable(g_pSyspllHandle);
        am_hal_syspll_deinitialize(g_pSyspllHandle);
        g_pSyspllHandle = NULL;

        if (g_sClkMgrClkConfig_SYSPLL.syspll.eFref == AM_HAL_SYSPLL_FREFSEL_XTAL32MHz)
        {
            am_hal_clkmgr_release_XTAL_HS(AM_HAL_CLKMGR_USER_ID_SYSPLL);
        }
        else
        {
            am_hal_clkmgr_release_EXTREF_CLK(AM_HAL_CLKMGR_USER_ID_SYSPLL);
        }
    }
    AM_CRITICAL_END

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
        case AM_HAL_CLKMGR_CLK_ID_HFRC2:
            ui32Status = am_hal_clkmgr_config_HFRC2(ui32RequestedClk, psClockConfig);
            break;
        case AM_HAL_CLKMGR_CLK_ID_SYSPLL:
            ui32Status = am_hal_clkmgr_config_SYSPLL(ui32RequestedClk, psClockConfig);
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

        case AM_HAL_CLKMGR_CLK_ID_HFRC2:
            *pui32RequestedClk = g_ui32ClkMgrClkCfgFreq_HFRC2;
            pCfg = &g_sClkMgrClkConfig_HFRC2;
            break;

        case AM_HAL_CLKMGR_CLK_ID_SYSPLL:
            *pui32RequestedClk = g_ui32ClkMgrClkCfgFreq_SYSPLL;
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

        case AM_HAL_CLKMGR_CLK_ID_HFRC2:
            ui32Status = am_hal_clkmgr_request_HFRC2(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_SYSPLL:
            ui32Status = am_hal_clkmgr_request_SYSPLL(eUserId);
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

        case AM_HAL_CLKMGR_CLK_ID_HFRC2:
            ui32Status = am_hal_clkmgr_release_HFRC2(eUserId);
            break;

        case AM_HAL_CLKMGR_CLK_ID_SYSPLL:
            ui32Status = am_hal_clkmgr_release_SYSPLL(eUserId);
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
// Handle Call to Legacy CLKGEN AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE
//
//*****************************************************************************
uint32_t am_hal_clkmgr_private_clkgen_hf2adj_apply()
{
    return am_hal_clkmgr_config_HFRC2(AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, NULL);
}

//*****************************************************************************
//
// Handle Call to Legacy CLKGEN AM_HAL_CLKGEN_CONTROL_HF2ADJ_DISABLE
//
//*****************************************************************************
uint32_t am_hal_clkmgr_private_clkgen_hf2adj_disable()
{
    return am_hal_clkmgr_config_HFRC2(AM_HAL_CLKMGR_HFRC2_FREQ_FREE_RUN_APPROX_250MHZ, NULL);
}
//! @endcond CLKMGR_PRIVATE_FUNC

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
