//*****************************************************************************
//
//! @file am_hal_spotmgr.c
//!
//! @brief Functions for managing System Power Optimization (SPOT).
//!
//! @addtogroup spotmgr_ap510L SPOT Manager - System Power Optimization
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module manages system power optimization features in Apollo5
//! devices. It provides functionality to optimize power consumption while
//! maintaining required system performance through dynamic voltage and
//! frequency scaling.
//!
//! @section hal_spotmgr_features Key Features
//!
//! 1. @b Power @b Optimization: Dynamic power management strategies.
//! 2. @b Performance @b Scaling: Voltage and frequency adjustment.
//! 3. @b System @b Monitoring: Real-time performance and power tracking.
//! 4. @b Profile @b Management: Power profile configuration and switching.
//! 5. @b Temperature @b Control: Thermal management integration.
//!
//! @section hal_spotmgr_functionality Functionality
//!
//! - Initialize and configure SPOT management system
//! - Adjust system voltage and frequency
//! - Monitor system performance metrics
//! - Manage power optimization profiles
//! - Handle thermal constraints
//!
//! @section hal_spotmgr_usage Usage
//!
//! 1. Initialize SPOT manager using setup functions
//! 2. Configure power profiles and thresholds
//! 3. Monitor system metrics
//! 4. Adjust power settings as needed
//!
//! @section hal_spotmgr_configuration Configuration
//!
//! - @b Voltage @b Levels: Configure operating voltage ranges
//! - @b Frequency @b Settings: Set system frequency thresholds
//! - @b Profile @b Options: Define power optimization profiles
//! - @b Thermal @b Limits: Set temperature-based constraints
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//
//! Function pointer type for SPOT manager functions
//
typedef uint32_t (*am_hal_spotmgr_pcm_init_t)(void);
typedef uint32_t (*am_hal_spotmgr_pcm_power_state_update_t)(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs);
typedef uint32_t (*am_hal_spotmgr_default_reset_t)(void);
typedef uint32_t (*am_hal_spotmgr_simobuck_init_bfr_enable_t)(void);
typedef uint32_t (*am_hal_spotmgr_simobuck_init_aft_enable_t)(void);
typedef uint32_t (*am_hal_spotmgr_tempco_suspend_t)(void);

//
//! SPOT manager state data structure
//
typedef struct
{
    // SPOT Manager Initialization
    // Applicable PCM version: ALL
    am_hal_spotmgr_pcm_init_t pfnSpotmgrInit;

    // SPOT Manager Power State Update
    // Applicable PCM version: ALL
    am_hal_spotmgr_pcm_power_state_update_t pfnSpotmgrPSUpdate;

    // Restore power state to POR default
    // Applicable PCM version: ALL
    am_hal_spotmgr_default_reset_t pfnSpotMgrDefaultRst;

    // SIMOBUCK initialization handlings just before/after enabling SIMOBUCK
    // Applicable PCM version: ALL
    am_hal_spotmgr_simobuck_init_bfr_enable_t pfnSpotMgrSimobuckInitBfrEnable;
    am_hal_spotmgr_simobuck_init_aft_enable_t pfnSpotMgrSimobuckInitAftEnable;

#if NO_TEMPSENSE_IN_DEEPSLEEP
    // Pre-DeepSleep handling to allow temperature sensing activity to be
    // suspended after deep sleep is entered
    // Applicable PCM version: ALL
    am_hal_spotmgr_tempco_suspend_t pfnSpotMgrTempcoSuspend;
#endif

#if AM_HAL_PWRCTRL_SIMOLP_AUTOSWITCH
    // SIMOBUCK LP autoswitch feature control handling
    // Applicable PCM version: ALL
    am_hal_spotmgr_simobuck_lp_autosw_init_t pfnSimobuckLpAutoSwInit;
    am_hal_spotmgr_simobuck_lp_autosw_enable_t pfnSimobuckLpAutoSwEnable;
    am_hal_spotmgr_simobuck_lp_autosw_disable_t pfnSimobuckLpAutoSwDisable;
#endif

} am_hal_spotmgr_state_t;

//*****************************************************************************
//
//! Global Variables
//
//*****************************************************************************
//! Trim Version booleans for optimization of trim version eval
bool g_bIsTrimver1;
bool g_bIsTrimver1OrNewer;
bool g_bIsTrimver2OrNewer;

//! This table will be populated with SPOT manager related INFO1 values and
//! will be used for easy lookup after OTP is powered down.
am_hal_spotmgr_info1_regs_t g_sSpotMgrINFO1regs;

//*****************************************************************************
//
//! Static Variables
//
//*****************************************************************************
static am_hal_spotmgr_state_t g_sSpotMgr;

//*****************************************************************************
//
//! @brief Power states update
//!        This API should be called before turning things on and after turning
//!        things off, or before switching to high performance mode and after
//!        switching to low power mode, or when temperature range is changed.
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
am_hal_spotmgr_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    if (g_sSpotMgr.pfnSpotmgrPSUpdate)
    {
        return g_sSpotMgr.pfnSpotmgrPSUpdate(eStimulus, bOn, pArgs);
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Restore power state to POR default
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_default_reset(void)
{
    if (g_sSpotMgr.pfnSpotMgrDefaultRst)
    {
        return g_sSpotMgr.pfnSpotMgrDefaultRst();
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just before enabling
//!        SIMOBUCK
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_simobuck_init_bfr_enable(void)
{
    if (g_sSpotMgr.pfnSpotMgrSimobuckInitBfrEnable)
    {
        return g_sSpotMgr.pfnSpotMgrSimobuckInitBfrEnable();
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just after enabling
//!        SIMOBUCK
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_simobuck_init_aft_enable(void)
{
    if (g_sSpotMgr.pfnSpotMgrSimobuckInitAftEnable)
    {
        return g_sSpotMgr.pfnSpotMgrSimobuckInitAftEnable();
    }
    return AM_HAL_STATUS_SUCCESS;
}

#if NO_TEMPSENSE_IN_DEEPSLEEP
//*****************************************************************************
//
//! @brief Prepare SPOT manager for suspended tempco during deep sleep
//!
//! @return SUCCESS or other Failures.
//!
//! This API is to be called before entering deepsleep if there is no handling
//! to execute temperature sensing periodically after the deepsleep is entered.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_tempco_suspend(void)
{
    if (g_sSpotMgr.pfnSpotMgrTempcoSuspend)
    {
        return g_sSpotMgr.pfnSpotMgrTempcoSuspend();
    }
    return AM_HAL_STATUS_SUCCESS;
}
#endif

#if AM_HAL_PWRCTRL_SIMOLP_AUTOSWITCH
//*****************************************************************************
//
//! @brief Initialize registers for SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_simobuck_lp_autosw_init(void)
{
    if (g_sSpotMgr.pfnSimobuckLpAutoSwInit)
    {
        return g_sSpotMgr.pfnSimobuckLpAutoSwInit();
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Enable SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_simobuck_lp_autosw_enable(void)
{
    if (g_sSpotMgr.pfnSimobuckLpAutoSwEnable)
    {
        return g_sSpotMgr.pfnSimobuckLpAutoSwEnable();
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Disable SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_simobuck_lp_autosw_disable(void)
{
    if (g_sSpotMgr.pfnSimobuckLpAutoSwDisable)
    {
        return g_sSpotMgr.pfnSimobuckLpAutoSwDisable();
    }
    return AM_HAL_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
//! @brief SPOT manager init
//!        This API should be called from am_hal_pwrctrl_low_power_init, to
//!        initialise SPOT manager.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_init(void)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Clear spotmgr storage before initialization
    //
    memset(&g_sSpotMgr, 0, sizeof(am_hal_spotmgr_state_t));

    //
    // Populate cached booleans for trim version eval optimization
    //
    // A0 trimver1
    //
    g_bIsTrimver1 = APOLLO510L_A0_TRIMVER1;
    //
    // A0 trimver1 and newer trim, and silicon newer than A0
    //
    g_bIsTrimver1OrNewer = APOLLO510L_A0_GE_TRIMVER1 || APOLLO510L_GT_A0;
    //
    // A0 trimver2 and newer trim, and silicon newer than A0
    //
    g_bIsTrimver2OrNewer = APOLLO510L_A0_GE_TRIMVER2 || APOLLO510L_GT_A0;
    //
    // Populate SPOTmanager function pointer according to PCM version
    //
    if (g_bIsTrimver1)
    {
        #if !AM_HAL_SPOTMGR_TRIMVER_1_DISABLE
        g_sSpotMgr.pfnSpotmgrInit = am_hal_spotmgr_trimver_1_init;
        g_sSpotMgr.pfnSpotmgrPSUpdate = am_hal_spotmgr_trimver_1_power_state_update;
        g_sSpotMgr.pfnSpotMgrDefaultRst = am_hal_spotmgr_trimver_1_default_reset;
        #if NO_TEMPSENSE_IN_DEEPSLEEP
        g_sSpotMgr.pfnSpotMgrTempcoSuspend = am_hal_spotmgr_trimver_1_tempco_suspend;
        #endif
        #endif
    }
    else if (g_bIsTrimver2OrNewer)
    {
        #if !AM_HAL_SPOTMGR_TRIMVER_2_DISABLE
        g_sSpotMgr.pfnSpotmgrInit = am_hal_spotmgr_trimver_2_init;
        g_sSpotMgr.pfnSpotmgrPSUpdate = am_hal_spotmgr_trimver_2_power_state_update;
        g_sSpotMgr.pfnSpotMgrDefaultRst = am_hal_spotmgr_trimver_2_default_reset;
        g_sSpotMgr.pfnSpotMgrSimobuckInitBfrEnable = am_hal_spotmgr_trimver_2_simobuck_init_bfr_enable;
        g_sSpotMgr.pfnSpotMgrSimobuckInitAftEnable = am_hal_spotmgr_trimver_2_simobuck_init_aft_enable;
        #endif
    }
    //
    // Execute SPOTmanager init
    //
    if (g_sSpotMgr.pfnSpotmgrInit)
    {
        ui32Status = g_sSpotMgr.pfnSpotmgrInit();
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Dummy Weak function for spotmgr log change event handler
//!
//! @param pChangeLog - Pointer of am_hal_spotmgr_changelog_t struct.
//!
//! @return None.
//
//*****************************************************************************
#ifdef AM_HAL_SPOTMGR_PROFILING
#if defined (__IAR_SYSTEMS_ICC__)
__weak void
#else
void __attribute__((weak))
#endif
am_hal_spotmgr_log_change(am_hal_spotmgr_changelog_t *pChangeLog)
{
    // Dummy weak function - Do nothing
}
#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
