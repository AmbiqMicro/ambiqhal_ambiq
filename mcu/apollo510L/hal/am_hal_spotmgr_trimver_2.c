//*****************************************************************************
//
//! @file am_hal_spotmgr_trimver_2.c
//!
//! @brief SPOT Manager Trim Version 2 Implementation.
//!
//! @addtogroup spotmgr_trim2_ap510L SPOT Manager Trim Version 2
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module implements version 2 of the SPOT (System Power Optimization)
//! trim settings. It provides enhanced voltage, frequency, and power optimization
//! configurations for Apollo5 devices using the version 2 trim specifications.
//!
//! @section hal_spotmgr_trim2_features Key Features
//!
//! 1. @b Enhanced @b Trim: Version 2 advanced trim configurations.
//! 2. @b Improved @b Calibration: Refined power parameter calibration.
//! 3. @b Optimized @b Curves: Enhanced voltage-frequency relationships.
//! 4. @b Extended @b Points: Additional performance-power points.
//! 5. @b Advanced @b Compensation: Improved thermal adjustment methods.
//!
//! @section hal_spotmgr_trim2_functionality Functionality
//!
//! - Load and apply version 2 trim settings
//! - Manage enhanced calibrated operating points
//! - Implement advanced temperature compensation
//! - Provide version 2 specific optimizations
//! - Support extended power profile transitions
//!
//! @section hal_spotmgr_trim2_usage Usage
//!
//! 1. Initialize trim version 2 settings
//! 2. Apply enhanced calibrated parameters
//! 3. Monitor and adjust extended operating points
//! 4. Handle advanced temperature variations
//!
//! @section hal_spotmgr_trim2_configuration Configuration
//!
//! - @b Trim @b Values: Version 2 enhanced calibration data
//! - @b Operating @b Points: Extended performance levels
//! - @b Temperature @b Ranges: Advanced compensation settings
//! - @b Version @b Control: Enhanced trim version management
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
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"

#if !AM_HAL_SPOTMGR_TRIMVER_2_DISABLE


//
//! Enum for power states descriptions, only bit[4:7] are used to determine power state.
//
typedef enum
{
    AM_HAL_SPOTMGR_POWER_STATE_DESC_0  = 0x00,
    AM_HAL_SPOTMGR_POWER_STATE_DESC_1  = 0x10,
    AM_HAL_SPOTMGR_POWER_STATE_DESC_2  = 0x20,
} am_hal_spotmgr_power_state_desc_e;

//
//! Enum for power states transition sequences.
//
typedef enum
{
    AM_HAL_SPOTMGR_TRANS_SEQ_0  , // From power state 0 to power state 1
    AM_HAL_SPOTMGR_TRANS_SEQ_1  , // From power state 1 to power state 0
    AM_HAL_SPOTMGR_TRANS_SEQ_2  , // From power state 0 to power state 2
    AM_HAL_SPOTMGR_TRANS_SEQ_3  , // From power state 2 to power state 0
    AM_HAL_SPOTMGR_TRANS_SEQ_4  , // From power state 1 to power state 2
    AM_HAL_SPOTMGR_TRANS_SEQ_5  , // From power state 2 to power state 1
    AM_HAL_SPOTMGR_TRANS_SEQ_INVALID, // Invalid state transitions
} am_hal_spotmgr_transition_sequence_e;

//
//! Bitfield definitions for power states descriptions
//
typedef union
{
    am_hal_spotmgr_power_state_desc_e ePwrStateDesc;

    struct
    {
        //! CPU performance mode
        uint32_t CPUMODE        : 4;
        //! All peripheral (inluding GPU) power state
        uint32_t ALLPERIPHMODE  : 4;
        //! Temperature range
        uint32_t TEMPRANGE      : 4;
        //! GPU power state
        uint32_t GPUMODE        : 4;
        //! Peripheral power state
        uint32_t PERIPHMODE     : 4;
        //! GPU or SDIO power state
        uint32_t GPUSDIOMODE    : 4;
        //! Reserved
        uint32_t                : 8;
    } PWRSTATEDESC_b;
} am_hal_spotmgr_power_state_desc_t;

//
//! Function pointer of power state transition sequence.
//
typedef uint32_t (*TransitionSequencePtr)(uint32_t, uint32_t);

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//! Bit[7:4] are used to determine power state.
#define PWR_STATE_DESC_MASK (0x000000F0)

//! Delay in uSec for waiting for VDDC and/or VDDF ramp up.
#define STEP_UP_VDDC_VDDF_DELAY_IN_US (15)

//! Default VDDCRXCOMPTRIMMINUS for power state 1/2
#define DEFAULT_VDDCRXCOMPTRIMMINUS_PS1_2 (0)

//! Default VDDFRXCOMPTRIMMINUS for power state 1
#define DEFAULT_VDDFRXCOMPTRIMMINUS_PS1 (0)

//*****************************************************************************
//
//! Globals
//
//*****************************************************************************
//! The default power state for trimver2 is 1.
static uint32_t g_ui32CurPowerStateStatic = 1;
//! Static variable for storing SYSPLL status
static bool g_bSysPllEnableStatic;
//! Static variable for temperature range
static am_hal_spotmgr_tempco_range_e g_eCurTempRangeStatic = AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH;

//*****************************************************************************
//
//! Timer interrupt service for spotmgr
//
//*****************************************************************************
void
am_hal_spotmgr_trimver_2_internal_timer_interrupt_service(void)
{
    AM_CRITICAL_BEGIN

    //
    // Clear the flag on timeout.
    //
    g_bCm4Sleep = false;

    //
    // Clear interrupt status for this timer.
    // Disable timer.
    //
    am_hal_spotmgr_timer_stop();

    AM_CRITICAL_END
}

//*****************************************************************************
//
//! @brief Determine the buck state in deepsleep by setting g_bFrcBuckAct
//!
//! @param ui32PwrState - Target power state.
//!
//! @return None.
//
//*****************************************************************************
static inline void
spotmgr_buck_deepsleep_state_determine(uint32_t ui32PwrState)
{
    //
    // Force simobuck into active if:
    // 1. Any peripheral on (Power State != 0)
    // 2. Temperature > 50C
    // 3. SYSPLL is Enabled
    // 4. In PS0, only NETAOL is ON and CM4 is not going to sleep
    //
    if ((ui32PwrState != 0) ||
        (g_eCurTempRangeStatic == AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH) ||
        (g_bSysPllEnableStatic))
    {
        g_bFrcBuckAct = true;
    }
    else if ((PWRCTRL->DEVPWREN_b.PWRENNETAOL == PWRCTRL_DEVPWREN_PWRENNETAOL_ON) &&
             (!g_bCm4Sleep))
    {
        g_bFrcBuckAct = true;
    }
    else
    {
        g_bFrcBuckAct = false;
    }
}

//*****************************************************************************
//
//! Sequences for the transition from power state 0 to
//! power state 1.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_0(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = DEFAULT_VDDCRXCOMPTRIMMINUS_PS1_2;
    MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = DEFAULT_VDDFRXCOMPTRIMMINUS_PS1;
    am_hal_delay_us(STEP_UP_VDDC_VDDF_DELAY_IN_US);
    //
    // Enable peripheral after applying the trims above
    //

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Sequences for the transition from power state 1 to
//! power state 0.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_1(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    //
    // Disable peripheral, then apply the trims below
    //
    MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDCRXCOMPTRIMMINUS;
    MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDFRXCOMPTRIMMINUS;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Sequences for the transition from power state 0 to
//! power state 2.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_2(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = DEFAULT_VDDCRXCOMPTRIMMINUS_PS1_2;
    am_hal_delay_us(STEP_UP_VDDC_VDDF_DELAY_IN_US);
    //
    // Enable peripheral after applying the trims above
    //

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Sequences for the transition from power state 2 to
//! power state 0.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_3(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    //
    // Disable peripheral, then apply the trims below
    //
    MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDCRXCOMPTRIMMINUS;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Sequences for the transition from power state 1 to
//! power state 2.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_4(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    //
    // Disable peripheral, then apply the trims below
    //
    MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDFRXCOMPTRIMMINUS;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Sequences for the transition from power state 2 to
//! power state 1.
//
//*****************************************************************************
static inline uint32_t
transition_sequence_5(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = DEFAULT_VDDFRXCOMPTRIMMINUS_PS1;
    am_hal_delay_us(STEP_UP_VDDC_VDDF_DELAY_IN_US);
    //
    // Enable peripheral after applying the trims above
    //

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Function pointer array for transition sequences.
//
//*****************************************************************************
TransitionSequencePtr powerStateTransitionSeq[] =
{
    //
    // Transition sequences
    //
    transition_sequence_0,  transition_sequence_1, transition_sequence_2, transition_sequence_3, transition_sequence_4, transition_sequence_5
};

//*****************************************************************************
//
//! @brief Determine sequences of power state transition
//!
//! @param ui32TarPwrState - Target power state.
//! @param ui32CurPwrState - Current/Last power state.
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
static uint32_t
spotmgr_state_transition_sequence_determine(uint32_t ui32TarPwrState, uint32_t ui32CurPwrState, am_hal_spotmgr_transition_sequence_e * peSeqNum)
{
    am_hal_spotmgr_transition_sequence_e eTransitionSeqTable[3][3] =
    {
        {AM_HAL_SPOTMGR_TRANS_SEQ_INVALID, AM_HAL_SPOTMGR_TRANS_SEQ_0,       AM_HAL_SPOTMGR_TRANS_SEQ_2      },
        {AM_HAL_SPOTMGR_TRANS_SEQ_1,       AM_HAL_SPOTMGR_TRANS_SEQ_INVALID, AM_HAL_SPOTMGR_TRANS_SEQ_4      },
        {AM_HAL_SPOTMGR_TRANS_SEQ_3,       AM_HAL_SPOTMGR_TRANS_SEQ_5,       AM_HAL_SPOTMGR_TRANS_SEQ_INVALID}
    };

    //
    // Get the sequence index from the table
    //
    *peSeqNum = eTransitionSeqTable[ui32CurPwrState][ui32TarPwrState];
    if (*peSeqNum == AM_HAL_SPOTMGR_TRANS_SEQ_INVALID)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Power trims init
//!        Before enabling SIMOBUCK, initialise power trims
//!        according to the power state and Ton state.
//!
//! @param ui32PwrState    - Power state.
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
static uint32_t
spotmgr_power_trims_init(uint32_t ui32PwrState)
{
    if (ui32PwrState == 1)
    {
        MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = DEFAULT_VDDCRXCOMPTRIMMINUS_PS1_2;
        MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = DEFAULT_VDDFRXCOMPTRIMMINUS_PS1;
    }
    else if (ui32PwrState == 2)
    {
        MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = DEFAULT_VDDCRXCOMPTRIMMINUS_PS1_2;
        MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDFRXCOMPTRIMMINUS;
    }
    else
    {
        MCUCTRL->SIMOBUCK7_b.VDDCRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDCRXCOMPTRIMMINUS;
        MCUCTRL->SIMOBUCK6_b.VDDFRXCOMPTRIMMINUS = g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM_b.VDDFRXCOMPTRIMMINUS;
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Power trims update
//!        Update power trims accodrding to the power state.
//!
//! @param ui32PwrState    - Target power state.
//! @param ui32CurPwrState - Current/Last power state.
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
static uint32_t
spotmgr_power_trims_update(uint32_t ui32PwrState, uint32_t ui32CurPwrState)
{
    uint32_t ui32Status;
    am_hal_spotmgr_transition_sequence_e eSeqNum = AM_HAL_SPOTMGR_TRANS_SEQ_INVALID;

    ui32Status = spotmgr_state_transition_sequence_determine(ui32PwrState, ui32CurPwrState, &eSeqNum);
    //
    // Execute the power state transition sequences.
    //
    if (AM_HAL_STATUS_SUCCESS == ui32Status)
    {
        //
        // Call the corresponding transition sequence
        //
        ui32Status = powerStateTransitionSeq[eSeqNum](ui32PwrState, ui32CurPwrState);
    }

    return ui32Status;
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
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
static int32_t
spotmgr_power_state_determine(am_hal_spotmgr_power_status_t * psPwrStatus, uint32_t * pui32PwrState)
{
    am_hal_spotmgr_power_state_desc_t sPwrStatDesc =
    {
        .ePwrStateDesc = (am_hal_spotmgr_power_state_desc_e)0
    };

    if ((psPwrStatus->eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE) ||
        (psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK_PS1)    ||
        (psPwrStatus->ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK_PS1))
    {
        sPwrStatDesc.PWRSTATEDESC_b.ALLPERIPHMODE = 1;
    }
    else if (psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK_PS2)
    {
        sPwrStatDesc.PWRSTATEDESC_b.ALLPERIPHMODE = 2;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.ALLPERIPHMODE = 0;
    }

    //
    // Determine the power state.
    //
    switch (sPwrStatDesc.ePwrStateDesc & PWR_STATE_DESC_MASK)
    {
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_0:
            *pui32PwrState = 0;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_1:
            *pui32PwrState = 1;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_2:
            *pui32PwrState = 2;
            break;

        default:
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Inline function to convert temperature in float to temperature range
//
//*****************************************************************************
static inline am_hal_spotmgr_tempco_range_e
spotmgr_temp_to_range(float fTemp)
{
    if ((fTemp < BUCK_LP_TEMP_THRESHOLD) && (fTemp >= LOW_LIMIT))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW;
    }
    else if ((fTemp >= BUCK_LP_TEMP_THRESHOLD) && (fTemp < HIGH_LIMIT))
    {
        return AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH;
    }
    return AM_HAL_SPOTMGR_TEMPCO_OUT_OF_RANGE;
}

//*****************************************************************************
//
//! @brief Power states update
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
//! @return SUCCESS or Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_trimver_2_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32PowerState = 0;

    am_hal_spotmgr_power_status_t sPwrStatus;
    bool bSkipAllUpdates = false, bReqPwrStateChg = true;
    static bool bIsRadioSleepStatic = true;
#ifdef AM_HAL_SPOTMGR_PROFILING
    bool bLogSleepChangeEvt = false;
#endif

    //
    // Check if SIMOBUCK is enabled
    //
    if ((PWRCTRL->VRSTATUS_b.SIMOBUCKST != PWRCTRL_VRSTATUS_SIMOBUCKST_ACT) &&
        (eStimulus != AM_HAL_SPOTMGR_STIM_INIT_STATE))
    {
        //
        // If the stimulus is temperature, note it down even if SIMOBUCK is not active
        //
        if (eStimulus == AM_HAL_SPOTMGR_STIM_TEMP)
        {
            if (pArgs != NULL)
            {
                am_hal_spotmgr_tempco_param_t *psTemp = (am_hal_spotmgr_tempco_param_t *)pArgs;
                g_eCurTempRangeStatic = spotmgr_temp_to_range(psTemp->fTemperature);

                switch(g_eCurTempRangeStatic)
                {
                    case AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW:
                        psTemp->fRangeLower = LOW_LIMIT;
                        psTemp->fRangeHigher = BUCK_LP_TEMP_THRESHOLD;
                        break;
                    case AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH:
                        psTemp->fRangeLower = BUCK_LP_TEMP_THRESHOLD - TEMP_HYSTERESIS;
                        psTemp->fRangeHigher = HIGH_LIMIT;
                        break;
                    case AM_HAL_SPOTMGR_TEMPCO_OUT_OF_RANGE:
                        psTemp->fRangeLower = 0.0f;
                        psTemp->fRangeHigher = 0.0f;
                        return AM_HAL_STATUS_INVALID_ARG;
                }
            }
            else
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
        }

        if (eStimulus == AM_HAL_SPOTMGR_STIM_SYSPLL)
        {
            g_bSysPllEnableStatic = bOn;
        }
        //
        // Return success when SIMOBUCK is not active
        //
        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // Check if INFO1 regs for power trims are valid
    //
    if (g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid != INFO1GLOBALVALID)
    {
        return AM_HAL_STATUS_FAIL;
    }
    static am_hal_spotmgr_cpu_state_e eLastCpuStateStatic = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;          // The default CPU state after MCU powering up is LP.
    AM_CRITICAL_BEGIN
#if defined(AM_HAL_SPOTMGR_PROFILING) && defined(AM_HAL_SPOTMGR_PROFILING_VERBOSE)
    // Log a verbose entry alongside the normal entries, with markings on tonState for verbose entry indication
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = AM_HAL_SPOTMGR_PROFILING_PWRST_INVALID;
    changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_PWRST_INVALID;
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
             (eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP)  ||
             (eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEPER)) &&
            ((sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP)  ||
             (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP1) ||
             (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2)))
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
        sPwrStatus.eTempRange = g_eCurTempRangeStatic;
        sPwrStatus.eGpuState = (sPwrStatus.ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTGFX_Msk) ?
                               AM_HAL_SPOTMGR_GPUSTATE_ACTIVE :
                               AM_HAL_SPOTMGR_GPUSTATE_OFF;
        sPwrStatus.eCpuState = (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER) ? AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP :
                               ((PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1) ? AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP1 : AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2);
        //
        // Get the requested/target status
        //
        switch (eStimulus)
        {
            case AM_HAL_SPOTMGR_STIM_INIT_STATE:
                break;

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
                bReqPwrStateChg = false;
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
                bReqPwrStateChg = false;
                break;

            case AM_HAL_SPOTMGR_STIM_TEMP:
                if (pArgs != NULL)
                {
                    am_hal_spotmgr_tempco_param_t *psTempCo = (am_hal_spotmgr_tempco_param_t *)pArgs;
                    sPwrStatus.eTempRange = spotmgr_temp_to_range(psTempCo->fTemperature);
                    g_eCurTempRangeStatic = sPwrStatus.eTempRange;

                    switch(g_eCurTempRangeStatic)
                    {
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW:
                            psTempCo->fRangeLower = LOW_LIMIT;
                            psTempCo->fRangeHigher = BUCK_LP_TEMP_THRESHOLD;
                            break;
                        case AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH:
                            psTempCo->fRangeLower = BUCK_LP_TEMP_THRESHOLD - TEMP_HYSTERESIS;
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
                bReqPwrStateChg = false;
                break;

            case AM_HAL_SPOTMGR_STIM_CPU_STATE:
                if (pArgs != NULL)
                {
                    sPwrStatus.eCpuState = *((am_hal_spotmgr_cpu_state_e *)pArgs);
                    if (eLastCpuStateStatic != sPwrStatus.eCpuState)
                    {
                        if ((eLastCpuStateStatic <= AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2) &&
                            ((sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP) ||
                             (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEPER)))
                        {
                            spotmgr_buck_deepsleep_state_determine(g_ui32CurPowerStateStatic);
                            am_hal_clkmgr_clock_status_get(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, &g_ui32HfxtalUserCount);
                        }
                        #ifdef AM_HAL_SPOTMGR_PROFILING
                        if ((eLastCpuStateStatic >= AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP) ||
                            (sPwrStatus.eCpuState >= AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
                        {
                            bLogSleepChangeEvt = true;
                        }
                        #endif

                        eLastCpuStateStatic = sPwrStatus.eCpuState;
                    }
                }
                else
                {
                    ui32Status = AM_HAL_STATUS_INVALID_ARG;
                }
                bReqPwrStateChg = false;
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

            case AM_HAL_SPOTMGR_STIM_SYSPLL:
                if (bOn != g_bSysPllEnableStatic)
                {
                    g_bSysPllEnableStatic = bOn;
                }
                bReqPwrStateChg = false;
                break;
            case AM_HAL_SPOTMGR_STIM_CM4_SLEEP:
                bReqPwrStateChg = false;
                if (bOn)  // request Buck_LP
                {
                    if (pArgs == NULL)
                    {
                        ui32Status = AM_HAL_STATUS_INVALID_ARG;
                        break;
                    }
                    if (*((uint32_t *)pArgs) == AM_HAL_SYSCTRL_CM4_NO_RADIO_INDICATOR)
                    {
                        bIsRadioSleepStatic = false;
                        //
                        // If switch from radio to no-radio,
                        // stop the timer and clear g_bCm4Sleep flag firstly.
                        //
                        if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
                        {
                            am_hal_spotmgr_trimver_2_internal_timer_interrupt_service();
                        }
                        //
                        // For no-radio, set g_bCm4Sleep to 1 if IPC is not pending
                        //
                        if (!g_bIpcPending)
                        {
                            g_bCm4Sleep = true;
                        }
                        else
                        {
                            ui32Status = AM_HAL_STATUS_IN_USE;
                        }
                    }
                    else
                    {
                        uint32_t ui32TimerDelayInMs = *((uint32_t *)pArgs);
                        bIsRadioSleepStatic = true;

                        if (!g_bIpcPending)
                        {
                            //
                            // Set the flag g_bCm4Sleep if IPC is not pending
                            //
                            g_bCm4Sleep = true;
                            //
                            // start timer if it is off, restart timer if it is running
                            //
                            if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
                            {
                                am_hal_spotmgr_timer_restart(ui32TimerDelayInMs);
                            }
                            else
                            {
                                am_hal_spotmgr_timer_start(ui32TimerDelayInMs);
                            }
                        }
                        else
                        {
                            ui32Status = AM_HAL_STATUS_IN_USE;
                        }
                    }
                }
                else // cancel the request
                {
                    //
                    // Clear g_bCm4Sleep, stop and disable the timer if it is running.
                    // cm4 may switch between radio and no-radio, we still need to check the timer status even for no-radio.
                    //
                    if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
                    {
                        am_hal_spotmgr_trimver_2_internal_timer_interrupt_service();
                    }
                    else if ( !bIsRadioSleepStatic )
                    {
                        //
                        // Clear the flag if it is no-radio image
                        //
                        g_bCm4Sleep = false;
                    }
                }
                break;
            default:
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
                bReqPwrStateChg = false;
                break;
        }

        if ((ui32Status == AM_HAL_STATUS_SUCCESS) && bReqPwrStateChg)
        {
            //
            // Determine the requested/target power state
            //
            ui32Status = spotmgr_power_state_determine(&sPwrStatus, &ui32PowerState);
            if (ui32Status == AM_HAL_STATUS_SUCCESS)
            {
                //
                // If the power state needs to be changed, call spotmgr_power_trims_update().
                //
                if ((ui32PowerState != g_ui32CurPowerStateStatic))
                {
#ifdef AM_HAL_SPOTMGR_PROFILING
                    am_hal_spotmgr_changelog_t changeLog;
                    changeLog.u.s.pwrState = ui32PowerState;
                    changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_TONST_INVALID;
                    changeLog.u.s.eStimulus = eStimulus;
                    changeLog.u.s.bOn = bOn;
                    changeLog.args = pArgs ? *((uint32_t *)pArgs) : 0xDEADBEEF;
                    am_hal_spotmgr_log_change(&changeLog);
#endif
                    if (eStimulus == AM_HAL_SPOTMGR_STIM_INIT_STATE)
                    {
                        //
                        // Initialise trims
                        //
                        ui32Status = spotmgr_power_trims_init(ui32PowerState);
                    }
                    else
                    {
                        //
                        // Update trims
                        //
                        ui32Status = spotmgr_power_trims_update(ui32PowerState, g_ui32CurPowerStateStatic);
                    }
                }
                //
                // Maintain a static variable with current trim settings.
                //
                g_ui32CurPowerStateStatic = ui32PowerState;

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
//!        SIMOBUCK
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_trimver_2_simobuck_init_aft_enable(void)
{
    //
    // The initial value of SIMOBUCK11 for Ton clock handoff.
    //
    MCUCTRL->SIMOBUCK11 = 0x8;

    return AM_HAL_STATUS_SUCCESS;
}

#if NO_TEMPSENSE_IN_DEEPSLEEP
//*****************************************************************************
//
//! @brief Prepare SPOT manager for suspended tempco during deep sleep
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_trimver_2_tempco_suspend(void)
{
    g_bFrcBuckAct = true;
    return AM_HAL_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
//! @brief SPOT manager init
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_trimver_2_init(void)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t info1buf[1];

//
// Helper macros for INFO1 populate.
// RD_INFO1: Macro to call am_hal_info1_read() and check return status.
//

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

    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_POWERSTATE0_O  / 4), 1, &info1buf[0]);

    g_sSpotMgrINFO1regs.sPowerState0Trims.PWRSTATE0TRIM    = info1buf[0];

    //
    // All done, mark the data as valid
    //
    g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid = INFO1GLOBALVALID;

    //
    // Initialise the timer for buck state control when CM4 is in sleep.
    //
    am_hal_spotmgr_timer_init();

#ifdef AM_HAL_SPOTMGR_PROFILING
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = 1;
    changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_TONST_INVALID;
    changeLog.u.s.eStimulus = AM_HAL_SPOTMGR_PROFILING_ESTIM_INVALID;
    changeLog.u.s.bOn = AM_HAL_SPOTMGR_PROFILING_BON_INVALID;
    changeLog.args = 0xDEADBEEF;
    am_hal_spotmgr_log_change(&changeLog);
#endif

    //
    // Initialise SYSPLL status
    //
    g_bSysPllEnableStatic = (MCUCTRL->PLLCTL0_b.SYSPLLPDB == MCUCTRL_PLLCTL0_SYSPLLPDB_ENABLE);

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Reset power state to POR default - Power State 1
//!
//! @return SUCCESS or Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_trimver_2_default_reset(void)
{
    bool bEnabled = false;
    //
    // Make sure all peripherals are off except OTP, and turn on OTP if it is off.
    //
    if (((PWRCTRL->DEVPWRSTATUS & ~PWRCTRL_DEVPWRSTATUS_PWRSTOTP_Msk) == 0) &&
        (PWRCTRL->AUDSSPWRSTATUS == 0 ))
    {
        if (am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled) == AM_HAL_STATUS_SUCCESS)
        {
            if (!bEnabled)
            {
                if (am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP) != AM_HAL_STATUS_SUCCESS)
                {
                    //
                    // Failed to turn on OTP
                    //
                    return AM_HAL_STATUS_FAIL;
                }
            }
        }
        else
        {
            //
            // Failed to get OTP power status
            //
            return AM_HAL_STATUS_FAIL;
        }
    }
    else
    {
        //
        // Return error, users should turn off all peripherals
        //
        return AM_HAL_STATUS_FAIL;
    }
#ifdef AM_HAL_SPOTMGR_PROFILING
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = 1;
    changeLog.u.s.tonState = AM_HAL_SPOTMGR_PROFILING_TONST_INVALID;
    changeLog.u.s.eStimulus = AM_HAL_SPOTMGR_PROFILING_ESTIM_INVALID;
    changeLog.u.s.bOn = AM_HAL_SPOTMGR_PROFILING_BON_INVALID;
    changeLog.args = 0xDEADBEEF;
    am_hal_spotmgr_log_change(&changeLog);
#endif
    return AM_HAL_STATUS_SUCCESS;
}

#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
