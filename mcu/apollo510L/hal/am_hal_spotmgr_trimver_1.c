//*****************************************************************************
//
//! @file am_hal_spotmgr_trimver_1.c
//!
//! @brief SPOT Manager Trim Version 1 Implementation.
//!
//! @addtogroup spotmgr_trim1_ap510L SPOT Manager Trim Version 1
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module implements version 1 of the SPOT (System Power Optimization)
//! trim settings. It provides specific voltage, frequency, and power optimization
//! configurations for Apollo5 devices using the version 1 trim specifications.
//!
//! @section hal_spotmgr_trim1_features Key Features
//!
//! 1. @b Trim @b Settings: Version 1 specific trim configurations.
//! 2. @b Calibration @b Data: Factory-calibrated power parameters.
//! 3. @b Power @b Curves: Optimized voltage-frequency relationships.
//! 4. @b Operating @b Points: Pre-defined performance-power points.
//! 5. @b Temperature @b Compensation: Thermal adjustment parameters.
//!
//! @section hal_spotmgr_trim1_functionality Functionality
//!
//! - Load and apply version 1 trim settings
//! - Manage calibrated operating points
//! - Handle temperature compensation
//! - Provide version-specific optimizations
//! - Support power profile transitions
//!
//! @section hal_spotmgr_trim1_usage Usage
//!
//! 1. Initialize trim version 1 settings
//! 2. Apply calibrated parameters
//! 3. Monitor and adjust operating points
//! 4. Handle temperature variations
//!
//! @section hal_spotmgr_trim1_configuration Configuration
//!
//! - @b Trim @b Values: Version 1 specific calibration data
//! - @b Operating @b Points: Pre-defined performance levels
//! - @b Temperature @b Ranges: Thermal compensation settings
//! - @b Version @b Control: Trim version management
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
// This is part of revision release_sdk5_2_a_1-29944d3085 of the AmbiqSuite Development Package.
//
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"

#if !AM_HAL_SPOTMGR_TRIMVER_1_DISABLE

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
    // peripheral or SYSPLL enabled in deepsleep or temperature range is HIGH, the
    // simobuck must be forced to stay in active mode in deepsleep.
    //
    if ((psPwrStatus->eTempRange == AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH)   ||
        (psPwrStatus->ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK)      ||
        (psPwrStatus->ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK)  ||
        (MCUCTRL->PLLCTL0_b.SYSPLLPDB == MCUCTRL_PLLCTL0_SYSPLLPDB_ENABLE))
    {
        g_bFrcBuckAct = true;
        return;
    }
    else
    {
        //
        // Check stimer status and clock source, if it is using either the HFRC
        // or a GPIO external clock input as the clock source in
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
            // either the HFRC or a GPIO external clock input as the clock
            // source in deepsleep, the simobuck must be forced to stay in active
            // mode in deepsleep.
            //
            for (uint32_t ui32TimerNumber = 0; ui32TimerNumber < AM_REG_NUM_TIMERS; ui32TimerNumber++)
            {
                if ((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0EN == TIMER_CTRL0_TMR0EN_EN) &&
                    (TIMER->GLOBEN & (TIMER_GLOBEN_ENB0_EN << ui32TimerNumber))        &&
                    (((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK >= AM_HAL_TIMER_CLOCK_HFRC_DIV4)            &&
                      (TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK <= AM_HAL_TIMER_CLOCK_HFRC_DIV4K))          ||
                     ((TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK >= AM_HAL_TIMER_CLOCK_GPIO0)                &&
                      (TIMERn(ui32TimerNumber)->CTRL0_b.TMR0CLK <= AM_HAL_TIMER_CLOCK_GPIO99))))
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
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_trimver_1_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_spotmgr_power_status_t sPwrStatus;
    bool bSkipAllUpdates = false;
#ifdef AM_HAL_SPOTMGR_PROFILING
    bool bLogSleepChangeEvt = false;
#endif
    //
    // static variable for temperature range
    //
    static am_hal_spotmgr_tempco_range_e eCurTempRangeStatic = AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH; // For safety, set the default range to HIGH
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
                eCurTempRangeStatic = spotmgr_temp_to_range(psTemp->fTemperature);

                switch(eCurTempRangeStatic)
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
        //
        // Return success when SIMOBUCK is not active
        //
        return AM_HAL_STATUS_SUCCESS;
    }
    //
    // Static variables for storing the last/current status, initialise them to the default values after MCU powering up.
    //
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
        sPwrStatus.eTempRange = eCurTempRangeStatic;
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

                    switch(eCurTempRangeStatic)
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
                        // If CPU is going to deep or deeper sleep, need to determine the buck state
                        // in deep sleep.
                        //
                        if ( (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP) ||
                             (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEPER) )
                        {
                            spotmgr_buck_deepsleep_state_determine(&sPwrStatus);
                        }

                        eLastCpuStateStatic = sPwrStatus.eCpuState;
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


#if NO_TEMPSENSE_IN_DEEPSLEEP
//*****************************************************************************
//
//! @brief Prepare SPOT manager for suspended tempco during deep sleep
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_trimver_1_tempco_suspend(void)
{
    g_bFrcBuckAct = true;
    return AM_HAL_STATUS_SUCCESS;
}
#endif

//*****************************************************************************
//
//! @brief SPOT manager init
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_trimver_1_init(void)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
#ifdef AM_HAL_SPOTMGR_PROFILING
    am_hal_spotmgr_changelog_t changeLog;
    changeLog.u.s.pwrState = 0;
    changeLog.u.s.tonState = 0;
    changeLog.u.s.eStimulus = AM_HAL_SPOTMGR_PROFILING_ESTIM_INVALID;
    changeLog.u.s.bOn = AM_HAL_SPOTMGR_PROFILING_BON_INVALID;
    changeLog.args = 0xDEADBEEF;
    am_hal_spotmgr_log_change(&changeLog);
#endif

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Reset power state to POR default
//!        Do not report a fixed temperature here, user must report temperature
//!        after transioning to a new image to make sure SIMOBUCK state is
//!        correct in deep sleep or deeper sleep.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_trimver_1_default_reset(void)
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

    return AM_HAL_STATUS_SUCCESS;
}

#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
