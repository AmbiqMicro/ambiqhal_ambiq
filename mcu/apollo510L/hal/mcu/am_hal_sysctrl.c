//*****************************************************************************
//
//! @file am_hal_sysctrl.c
//!
//! @brief Functions for interfacing with the M55 system control registers
//!
//! @addtogroup sysctrl4_ap510L SYSCTRL - System Control
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides system control functions for Apollo5
//! devices, including power management, sleep mode control, and system
//! bus synchronization. It enables efficient power management and
//! reliable system operation across different power states.
//!
//! @section hal_sysctrl_features Key Features
//!
//! 1. @b Sleep @b Management: Control normal and deep sleep modes.
//! 2. @b Power @b Control: Manage buck converter and power states.
//! 3. @b FPU @b Control: Enable/disable floating-point unit operations.
//! 4. @b System @b Reset: Provide system reset functionality.
//! 5. @b Clock @b Management: Control clock multiplexer and reset operations.
//!
//! @section hal_sysctrl_functionality Functionality
//!
//! - Control system sleep and deep sleep modes
//! - Manage power states and buck converter operation
//! - Enable/disable FPU and configure stacking
//! - Handle system reset operations
//! - Control clock multiplexer and reset functionality
//!
//! @section hal_sysctrl_usage Usage
//!
//! 1. Configure sleep modes using am_hal_sysctrl_sleep()
//! 2. Control FPU operations as needed
//! 3. Manage power states and buck converter
//! 4. Handle system reset when required
//! 5. Configure clock multiplexer operations
//!
//! @section hal_sysctrl_configuration Configuration
//!
//! - @b Sleep @b Modes: Configure normal and deep sleep parameters
//! - @b Power @b States: Set up buck converter and power management
//! - @b FPU @b Settings: Configure floating-point unit operations
//! - @b Clock @b Control: Set up clock multiplexer and reset operations
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
#include "am_mcu_apollo.h"
#include "../am_hal_clkmgr_private.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************

//*****************************************************************************
//
//  Globals
//
//*****************************************************************************

extern void buck_ldo_update_override(bool bEnable);

bool g_bFrcBuckAct  = false;
static bool g_bAppFrcBuckAct = false;

//*****************************************************************************
//
//! @brief Control the buck state in deepsleep
//!
//! @param bFrcBuckAct - True for forcing buck active in deepsleep
//!                    - False for not forcing buck active in deepsleep
//!
//! If you want to manually force the buck stay active in deepsleep mode,
//! am_hal_sysctrl_force_buck_active_in_deepsleep must
//! be called for setting g_bAppFrcBuckAct to true before
//! calling am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP).
//! If anyone of spotmgr and
//! am_hal_sysctrl_force_buck_active_in_deepsleep forced buck stay active, buck
//! will stay active in deepsleep.
//
//*****************************************************************************
void
am_hal_sysctrl_force_buck_active_in_deepsleep(bool bFrcBuckAct)
{
    g_bAppFrcBuckAct = bFrcBuckAct;
}

//
// Instrumentation hook for collecting the Register Settings values such as PWRCTRL, MCUCTRL and CLKGEN
//
__attribute__((weak)) void am_hal_PRE_SLEEP_PROCESSING(void){}

//*****************************************************************************
//
// Place the core into sleep, deepsleep or deepersleep.
//
// This function puts the MCU to sleep, deepsleep or deepersleep depending on eSleepType.
//
// Valid values for eSleepType are:
//     AM_HAL_SYSCTRL_SLEEP_NORMAL
//     AM_HAL_SYSCTRL_SLEEP_DEEP
//     AM_HAL_SYSCTRL_SLEEP_DEEPER
//
//*****************************************************************************

void
am_hal_sysctrl_sleep(am_hal_sysctrl_sleep_type_e eSleepType)
{
    bool bSimobuckAct = false;
    bool bReportedDeepSleep = false;
    bool bBuckIntoLPinDS = false, bBuckIntoACTinDS = false;
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sActCpdlpConfig;
    am_hal_spotmgr_cpu_state_e eCpuSt;
    uint32_t  ui32CpdlpConfig = 0;

    //
    // Inform clkmgr to release pre-started clocks that is not yet claimed.
    //
    if (eSleepType >= AM_HAL_SYSCTRL_SLEEP_DEEP)
    {
        am_hal_clkmgr_control(AM_HAL_CLKMGR_RELEASE_PRESTART_CLK, NULL);
    }

    //
    // Disable interrupts and save the previous interrupt state.
    //
    AM_CRITICAL_BEGIN

    //
    // Clear CPUPWRSTATUS FULLRETCACHE and FUNCRETCACHE status bits
    //
    PWRCTRL->CPUPWRSTATUS =PWRCTRL_CPUPWRSTATUS_FULLRETCACHE_Msk | PWRCTRL_CPUPWRSTATUS_FUNCRETCACHE_Msk;

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
    // Get current mode.
    //
    bSimobuckAct = ( PWRCTRL->VRSTATUS_b.SIMOBUCKST == PWRCTRL_VRSTATUS_SIMOBUCKST_ACT );

    //
    // If the user selected DEEPSLEEP and OTP & ROM are off, attempt to enter
    // DEEP SLEEP.
    // CPU cannot go to deepsleep if either OTP or ROM is still powered on
    //
    if ((eSleepType >= AM_HAL_SYSCTRL_SLEEP_DEEP)
        && (!PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP))
    {
        if (eSleepType >= AM_HAL_SYSCTRL_SLEEP_DEEPER)
        {
            PWRCTRL->CPUPWRCTRL_b.DEEPERSLEEPEN = 1;
            eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEPER;
        }
        else
        {
            PWRCTRL->CPUPWRCTRL_b.DEEPERSLEEPEN = 0;
            eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP;
        }
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
        am_hal_pwrctrl_pwrmodctl_cpdlp_config(sDSCpdlpConfig);

#if NO_TEMPSENSE_IN_DEEPSLEEP
        am_hal_spotmgr_tempco_suspend();
#endif
        //
        // Prepare clock manager for deepsleep
        //
        am_hal_clkmgr_private_deepsleep_enter();

        //
        // Report CPU state change
        //
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eCpuSt);
        //
        // Prepare the data for reporting CPU status after waking up.
        //
        if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1)
        {
            eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP1;
        }
        else if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2)
        {
            eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2;
        }
        else
        {
            eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;
        }

        bReportedDeepSleep = true;

        //
        // Check if SIMOBUCK needs to stay in Active mode in DeepSleep
        //
        if ( bSimobuckAct )
        {
            //
            // Check if SIMOBUCK would go into LP mode in DeepSleep
            //
            if (g_bIsTrimver2OrNewer)
            {
                if (g_bAppFrcBuckAct || g_bFrcBuckAct)
                {
                    //
                    // This implies upon deepsleep, buck can keep in active mode
                    //
                    bBuckIntoACTinDS = true;

                    //
                    // Force buck to go in active mode
                    //
                    buck_ldo_update_override(true);

                }
            }
            else
            {
                if (!g_bAppFrcBuckAct && !g_bFrcBuckAct)
                {
                    //
                    // This implies upon deepsleep, buck can transition into LP mode
                    //
                    bBuckIntoLPinDS = true;

                    //
                    // Remove overrides to allow buck to go in LP mode
                    //
                    buck_ldo_update_override(false);

#if AM_HAL_PWRCTRL_SIMOLP_AUTOSWITCH
                    am_hal_spotmgr_simobuck_lp_autosw_enable();
#endif
                }
            }
        }
        //
        // Prepare the core for deepsleep (write 1 to the DEEPSLEEP bit).
        //
        SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
        //
        // Clear the following bits before entering deepsleep.
        // This is required to reduce the deepsleep power consumption.
        //
        MCUCTRL->VREFGEN2_b.TVRGCHSENRESDIV = 0;
        MCUCTRL->VREFGEN3_b.TVRGCLVHSENRESDIV = 0;
        MCUCTRL->VREFGEN4_b.TVRGFHSENRESDIV = 0;
        MCUCTRL->VREFGEN5_b.TVRGSHSENRESDIV = 0;
    }
    else
    {
        //
        // Set the CPDLPSTATE configuration in normal sleep mode
        //
        am_hal_pwrctrl_pwrmodctl_cpdlp_t sNSCpdlpConfig;
        if (PWRCTRL->CPUPWRCTRL_b.SLEEPMODE) // ARM sleep
        {
            sNSCpdlpConfig.eRlpConfig = sActCpdlpConfig.eRlpConfig;
            sNSCpdlpConfig.eElpConfig = AM_HAL_PWRCTRL_ELP_RET;
            sNSCpdlpConfig.eClpConfig = AM_HAL_PWRCTRL_CLP_ON_CLK_OFF;
        }
        else // Ambiq sleep
        {
            sNSCpdlpConfig.eRlpConfig = sActCpdlpConfig.eRlpConfig;
            sNSCpdlpConfig.eElpConfig = AM_HAL_PWRCTRL_ELP_RET; // or can leave at 0x0 as we will turn the clocks off at the source
            sNSCpdlpConfig.eClpConfig = AM_HAL_PWRCTRL_CLP_ON_CLK_OFF; // or can leave at 0x0 as we will turn the clocks off at the source
        }
        //
        // If ELP is OFF or RET in active state, keep it OFF or RET.
        //
        if ((sActCpdlpConfig.eElpConfig == AM_HAL_PWRCTRL_ELP_OFF) || (sActCpdlpConfig.eElpConfig == AM_HAL_PWRCTRL_ELP_RET))
        {
            sNSCpdlpConfig.eElpConfig = sActCpdlpConfig.eElpConfig;
        }
        am_hal_pwrctrl_pwrmodctl_cpdlp_config(sNSCpdlpConfig);

        //
        // Prepare the core for normal sleep (write 0 to the DEEPSLEEP bit).
        //
        SCB->SCR &= ~_VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
    }

    //
    // Wait (up to 1 ms) for either FULLRETCACHE (bit 19) or
    // FUNCRETCACHE (bit 21) in CPUPWRSTATUS to be set.
    //
    if(sActCpdlpConfig.eElpConfig != AM_HAL_PWRCTRL_ELP_OFF)
    {
        am_hal_delay_us_status_check(1000,
                                    (uint32_t) &PWRCTRL->CPUPWRSTATUS,
                                    PWRCTRL_CPUPWRSTATUS_FULLRETCACHE_Msk |
                                    PWRCTRL_CPUPWRSTATUS_FUNCRETCACHE_Msk,
                                    0,
                                    false);
    }

    //
    // Before executing WFI, flush APB writes.
    //
    am_hal_sysctrl_sysbus_write_flush();

    //
    // Weak am_hal_PRE_SLEEP_PROCESSING function to be overwritten in the application. Used by pwrctrl_state_transition_trim_regdump_test_cases
    // to collect the Register Settings for PWRCTRL, MCUCTRL and CLKGEN before going into deepsleep
    //
    am_hal_PRE_SLEEP_PROCESSING();

    //
    // Execute the sleep instruction.
    //
    __WFI();

    //
    // Upon wake, execute the Instruction Sync Barrier instruction.
    //
    __ISB();
    //
    // Set the bits back to 1 immediately after exiting deepsleep
    //
    if (bReportedDeepSleep)
    {
        MCUCTRL->VREFGEN2_b.TVRGCHSENRESDIV = 1;
        MCUCTRL->VREFGEN3_b.TVRGCLVHSENRESDIV = 1;
        MCUCTRL->VREFGEN4_b.TVRGFHSENRESDIV = 1;
        MCUCTRL->VREFGEN5_b.TVRGSHSENRESDIV = 1;
    }
    //
    // Report CPU state change
    //
    if (bReportedDeepSleep)
    {
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eCpuSt);

        //
        // Recover clock manager after deepsleep
        //
        am_hal_clkmgr_private_deepsleep_exit();
    }
    if ( bBuckIntoACTinDS )
    {
        //
        // Disable overrides
        //
        MCUCTRL->VRCTRL_b.SIMOBUCKOVER = false;
        //
        // Set SCM mode to 2
        //
        SCM->SCMCNTRCTRL1 = SCMCNTRCTRL1_SETTING_DEFAULT;
    }
    else if ( bBuckIntoLPinDS )
    {
        //
        // Re-enable overrides
        //
        MCUCTRL->VRCTRL_b.SIMOBUCKOVER   = true;
#if AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
        MCUCTRL->VRCTRL_b.CORELDOOVER    = true;
        MCUCTRL->VRCTRL_b.MEMLDOOVER     = true;
#endif // AM_HAL_PWRCTL_SET_CORELDO_MEMLDO_IN_PARALLEL
    }
#if AM_HAL_PWRCTRL_SIMOLP_AUTOSWITCH
    am_hal_spotmgr_simobuck_lp_autosw_disable();
#endif

    //
    // Restore the CPDLPSTATE
    // am_hal_pwrctrl_pwrmodctl_cpdlp_config(sActCpdlpConfig);
    //
    PWRMODCTL->CPDLPSTATE = ui32CpdlpConfig;

    //
    // Restore the interrupt state.
    //
    AM_CRITICAL_END

    //
    // Inform clkmgr to resume pre-started clocks that was released.
    //
    if (eSleepType >= AM_HAL_SYSCTRL_SLEEP_DEEP)
    {
        am_hal_clkmgr_control(AM_HAL_CLKMGR_RESUME_PRESTART_CLK, NULL);
    }
}

//*****************************************************************************
//
// Enable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_enable(void)
{
    //
    //! Enable the EPU. See section 6.3 of the CM55 TRM.

    //
    SCB->CPACR |= (0xF << 20);
    __DSB();
    __ISB();
}

//*****************************************************************************
//
// Disable the floating point module.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_disable(void)
{
    //
    // Disable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    SCB->CPACR |= ~(0xF << 20);
    __DSB();
    __ISB();
}

//*****************************************************************************
//
// Enable stacking of FPU registers on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_enable(bool bLazy)
{
    uint32_t ui32fpccr;

    //
    // Set the requested FPU stacking mode in ISRs.
    //
    AM_CRITICAL_BEGIN
#define SYSCTRL_FPCCR_LAZY  (FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk)
    ui32fpccr  = FPU->FPCCR;
    ui32fpccr &= ~SYSCTRL_FPCCR_LAZY;
    ui32fpccr |= (bLazy ? SYSCTRL_FPCCR_LAZY : FPU_FPCCR_ASPEN_Msk);
    FPU->FPCCR = ui32fpccr;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Disable FPU register stacking on exception entry.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_disable(void)
{
    //
    // Completely disable FPU context save on entry to ISRs.
    //
    AM_CRITICAL_BEGIN
    FPU->FPCCR &= ~SYSCTRL_FPCCR_LAZY;
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//
//*****************************************************************************
void
am_hal_sysctrl_aircr_reset(void)
{
    //
    // Set the system reset bit in the AIRCR register
    //
    __NVIC_SystemReset();
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
