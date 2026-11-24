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
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_clkmgr_private.h"

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
//
// Change the CLK sources for the simobuck RX_CLK and TON_CLK from HFRC to Local
// CLKs. However, keep in mind that during normal CM55 Active operation we want
// to have those CLKs sourced from HFRC.
//
#define TON_CLK_HANDOFF_BEFORE_SLEEP()                                        \
do                                                                            \
{                                                                             \
    MCUCTRL->SIMOBUCK1_b.TONCLKLOCALCLKENFORCE = 1;                           \
    am_hal_delay_us(10);                                                      \
    MCUCTRL->SIMOBUCK1_b.TONCLKFORCEUSELOCALCLK = 1;                          \
    am_hal_delay_us(10);                                                      \
    MCUCTRL->SIMOBUCK11 |= (1 << 4);                                          \
    am_hal_delay_us(1);                                                       \
    MCUCTRL->SIMOBUCK11 &= ~(1 << 3);                                         \
    am_hal_delay_us(1);                                                       \
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 0;                                     \
    MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 1;                                     \
    am_hal_delay_us(1);                                                       \
    MCUCTRL->SIMOBUCK1_b.TONCLKLOCALCLKENFORCE = 0;                           \
} while (0);

//
// Change back to using HFRC as the CLK source.
//
#define TON_CLK_HANDOFF_AFTER_SLEEP()                                         \
do                                                                            \
{                                                                             \
    MCUCTRL->SIMOBUCK1_b.TONCLKLOCALCLKENFORCE = 1;                           \
    MCUCTRL->SIMOBUCK11 |= (1 << 3);                                          \
    MCUCTRL->SIMOBUCK1_b.TONCLKFORCEUSELOCALCLK = 0;                          \
    MCUCTRL->SIMOBUCK11 &= ~(1 << 4);                                         \
    MCUCTRL->SIMOBUCK1_b.TONCLKLOCALCLKENFORCE = 0;                           \
} while (0);

//*****************************************************************************
//
//  Globals
//
//*****************************************************************************

extern void buck_ldo_update_override(bool bEnable);

bool g_bFrcBuckAct  = false;
static bool g_bAppFrcBuckAct = false;
bool g_bIpcPending = false;
uint32_t g_ui32HfxtalUserCount = 0;

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
    am_hal_pwrctrl_pwrmodctl_cpdlp_t sActCpdlpConfig;
    uint32_t ui32InternalTimerAIRQNum = TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A + NVIC_USER_IRQ_OFFSET;
    am_hal_spotmgr_cpu_state_e eCpuSt = AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP, eOrigCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;
    bool bOtherIsrPending = false;
    uint32_t ui32CpdlpConfig = 0;

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
    PWRCTRL->CPUPWRSTATUS = PWRCTRL_CPUPWRSTATUS_FULLRETCACHE_Msk | PWRCTRL_CPUPWRSTATUS_FUNCRETCACHE_Msk;

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

        //
        // Prepare clock manager for deepsleep
        //
        am_hal_clkmgr_private_deepsleep_enter();

        //
        // Report CPU state change
        //
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eCpuSt);

#if NO_TEMPSENSE_IN_DEEPSLEEP
        am_hal_spotmgr_tempco_suspend();
#endif
        //
        // Prepare the data for reporting CPU status after waking up.
        //
        if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1)
        {
            eOrigCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP1;
        }
        else if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2)
        {
            eOrigCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2;
        }
        else
        {
            eOrigCpuSt = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;
        }

        bReportedDeepSleep = true;

        if (g_bIsTrimver1)
        {
            //
            // Check if SIMOBUCK needs to stay in Active mode in DeepSleep
            //
            if ( bSimobuckAct )
            {
                if (!g_bAppFrcBuckAct && !g_bFrcBuckAct)
                {
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
#if !AM_HAL_STALL_CPU_HP2WAKE
        //
        // If in HP2 mode, we need to wait till SYSPLL is ready and CPU is fully back in HP2 mode, before attempting deep sleep or deeper sleep.
        //
        if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2)
        {
            while ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2 )
            {
                am_hal_delay_us(1);
            }
        }
#endif // !AM_HAL_STALL_CPU_HP2WAKE
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
    if ( sActCpdlpConfig.eElpConfig != AM_HAL_PWRCTRL_ELP_OFF )
    {
        am_hal_delay_us_status_check(1000,
                                    (uint32_t) &PWRCTRL->CPUPWRSTATUS,
                                    PWRCTRL_CPUPWRSTATUS_FULLRETCACHE_Msk |
                                    PWRCTRL_CPUPWRSTATUS_FUNCRETCACHE_Msk,
                                    0,
                                    false);
    }

    if ( g_bIsTrimver2OrNewer && bReportedDeepSleep )
    {
        if (g_bFrcBuckAct || g_bAppFrcBuckAct)
        {
            //
            // Set SIMOBUCKOVER to 1 before entering deepsleep
            //
            MCUCTRL->VRCTRL_b.SIMOBUCKOVER = true;
            TON_CLK_HANDOFF_BEFORE_SLEEP();
        }
        else
        {
            //
            // If xxxFrcBuckAct are false, the buck state is determine by the hardware
            // instead of the software. IGNORENETAOL is asserted here so that
            // the buck can switch to Buck_LP even when NETAOL is on.
            //
            PWRCTRL->LEGACYVRLPOVR_b.IGNORENETAOL = 1;
            //
            // If HFXTAL_48M is used in Buck_LP, maintain VDDRF by setting bit 6.
            // Only consider the HFXTAL user on the CM55 side. We do not take the CM4 side
            // into account because CM4 may be either not powered on, or even if powered,
            // it has no HFXTAL users when it is in sleep mode, as described in am_hal_sysctrl_cm4_sleep_notify().
            //
            if (g_ui32HfxtalUserCount != 0)
            {
                MCUCTRL->SIMOBUCK11 |= (1 << 6);
            }

            MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 0;
            MCUCTRL->VRCTRL_b.SIMOBUCKOVER = true;
        }
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

    if ( g_bIsTrimver2OrNewer && bReportedDeepSleep )
    {
        if (g_bFrcBuckAct || g_bAppFrcBuckAct)
        {
            //
            // Set SIMOBUCKOVER back to 0 immediately after exiting deepsleep
            //
            MCUCTRL->VRCTRL_b.SIMOBUCKOVER = false;
            TON_CLK_HANDOFF_AFTER_SLEEP();
        }
        else
        {
            MCUCTRL->VRCTRL_b.SIMOBUCKOVER = false;
            PWRCTRL->LEGACYVRLPOVR_b.IGNORENETAOL = 0;
            if (g_ui32HfxtalUserCount != 0)
            {
                MCUCTRL->SIMOBUCK11 &= ~(1 << 6);
            }
        }
        am_hal_sysctrl_sysbus_write_flush();
    }
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
    // Check if the core is being woken up by AM_HAL_INTERNAL_TIMER_NUM_A interrupt; if so, attempt to re-enter sleep.
    //
    if (g_bIsTrimver2OrNewer &&
        (_FLD2VAL(SCB_ICSR_VECTPENDING, SCB->ICSR) == ui32InternalTimerAIRQNum))
    {
        //
        // Call am_hal_spotmgr_internal_timer_interrupt_service manually, we stopped the timer,
        // cleared the timer interrupt status and cleared g_bCm4Sleep flag in this function.
        //
        am_hal_spotmgr_internal_timer_interrupt_service();
        if (!(_FLD2VAL(SCB_ICSR_VECTPENDING, SCB->ICSR)))
        {
            if (bReportedDeepSleep)
            {
                //
                // Report CPU state change
                //
                am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eOrigCpuSt);
                //
                // Recover clock manager after deepsleep
                //
                am_hal_clkmgr_private_deepsleep_exit();
                //
                // If in HP2 mode, we need to wait till CPU is fully back in HP2 mode, before attempting deepsleep
                //
                if (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2)
                {
                    while ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2 )
                    {
                        //
                        // Check if any other interrupts are pending.
                        //
                        if (_FLD2VAL(SCB_ICSR_VECTPENDING, SCB->ICSR))
                        {
                            bOtherIsrPending = true;
                            break;
                        }
                        else
                        {
                            am_hal_delay_us(1);
                        }
                    }
                }
            }
            //
            // Enter sleep mode again only if no other interrupts are pending.
            //
            if (!bOtherIsrPending)
            {
                if (bReportedDeepSleep)
                {
                    //
                    // Prepare clock manager for deepsleep
                    //
                    am_hal_clkmgr_private_deepsleep_enter();
                    //
                    // Report deep/deeper sleep state again
                    //
                    am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eCpuSt);
                    //
                    // Clear the following bits before entering deepsleep.
                    // This is required to reduce the deepsleep power consumption.
                    //
                    MCUCTRL->VREFGEN2_b.TVRGCHSENRESDIV = 0;
                    MCUCTRL->VREFGEN3_b.TVRGCLVHSENRESDIV = 0;
                    MCUCTRL->VREFGEN4_b.TVRGFHSENRESDIV = 0;
                    MCUCTRL->VREFGEN5_b.TVRGSHSENRESDIV = 0;

                    if (g_bFrcBuckAct || g_bAppFrcBuckAct)
                    {
                        //
                        // Set SIMOBUCKOVER to 1 before entering deepsleep
                        //
                        MCUCTRL->VRCTRL_b.SIMOBUCKOVER = true;
                        TON_CLK_HANDOFF_BEFORE_SLEEP();
                    }
                    else
                    {
                        PWRCTRL->LEGACYVRLPOVR_b.IGNORENETAOL = 1;
                        if (g_ui32HfxtalUserCount != 0)
                        {
                            MCUCTRL->SIMOBUCK11 |= (1 << 6);
                        }
                        MCUCTRL->VRCTRL_b.SIMOBUCKACTIVE = 0;
                        MCUCTRL->VRCTRL_b.SIMOBUCKOVER = true;
                    }
                }

                //
                // Before executing WFI, flush APB writes.
                //
                am_hal_sysctrl_sysbus_write_flush();

                //
                // Execute the sleep instruction.
                //
                __WFI();

                //
                // Upon wake, execute the Instruction Sync Barrier instruction.
                //
                __ISB();

                if (bReportedDeepSleep)
                {
                    if (g_bFrcBuckAct || g_bAppFrcBuckAct)
                    {
                        //
                        // Set SIMOBUCKOVER back to 0 immediately after exiting deepsleep
                        //
                        MCUCTRL->VRCTRL_b.SIMOBUCKOVER = false;
                        TON_CLK_HANDOFF_AFTER_SLEEP();
                    }
                    else
                    {
                        MCUCTRL->VRCTRL_b.SIMOBUCKOVER = false;
                        PWRCTRL->LEGACYVRLPOVR_b.IGNORENETAOL = 0;
                        if (g_ui32HfxtalUserCount != 0)
                        {
                            MCUCTRL->SIMOBUCK11 &= ~(1 << 6);
                        }
                    }
                    am_hal_sysctrl_sysbus_write_flush();

                    //
                    // Set the bits back to 1 immediately after exiting deepsleep
                    //
                    MCUCTRL->VREFGEN2_b.TVRGCHSENRESDIV = 1;
                    MCUCTRL->VREFGEN3_b.TVRGCLVHSENRESDIV = 1;
                    MCUCTRL->VREFGEN4_b.TVRGFHSENRESDIV = 1;
                    MCUCTRL->VREFGEN5_b.TVRGSHSENRESDIV = 1;
                }
            }
        }
    }
    //
    // Report CPU state change
    //
    if (bReportedDeepSleep)
    {
        am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CPU_STATE, false, (void *) &eOrigCpuSt);

        //
        // Recover clock manager after deepsleep
        //
        am_hal_clkmgr_private_deepsleep_exit();
    }
#if AM_HAL_STALL_CPU_HP2WAKE
    //
    // If in HP2 mode, we need to wait till SYSPLL is ready and CPU is fully back in HP2 mode
    //
    if ((eSleepType >= AM_HAL_SYSCTRL_SLEEP_DEEP) && (PWRCTRL->MCUPERFREQ_b.MCUPERFREQ == AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2))
    {
        while ( PWRCTRL->MCUPERFREQ_b.MCUPERFSTATUS != AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2 )
        {
            am_hal_delay_us(1);
        }
    }
#endif // AM_HAL_STALL_CPU_HP2WAKE
    if (g_bIsTrimver1)
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
    SCB->CPACR &= ~(0xF << 20);
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
//! @brief Inform CM55 SPOT Manager that CM4 is going to sleep
//!        for ui32SleepDurationInMs
//!
//! @param ui32SleepDurationInMs Sleep duration in milliseconds
//! @param ui32BuckActInAdvInMs Switch buck to active by N ms before CM4 waking up.
//!
//! @return Status code
//!
//! This function is typically called in the CM55 IPC message handler.
//! It is used to notify CM55 via IPC about CM4 system power state changes:
//! 1. CM4 is entering deep sleep.
//! 2. HFXTAL 48M status:
//!    - If not used as the clock source by CM55 or its peripherals:
//!        it is either being turned off or already turned off.
//!    - Otherwise:
//!        it remains enabled.
//! 3. Baseband/RF subsystem is being powered down or already powered off.
//! 4. CM4 is required to enter deep sleep earlier than CM55 and exit sleep
//!    later than CM55.
//
//*****************************************************************************
uint32_t
am_hal_sysctrl_cm4_sleep_notify(uint32_t ui32SleepDurationInMs, uint32_t ui32BuckActInAdvInMs)
{
    bool bNotifyCm4Sleep = true;
    uint32_t ui32TimerDelayInMs = 0;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN
    //
    // If ui32SleepDurationInMs == 0xFFFFFFFF, it is a sleep notification from no-radio image.
    //
    if (ui32SleepDurationInMs == AM_HAL_SYSCTRL_CM4_NO_RADIO_INDICATOR)
    {
        //
        // set ui32TimerDelayInMs to 0xFFFFFFFF to indicate no-radio image.
        //
        ui32TimerDelayInMs = AM_HAL_SYSCTRL_CM4_NO_RADIO_INDICATOR;
        //
        // Notify SPOTMGR that CM4 is going to sleep.
        //
        ui32Status = am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CM4_SLEEP, bNotifyCm4Sleep, (void *) &ui32TimerDelayInMs);
    }
    else // Radio image
    {
        if (ui32SleepDurationInMs > ui32BuckActInAdvInMs)
        {
            ui32TimerDelayInMs = ui32SleepDurationInMs - ui32BuckActInAdvInMs;
            //
            // Report CM4 sleep duration (ui32SleepDurationInMs - ui32BuckActInAdvInMs) to SPOTMGR.
            //
            ui32Status = am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CM4_SLEEP, bNotifyCm4Sleep, (void *) &ui32TimerDelayInMs);
        }
        else
        {
            ui32Status = AM_HAL_STATUS_INVALID_ARG;
        }
    }
    AM_CRITICAL_END

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Report whether all IPC from CM55 to CM4 got replied.
//! Must report pending before CM55 sends an IPC msg,
//! and report idle after CM55 received all replies from CM4.
//!
//! @param bPending True for pending, False for back to idle
//!
//! @return Status code
//
//*****************************************************************************
uint32_t
am_hal_sysctrl_ipc_pending_notify(bool bPending)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN
    g_bIpcPending = bPending;
    if (bPending)
    {
        bool bReqBuckLP = false;
        //
        // Cancel the request for buck to go to LP mode if the request is ongoing
        //
        ui32Status = am_hal_spotmgr_power_state_update(AM_HAL_SPOTMGR_STIM_CM4_SLEEP, bReqBuckLP, NULL);
    }
    AM_CRITICAL_END

    return ui32Status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
