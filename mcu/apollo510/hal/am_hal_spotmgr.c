// ****************************************************************************
//
//! @file am_hal_spotmgr.c
//!
//! @brief SPOT manager functions that manage power states.
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
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
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
// This is part of revision release_sdk5p2-040c7863bb of the AmbiqSuite Development Package.
//
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//! CoreLDO boost code for Ton adjustment
#define CORELDO_BOOST_FOR_TON_ADJ   14
//! MemLDO boost code for Ton adjustment
#define MEMLDO_BOOST_FOR_TON_ADJ    6
//! CoreLDO boost code for TVRG adjustment
#define CORELDO_BOOST_FOR_TVRG_ADJ  7
//! LDOs boost duration in us
#define LDO_BOOST_DURATION_IN_US    2000
//! LDOs boost duration in us for trim optmized parts
#define LDO_BOOST_DURATION_OPTIMIZED_IN_US    200
//! LDOs boost duration in us for double boost case on trim optmized parts
#define LDO_BOOST_DURATION_OPTIMIZED_DOUBLE_BOOST_IN_US    50
//! Boost code for VDDCACTLOWTONTRIM in power state 14
#define VDDCACTLOWTONTRIM_BOOST_STATE14 6
//! Boost code for VDDCACTLOWTONTRIM in power state 15
#define VDDCACTLOWTONTRIM_BOOST_STATE15 12
//! The B1/B2 PCM2.1 without patch parts
#define PCM2P1_WO_PATCH ((APOLLO5_B1_PCM2P1 || APOLLO5_B2_PCM2P1) && ((g_sINFO1regs.ui32PATCH_TRACKER0 & 0x1) == 0))
//! Check if it is the profile for collapsing STM and STM+Periph
#define IS_PROFILE_COLLAPSE_STM_AND_STMP (g_sSpotMgrProfile.PROFILE_b.COLLAPSESTMANDSTMP == AM_HAL_SPOTMGR_COLLAPSE_STM_STMP_EN)

//*****************************************************************************
//
//! Globals
//
//*****************************************************************************

//! This table will be populated with SPOT manager related INFO1 values and
//! will be used for easy lookup after OTP is powered down.
am_hal_spotmgr_info1_regs_t g_sSpotMgrINFO1regs;

//! Flag to clear or set PWRSWVDDCAOROVERRIDE and PWRSWVDDCPUOVERRIDE when
//! switching between HP and deepsleep
bool g_bVddcaorVddcpuOverride = false;

//! Flag for setting VDDF LP minus offset trim to reduce VDDF when entering
//! deepsleep from LP mode.
bool g_bVddfLpMinusForLp = false;

//! CoreLDO actual boost code for TVRG adjustment
int32_t g_i32CORELDODiff = 0;

//! The flag indicates MCU LP to HP switching is ongoing
bool g_bSwitchingToHp = false;

//! VDDC voltage level for power states, a lager number indicates the voltage is higher
uint32_t g_ui32PwrStVddcVoltLvl[16] = {0, 0, 1, 2, 1, 1, 2, 3, 6, 4, 4, 4, 8, 7, 5, 5};

//! VDDF voltage level for power states, a lager number indicates the voltage is higher
uint32_t g_ui32PwrStVddfVoltLvl[16] = {1, 0, 0, 0, 3, 2, 2, 2, 5, 4, 4, 4, 5, 4, 4, 4};

//! New VDDC TVRGCVREFTRIM trim value
uint32_t g_ui32NewVddcTrim = 0;

//! New VDDF TVRGFVREFTRIM trim value
uint32_t g_ui32NewVddfTrim = 0;

//! SPOT manager profile
static am_hal_spotmgr_profile_t g_sSpotMgrProfile = {.PROFILE = 0};

//! Flag for current temperature is less than 50C
bool g_bTempLessThan50C = true;

//*****************************************************************************
//
//! Inline function for boosting LDOs for power state transitions
//
//*****************************************************************************
static inline void
spotmgr_ldo_boost(bool bBoostMemLdo)
{
    //
    // Boost memldo
    //
    if (bBoostMemLdo)
    {
        MCUCTRL->D2ASPARE_b.D2ASPARE |= (0x3UL << 15);
        MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.DFLTMEMLDOACTTRIM;
    }
    //
    // Overflow checks for boosting coreldo
    //
    DIFF_OVF_CAP(g_i32CORELDODiff, CORELDO_BOOST_FOR_TVRG_ADJ, MCUCTRL, LDOREG1, CORELDOACTIVETRIM);
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM += g_i32CORELDODiff;
}

//*****************************************************************************
//
//! Inline function for removing LDOs boost for power state transitions
//
//*****************************************************************************
static inline void
spotmgr_ldo_boost_remove(bool bReduceCoreLdo)
{
    //
    // Change memldo trim to support switching memldo reference to tvrgf
    //
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM = g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.MEMLDOACTTRIM;
    MCUCTRL->D2ASPARE = (MCUCTRL->D2ASPARE & ~(0x3UL << 15)) | (g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.MEMLDOD2ASPARE << 15);
    //
    // Reduce Core LDO
    //
    if (bReduceCoreLdo)
    {
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM -= g_i32CORELDODiff;
    }
}

//*****************************************************************************
//
//! Inline function for updating TVRGC and TVRGF trims based on current state
//
//*****************************************************************************
static inline void
spotmgr_buck_trims_update(void)
{
    //
    // Update trims based on current state
    //
    if (!PCM2P1_WO_PATCH)
    {
        MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
        MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
    }
}

//*****************************************************************************
//
//! Inline function for initializing the timer for LDOs boost.
//
//*****************************************************************************
static inline void
ldo_boost_timer_init(void)
{
    //
    // Disable the timer.
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN = 0;
    //
    // Apply the settings.
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0 = _VAL2FLD(TIMER_CTRL0_TMR0CLK,     AM_HAL_TIMER_CLOCK_HFRC_DIV16)  |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0FN,      AM_HAL_TIMER_FN_EDGE)           |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0POL1,    false)                          |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0POL0,    false)                          |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0TMODE,   AM_HAL_TIMER_TRIGGER_DIS)       |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0LMT,     0)                              |
                                                 _VAL2FLD(TIMER_CTRL0_TMR0EN,      0);
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->MODE0 = _VAL2FLD(TIMER_MODE0_TMR0TRIGSEL, AM_HAL_TIMER_TRIGGER_TMR0_OUT1);
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->TMR0CMP0 = LDO_BOOST_DURATION_IN_US * (AM_HAL_CLKGEN_FREQ_MAX_MHZ / 16);
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->TMR0CMP1 = 0xFFFFFFFF;
    //
    // Clear the timer Interrupt
    //
    TIMER->INTCLR = AM_HAL_TIMER_MASK(AM_HAL_INTERNAL_TIMER_NUM_A, AM_HAL_TIMER_COMPARE_BOTH);
    //
    // Enable the timer Interrupt.
    //
    TIMER->INTEN |= AM_HAL_TIMER_MASK(AM_HAL_INTERNAL_TIMER_NUM_A, AM_HAL_TIMER_COMPARE0);
}

//*****************************************************************************
//
//! Inline function for starting the timer for LDOs boost.
//
//*****************************************************************************
static inline void
ldo_boost_timer_start(uint32_t ui32BoostDelayInUs)
{
    //
    // Request clock
    //
    am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_TIMER0 + AM_HAL_INTERNAL_TIMER_NUM_A));
    //
    //  Set timer compare value
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->TMR0CMP0 = ui32BoostDelayInUs * (AM_HAL_CLKGEN_FREQ_MAX_MHZ / 16);
    //
    // Toggle the clear bit (required by the hardware), and then enable the timer.
    //
    TIMER->GLOBEN |= 1UL <<  AM_HAL_INTERNAL_TIMER_NUM_A;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0CLR = 1;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0CLR = 0;
    //
    // Set the timer interrupt
    //
    NVIC->ISER[(TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) / 32] = (1 << ((TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) % 32));
    //
    // Enable the timer
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN = 1;
}

//*****************************************************************************
//
//! Inline function for restarting the timer for LDOs boost.
//
//*****************************************************************************
static inline void
ldo_boost_timer_restart(uint32_t ui32BoostDelayInUs)
{
    //
    // Disable the timer, then toggle the clear bit
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN  = 0;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0CLR = 1;
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0CLR = 0;
    //
    //  Set timer compare value
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->TMR0CMP0 = ui32BoostDelayInUs * (AM_HAL_CLKGEN_FREQ_MAX_MHZ / 16);
    //
    // clear the timer interrupt status
    //
    TIMER->INTCLR = AM_HAL_TIMER_MASK(AM_HAL_INTERNAL_TIMER_NUM_A, AM_HAL_TIMER_COMPARE_BOTH);
    //
    // Clear pending NVIC interrupt for the timer-specific IRQ.
    //
    NVIC->ICPR[(((uint32_t)TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) & 0x1FUL));
    //
    // Enable the timer
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN  = 1;
}

//*****************************************************************************
//
//! Inline function for stopping the timer for LDOs boost.
//
//*****************************************************************************
static inline void
ldo_boost_timer_stop(void)
{
    //
    // Deinit the timer
    //
    TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN  = 0;
    TIMER->GLOBEN &= ~(1UL <<  AM_HAL_INTERNAL_TIMER_NUM_A);
    //
    // Release clock
    //
    am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_TIMER0 + AM_HAL_INTERNAL_TIMER_NUM_A));
    //
    // Disable the timer interrupt
    //
    NVIC->ICER[(TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) / 32] = (1 << ((TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) % 32));
    //
    // clear the timer interrupt status
    //
    TIMER->INTCLR = AM_HAL_TIMER_MASK(AM_HAL_INTERNAL_TIMER_NUM_A, AM_HAL_TIMER_COMPARE_BOTH);
    //
    // Clear pending NVIC interrupt for the timer-specific IRQ.
    //
    NVIC->ICPR[(((uint32_t)TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)TIMER0_IRQn + AM_HAL_INTERNAL_TIMER_NUM_A) & 0x1FUL));
}

//*****************************************************************************
//
//! Timer interrupt service for boosting LDOs and bucks
//
//*****************************************************************************
void
am_hal_spotmgr_boost_timer_interrupt_service(void)
{
    AM_CRITICAL_BEGIN
    //
    // Remove double boost if needed
    //
    spotmgr_buck_trims_update();
    //
    // If temperature < 50C, change the power to VDCC.
    // Else, we leave the CPU running off VDDF.
    //
    if (g_bSwitchingToHp)
    {
        if (g_bTempLessThan50C)
        {
            MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
            MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
        }
        g_bSwitchingToHp = false;
    }
    //
    // Clear interrupt status for this timer
    // Disable timer
    //
    ldo_boost_timer_stop();
    //
    // Remove LDOs boost
    //
    spotmgr_ldo_boost_remove(true);

    AM_CRITICAL_END
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
spotmgr_buck_state_determine(am_hal_spotmgr_power_status_t * psPwrStatus)
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
    // If entering deepsleep in CPU HP mode, g_bVddcaorVddcpuOverride must be set
    //
    if ((eLastCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP) && (eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
    {
        //
        // Set this bit to enable Vddcaor Vddcpu Override clear and set when switching between HP and deepsleep.
        //
        g_bVddcaorVddcpuOverride = true;
    }
    //
    // When switching from CPU HP mode to LP mode the following fields must be cleared after entering CPU LP mode
    //
    if ((eLastCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP) && (eCpuState == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP))
    {
        MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = MCUCTRL_PWRSW0_PWRSWVDDMCPUSTATSEL_VDDC;
        MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 0;
        MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 0;
        g_bSwitchingToHp = false;
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
spotmgr_power_ton_adjust(uint32_t ui32TonState, uint32_t ui32TarPwrState)
{
    int32_t i32CORELDODiff, i32MEMLDODiff;
    uint32_t ui32VDDCACTLOWTONTRIM, ui32VDDFACTLOWTONTRIM;
    //
    // Overflow checks for boosting coreldo and memldo
    //
    DIFF_OVF_CAP(i32CORELDODiff, CORELDO_BOOST_FOR_TON_ADJ, MCUCTRL, LDOREG1, CORELDOACTIVETRIM);
    DIFF_OVF_CAP(i32MEMLDODiff, MEMLDO_BOOST_FOR_TON_ADJ, MCUCTRL, LDOREG2, MEMLDOACTIVETRIM);
    //
    // Boost coreldo and memldo output voltage
    //
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM += i32CORELDODiff;
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM += i32MEMLDODiff;
    //
    // Delay 20us for LDO boost
    //
    am_hal_delay_us(20);
    //
    // Short VDDC to VDDCLV
    //
    MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVORVAL = 1;
    MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVOREN = 1;
    //
    // Delay 20us for waiting for power stabilization
    //
    am_hal_delay_us(20);
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
    //
    // Update ui32VDDCACTLOWTONTRIM for the specical cases (power state 8 and 12, 14, 15).
    //
    if (ui32TarPwrState == 8)
    {
        ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sStmTon.STMTON_b.PWRSTATE8VDDCTON;
    }
    else if (ui32TarPwrState == 12)
    {
        ui32VDDCACTLOWTONTRIM = g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON_b.PWRSTATE12VDDCTON;
    }
    else if (ui32TarPwrState == 14)
    {
        ui32VDDCACTLOWTONTRIM = (ui32VDDCACTLOWTONTRIM + VDDCACTLOWTONTRIM_BOOST_STATE14) > (MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Msk >> MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Pos) ?
                                (MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Msk >> MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Pos) :
                                (ui32VDDCACTLOWTONTRIM + VDDCACTLOWTONTRIM_BOOST_STATE14);
    }
    else if (ui32TarPwrState == 15)
    {
        ui32VDDCACTLOWTONTRIM = (ui32VDDCACTLOWTONTRIM + VDDCACTLOWTONTRIM_BOOST_STATE15) > (MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Msk >> MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Pos) ?
                                (MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Msk >> MCUCTRL_SIMOBUCK2_VDDCACTLOWTONTRIM_Pos) :
                                (ui32VDDCACTLOWTONTRIM + VDDCACTLOWTONTRIM_BOOST_STATE15);
    }

    MCUCTRL->SIMOBUCK2_b.VDDCACTLOWTONTRIM = ui32VDDCACTLOWTONTRIM;
    MCUCTRL->SIMOBUCK7_b.VDDFACTLOWTONTRIM = ui32VDDFACTLOWTONTRIM;
    //
    // VDDC_LV Ton adjustments for power state 1 and 5
    //
    if ((ui32TarPwrState == 1) || (ui32TarPwrState == 5))
    {
        MCUCTRL->SIMOBUCK4_b.VDDCLVACTLOWTONTRIM = VDDCLVACTLOWTONTRIM_POWER_STATE_1_5;
    }
    else
    {
        MCUCTRL->SIMOBUCK4_b.VDDCLVACTLOWTONTRIM = VDDCLVACTLOWTONTRIM_DEFAULT;
    }
    //
    // Remove VDDC and VDDCLV short
    //
    MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVORVAL = 0;
    MCUCTRL->PWRSW1_b.SHORTVDDCVDDCLVOREN = 0;
    //
    // Reduce coreldo and memldo output voltage
    //
    MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM -= i32CORELDODiff;
    MCUCTRL->LDOREG2_b.MEMLDOACTIVETRIM -= i32MEMLDODiff;
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
    bool bUp = false;
    static uint32_t ui32BasePwrStateStatic = 7;
    am_hal_spotmgr_trim_settings_t *pTrimSettings = &g_sSpotMgrINFO1regs.sPowerStateArray[ui32PwrState];
    am_hal_spotmgr_trim_settings_t *pCurTrimSettings = &g_sSpotMgrINFO1regs.sPowerStateArray[ui32CurPwrState];
    am_hal_spotmgr_trim_settings_t *pBaseTrimSettings = &g_sSpotMgrINFO1regs.sPowerStateArray[ui32BasePwrStateStatic];
    bool bEnableICache = false;
    uint32_t ui32BaseVddfTrim = 0, ui32BaseVddcTrim = 0;
    uint32_t ui32BoostDelayInUs = 0;
    int32_t  i32DblBstVddcDiff = 0, i32DblBstVddfDiff = 0;

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
        //
        // Set bUp to true when stepping up to the higher voltage level
        //
        if ((g_ui32PwrStVddcVoltLvl[ui32PwrState] > g_ui32PwrStVddcVoltLvl[ui32CurPwrState]) ||
            (g_ui32PwrStVddfVoltLvl[ui32PwrState] > g_ui32PwrStVddfVoltLvl[ui32CurPwrState]))
        {
            bUp = true;
        }
        //
        // Step up to the higher voltage level
        //
        if (bUp)
        {
            //
            // Adjust VDDC and VDDF Simobuck Tons if Ton state is changing
            //
            if ((ui32TonState != ui32CurTonState) ||
                (ui32PwrState == 1)               ||
                (ui32PwrState == 5)               ||
                (ui32PwrState == 8)               ||
                (ui32PwrState == 12)              ||
                (ui32PwrState == 14)              ||
                (ui32PwrState == 15)              ||
                (ui32CurPwrState == 1)            ||
                (ui32CurPwrState == 5)            ||
                (ui32CurPwrState == 8)            ||
                (ui32CurPwrState == 12)           ||
                (ui32CurPwrState == 14)           ||
                (ui32CurPwrState == 15))

            {
                spotmgr_power_ton_adjust(ui32TonState, ui32PwrState);
            }
            //
            // Assign new TVRGC trims
            //
            g_ui32NewVddcTrim = pTrimSettings->PWRSTATE_b.TVRGCACTTRIM;
            //
            // Assign new TVRGF trims
            //
            g_ui32NewVddfTrim = pTrimSettings->PWRSTATE_b.TVRGFACTTRIM;
            //
            // CoreLDO trims
            //
            MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM = pTrimSettings->PWRSTATE_b.CORELDOTEMPCOTRIM;
            if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
            {
                //
                // Update the LDO trims and boost in single assignment.
                //
                // Overflow checks for boosting coreldo
                //
                if ((pTrimSettings->PWRSTATE_b.CORELDOACTTRIM + CORELDO_BOOST_FOR_TVRG_ADJ) > (MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Msk >> MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Pos))
                {
                    g_i32CORELDODiff = (MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Msk >> MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Pos) - pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;
                }
                else
                {
                    g_i32CORELDODiff = CORELDO_BOOST_FOR_TVRG_ADJ;
                }
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM + g_i32CORELDODiff;
                ui32BaseVddcTrim = pBaseTrimSettings->PWRSTATE_b.TVRGCACTTRIM;
                ui32BaseVddfTrim = pBaseTrimSettings->PWRSTATE_b.TVRGFACTTRIM;
            }
            else
            {
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;
                ui32BaseVddcTrim = pCurTrimSettings->PWRSTATE_b.TVRGCACTTRIM;
                ui32BaseVddfTrim = pCurTrimSettings->PWRSTATE_b.TVRGFACTTRIM;
            }
            //
            // Calculate the trim difference for double boost.
            // If voltage is being reduced, calculate the actual difference without double but with the sign
            //
            if (g_ui32NewVddcTrim > ui32BaseVddcTrim)
            {
                i32DblBstVddcDiff = 2 * (g_ui32NewVddcTrim - ui32BaseVddcTrim);
            }
            else
            {
                i32DblBstVddcDiff = (int32_t)(ui32BaseVddcTrim - g_ui32NewVddcTrim);
                i32DblBstVddcDiff = -i32DblBstVddcDiff;
            }
            if (g_ui32NewVddfTrim > ui32BaseVddfTrim)
            {
                i32DblBstVddfDiff = 2 * (g_ui32NewVddfTrim - ui32BaseVddfTrim);
            }
            else
            {
                i32DblBstVddfDiff = (int32_t)(ui32BaseVddfTrim - g_ui32NewVddfTrim);
                i32DblBstVddfDiff = -i32DblBstVddfDiff;
            }
            //
            // Apply TVRGC and TVRGF trims, assign delay for boost.
            //
            if ((ui32BaseVddcTrim + i32DblBstVddcDiff > (MCUCTRL_VREFGEN2_TVRGCVREFTRIM_Msk >> MCUCTRL_VREFGEN2_TVRGCVREFTRIM_Pos)) ||
                (ui32BaseVddfTrim + i32DblBstVddfDiff > (MCUCTRL_VREFGEN4_TVRGFVREFTRIM_Msk >> MCUCTRL_VREFGEN4_TVRGFVREFTRIM_Pos)) ||
                PCM2P1_WO_PATCH)
            {
                MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
                MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
                if (PCM2P1_WO_PATCH)
                {
                    ui32BoostDelayInUs = LDO_BOOST_DURATION_IN_US;
                }
                else
                {
                    ui32BoostDelayInUs = LDO_BOOST_DURATION_OPTIMIZED_IN_US;
                }
            }
            else
            {
                MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = ui32BaseVddcTrim + i32DblBstVddcDiff;
                MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = ui32BaseVddfTrim + i32DblBstVddfDiff;
                ui32BoostDelayInUs = LDO_BOOST_DURATION_OPTIMIZED_DOUBLE_BOOST_IN_US;
            }
            //
            // If timer is running, restart the timer if voltage of next power
            // state is higher than current power state.
            //
            if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
            {
                //
                // Restart timer
                //
                ldo_boost_timer_restart(ui32BoostDelayInUs);
            }
            else
            {
                //
                // Boost LDOs
                //
                spotmgr_ldo_boost(true);
                //
                // Start timer
                //
                ldo_boost_timer_start(ui32BoostDelayInUs);
                //
                // Update ui32BasePwrStateStatic
                //
                ui32BasePwrStateStatic = ui32CurPwrState;
            }
            //
            // As it stands, none of the CPU HP states are same or lower power than CPU LP states
            // If switching from CPU LP to HP mode, switch the VDDMCPU domain from VDDC to VDDF
            //
            if ((ui32CurPwrState <= 7) && (ui32PwrState > 7))
            {
                if (SCB->CCR & SCB_CCR_IC_Msk)
                {
                    am_hal_cachectrl_icache_disable();
                    bEnableICache = true;
                }
                MCUCTRL->PWRSW0_b.PWRSWVDDMCPUSTATSEL = MCUCTRL_PWRSW0_PWRSWVDDMCPUSTATSEL_VDDF;
                g_bSwitchingToHp = true;
            }
            //
            // Delay 20 us when stepping up to the higher voltage level
            //
            am_hal_delay_us(20);
            //
            //  Enable the I-Cache.
            //
            if (bEnableICache)
            {
                am_hal_cachectrl_icache_enable();
            }
        }
        else // Lower down to the lower voltage level
        {
            //
            // CoreLDO trims
            //
            if ((TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN) &&
                ((g_ui32PwrStVddcVoltLvl[ui32PwrState] > g_ui32PwrStVddcVoltLvl[ui32BasePwrStateStatic]) ||
                 (g_ui32PwrStVddfVoltLvl[ui32PwrState] > g_ui32PwrStVddfVoltLvl[ui32BasePwrStateStatic])))
            {
                //
                // Update the LDO trims and boost in single assignment.
                //
                // Overflow checks for boosting coreldo
                //
                if ((pTrimSettings->PWRSTATE_b.CORELDOACTTRIM + CORELDO_BOOST_FOR_TVRG_ADJ) > (MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Msk >> MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Pos))
                {
                    g_i32CORELDODiff = (MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Msk >> MCUCTRL_LDOREG1_CORELDOACTIVETRIM_Pos) - pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;
                }
                else
                {
                    g_i32CORELDODiff = CORELDO_BOOST_FOR_TVRG_ADJ;
                }
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM + g_i32CORELDODiff;
            }
            else
            {
                MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = pTrimSettings->PWRSTATE_b.CORELDOACTTRIM;
            }
            MCUCTRL->LDOREG1_b.CORELDOTEMPCOTRIM  = pTrimSettings->PWRSTATE_b.CORELDOTEMPCOTRIM;
            //
            // TVRGC and TVRGF trims
            //
            if (PCM2P1_WO_PATCH || !(TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN))
            {
                MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = pTrimSettings->PWRSTATE_b.TVRGCACTTRIM;
                MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = pTrimSettings->PWRSTATE_b.TVRGFACTTRIM;
            }
            else
            {
                g_ui32NewVddcTrim = pTrimSettings->PWRSTATE_b.TVRGCACTTRIM;
                g_ui32NewVddfTrim = pTrimSettings->PWRSTATE_b.TVRGFACTTRIM;
            }
            //
            // Adjust VDDC and VDDF Simobuck Tons if Ton state is changing
            //
            if ((ui32TonState != ui32CurTonState) ||
                (ui32PwrState == 1)               ||
                (ui32PwrState == 5)               ||
                (ui32PwrState == 8)               ||
                (ui32PwrState == 12)              ||
                (ui32PwrState == 14)              ||
                (ui32PwrState == 15)              ||
                (ui32CurPwrState == 1)            ||
                (ui32CurPwrState == 5)            ||
                (ui32CurPwrState == 8)            ||
                (ui32CurPwrState == 12)           ||
                (ui32CurPwrState == 14)           ||
                (ui32CurPwrState == 15))
            {
                spotmgr_power_ton_adjust(ui32TonState, ui32PwrState);
            }
            //
            // If timer is running, compare the voltage of target power state
            // and the base voltage of the last power rail ramping up
            //
            if (TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN)
            {
                //
                // If target voltage is less than or equal to base voltage, stop the timer and remove LDOs boost.
                //
                if ((g_ui32PwrStVddcVoltLvl[ui32PwrState] <= g_ui32PwrStVddcVoltLvl[ui32BasePwrStateStatic]) &&
                    (g_ui32PwrStVddfVoltLvl[ui32PwrState] <= g_ui32PwrStVddfVoltLvl[ui32BasePwrStateStatic]))
                {
                    //
                    // As it stands, none of the CPU HP states are same or lower power than CPU LP states
                    //
                    // Stop timer, clear the timer interrupt status
                    //
                    ldo_boost_timer_stop();
                    //
                    // Update TVRGC and TCRGF trims based on current state
                    //
                    spotmgr_buck_trims_update();
                    //
                    // Remove boost, keep the CORELDOACTIVETRIM at the value of the current power state
                    //
                    spotmgr_ldo_boost_remove(false);
                }
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
        sPwrStatDesc.PWRSTATEDESC_b.GPUPERIPHMODE = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.GPUPERIPHMODE = 0;
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

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_4: // CPULP + G/P, Temp 3
            *pui32PwrState = 4;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_5: // CPULP + G/P, Temp 2
            *pui32PwrState = 5;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_6: // CPULP + G/P, Temp 1
            *pui32PwrState = 6;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_7: // CPULP + G/P, Temp 0
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

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_12: // CPUHP + G/P, Temp 3
            *pui32PwrState = 12;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_13: // CPUHP + G/P, Temp 2
            *pui32PwrState = 13;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_14: // CPUHP + G/P, Temp 1
            *pui32PwrState = 14;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_15: // CPUHP + G/P, Temp 0
            *pui32PwrState = 15;
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
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_tempco_range_e enum.
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

int32_t
am_hal_spotmgr_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32PowerState = 0, ui32TonState = 0;
    am_hal_spotmgr_power_status_t sPwrStatus;
    bool bSetIntPwrAfterPwrState = false;
    bool bSkipAllUpdates = false, bReqPwrOrTonStateChg = true;
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
    static uint32_t ui32CurPowerStateStatic = 7; // Per Validation's reply, the default power state is 7.
    static uint32_t ui32CurTonStateStatic = 5; // The default Ton state is 5.
    static am_hal_spotmgr_tempco_range_e eCurTempRangeStatic = AM_HAL_SPOTMGR_TEMPCO_RANGE_VERY_LOW;    // Per Validation's reply, the default state is 7. The temp range is VERY_LOW in state 7.
    static am_hal_spotmgr_cpu_state_e eLastCpuStateStatic = AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP;          // The default CPU state after MCU powering up is LP.
    AM_CRITICAL_BEGIN
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
                    sPwrStatus.eTempRange = *((am_hal_spotmgr_tempco_range_e *)pArgs);
                    eCurTempRangeStatic = sPwrStatus.eTempRange;
                    g_bTempLessThan50C = (eCurTempRangeStatic <= AM_HAL_SPOTMGR_TEMPCO_RANGE_MID);
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
                        //
                        // If CPU is going to deep sleep, need to determine the buck state
                        // in deep sleep.
                        //
                        if (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP)
                        {
                            spotmgr_buck_state_determine(&sPwrStatus);
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
                            bSetIntPwrAfterPwrState = true;
                        }
                        //
                        // If entering deepsleep in CPU LP mode, and STM state was collapsed to
                        // STM+periph state, g_bVddfLpMinusForLp must be set to save power.
                        //
                        else if ((eLastCpuStateStatic == AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP) &&
                                 (sPwrStatus.eCpuState == AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP))
                        {
                            if (IS_PROFILE_COLLAPSE_STM_AND_STMP &&
                                !((sPwrStatus.eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_LP) ||
                                  (sPwrStatus.eGpuState == AM_HAL_SPOTMGR_GPUSTATE_ACTIVE_HP) ||
                                  (sPwrStatus.ui32DevPwrSt & DEVPWRST_MONITOR_PERIPH_MASK)    ||
                                  (sPwrStatus.ui32AudSSPwrSt & AUDSSPWRST_MONITOR_PERIPH_MASK)))
                            {
                                //
                                // Set this bit to set VDDF LP minus offset trim to reduce VDDF
                                //
                                g_bVddfLpMinusForLp = true;
                            }
                        }
                        //
                        // If eLastCpuStateStatic or sPwrStatus.eCpuState is sleep, PwrState does not change,
                        // only set internal power domain here.
                        // It is a special case that entering deepsleep in CPU LP mode, because when entering
                        // deepsleep in CPU LP mode, we must update g_bVddfLpMinusForLp and power state
                        // according to the status and profile.
                        // So, I created a separate conditional branch above for entering deepsleep from CPU LP mode,
                        // and did not clear bReqPwrOrTonStateChg there.
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
                if ((ui32PowerState != ui32CurPowerStateStatic) || (ui32TonState != ui32CurTonStateStatic))
                {
                    //
                    // Update trims
                    //
                    spotmgr_power_trims_update(ui32PowerState, ui32CurPowerStateStatic, ui32TonState, ui32CurTonStateStatic);
                    //
                    // Set internal power domain for CPU HP to LP switch, PwrState will be changed.
                    //
                    if (bSetIntPwrAfterPwrState)
                    {
                        spotmgr_internal_power_domain_set(sPwrStatus.eCpuState, eLastCpuStateStatic);
                        eLastCpuStateStatic = sPwrStatus.eCpuState;
                    }
                }
                //
                // Maintain a static variable with current trim settings.
                //
                ui32CurPowerStateStatic = ui32PowerState;
                ui32CurTonStateStatic   = ui32TonState;
            }
        }
    }

    AM_CRITICAL_END

    return ui32Status;
}

//*****************************************************************************
//
//! @brief SPOT manager init
//!        This API should be called from am_hal_pwrctrl_low_power_init, to
//!        initialise SPOT manager.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
int32_t
am_hal_spotmgr_init(void)
{
    uint32_t ui32Status;
    uint32_t info1buf[4];

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

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_POWERSTATE15_O , AM_REG_OTP_INFO1_POWERSTATE0_O , sizeof(g_sSpotMgrINFO1regs.sPowerStateArray) / sizeof(am_hal_spotmgr_trim_settings_t));
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_POWERSTATE0_O  / 4), sizeof(g_sSpotMgrINFO1regs.sPowerStateArray) / sizeof(am_hal_spotmgr_trim_settings_t), (uint32_t *) &(g_sSpotMgrINFO1regs.sPowerStateArray[0]));

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_DEFAULTTON_O, AM_REG_OTP_INFO1_GPUVDDCTON_O, 4 );
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_GPUVDDCTON_O  / 4), 4, &info1buf[0]);
    g_sSpotMgrINFO1regs.sGpuVddcTon.GPUVDDCTON    = info1buf[0];
    g_sSpotMgrINFO1regs.sGpuVddfTon.GPUVDDFTON    = info1buf[1];
    g_sSpotMgrINFO1regs.sStmTon.STMTON            = info1buf[2];
    g_sSpotMgrINFO1regs.sDefaultTon.DEFAULTTON    = info1buf[3];

    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_MEMLDOCONFIG_O  / 4), 1, &info1buf[0]);
    g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG   = info1buf[0];

    g_sSpotMgrINFO1regs.sPowerStateArray[8].PWRSTATE_b.TVRGCACTTRIM =
        (g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.TVRGCACTTRIM +
         g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.TVRGCACTTRIM) / 2;
    g_sSpotMgrINFO1regs.sPowerStateArray[9].PWRSTATE_b.TVRGCACTTRIM =
        g_sSpotMgrINFO1regs.sPowerStateArray[10].PWRSTATE_b.TVRGCACTTRIM;
    //
    // As such these files are no longer used - but just for consistency, we update it here, in case we ever go back to using this
    //
    g_sSpotMgrINFO1regs.sPowerStateArray[8].PWRSTATE_b.TVRGCVREFSEL = g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.TVRGCVREFSEL;
    g_sSpotMgrINFO1regs.sPowerStateArray[9].PWRSTATE_b.TVRGCVREFSEL = g_sSpotMgrINFO1regs.sPowerStateArray[10].PWRSTATE_b.TVRGCVREFSEL;

    //
    // All done, mark the data as valid
    //
    g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid = INFO1GLOBALVALID;

    //
    // Initialise the timer for power boost
    //
    ldo_boost_timer_init();

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Set profile of SPOT manager
//!        This API is used to set SPOT manager profile to change the logic of
//!        power state determination. We implemented 2 profiles as below.
//!
//!        When psProfile->PROFILE_b.COLLAPSESTMANDSTMP ==
//!        AM_HAL_SPOTMGR_COLLAPSE_STM_STMP_DIS (the default profile), keep the
//!        original power state.
//!
//!        When psProfile->PROFILE_b.COLLAPSESTMANDSTMP ==
//!        AM_HAL_SPOTMGR_COLLAPSE_STM_STMP_EN, collapse the STM and STM+periph
//!        power states(0&4, 1&5, 2&6, 3&7, 8&12, 9&13, 10&14, 11&15). We suggest
//!        falling back to this profile for scenarios with frequent peripheral
//!        bursts and the overall time of all peripherals off state is short,
//!        which result in a lower overall average power compared to the default.
//!
//! Important:
//!        After setting the profile, the new profile
//!        takes effect in the next calling to am_hal_spotmgr_power_state_update,
//!        except the calling to am_hal_spotmgr_power_state_update when waking up MCU
//!        from normal sleep or deep sleep, entering normal sleep from CPU HP or LP
//!        mode, entering deepsleep from HP mode.
//!
//! @param psProfile - Pointer of am_hal_spotmgr_profile_t struct.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_spotmgr_profile_set(am_hal_spotmgr_profile_t * psProfile)
{
    g_sSpotMgrProfile = *psProfile;
}

//*****************************************************************************
//
//! @brief Get current profile which is saved to g_sSpotMgrProfile
//!
//! @param psProfile - Pointer of am_hal_spotmgr_profile_t struct.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_spotmgr_profile_get(am_hal_spotmgr_profile_t * psProfile)
{
    *psProfile = g_sSpotMgrProfile;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
