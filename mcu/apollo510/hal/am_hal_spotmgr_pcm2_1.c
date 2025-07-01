// ****************************************************************************
//
//! @file am_hal_spotmgr_pcm2_1.c
//!
//! @brief SPOT manager functions that manage power states for PCM2.1 parts
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

#if !AM_HAL_SPOTMGR_PCM2_1_DISABLE

//
//! Enum for power states descriptions, only bit[0:11] are used to determine power state.
//
typedef enum
{
    AM_HAL_SPOTMGR_POWER_STATE_DESC_0  = 0x000300, // CPULP, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_1  = 0x000200, // CPULP, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_2  = 0x000100, // CPULP, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_3  = 0x000000, // CPULP, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_4  = 0x000310, // CPULP + G/P, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_5  = 0x000210, // CPULP + G/P, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_6  = 0x000110, // CPULP + G/P, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_7  = 0x000010, // CPULP + G/P, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_8  = 0x000301, // CPUHP, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_9  = 0x000201, // CPUHP, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_10 = 0x000101, // CPUHP, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_11 = 0x000001, // CPUHP, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_12 = 0x000311, // CPUHP + G/P, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_13 = 0x000211, // CPUHP + G/P, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_14 = 0x000111, // CPUHP + G/P, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_15 = 0x000011, // CPUHP + G/P, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_16 = 0x100310, // CPULP + G/P + SDIO, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_17 = 0x100210, // CPULP + G/P + SDIO, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_18 = 0x100110, // CPULP + G/P + SDIO, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_19 = 0x100010, // CPULP + G/P + SDIO, Temp 0
    AM_HAL_SPOTMGR_POWER_STATE_DESC_20 = 0x100311, // CPUHP + G/P + SDIO, Temp 3
    AM_HAL_SPOTMGR_POWER_STATE_DESC_21 = 0x100211, // CPUHP + G/P + SDIO, Temp 2
    AM_HAL_SPOTMGR_POWER_STATE_DESC_22 = 0x100111, // CPUHP + G/P + SDIO, Temp 1
    AM_HAL_SPOTMGR_POWER_STATE_DESC_23 = 0x100011  // CPUHP + G/P + SDIO, Temp 0
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
        //! GPU and all peripheral power state
        uint32_t GPUPERIPHMODE  : 4;
        //! Temperature range
        uint32_t TEMPRANGE      : 4;
        //! GPU power state
        uint32_t GPUMODE        : 4;
        //! Peripheral power state
        uint32_t PERIPHMODE     : 4;
        //! SDIO power state
        uint32_t SDIOMODE       : 4;
        //! Reserved
        uint32_t                : 8;
    } PWRSTATEDESC_b;
} am_hal_spotmgr_power_state_desc_t;

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//! Only bit[0:11] and bit[20:23] are used to determine power state.
#define PWR_STATE_DESC_MASK 0x00F00FFF
//! Only bit[0:3] and bit[12:19] are used to determine ton state.
#define TON_STATE_DESC_MASK 0x000FF00F
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
//! VDDCLVACTLOWTONTRIM trim code for power state 1 5 17
#define VDDCLVACTLOWTONTRIM_POWER_STATE_1_5_17 4
//! VDDCLVACTLOWTONTRIM trim code for all other power states except power state 1 (and 5).
#define VDDCLVACTLOWTONTRIM_DEFAULT 6
//! Maximum delay for waiting for timer AM_HAL_INTERNAL_TIMER_NUM_A is disabled
#define TIMER_A_DISABLE_WAIT_IN_US   2500

//*****************************************************************************
//
//! Static Variable
//
//*****************************************************************************

//! CoreLDO actual boost code for TVRG adjustment
static int32_t g_i32CORELDODiff = 0;

//! VDDC voltage level for power states, a lager number indicates the voltage is higher
static uint32_t g_ui32PwrStVddcVoltLvl[20] = {0, 0, 1, 2, 1, 1, 2, 3, 6, 4, 4, 4, 8, 7, 5, 5, 1, 1, 2, 3};

//! VDDF voltage level for power states, a lager number indicates the voltage is higher
static uint32_t g_ui32PwrStVddfVoltLvl[20] = {1, 0, 0, 0, 3, 2, 2, 2, 5, 4, 4, 4, 6, 4, 4, 4, 6, 4, 4, 4};

//! New VDDC TVRGCVREFTRIM trim value
static uint32_t g_ui32NewVddcTrim = 0;

//! New VDDF TVRGFVREFTRIM trim value
static uint32_t g_ui32NewVddfTrim = 0;

//! The default power state is 7.
static uint32_t g_ui32CurPowerStateStatic = 7;

//*****************************************************************************
//
//! Inline function to convert temperature in float to PCM2.1 temperature range
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
        MCUCTRL->D2ASPARE_b.MEMLDOREF = 0x3;
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
    MCUCTRL->D2ASPARE_b.MEMLDOREF = g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.MEMLDOD2ASPARE;

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
    if (!g_bIsPCM2p1WoPatch)
    {
        MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
        MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
    }
}

//*****************************************************************************
//
//! Timer interrupt service for spotmgr
//
//*****************************************************************************
void
am_hal_spotmgr_pcm2_1_boost_timer_interrupt_service(void)
{
    AM_CRITICAL_BEGIN
    //
    // Remove double boost if needed
    //
    spotmgr_buck_trims_update();
    //
    // If temperature < 50C, change the power to VDDC.
    // Else, we leave the CPU running off VDDF.
    //
    if (g_bSwitchingToHp)
    {
        if ((g_ui32CurPowerStateStatic != 8) && (g_ui32CurPowerStateStatic != 12))
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
    am_hal_spotmgr_timer_stop();
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
        if (!g_bSwitchingToHp)
        {
            //
            // Set this bit to enable Vddcaor Vddcpu Override clear and set when switching between HP and deepsleep.
            //
            g_bHpToDeepSleep = true;
        }
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
    // VDDC_LV Ton adjustments for power state 1 and 5, 17
    //
    if ((ui32TarPwrState == 1) || (ui32TarPwrState == 5) || (ui32TarPwrState == 17))
    {
        MCUCTRL->SIMOBUCK4_b.VDDCLVACTLOWTONTRIM = VDDCLVACTLOWTONTRIM_POWER_STATE_1_5_17;
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
    uint32_t ui32TimerDelayInUs = 0;
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
                (ui32PwrState == 17)              ||
                (ui32PwrState == 8)               ||
                (ui32PwrState == 12)              ||
                (ui32PwrState == 14)              ||
                (ui32PwrState == 15)              ||
                (ui32CurPwrState == 1)            ||
                (ui32CurPwrState == 5)            ||
                (ui32CurPwrState == 17)           ||
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
                g_bIsPCM2p1WoPatch)
            {
                MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = g_ui32NewVddcTrim;
                MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = g_ui32NewVddfTrim;
                if (g_bIsPCM2p1WoPatch)
                {
                    ui32TimerDelayInUs = LDO_BOOST_DURATION_IN_US;
                }
                else
                {
                    ui32TimerDelayInUs = LDO_BOOST_DURATION_OPTIMIZED_IN_US;
                }
            }
            else
            {
                MCUCTRL->VREFGEN2_b.TVRGCVREFTRIM = ui32BaseVddcTrim + i32DblBstVddcDiff;
                MCUCTRL->VREFGEN4_b.TVRGFVREFTRIM = ui32BaseVddfTrim + i32DblBstVddfDiff;
                ui32TimerDelayInUs = LDO_BOOST_DURATION_OPTIMIZED_DOUBLE_BOOST_IN_US;
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
                am_hal_spotmgr_timer_restart(ui32TimerDelayInUs);
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
                am_hal_spotmgr_timer_start(ui32TimerDelayInUs);
                //
                // Update ui32BasePwrStateStatic
                //
                ui32BasePwrStateStatic = ui32CurPwrState;
            }
            //
            // As it stands, none of the CPU HP states are same or lower power than CPU LP states
            // If switching from CPU LP to HP mode, switch the VDDMCPU domain from VDDC to VDDF
            //
            if (((ui32CurPwrState <= 7) || ((ui32CurPwrState >= 16) && (ui32CurPwrState <= 19))) &&
                ((ui32PwrState >= 8) && (ui32PwrState <= 15)))
            {
                //
                // If icache is enabled, disable icache
                //
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
            if (g_bIsPCM2p1WoPatch || !(TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN))
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
                (ui32PwrState == 17)              ||
                (ui32PwrState == 8)               ||
                (ui32PwrState == 12)              ||
                (ui32PwrState == 14)              ||
                (ui32PwrState == 15)              ||
                (ui32CurPwrState == 1)            ||
                (ui32CurPwrState == 5)            ||
                (ui32CurPwrState == 17)           ||
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
                    am_hal_spotmgr_timer_stop();
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

    if ((psPwrStatus->ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTSDIO0_Msk) ||
        (psPwrStatus->ui32DevPwrSt & PWRCTRL_DEVPWRSTATUS_PWRSTSDIO1_Msk))
    {
        sPwrStatDesc.PWRSTATEDESC_b.SDIOMODE = 1;
    }
    else
    {
        sPwrStatDesc.PWRSTATEDESC_b.SDIOMODE = 0;
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
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_20: // CPUHP + G/P + SDIO, Temp 3
            *pui32PwrState = 12;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_13: // CPUHP + G/P, Temp 2
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_21: // CPUHP + G/P + SDIO, Temp 2
            *pui32PwrState = 13;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_14: // CPUHP + G/P, Temp 1
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_22: // CPUHP + G/P + SDIO, Temp 1
            *pui32PwrState = 14;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_15: // CPUHP + G/P, Temp 0
        case AM_HAL_SPOTMGR_POWER_STATE_DESC_23: // CPUHP + G/P + SDIO, Temp 0
            *pui32PwrState = 15;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_16: // CPULP + G/P + SDIO, Temp 3
            *pui32PwrState = 16;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_17: // CPULP + G/P + SDIO, Temp 2
            *pui32PwrState = 17;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_18: // CPULP + G/P + SDIO, Temp 1
            *pui32PwrState = 18;
            break;

        case AM_HAL_SPOTMGR_POWER_STATE_DESC_19: // CPULP + G/P + SDIO, Temp 0
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
//! @brief Power states update for PCM2.1
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
am_hal_spotmgr_pcm2_1_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    uint32_t ui32PowerState = 0, ui32TonState = 0;
    am_hal_spotmgr_power_status_t sPwrStatus;
    bool bSetIntPwrAfterPwrState = false;
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
    static uint32_t ui32CurTonStateStatic = 6;      // The default Ton state is 6 - CPU LP, GPU off and any peripheral on.
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
                            if ((g_ui32CurPowerStateStatic != 8)  &&
                                (g_ui32CurPowerStateStatic != 12) &&
                                (!TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0_b.TMR0EN) )
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
                            eLastCpuStateStatic = sPwrStatus.eCpuState;
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
                    //
                    // Set internal power domain for CPU HP to LP switch, PwrState will be changed.
                    //
                    if (bSetIntPwrAfterPwrState)
                    {
                        spotmgr_internal_power_domain_set(sPwrStatus.eCpuState, eLastCpuStateStatic);
                        eLastCpuStateStatic = sPwrStatus.eCpuState;
                    }
                }

                if (((g_ui32CurPowerStateStatic == 12) && (ui32PowerState == 13 || ui32PowerState == 14 || ui32PowerState == 15)) ||
                    ((g_ui32CurPowerStateStatic ==  8) && (ui32PowerState ==  9 || ui32PowerState == 10 || ui32PowerState == 11)))
                {
                    MCUCTRL->PWRSW0_b.PWRSWVDDCAOROVERRIDE = 1;
                    MCUCTRL->PWRSW0_b.PWRSWVDDCPUOVERRIDE = 1;
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
//! @brief SIMOBUCK initialziation handling at stage just before overriding
//!        LDO/SIMOBUCK
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_1_simobuck_init_bfr_ovr(void)
{
    MCUCTRL->SIMOBUCK4_b.VDDCLVACTLOWTONTRIM = VDDCLVACTLOWTONTRIM_DEFAULT;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just before enabling
//!        SIMOBUCK for PCM2.1
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_1_simobuck_init_bfr_enable(void)
{
    if ( g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid == INFO1GLOBALVALID )
    {
        //
        // Reduce CORELDOACTIVETRIM
        //
        MCUCTRL->LDOREG1_b.CORELDOACTIVETRIM = g_sSpotMgrINFO1regs.sPowerStateArray[7].PWRSTATE_b.CORELDOACTTRIM;
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just after enabling
//!        SIMOBUCK for PCM2.1
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_1_simobuck_init_aft_enable(void)
{
    if ( g_sSpotMgrINFO1regs.ui32SpotMgrINFO1GlobalValid == INFO1GLOBALVALID )
    {
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
//!        PCM2.1
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_1_tempco_suspend(void)
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
//! @brief SPOT manager init for PCM2.1
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t
am_hal_spotmgr_pcm2_1_init(void)
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

    CHK_OFFSET_DELTA(AM_REG_OTP_INFO1_POWERSTATE15_O , AM_REG_OTP_INFO1_POWERSTATE0_O , 16);
    RD_INFO1(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, (AM_REG_OTP_INFO1_POWERSTATE0_O  / 4), 16, (uint32_t *) &(g_sSpotMgrINFO1regs.sPowerStateArray[0]));
    //
    // Trims for power state 16~19
    //
    g_sSpotMgrINFO1regs.sPowerStateArray[16] = g_sSpotMgrINFO1regs.sPowerStateArray[4];
    g_sSpotMgrINFO1regs.sPowerStateArray[17] = g_sSpotMgrINFO1regs.sPowerStateArray[5];
    g_sSpotMgrINFO1regs.sPowerStateArray[18] = g_sSpotMgrINFO1regs.sPowerStateArray[6];
    g_sSpotMgrINFO1regs.sPowerStateArray[19] = g_sSpotMgrINFO1regs.sPowerStateArray[7];
    g_sSpotMgrINFO1regs.sPowerStateArray[16].PWRSTATE_b.TVRGFACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.TVRGFACTTRIM;
    g_sSpotMgrINFO1regs.sPowerStateArray[17].PWRSTATE_b.TVRGFACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.TVRGFACTTRIM;
    g_sSpotMgrINFO1regs.sPowerStateArray[18].PWRSTATE_b.TVRGFACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[14].PWRSTATE_b.TVRGFACTTRIM;
    g_sSpotMgrINFO1regs.sPowerStateArray[19].PWRSTATE_b.TVRGFACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[15].PWRSTATE_b.TVRGFACTTRIM;

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
    // The VDDC and CORELDO settings for power state 12 are the same as those for power state 13.
    //
    g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.TVRGCACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.TVRGCACTTRIM;
    g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.TVRGCVREFSEL = g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.TVRGCVREFSEL;
    g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.CORELDOTEMPCOTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.CORELDOTEMPCOTRIM;
    g_sSpotMgrINFO1regs.sPowerStateArray[12].PWRSTATE_b.CORELDOACTTRIM = g_sSpotMgrINFO1regs.sPowerStateArray[13].PWRSTATE_b.CORELDOACTTRIM;

    //
    // PCM2.1 does not have this bit field in INFO1, we initialise this field in global variable.
    //
    g_sSpotMgrINFO1regs.sMemldoCfg.MEMLDOCONFIG_b.VDDFCOMPTRIMMINUS = 31;

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
//! @brief Reset power state to POR default for PCM2.1
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
uint32_t am_hal_spotmgr_pcm2_1_default_reset(void)
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
