// ****************************************************************************
//
//! @file am_hal_clkgen.c
//!
//! @brief Functions for interfacing with the CLKGEN.
//!
//! @addtogroup clkgen4 CLKGEN - Clock Generator
//! @ingroup apollo510L_hal
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
// This is part of revision release_sdk5_2_a_0-438c93f352 of the AmbiqSuite Development Package.
//
// ****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "../am_hal_clkmgr_private.h"

// ****************************************************************************
//  Global variables
// ****************************************************************************
static bool g_bClkGenFirstClkOutInvoke = false;

// ****************************************************************************
//
//  am_hal_clkgen_control()
//      Apply various specific commands/controls on the CLKGEN module.
//
// ****************************************************************************
uint32_t
am_hal_clkgen_control(am_hal_clkgen_control_e eControl, void *pArgs)
{
    switch ( eControl )
    {
        case AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC:
            CLKGEN->OCTRL_b.RTCOSEL = CLKGEN_OCTRL_RTCOSEL_LFRC_DIV2;
            break;

        case AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL:
            CLKGEN->OCTRL_b.RTCOSEL = CLKGEN_OCTRL_RTCOSEL_XT_512Hz;
            break;

        case AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE:
            return am_hal_clkmgr_private_clkgen_hfadj_apply(pArgs);

        case AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE:
            return am_hal_clkmgr_private_clkgen_hfadj_disable();

        case AM_HAL_CLKGEN_CONTROL_I3CCLKEN_ENABLE:
            CLKGEN->CLKCTRL_b.I3CCLKEN = CLKGEN_CLKCTRL_I3CCLKEN_ENABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_I3CCLKEN_DISABLE:
            CLKGEN->CLKCTRL_b.I3CCLKEN = CLKGEN_CLKCTRL_I3CCLKEN_DISABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLVCOEN_ENABLE:
            CLKGEN->CLKCTRL_b.PLLVCOEN = CLKGEN_CLKCTRL_PLLVCOEN_ENABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLVCOEN_NORMALOP:
            CLKGEN->CLKCTRL_b.PLLVCOEN = CLKGEN_CLKCTRL_PLLVCOEN_NORMALOP;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLDIVEN_ENABLE:
            CLKGEN->CLKCTRL_b.PLLDIVEN = CLKGEN_CLKCTRL_PLLDIVEN_ENABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLDIVEN_NORMALOP:
            CLKGEN->CLKCTRL_b.PLLDIVEN = CLKGEN_CLKCTRL_PLLDIVEN_NORMALOP;
            break;

        case AM_HAL_CLKGEN_CONTROL_SELECT_EXTREF:
            CLKGEN->CLKCTRL_b.XTHSMUXSEL = CLKGEN_CLKCTRL_XTHSMUXSEL_EXTREF;
            break;

        case AM_HAL_CLKGEN_CONTROL_SELECT_XT:
            CLKGEN->CLKCTRL_b.XTHSMUXSEL = CLKGEN_CLKCTRL_XTHSMUXSEL_XTHS;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_ENABLE:
            CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_ENABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCTRLCLK_DISABLE:
            CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_DISABLE;
            break;

        case AM_HAL_CLKGEN_CONTROL_GFXCORECLK_ENABLE:
            CLKGEN->CLKCTRL_b.GFXCORECLKEN = CLKGEN_CLKCTRL_GFXCORECLKEN_EN;
            break;

        case AM_HAL_CLKGEN_CONTROL_GFXCORECLK_DISABLE:
            CLKGEN->CLKCTRL_b.GFXCORECLKEN = CLKGEN_CLKCTRL_GFXCORECLKEN_DIS;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_clkgen_control()

// ****************************************************************************
//
//  am_hal_clkgen_status_get()
//  This function returns the current value of various CLKGEN statuses.
//
// ****************************************************************************
uint32_t
am_hal_clkgen_status_get(am_hal_clkgen_status_t *psStatus)
{
    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    psStatus->ui32SysclkFreq = AM_HAL_CLKGEN_FREQ_MAX_HZ;

    psStatus->eRTCOSC = AM_HAL_CLKGEN_STATUS_RTCOSC_LFRC;
    psStatus->bXtalFailure = false;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_clkgen_status_get()

static am_hal_clkmgr_clock_id_e
am_hal_clkgen_clksrc_get(uint32_t clk)
{
    switch(clk)
    {
        // XTAL
        case AM_HAL_CLKGEN_CLKOUT_XTAL_32768:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_16384:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_8192:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_4096:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_2048:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_1024:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_128:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_4:
        case AM_HAL_CLKGEN_CLKOUT_XTAL_0_5:
            return AM_HAL_CLKMGR_CLK_ID_XTAL_LS;
        // LFRC
        case AM_HAL_CLKGEN_CLKOUT_LFRC_NOMINAL:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV2:
            return AM_HAL_CLKMGR_CLK_ID_LFRC;
        case AM_HAL_CLKGEN_CLKOUT_PLL_VCO250M:
        case AM_HAL_CLKGEN_CLKOUT_PLL_VCO125M:
            return AM_HAL_CLKMGR_CLK_ID_PLLVCO;
        // PLL_POSTDIV
        case AM_HAL_CLKGEN_CLKOUT_PLL_POSTDIV:
            return AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV;
        // HS XTAL
        case AM_HAL_CLKGEN_CLKOUT_RF_XTAL_48M:
        case AM_HAL_CLKGEN_CLKOUT_RF_XTAL_24M:
        case AM_HAL_CLKGEN_CLKOUT_RF_XTAL_12M:
            return AM_HAL_CLKMGR_CLK_ID_XTAL_HS;
        default:
            return AM_HAL_CLKMGR_CLK_ID_MAX;
    }
}

// ****************************************************************************
//
//  am_hal_clkgen_clkout_enable()
//  This function is used to select and enable CLKOUT.
//
// ****************************************************************************
uint32_t
am_hal_clkgen_clkout_enable(bool bEnable, am_hal_clkgen_clkout_e eClkSelect)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Do a basic validation of the eClkSelect parameter.
    // Not every value in the range is valid, but at least this simple check
    // provides a reasonable chance that the parameter is valid.
    //
    if (bEnable && (eClkSelect > (am_hal_clkgen_clkout_e)AM_HAL_CLKGEN_CLKOUT_MAX))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Disable CLKOUT and release existing clock if:
    // - CLKOUT disable is requested
    // - different clock source is requested when CLKOUT is still enabled
    // - this is the first invocation after bootup
    //
    uint32_t ui32CurClkSrc = CLKGEN->CLKOUT_b.CKSEL;
    if (!bEnable ||
        (CLKGEN->CLKOUT_b.CKEN && ((ui32CurClkSrc != eClkSelect) || g_bClkGenFirstClkOutInvoke)))
    {
        am_hal_clkmgr_clock_id_e eClkMgrCurClkSrc = am_hal_clkgen_clksrc_get(ui32CurClkSrc);
        CLKGEN->CLKOUT_b.CKEN = CLKGEN_CLKOUT_CKEN_DIS;
        if (eClkMgrCurClkSrc != AM_HAL_CLKMGR_CLK_ID_MAX)
        {
            ui32Status = am_hal_clkmgr_clock_release(eClkMgrCurClkSrc, AM_HAL_CLKMGR_USER_ID_CLKOUT);
            if (ui32Status != AM_HAL_STATUS_SUCCESS)
            {
                g_bClkGenFirstClkOutInvoke = false;
                return ui32Status;
            }
        }
    }
    g_bClkGenFirstClkOutInvoke = false;

    //
    // Request clock and enable CLKOUT for clock selected
    //
    if (bEnable)
    {
        // Request for clock needed, and enable the CLKOUT selected
        am_hal_clkmgr_clock_id_e eClkMgrClkSrcSel = am_hal_clkgen_clksrc_get(eClkSelect);
        if (eClkMgrClkSrcSel != AM_HAL_CLKMGR_CLK_ID_MAX)
        {
            am_hal_clkmgr_clock_request(eClkMgrClkSrcSel, AM_HAL_CLKMGR_USER_ID_CLKOUT);
            if (ui32Status != AM_HAL_STATUS_SUCCESS)
            {
                return ui32Status;
            }
        }
        CLKGEN->CLKOUT_b.CKSEL = eClkSelect;
        CLKGEN->CLKOUT_b.CKEN = CLKGEN_CLKOUT_CKEN_EN;
    }

    //
    // Return success status.
    //
    return ui32Status;

} // am_hal_clkgen_clkout_enable()

// ****************************************************************************
//
//  am_hal_clkgen_hfrcadj_target_calculate()
//  Calculate HFADJ adjust target according to the parameters provided.
//  Note: Avoid using this API directly, use Clock Manager to control HFADJ
//        with automatic configuration for HFRC instead.
//
// ****************************************************************************
uint32_t
am_hal_clkgen_hfrcadj_target_calculate(uint32_t ui32RefFreq, uint32_t ui32TargetFrequency, uint32_t *pui32AdjTarget)
{
    *pui32AdjTarget = (ui32TargetFrequency / ui32RefFreq);
    return AM_HAL_STATUS_SUCCESS;
}
// ****************************************************************************
//
//  am_hal_clkgen_private_hfrc_force_on()
//  This private function is used to enable/disable of force-on for HFRC
//  oscillator block.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hfrc_force_on(bool bForceOn)
{
    CLKGEN->MISC_b.FRCHFRC = bForceOn ? CLKGEN_MISC_FRCHFRC_FRC: CLKGEN_MISC_FRCHFRC_NOFRC;
    return AM_HAL_STATUS_SUCCESS;
}
// ****************************************************************************
//
//  am_hal_clkgen_private_hfadj_apply()
//  This private function is used to configure and enable HFADJ.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hfadj_apply(uint32_t ui32RegVal)
{
    //
    // Make sure the ENABLE bit is set, and set config into register
    //
    ui32RegVal |= _VAL2FLD(CLKGEN_HFADJ_HFADJEN, CLKGEN_HFADJ_HFADJEN_EN);
    CLKGEN->HFADJ = ui32RegVal;
    return AM_HAL_STATUS_SUCCESS;
}
// ****************************************************************************
//
//  am_hal_clkgen_private_hfadj_disable()
//  This private function is used to disable HFADJ.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hfadj_disable()
{
    CLKGEN->HFADJ_b.HFADJEN = CLKGEN_HFADJ_HFADJEN_DIS;
    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
