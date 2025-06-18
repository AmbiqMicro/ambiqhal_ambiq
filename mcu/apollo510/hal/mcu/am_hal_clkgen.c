// ****************************************************************************
//
//! @file am_hal_clkgen.c
//!
//! @brief Functions for interfacing with the CLKGEN.
//!
//! @addtogroup clkgen4 CLKGEN - Clock Generator
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
#include "am_mcu_apollo.h"
#include "../am_hal_clkmgr_private.h"

uint32_t g_ui32HFRC2ReqCnt = 0;

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
            CLKGEN->OCTRL_b.OSEL = CLKGEN_OCTRL_OSEL_LFRC_DIV2;
            break;

        case AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL:
            CLKGEN->OCTRL_b.OSEL = CLKGEN_OCTRL_OSEL_XT_512Hz;
            break;

        case AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE:
            return am_hal_clkmgr_private_clkgen_hfadj_apply(pArgs);
            break;

        case AM_HAL_CLKGEN_CONTROL_HFADJ_DISABLE:
            return am_hal_clkmgr_private_clkgen_hfadj_disable();
            break;

        case AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE:
            return am_hal_clkmgr_private_clkgen_hf2adj_apply();
            break;

        case AM_HAL_CLKGEN_CONTROL_HF2ADJ_DISABLE:
            return am_hal_clkmgr_private_clkgen_hf2adj_disable();
            break;

        case AM_HAL_CLKGEN_CONTROL_HFRC2_ON_REQ:
            AM_CRITICAL_BEGIN
            //
            // If this is the first request, turn HFRC2 on.
            //
            if (g_ui32HFRC2ReqCnt == 0)
            {
                AM_REGVAL(0x400200c4) &= ~(1 << 5);
                AM_REGVAL(0x400200c4) |=  (1 << 0);
                am_hal_delay_us(5);
            }

            //
            // Increase the request count.
            //
            g_ui32HFRC2ReqCnt++;
            AM_CRITICAL_END
            break;

        case AM_HAL_CLKGEN_CONTROL_HFRC2_OFF_REQ:
            AM_CRITICAL_BEGIN
            if (g_ui32HFRC2ReqCnt > 0)
            {
                //
                // Decrease the request count.
                //
                g_ui32HFRC2ReqCnt--;
            }

            if (g_ui32HFRC2ReqCnt == 0)
            {
                AM_REGVAL(0x400200c4) |=  (1 << 5);
            }
            AM_CRITICAL_END
            break;

        case AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE:
            CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_ENABLE;
            if ( CLKGEN->DISPCLKCTRL_b.DISPCLKSEL > CLKGEN_DISPCLKCTRL_DISPCLKSEL_DPHYPLL )
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            else
            {
                CLKGEN->DISPCLKCTRL_b.DCCLKEN = 1;
                return am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_DC);
            }
            break;

        case AM_HAL_CLKGEN_CONTROL_DCCLK_DISABLE:
            CLKGEN->DISPCLKCTRL_b.DCCLKEN = 0;
            CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_DISABLE;
            return am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_DC);
            break;

        case AM_HAL_CLKGEN_CONTROL_DBICLKSEL_DBIB_CLK:
            CLKGEN->DISPCLKCTRL_b.DBICLKSEL = CLKGEN_DISPCLKCTRL_DBICLKSEL_DBIB_CLK;
            break;

        case AM_HAL_CLKGEN_CONTROL_DBICLKSEL_FORMAT_CLK:
            CLKGEN->DISPCLKCTRL_b.DBICLKSEL = CLKGEN_DISPCLKCTRL_DBICLKSEL_FORMAT_CLK;
            break;

        case AM_HAL_CLKGEN_CONTROL_DBICLKSEL_PLL_CLK:
            CLKGEN->DISPCLKCTRL_b.DBICLKSEL = CLKGEN_DISPCLKCTRL_DBICLKSEL_PLL_CLK;
            break;

        case AM_HAL_CLKGEN_CONTROL_DBICLKDIV2EN_ENABLE:
            CLKGEN->DISPCLKCTRL_b.DBICLKDIV2EN = 1;
            break;

        case AM_HAL_CLKGEN_CONTROL_DBICLKDIV2EN_DISABLE:
            CLKGEN->DISPCLKCTRL_b.DBICLKDIV2EN = 0;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_OFF:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_OFF;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC12:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC12;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC24:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC24;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC48:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC48;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC96;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC192:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_HFRC192;
            break;

        case AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_DPHYPLL:
            CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = CLKGEN_DISPCLKCTRL_DISPCLKSEL_DPHYPLL;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLK_ENABLE:
            CLKGEN->DISPCLKCTRL_b.PLLCLKEN = 1;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLK_DISABLE:
            CLKGEN->DISPCLKCTRL_b.PLLCLKEN = 0;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_OFF:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_OFF;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC24:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC_24MHz;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC12:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_HFRC_12MHz;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFXT:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_XTAL_HS;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFXT_DIV2:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_XTAL_HS_div2;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_EXTREFCLK:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_EXTREFCLK;
            break;

        case AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_EXTREFCLK_DIV2:
            CLKGEN->DISPCLKCTRL_b.PLLCLKSEL = CLKGEN_DISPCLKCTRL_PLLCLKSEL_EXTREFCLK_div2;
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
#ifdef AM_HAL_CLKMGR_MANAGE_XTAL_LS
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
        case AM_HAL_CLKGEN_CLKOUT_XTAL_0_015:
        // Not Autoenabled ("NE")
        case AM_HAL_CLKGEN_CLKOUT_XTALNE_32768:
        case AM_HAL_CLKGEN_CLKOUT_XTALNE_2048:
            return AM_HAL_CLKMGR_CLK_ID_XTAL_LS;
#endif
        // LFRC
        case AM_HAL_CLKGEN_CLKOUT_LFRC_NOMINAL:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV2:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV32:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV512:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV32K:
        case AM_HAL_CLKGEN_CLKOUT_LFRC_DIV1M:
        // Uncalibrated LFRC
        case AM_HAL_CLKGEN_CLKOUT_ULFRC_DIV16:
        case AM_HAL_CLKGEN_CLKOUT_ULFRC_DIV128:
        case AM_HAL_CLKGEN_CLKOUT_ULFRC_DIV1K:
        case AM_HAL_CLKGEN_CLKOUT_ULFRC_DIV4K:
        case AM_HAL_CLKGEN_CLKOUT_ULFRC_DIV1M:
        case AM_HAL_CLKGEN_CLKOUT_LFRCNE:
        case AM_HAL_CLKGEN_CLKOUT_LFRCNE_32:
            return AM_HAL_CLKMGR_CLK_ID_LFRC;
        // HFRC2
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_250M:
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_125M:
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_31M:
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_16M:
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_8M:
        case AM_HAL_CLKGEN_CLKOUT_HFRC2_4M:
            return AM_HAL_CLKMGR_CLK_ID_HFRC2;
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
    if ( !bEnable )
    {
        CLKGEN->CLKOUT_b.CKEN = 0;
        am_hal_clkmgr_clock_release(am_hal_clkgen_clksrc_get(CLKGEN->CLKOUT_b.CKSEL), AM_HAL_CLKMGR_USER_ID_CLKOUT);
    }

    //
    // Do a basic validation of the eClkSelect parameter.
    // Not every value in the range is valid, but at least this simple check
    // provides a reasonable chance that the parameter is valid.
    //
    if ( eClkSelect <= (am_hal_clkgen_clkout_e)AM_HAL_CLKGEN_CLKOUT_MAX )
    {
        //
        // Are we actually changing the frequency?
        //
        if ( CLKGEN->CLKOUT_b.CKSEL != eClkSelect )
        {
            //
            // Disable before changing the clock
            //
            CLKGEN->CLKOUT_b.CKEN = CLKGEN_CLKOUT_CKEN_DIS;
            if ( bEnable )
            {
                am_hal_clkmgr_clock_release(am_hal_clkgen_clksrc_get(CLKGEN->CLKOUT_b.CKSEL), AM_HAL_CLKMGR_USER_ID_CLKOUT);
            }
            //
            // Set the new clock select
            //
            CLKGEN->CLKOUT_b.CKSEL = eClkSelect;
            if ( bEnable )
            {
                if ( (eClkSelect != AM_HAL_CLKGEN_CLKOUT_RTC_1HZ) && (eClkSelect != AM_HAL_CLKGEN_CLKOUT_CG_100) )
                {
                    am_hal_clkmgr_clock_request(am_hal_clkgen_clksrc_get(eClkSelect), AM_HAL_CLKMGR_USER_ID_CLKOUT);
                }
            }
        }

        //
        // Enable/disable as requested.
        //
        CLKGEN->CLKOUT_b.CKEN = bEnable ? CLKGEN_CLKOUT_CKEN_EN : CLKGEN_CLKOUT_CKEN_DIS;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_clkgen_clkout_enable()

//! @cond CLKGEN_PRIVATE_FUNC

// ****************************************************************************
//
//  am_hal_clkgen_hfrc2adj_ratio_calculate()
//  Calculate HF2ADJ adjust ratio according to the parameters provided.
//  Note: Avoid using this API directly, use Clock Manager to control HF2ADJ
//        with automatic configuration for HFRC2 instead.
//
// ****************************************************************************
uint32_t
am_hal_clkgen_hfrc2adj_ratio_calculate(uint32_t ui32RefFreq, uint32_t ui32TargetFreq, am_hal_clkgen_hfrc2adj_refclk_div_e eXtalDiv, uint32_t *pui32AdjRatio)
{
    //
    // Calculate target frequency ratio to reference frequency
    //
    float fRatio = (float)ui32TargetFreq;
    fRatio /= (ui32RefFreq / (1 << (uint8_t)eXtalDiv));

    //
    // Convert to register representation
    //
    *pui32AdjRatio = (uint32_t)(fRatio * (1 << 15));

    return AM_HAL_STATUS_SUCCESS;
}

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

// ****************************************************************************
//
//  am_hal_clkgen_private_hfrc2_force_on()
//  This private function is used to enable/disable of force-on for HFRC2
//  oscillator block.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hfrc2_force_on(bool bForceOn)
{
    if (bForceOn)
    {
        if (CLKGEN->MISC_b.FRCHFRC2 != CLKGEN_MISC_FRCHFRC2_FRC)
        {
            CLKGEN->MISC_b.FRCHFRC2 = CLKGEN_MISC_FRCHFRC2_FRC;
            return am_hal_delay_us_status_check(100,
                                (uint32_t)&CLKGEN->CLOCKENSTAT,
                                CLKGEN_CLOCKENSTAT_HFRC2READY_Msk,
                                CLKGEN_CLOCKENSTAT_HFRC2READY_Msk,
                                true);
        }
    }
    else
    {

        CLKGEN->MISC_b.FRCHFRC2 = CLKGEN_MISC_FRCHFRC2_NOFRC;
    }

    return AM_HAL_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  am_hal_clkgen_private_hf2adj_apply()
//  This private function is used to configure and enable HFADJ2.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hf2adj_apply(am_hal_clkgen_hfrc2adj_t* psHFRC2Adj)
{
    //
    // Check pointer validity
    //
    if (psHFRC2Adj == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Set HF2ADJ configuration into CLKGEN registers
    //
    CLKGEN->HF2ADJ1_b.HF2ADJTRIMEN       = CLKGEN_HF2ADJ1_HF2ADJTRIMEN_TRIM_EN7;
    CLKGEN->HF2ADJ0_b.HF2ADJXTHSMUXSEL   = (uint32_t)psHFRC2Adj->eRefClkSel;
    CLKGEN->HF2ADJ2_b.HF2ADJXTALDIVRATIO = (uint32_t)psHFRC2Adj->eRefClkDiv;
    CLKGEN->HF2ADJ2_b.HF2ADJRATIO        = psHFRC2Adj->ui32AdjRatio;
    CLKGEN->HF2ADJ0_b.HF2ADJEN           = CLKGEN_HF2ADJ0_HF2ADJEN_EN;

    return AM_HAL_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  am_hal_clkgen_private_hf2adj_disable()
//  This private function is used to disable HFADJ2.
//  Note: This API is inteded for use by HAL only. Do not call this API from
//  Application/BSP.
//
// ****************************************************************************
uint32_t am_hal_clkgen_private_hf2adj_disable()
{
    CLKGEN->HF2ADJ0_b.HF2ADJEN = CLKGEN_HF2ADJ0_HF2ADJEN_DIS;
    return AM_HAL_STATUS_SUCCESS;
}
//! @endcond CLKGEN_PRIVATE_FUNC


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
