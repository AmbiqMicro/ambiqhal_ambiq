// ****************************************************************************
//
//! @file am_hal_crm.c
//!
//! @brief Functions for interfacing with the CRM.
//!
//! @addtogroup CRM - Peripheral Clock and Reset Controls
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
#include "am_hal_crm_private.h"

#define CRM_REG_DEFAULT             0x00000008

#define ADCCRM_REG_DEFAULT          CRM_REG_DEFAULT
#define APBDISPCRM_REG_DEFAULT      CRM_REG_DEFAULT
#define APBDISPPHYCRM_REG_DEFAULT   CRM_REG_DEFAULT
#define APBUARTCRM_REG_DEFAULT      CRM_REG_DEFAULT
#define APBI2SCRM_REG_DEFAULT       CRM_REG_DEFAULT
#define APBPDMCRM_REG_DEFAULT       CRM_REG_DEFAULT
#define APBUSBCRM_REG_DEFAULT       CRM_REG_DEFAULT
#define DISPCLKCRM_REG_DEFAULT      CRM_REG_DEFAULT
#define DPHYPLLREFCRM_REG_DEFAULT   CRM_REG_DEFAULT
#define DSPPDM0CRM_REG_DEFAULT      CRM_REG_DEFAULT
#define I2S0MCLKCRM_REG_DEFAULT     CRM_REG_DEFAULT
#define I2S0MCLKOUTCRM_REG_DEFAULT  CRM_REG_DEFAULT
#define I2S0REFCLKCRM_REG_DEFAULT   CRM_REG_DEFAULT
#define UART0HFCRM_REG_DEFAULT      CRM_REG_DEFAULT
#define UART1HFCRM_REG_DEFAULT      CRM_REG_DEFAULT
#define USBREFCLKCRM_REG_DEFAULT    CRM_REG_DEFAULT

uint32_t am_hal_crm_initialize(void)
{
    //
    // Set CRM registers to default
    //
    CRM->ADCCRM           = ADCCRM_REG_DEFAULT        ;
    CRM->APBDISPCRM       = APBDISPCRM_REG_DEFAULT    ;
    CRM->APBDISPPHYCRM    = APBDISPPHYCRM_REG_DEFAULT ;
    CRM->APBUARTCRM       = APBUARTCRM_REG_DEFAULT    ;
    CRM->APBI2SCRM        = APBI2SCRM_REG_DEFAULT     ;
    CRM->APBPDMCRM        = APBPDMCRM_REG_DEFAULT     ;
    CRM->APBUSBCRM        = APBUSBCRM_REG_DEFAULT     ;
    CRM->DISPCLKCRM       = DISPCLKCRM_REG_DEFAULT    ;
    CRM->DPHYPLLREFCRM    = DPHYPLLREFCRM_REG_DEFAULT ;
    CRM->DSPPDM0CRM       = DSPPDM0CRM_REG_DEFAULT    ;
    CRM->I2S0MCLKCRM      = I2S0MCLKCRM_REG_DEFAULT   ;
    CRM->I2S0MCLKOUTCRM   = I2S0MCLKOUTCRM_REG_DEFAULT;
    CRM->I2S0REFCLKCRM    = I2S0REFCLKCRM_REG_DEFAULT ;
    CRM->UART0HFCRM       = UART0HFCRM_REG_DEFAULT    ;
    CRM->UART1HFCRM       = UART1HFCRM_REG_DEFAULT    ;
    CRM->USBREFCLKCRM     = USBREFCLKCRM_REG_DEFAULT  ;

    return AM_HAL_STATUS_SUCCESS;
}

#define CHECK_CLK_ACTIVE_TIMEOUT 2000

uint32_t am_hal_crm_control_ADC_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->ADCCRM |= CRM_ADCCRM_ADCCLKEN_Msk;
    }
    else
    {
        CRM->ADCCRM &= ~(CRM_ADCCRM_ADCCLKEN_Msk);
    }

    am_hal_delay_us(1);

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_ADC(am_hal_crm_adc_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32ADCCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_ADCCRM_ADCCLKDIV_Msk >> CRM_ADCCRM_ADCCLKDIV_Pos) ||
        eClkSel > CRM_ADCCRM_ADCCLKSEL_PLLPOSTDIV)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32ADCCRM = CRM->ADCCRM;
    if (ui32ADCCRM | CRM_ADCCRM_ADCCLKEN_Msk)
    {
        bClockOn = true;
        am_hal_crm_control_ADC_CLOCK_SET(false);
    }

    ui32ADCCRM  &= ~(CRM_ADCCRM_ADCCLKSEL_Msk | CRM_ADCCRM_ADCCLKDIV_Msk | CRM_ADCCRM_ADCCLKEN_Msk);
    ui32ADCCRM  |= ui32Divider << CRM_ADCCRM_ADCCLKDIV_Pos;
    ui32ADCCRM  |= eClkSel << CRM_ADCCRM_ADCCLKSEL_Pos;
    ui32ADCCRM  |= CRM_ADCCRM_ADCRSTN_Msk;
    CRM->ADCCRM  = ui32ADCCRM;

    if (bClockOn)
    {
        am_hal_crm_control_ADC_CLOCK_SET(true);
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_ADC_RESET(bool bEnable)
{
    CRM->ADCCRM_b.ADCRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DC_CLOCK_SET(bool bEnable)
{
    CRM->APBDISPCRM_b.APBDISPRSTN  = 1;
    CRM->APBDISPCRM_b.APBDISPCLKEN = bEnable;

    uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                       (uint32_t)&CRM->APBDISPCRM,
                                                       CRM_APBDISPCRM_APBDISPCLKACTIVE_Msk,
                                                       bEnable << CRM_APBDISPCRM_APBDISPCLKACTIVE_Pos,
                                                       true);
    if (ui32Status)
    {
        return ui32Status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DC_RESET(bool bEnable)
{
    CRM->APBDISPCRM_b.APBDISPRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DSI_CLOCK_SET(bool bEnable)
{
    CRM->APBDISPPHYCRM_b.APBDISPPHYRSTN  = 1;
    CRM->APBDISPPHYCRM_b.APBDISPPHYCLKEN = bEnable;

    uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                       (uint32_t)&CRM->APBDISPPHYCRM,
                                                       CRM_APBDISPPHYCRM_APBDISPPHYCLKACTIVE_Msk,
                                                       bEnable << CRM_APBDISPPHYCRM_APBDISPPHYCLKACTIVE_Pos,
                                                       true);
    if (ui32Status)
    {
        return ui32Status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DSI_RESET(bool bEnable)
{
    CRM->APBDISPPHYCRM_b.APBDISPPHYRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

#define UART0STATMASK 0x1
#define UART1STATMASK 0x2

static uint32_t ui32UARTStat = 0;

uint32_t am_hal_crm_control_UART0_CLOCK_SET(bool bEnable)
{
    if (ui32UARTStat == 0)
    {
        if (bEnable)
        {
            AM_CRITICAL_BEGIN
            CRM->APBUARTCRM_b.APBUARTRSTN  = 1;
            CRM->APBUARTCRM_b.APBUARTCLKEN = bEnable;
            AM_CRITICAL_END
            uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                               (uint32_t)&CRM->APBUARTCRM,
                                                               CRM_APBUARTCRM_APBUARTCLKACTIVE_Msk,
                                                               bEnable << CRM_APBUARTCRM_APBUARTCLKACTIVE_Pos,
                                                               true);
            if (ui32Status)
            {
                return ui32Status;
            }
            AM_CRITICAL_BEGIN
            ui32UARTStat |= UART0STATMASK;
            AM_CRITICAL_END
        }
    }
    else
    {
        if (!bEnable)
        {
            if (ui32UARTStat == UART0STATMASK)
            {
                AM_CRITICAL_BEGIN
                CRM->APBUARTCRM_b.APBUARTRSTN  = 1;
                CRM->APBUARTCRM_b.APBUARTCLKEN = bEnable;
                AM_CRITICAL_END
                uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                                   (uint32_t)&CRM->APBUARTCRM,
                                                                   CRM_APBUARTCRM_APBUARTCLKACTIVE_Msk,
                                                                   bEnable << CRM_APBUARTCRM_APBUARTCLKACTIVE_Pos,
                                                                   true);
                if (ui32Status)
                {
                    return ui32Status;
                }
                ui32UARTStat = 0;
            }
            else
            {
                AM_CRITICAL_BEGIN
                ui32UARTStat &= ~UART0STATMASK;
                AM_CRITICAL_END
            }
        }
        else
        {
            AM_CRITICAL_BEGIN
            ui32UARTStat |= UART0STATMASK;
            AM_CRITICAL_END
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_UART1_CLOCK_SET(bool bEnable)
{
    if (ui32UARTStat == 0)
    {
        if (bEnable)
        {
            AM_CRITICAL_BEGIN
            CRM->APBUARTCRM_b.APBUARTRSTN  = 1;
            CRM->APBUARTCRM_b.APBUARTCLKEN = bEnable;
            AM_CRITICAL_END
            uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                               (uint32_t)&CRM->APBUARTCRM,
                                                               CRM_APBUARTCRM_APBUARTCLKACTIVE_Msk,
                                                               bEnable << CRM_APBUARTCRM_APBUARTCLKACTIVE_Pos,
                                                               true);
            if (ui32Status)
            {
                return ui32Status;
            }
            AM_CRITICAL_BEGIN
            ui32UARTStat |= UART1STATMASK;
            AM_CRITICAL_END
        }
    }
    else
    {
        if (!bEnable)
        {
            if (ui32UARTStat == UART1STATMASK)
            {
                AM_CRITICAL_BEGIN
                CRM->APBUARTCRM_b.APBUARTRSTN  = 1;
                CRM->APBUARTCRM_b.APBUARTCLKEN = bEnable;
                AM_CRITICAL_END
                uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                                   (uint32_t)&CRM->APBUARTCRM,
                                                                   CRM_APBUARTCRM_APBUARTCLKACTIVE_Msk,
                                                                   bEnable << CRM_APBUARTCRM_APBUARTCLKACTIVE_Pos,
                                                                   true);
                if (ui32Status)
                {
                    return ui32Status;
                }
                ui32UARTStat = 0;
            }
            else
            {
                AM_CRITICAL_BEGIN
                ui32UARTStat &= ~UART1STATMASK;
                AM_CRITICAL_END
            }
        }
        else
        {
            AM_CRITICAL_BEGIN
            ui32UARTStat |= UART1STATMASK;
            AM_CRITICAL_END
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_UART0_RESET(bool bEnable)
{
    if (ui32UARTStat != UART0STATMASK)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    AM_CRITICAL_BEGIN

    CRM->APBUARTCRM_b.APBUARTRSTN = !bEnable;

    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_UART1_RESET(bool bEnable)
{
    if (ui32UARTStat != UART1STATMASK)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    AM_CRITICAL_BEGIN

    CRM->APBUARTCRM_b.APBUARTRSTN = !bEnable;

    AM_CRITICAL_END

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_I2S0_CLOCK_SET(bool bEnable)
{
    CRM->APBI2SCRM_b.APBI2SRSTN  = 1;
    CRM->APBI2SCRM_b.APBI2SCLKEN = bEnable;

    uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                       (uint32_t)&CRM->APBI2SCRM,
                                                       CRM_APBI2SCRM_APBI2SCLKACTIVE_Msk,
                                                       bEnable << CRM_APBI2SCRM_APBI2SCLKACTIVE_Pos,
                                                       true);
    if (ui32Status)
    {
        return ui32Status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_I2S0(am_hal_crm_apbi2s_clksel_e eClkSel)
{
    uint32_t ui32APBI2SCRM;
    bool     bClockOn = false;

    ui32APBI2SCRM = CRM->APBI2SCRM;
    if (ui32APBI2SCRM | CRM_APBI2SCRM_APBI2SCLKEN_Msk)
    {
        bClockOn = true;
        am_hal_crm_control_I2S0_CLOCK_SET(false);
    }

    ui32APBI2SCRM  &= ~(CRM_APBI2SCRM_APBI2SCLKSEL_Msk | CRM_APBI2SCRM_APBI2SCLKEN_Msk);
    ui32APBI2SCRM  |= eClkSel << CRM_APBI2SCRM_APBI2SCLKSEL_Pos;
    ui32APBI2SCRM  |= CRM_APBI2SCRM_APBI2SRSTN_Msk;
    CRM->APBI2SCRM  = ui32APBI2SCRM;

    if (bClockOn)
    {
        am_hal_crm_control_I2S0_CLOCK_SET(true);
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_I2S0_RESET(bool bEnable)
{
    CRM->APBI2SCRM_b.APBI2SRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_PDM0_CLOCK_SET(bool bEnable)
{
    CRM->APBPDMCRM_b.APBPDMRSTN  = 1;
    CRM->APBPDMCRM_b.APBPDMCLKEN = bEnable;

    uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                       (uint32_t)&CRM->APBPDMCRM,
                                                       CRM_APBPDMCRM_APBPDMCLKACTIVE_Msk,
                                                       bEnable << CRM_APBPDMCRM_APBPDMCLKACTIVE_Pos,
                                                       true);
    if (ui32Status)
    {
        return ui32Status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_PDM0_RESET(bool bEnable)
{
    CRM->APBPDMCRM_b.APBPDMRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_USB_CLOCK_SET(bool bEnable)
{
    CRM->APBUSBCRM_b.APBUSBRSTN  = 1;
    CRM->APBUSBCRM_b.APBUSBCLKEN = bEnable;

    uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                       (uint32_t)&CRM->APBUSBCRM,
                                                       CRM_APBUSBCRM_APBUSBCLKACTIVE_Msk,
                                                       bEnable << CRM_APBUSBCRM_APBUSBCLKACTIVE_Pos,
                                                       true);
    if (ui32Status)
    {
        return ui32Status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_USB_RESET(bool bEnable)
{
    CRM->APBUSBCRM_b.APBUSBRSTN = !bEnable;

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DISPCLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->DISPCLKCRM |= CRM_DISPCLKCRM_DISPCLKCLKEN_Msk;
    }
    else
    {
        CRM->DISPCLKCRM &= ~CRM_DISPCLKCRM_DISPCLKCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->DISPCLKCRM,
                                                           CRM_DISPCLKCRM_DISPCLKCLKACTIVE_Msk,
                                                           bEnable << CRM_DISPCLKCRM_DISPCLKCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_DISPCLK(am_hal_crm_dispclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32DISPCLKCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_DISPCLKCRM_DISPCLKCLKDIV_Msk >> CRM_DISPCLKCRM_DISPCLKCLKDIV_Pos) ||
        eClkSel > CRM_DISPCLKCRM_DISPCLKCLKSEL_dphy_pll_clk)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32DISPCLKCRM = CRM->DISPCLKCRM;
    if (ui32DISPCLKCRM | CRM_DISPCLKCRM_DISPCLKCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_DISPCLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32DISPCLKCRM &= ~(CRM_DISPCLKCRM_DISPCLKCLKSEL_Msk | CRM_DISPCLKCRM_DISPCLKCLKDIV_Msk |
                        CRM_DISPCLKCRM_DISPCLKCLKEN_Msk);
    ui32DISPCLKCRM |= ui32Divider << CRM_DISPCLKCRM_DISPCLKCLKDIV_Pos;
    ui32DISPCLKCRM |= eClkSel << CRM_DISPCLKCRM_DISPCLKCLKSEL_Pos;
    CRM->DISPCLKCRM = ui32DISPCLKCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_DISPCLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_DSIPLLREFCLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->DPHYPLLREFCRM |= CRM_DPHYPLLREFCRM_DPHYPLLREFCLKEN_Msk;
    }
    else
    {
        CRM->DPHYPLLREFCRM &= ~CRM_DPHYPLLREFCRM_DPHYPLLREFCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->DPHYPLLREFCRM,
                                                           CRM_DPHYPLLREFCRM_DPHYPLLREFCLKACTIVE_Msk,
                                                           bEnable << CRM_DPHYPLLREFCRM_DPHYPLLREFCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_DSIPLLREFCLK(am_hal_crm_dsipllrefclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32DPHYPLLREFCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_DPHYPLLREFCRM_DPHYPLLREFCLKDIV_Msk >> CRM_DPHYPLLREFCRM_DPHYPLLREFCLKDIV_Pos) ||
        eClkSel > CRM_DPHYPLLREFCRM_DPHYPLLREFCLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32DPHYPLLREFCRM = CRM->DPHYPLLREFCRM;
    if (ui32DPHYPLLREFCRM | CRM_DPHYPLLREFCRM_DPHYPLLREFCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_DSIPLLREFCLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32DPHYPLLREFCRM &= ~(CRM_DPHYPLLREFCRM_DPHYPLLREFCLKSEL_Msk | CRM_DPHYPLLREFCRM_DPHYPLLREFCLKDIV_Msk |
                           CRM_DPHYPLLREFCRM_DPHYPLLREFCLKEN_Msk);
    ui32DPHYPLLREFCRM |= ui32Divider << CRM_DPHYPLLREFCRM_DPHYPLLREFCLKDIV_Pos;
    ui32DPHYPLLREFCRM |= eClkSel << CRM_DPHYPLLREFCRM_DPHYPLLREFCLKSEL_Pos;
    CRM->DPHYPLLREFCRM = ui32DPHYPLLREFCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_DSIPLLREFCLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_PDM0CLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->DSPPDM0CRM |= CRM_DSPPDM0CRM_DSPPDM0CLKEN_Msk;
    }
    else
    {
        CRM->DSPPDM0CRM &= ~CRM_DSPPDM0CRM_DSPPDM0CLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->DSPPDM0CRM,
                                                           CRM_DSPPDM0CRM_DSPPDM0CLKACTIVE_Msk,
                                                           bEnable << CRM_DSPPDM0CRM_DSPPDM0CLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_PDM0CLK(am_hal_crm_pdmclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32DSPPDM0CRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_DSPPDM0CRM_DSPPDM0CLKDIV_Msk >> CRM_DSPPDM0CRM_DSPPDM0CLKDIV_Pos) ||
        eClkSel > CRM_DSPPDM0CRM_DSPPDM0CLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32DSPPDM0CRM  = CRM->DSPPDM0CRM;
    if (ui32DSPPDM0CRM | CRM_DSPPDM0CRM_DSPPDM0CLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_PDM0CLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32DSPPDM0CRM &= ~(CRM_DSPPDM0CRM_DSPPDM0CLKSEL_Msk | CRM_DSPPDM0CRM_DSPPDM0CLKDIV_Msk |
                        CRM_DSPPDM0CRM_DSPPDM0CLKEN_Msk);
    ui32DSPPDM0CRM |= ui32Divider << CRM_DSPPDM0CRM_DSPPDM0CLKDIV_Pos;
    ui32DSPPDM0CRM |= eClkSel << CRM_DSPPDM0CRM_DSPPDM0CLKSEL_Pos;
    CRM->DSPPDM0CRM = ui32DSPPDM0CRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_PDM0CLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_I2S0MCLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->I2S0MCLKCRM |= CRM_I2S0MCLKCRM_I2S0MCLKCLKEN_Msk;
    }
    else
    {
        CRM->I2S0MCLKCRM &= ~CRM_I2S0MCLKCRM_I2S0MCLKCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->I2S0MCLKCRM,
                                                           CRM_I2S0MCLKCRM_I2S0MCLKCLKACTIVE_Msk,
                                                           bEnable << CRM_I2S0MCLKCRM_I2S0MCLKCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_I2S0MCLK(am_hal_crm_i2smclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32I2S0MCLKCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_I2S0MCLKCRM_I2S0MCLKCLKDIV_Msk >> CRM_I2S0MCLKCRM_I2S0MCLKCLKDIV_Pos) ||
        eClkSel > CRM_I2S0MCLKCRM_I2S0MCLKCLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32I2S0MCLKCRM = CRM->I2S0MCLKCRM;
    if (ui32I2S0MCLKCRM | CRM_I2S0MCLKCRM_I2S0MCLKCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_I2S0MCLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32I2S0MCLKCRM &= ~(CRM_I2S0MCLKCRM_I2S0MCLKCLKSEL_Msk | CRM_I2S0MCLKCRM_I2S0MCLKCLKDIV_Msk |
                         CRM_I2S0MCLKCRM_I2S0MCLKCLKEN_Msk);
    ui32I2S0MCLKCRM |= ui32Divider << CRM_I2S0MCLKCRM_I2S0MCLKCLKDIV_Pos;
    ui32I2S0MCLKCRM |= eClkSel << CRM_I2S0MCLKCRM_I2S0MCLKCLKSEL_Pos;
    CRM->I2S0MCLKCRM = ui32I2S0MCLKCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_I2S0MCLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_I2S0MCLKOUT_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->I2S0MCLKOUTCRM |= CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKEN_Msk;
    }
    else
    {
        CRM->I2S0MCLKOUTCRM &= ~CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->I2S0MCLKOUTCRM,
                                                           CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKACTIVE_Msk,
                                                           bEnable << CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_I2S0MCLKOUT(am_hal_crm_i2smclkout_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32I2S0MCLKOUTCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKDIV_Msk >> CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKDIV_Pos) ||
        eClkSel > CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32I2S0MCLKOUTCRM = CRM->I2S0MCLKOUTCRM;
    if (ui32I2S0MCLKOUTCRM | CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_I2S0MCLKOUT_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32I2S0MCLKOUTCRM &= ~(CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKSEL_Msk | CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKDIV_Msk |
                            CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKEN_Msk);
    ui32I2S0MCLKOUTCRM |= ui32Divider << CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKDIV_Pos;
    ui32I2S0MCLKOUTCRM |= eClkSel << CRM_I2S0MCLKOUTCRM_I2S0MCLKOUTCLKSEL_Pos;
    CRM->I2S0MCLKOUTCRM = ui32I2S0MCLKOUTCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_I2S0MCLKOUT_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_I2S0REFCLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->I2S0REFCLKCRM |= CRM_I2S0REFCLKCRM_I2S0REFCLKCLKEN_Msk;
    }
    else
    {
        CRM->I2S0REFCLKCRM &= ~CRM_I2S0REFCLKCRM_I2S0REFCLKCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->I2S0REFCLKCRM,
                                                           CRM_I2S0REFCLKCRM_I2S0REFCLKCLKACTIVE_Msk,
                                                           bEnable << CRM_I2S0REFCLKCRM_I2S0REFCLKCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_I2S0REFCLK(am_hal_crm_i2srefclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32I2S0REFCLKCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_I2S0REFCLKCRM_I2S0REFCLKCLKDIV_Msk >> CRM_I2S0REFCLKCRM_I2S0REFCLKCLKDIV_Pos) ||
        eClkSel > CRM_I2S0REFCLKCRM_I2S0REFCLKCLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32I2S0REFCLKCRM  = CRM->I2S0REFCLKCRM;
    if (ui32I2S0REFCLKCRM | CRM_I2S0REFCLKCRM_I2S0REFCLKCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_I2S0REFCLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32I2S0REFCLKCRM &= ~(CRM_I2S0REFCLKCRM_I2S0REFCLKCLKSEL_Msk | CRM_I2S0REFCLKCRM_I2S0REFCLKCLKDIV_Msk |
                           CRM_I2S0REFCLKCRM_I2S0REFCLKCLKEN_Msk);
    ui32I2S0REFCLKCRM |= ui32Divider << CRM_I2S0REFCLKCRM_I2S0REFCLKCLKDIV_Pos;
    ui32I2S0REFCLKCRM |= eClkSel << CRM_I2S0REFCLKCRM_I2S0REFCLKCLKSEL_Pos;
    CRM->I2S0REFCLKCRM = ui32I2S0REFCLKCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_I2S0REFCLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_UART0HF_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->UART0HFCRM |= CRM_UART0HFCRM_UART0HFCLKEN_Msk;
    }
    else
    {
        CRM->UART0HFCRM &= ~CRM_UART0HFCRM_UART0HFCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->UART0HFCRM,
                                                           CRM_UART0HFCRM_UART0HFCLKACTIVE_Msk,
                                                           bEnable << CRM_UART0HFCRM_UART0HFCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_UART0HF(am_hal_crm_uart0hf_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32UART0HFCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_UART0HFCRM_UART0HFCLKDIV_Msk >> CRM_UART0HFCRM_UART0HFCLKDIV_Pos) ||
        eClkSel > CRM_UART0HFCRM_UART0HFCLKSEL_PLLPOSTDIV)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32UART0HFCRM  = CRM->UART0HFCRM;
    if (ui32UART0HFCRM & CRM_UART0HFCRM_UART0HFCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_UART0HF_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32UART0HFCRM &= ~(CRM_UART0HFCRM_UART0HFCLKSEL_Msk | CRM_UART0HFCRM_UART0HFCLKDIV_Msk |
                        CRM_UART0HFCRM_UART0HFCLKEN_Msk);
    ui32UART0HFCRM |= ui32Divider << CRM_UART0HFCRM_UART0HFCLKDIV_Pos;
    ui32UART0HFCRM |= eClkSel << CRM_UART0HFCRM_UART0HFCLKSEL_Pos;
    CRM->UART0HFCRM = ui32UART0HFCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_UART0HF_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_UART1HF_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->UART1HFCRM |= CRM_UART1HFCRM_UART1HFCLKEN_Msk;
    }
    else
    {
        CRM->UART1HFCRM &= ~CRM_UART1HFCRM_UART1HFCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->UART1HFCRM,
                                                           CRM_UART1HFCRM_UART1HFCLKACTIVE_Msk,
                                                           bEnable << CRM_UART1HFCRM_UART1HFCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_UART1HF(am_hal_crm_uart1hf_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32UART1HFCRM;
    bool     bClockOn = false;

    if (ui32Divider > (CRM_UART1HFCRM_UART1HFCLKDIV_Msk >> CRM_UART1HFCRM_UART1HFCLKDIV_Pos) ||
        eClkSel > CRM_UART1HFCRM_UART1HFCLKSEL_PLLPOSTDIV)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32UART1HFCRM  = CRM->UART1HFCRM;
    if (ui32UART1HFCRM & CRM_UART1HFCRM_UART1HFCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_UART1HF_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    ui32UART1HFCRM &= ~(CRM_UART1HFCRM_UART1HFCLKSEL_Msk | CRM_UART1HFCRM_UART1HFCLKDIV_Msk |
                        CRM_UART1HFCRM_UART1HFCLKEN_Msk);
    ui32UART1HFCRM |= ui32Divider << CRM_UART1HFCRM_UART1HFCLKDIV_Pos;
    ui32UART1HFCRM |= eClkSel << CRM_UART1HFCRM_UART1HFCLKSEL_Pos;
    CRM->UART1HFCRM = ui32UART1HFCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_UART1HF_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_control_USBREFCLK_CLOCK_SET(bool bEnable)
{
    if (bEnable)
    {
        CRM->USBREFCLKCRM |= CRM_USBREFCLKCRM_USBREFCLKCLKEN_Msk;
    }
    else
    {
        CRM->USBREFCLKCRM &= ~CRM_USBREFCLKCRM_USBREFCLKCLKEN_Msk;
        uint32_t ui32Status = am_hal_delay_us_status_check(CHECK_CLK_ACTIVE_TIMEOUT,
                                                           (uint32_t)&CRM->USBREFCLKCRM,
                                                           CRM_USBREFCLKCRM_USBREFCLKCLKACTIVE_Msk,
                                                           bEnable << CRM_USBREFCLKCRM_USBREFCLKCLKACTIVE_Pos,
                                                           true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

uint32_t am_hal_crm_clock_config_USBREFCLK(am_hal_crm_usbrefclk_clksel_e eClkSel, uint32_t ui32Divider)
{
    uint32_t ui32USBREFCLKCRM;
    bool     bClockOn;

    if (ui32Divider > (CRM_USBREFCLKCRM_USBREFCLKCLKDIV_Msk >> CRM_USBREFCLKCRM_USBREFCLKCLKDIV_Pos) ||
        eClkSel > CRM_USBREFCLKCRM_USBREFCLKCLKSEL_EXTREF_CLK)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32USBREFCLKCRM  = CRM->USBREFCLKCRM;
    if (ui32USBREFCLKCRM | CRM_USBREFCLKCRM_USBREFCLKCLKEN_Msk)
    {
        bClockOn = true;
        uint32_t ui32Status = am_hal_crm_control_USBREFCLK_CLOCK_SET(false);
        if (ui32Status)
        {
            return ui32Status;
        }
    }
    ui32USBREFCLKCRM &= ~(CRM_USBREFCLKCRM_USBREFCLKCLKSEL_Msk | CRM_USBREFCLKCRM_USBREFCLKCLKDIV_Msk |
                          CRM_USBREFCLKCRM_USBREFCLKCLKEN_Msk);
    ui32USBREFCLKCRM |= ui32Divider << CRM_USBREFCLKCRM_USBREFCLKCLKDIV_Pos;
    ui32USBREFCLKCRM |= eClkSel << CRM_USBREFCLKCRM_USBREFCLKCLKSEL_Pos;
    CRM->USBREFCLKCRM = ui32USBREFCLKCRM;

    if (bClockOn)
    {
        uint32_t ui32Status = am_hal_crm_control_USBREFCLK_CLOCK_SET(true);
        if (ui32Status)
        {
            return ui32Status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
