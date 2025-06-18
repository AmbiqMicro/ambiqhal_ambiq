//*****************************************************************************
//
//! @file am_hal_dsi.c
//!
//! @brief Hardware abstraction for the Display Serial Interface
//!
//! @addtogroup dsi DSI - Display Serial Interface
//! @ingroup apollo510_hal
//! @{
//
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
// This is part of revision release_sdk5p0p0-5f68a8286b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include <math.h>

#define FORWARD_REVERSE     1                   // 1 for forward, 4 for reverse.

//
// Transmitted length of any Low-Power state period
//
static const double fTLPXMin = 50;

//
// Time that the transmitter drives LP-11 following a HS burst.
//
static const double fDataTHSExitMin = 100;

//
// Time that the transmitter drives the Clock Lane LP-00 Line state immediately before the HS-0 Line state starting the HS transmission.
//
static const double fCLKTHSPrepareMin = 38;
static const double fCLKTHSPrepareMax = 95;

//
// TCLK-PREPARE + time that the transmitter drives the HS-0 state prior to starting the Clock.
//
static const double fCLKTHSPrepareAddTHSZeroMin = 300;

//
// Time that the transmitter drives the HS-0 state after the last payload clock bit of a HS transmission burst.
//
static const double fCLKTHSTrailMin = 60;

//
// Time that the transmitter drives LP-11 following a HS burst.
//
static const double fCLKTHSExitMin = 100;

//
// The default reference clock frequency(MHz) of Apollo510 DSI.
//
static const uint32_t ui32RefFreq = 12;
//*****************************************************************************
//
//! VDD18 control callback function
//
//*****************************************************************************
am_hal_dsi_external_vdd18_callback external_vdd18_callback;

//*****************************************************************************
//
//! DSI state structure
//
//*****************************************************************************
typedef struct
{
    //
    // DSI frequency trim.
    //
    am_hal_dsi_freq_trim_e                  eTrim;

    //
    // DBI data width
    //
    am_hal_dsi_dbi_width_e                  eWidth;

    //
    // DSI lane(s)
    //
    uint8_t                                 ui8Lanes;

    //
    // This structure members are valid or not.
    //
    bool                                    bisValid;
}am_hal_dsi_state_t;

//*****************************************************************************
//
// Register callback function
//
//*****************************************************************************
uint32_t
am_hal_dsi_register_external_vdd18_callback(const am_hal_dsi_external_vdd18_callback cb)
{
    if (cb != NULL)
    {
        external_vdd18_callback = cb;
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure DSI frequency and timing
//
//*****************************************************************************
uint32_t
am_hal_dsi_timing(uint32_t ui32FreqTrim)
{
    static uint32_t ui32StoredTrim = 0;
    static uint8_t ui8Lpbyteclk = 0;
    //
    // DATA LANE parameters
    //
    static uint8_t ui8DataHSPrep = 0, ui8DataHSZero = 0, ui8DataHSTrail = 0, ui8DataHSExit = 0;
    //
    // CLOCK LANE parameters
    //
    static uint8_t ui8CLKHSPrep = 0, ui8CLKHSZero = 0, ui8CLKHSTrail = 0, ui8CLKHSExit = 0;

    ui32FreqTrim &= 0x7F;

    if (ui32StoredTrim != ui32FreqTrim)
    {
        ui32StoredTrim = ui32FreqTrim;

        uint32_t ui32Frequency;
        //
        // Convert frequency trim value to the actual frequency(MHz).
        //
        if (ui32FreqTrim & 0x40U)
        {
            ui32Frequency = ((ui32FreqTrim & ~0x40U) * 2 + 1) * ui32RefFreq;
        }
        else
        {
            ui32Frequency = ui32FreqTrim * 2 * ui32RefFreq;
        }

        //
        // Calculate the Unit Interval(UI) time(ns)
        //
        double fUI = 500.0 / ui32Frequency;

        //
        // Time that the transmitter drives the Data Lane LP-00 Line state immediately before the HS-0 Line state starting the HS transmission
        //
        double fDataTHSPrepareMin;
        double fDataTHSPrepareMax;

        //
        // THS-PREPARE + time that the transmitter drives the HS-0 state prior to transmitting the Sync sequence.
        //
        double fDataTHSPrepareAddTHSZeroMin;

        //
        // Time that the transmitter drives the flipped differential state after last payload data bit of a HS transmission burst
        //
        double fDataTHSTrailMin;

        //
        // The minimum time of HS prepare is 40 + 4 * UI, the maximum is 85 + 6 * UI.
        //
        fDataTHSPrepareMin = 40 + 4 * fUI;
        fDataTHSPrepareMax = 85 + 6 * fUI;

        //
        // Calculate the average time of HS prepare miminum and maximum.
        //
        double fAVG = (fDataTHSPrepareMin + fDataTHSPrepareMax) / 2;

        //
        // Intermediate temporary variables
        //
        uint8_t ui8H7, ui8I7, ui8J7, ui8K7, ui8L7, ui8M7, ui8R7, ui8S7, ui8J19, ui8L19;
        int8_t i8Temp;
        double fN7, fO7, fP7, fQ7, fK19, fTemp;

        fTemp = fAVG / 8 / fUI;
        i8Temp = (int8_t)floor(fTemp);
        ui8H7 = i8Temp < 0 ? 0 : i8Temp;
        i8Temp = (int8_t)ceil(fTemp);
        ui8K7 = i8Temp < 0 ? 0 : i8Temp;

        fTemp = (fAVG - 8 * fUI * ui8H7) / 2 / fUI;
        i8Temp = (int8_t)ceil(fTemp);
        ui8I7 = i8Temp < 0 ? 0 : (i8Temp >= 4 ? 3 : i8Temp);
        i8Temp = (int8_t)floor(fTemp);
        ui8J7 = i8Temp < 0 ? 0 : (i8Temp >= 4 ? 3 : i8Temp);

        fTemp = (fAVG - 8 * fUI * ui8K7) / 2 / fUI;
        i8Temp = (int8_t)floor(fTemp);
        ui8L7 = i8Temp < 0 ? 0 : (i8Temp >= 4 ? 3 : i8Temp);
        i8Temp = (int8_t)ceil(fTemp);
        ui8M7 = i8Temp < 0 ? 0 : (i8Temp >= 4 ? 3 : i8Temp);

        fN7 = fabs((fAVG - (ui8H7 * 8 * fUI + 2 * fUI * ui8I7)) / fAVG);
        fO7 = fabs((fAVG - (ui8H7 * 8 * fUI + 2 * fUI * ui8J7)) / fAVG);
        fP7 = fabs((fAVG - (ui8K7 * 8 * fUI + 2 * fUI * ui8L7)) / fAVG);
        fQ7 = fabs((fAVG - (ui8K7 * 8 * fUI + 2 * fUI * ui8M7)) / fAVG);

        ui8R7 = (fN7 <= fO7 && fN7 <= fP7 && fN7 <= fQ7) ? ui8H7 - 1 :
                (fO7 <= fN7 && fO7 <= fP7 && fO7 < fQ7) ? ui8H7 - 1 : ui8K7 - 1;

        ui8S7 = (fN7 <= fO7 && fN7 <= fP7 && fN7 <= fQ7) ? ui8I7 :
                (fO7 <= fN7 && fO7 <= fP7 && fO7 < fQ7) ? ui8J7 :
                (fP7 <= fN7 && fP7 < fO7 && fP7 < fQ7) ? ui8L7 : ui8M7;

        //
        // Calculate DPHY DATA timing parameters
        //
        ui8DataHSPrep = ui8S7 << 6 | ui8R7;

        //
        // The minimum value of HS prepare + HS zero is 145 + 10 * UI.
        //
        fDataTHSPrepareAddTHSZeroMin = 145 + 10 * fUI;
        fTemp = (fDataTHSPrepareAddTHSZeroMin - fDataTHSPrepareMin - 8 * fUI) / 8 / fUI;
        ui8DataHSZero = (int8_t)ceil(fTemp);

        //
        // The max(n*8*UI,  60 ns + n*4*UI) value is the minimum time of HS trail
        //
        fDataTHSTrailMin = (FORWARD_REVERSE * 8 * fUI > 60 + FORWARD_REVERSE * 4 * fUI) ? FORWARD_REVERSE * 8 * fUI : 60 + FORWARD_REVERSE * 4 * fUI;
        fTemp = (fDataTHSTrailMin - 2 * 8 * fUI) / 8 / fUI;
        i8Temp = (int8_t)ceil(fTemp);
        ui8DataHSTrail = i8Temp < 0 ? 0 : i8Temp;

        fTemp = (fDataTHSExitMin - 8 * fUI) / 8 / fUI;
        ui8DataHSExit = (int8_t)ceil(fTemp);

        //
        // Calculate DPHY CLK timing parameters
        //
        //
        // Calculate the average time of HS prepare miminum and maximum.
        //
        fAVG = (fCLKTHSPrepareMin + fCLKTHSPrepareMax) / 2.0;
        fTemp = (fAVG - 20 - 2 * fUI) / 8 / fUI;
        i8Temp = (int8_t)floor(fTemp);

        ui8J19 = i8Temp < 0 ? 0 : i8Temp;
        fK19 = ui8J19 * 8 * fUI + 20 + 2 * fUI;
        ui8L19 = fK19 < fCLKTHSPrepareMin ? (fK19 + 2 * fUI > fCLKTHSPrepareMin ? 1 : (fK19 + 2 * 2 * fUI > fCLKTHSPrepareMin ? 2 : (fK19 + 3 * 2 * fUI > fCLKTHSPrepareMin ? 3 : 0))) : 0;
        ui8CLKHSPrep = ui8L19 << 6 | ui8J19;

        fTemp = (fCLKTHSPrepareAddTHSZeroMin - fCLKTHSPrepareMin - fUI - 8 * fUI) / 8 / fUI;
        ui8CLKHSZero = (int8_t)ceil(fTemp);

        fTemp = (fCLKTHSTrailMin - 8 * fUI + fUI) / 8 / fUI;
        i8Temp = (int8_t)ceil(fTemp);
        ui8CLKHSTrail = i8Temp < 0 ? 0 : i8Temp;

        fTemp = (fCLKTHSExitMin - 8 * fUI) / 8 / fUI;
        ui8CLKHSExit = (int8_t)ceil(fTemp);

        //
        // Calculate transmitted length of any Low-Power state period
        //
        fTemp = (fTLPXMin - 8 * fUI) / 8 / fUI;
        i8Temp = (int8_t)ceil(fTemp);
        ui8Lpbyteclk = i8Temp < 0 ? 0 : i8Temp;
    }

    DSI->LPBYTECLK = _VAL2FLD(DSI_LPBYTECLK_VALBYTECLK, ui8Lpbyteclk);

    DSI->DPHYPARAM_b.HSPREP = ui8DataHSPrep;
    DSI->DPHYPARAM_b.HSZERO = ui8DataHSZero;
    DSI->DPHYPARAM_b.HSTRAIL = ui8DataHSTrail;
    DSI->DPHYPARAM_b.HSEXIT = ui8DataHSExit;

    DSI->CLKLANETIMPARM_b.HSPREP = ui8CLKHSPrep;
    DSI->CLKLANETIMPARM_b.HSZERO = ui8CLKHSZero;
    DSI->CLKLANETIMPARM_b.HSTRAIL = ui8CLKHSTrail;
    DSI->CLKLANETIMPARM_b.HSEXIT = ui8CLKHSExit;

    //
    // PLL settings
    //
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x0000007F);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ui32FreqTrim);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_para_config(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim, bool bSendUlpsPattern)
{

    //
    // ui32DSIFuncPrg_REG (DATA_WIDTH, RESERVED, FMT_VIDEO, CH_NO_CM, CH_NO_VM, DATA_LANE_CNT)
    //                     [15:13]     [12:10]   [9:7]      [6:5]     [4:3]     [2:0]
    //
    uint32_t ui32DSIFuncPrg = 0;

    //
    // check number of lanes parameters
    //
    switch (ui8LanesNum)
    {
        case DSI_DSIFUNCPRG_DATALANES_DATAL1:
            //
            // One data lane mode
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, DSI_DSIFUNCPRG_DATALANES_DATAL1);
        break;

        case DSI_DSIFUNCPRG_DATALANES_DATAL2:
            //
            // Two data lanes mode
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_DATALANES, DSI_DSIFUNCPRG_DATALANES_DATAL2);
        break;

        default:
            //
            // Other modes are not yet supported.
            //
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // check DBI bus width parameter
    //
    switch (ui8DBIBusWidth)
    {
        case 8:
            //
            // The 8bit Interface of DBI-Type B output formats
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_8BIT);
        break;

        case 9:
            //
            // The 9bit Interface of DBI-Type B output formats
            //
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_9BIT);
        break;

        case 16:
            //
            // There are two options for 16 bit interface of DBI-Type B output formats. We use option 1 as the default.
            //
            //ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_16BIT0);
            ui32DSIFuncPrg |= _VAL2FLD(DSI_DSIFUNCPRG_REGNAME, DSI_DSIFUNCPRG_REGNAME_16BIT1);

        break;

        default:
            //
            // Other modes are not yet supported.
            //
            return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);

    //
    // write into DSI functional programming register
    //
    DSI->DSIFUNCPRG = ui32DSIFuncPrg;


    //
    // write into HIGH SPEED RECEIVE TIMEOUT REGISTER
    //
    DSI->HSTXTIMEOUT = _VAL2FLD(DSI_HSTXTIMEOUT_MAXDURTOCNT, 0x00FFFFFF);

    //
    //write into LOW POWER RECEIVE TIMEOUT REGISTER
    //
    DSI->LPRXTO = _VAL2FLD(DSI_LPRXTO_TOCHKRVS, 0xFFFF);

    //
    // write into TURN AROUND TIMEOUT REGISTER
    //
    DSI->TURNARNDTO = _VAL2FLD(DSI_TURNARNDTO_TIMOUT, 0x1F);

    //
    // write into DEVICE RESET TIMER REGISTER
    //
    DSI->DEVICERESETTIMER = _VAL2FLD(DSI_DEVICERESETTIMER_TIMOUT, 0xFF);

    //
    // write into HIGH TO LOW SWITCH COUNT REGISTER
    //
    DSI->DATALANEHILOSWCNT = _VAL2FLD(DSI_DATALANEHILOSWCNT_DATALHLSWCNT, 0xFFFF);
    DSI->INITCNT = _VAL2FLD(DSI_INITCNT_MSTR, 0x7d0);

    DSI->CLKEOT = _VAL2FLD(DSI_CLKEOT_CLOCK, 1);
    DSI->CLKEOT |= _VAL2FLD(DSI_CLKEOT_EOT, 1);
    am_hal_dsi_timing(ui32FreqTrim);
    DSI->AFETRIM2 = _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x10000000);
    if (ui8LanesNum == 1)
    {
        DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x00480000); // trim_2<22> and trim_2<19> need to be set for DSI TX in 1-lane configuration.
    }
    else if (ui8LanesNum == 2)
    {
        DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x00400000); // clear power down bit for Data lane 1 to support DSI TX in 2lanes configuration.
    }
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00002000); // trim_1<13> needs to be set

    if (bSendUlpsPattern)
    {
        DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00030000);
    }
    if (!APOLLO5_B0)
    {
        DSI->AFETRIM0 |= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, 0x00020000); // trim_0<17> needs to be set for B1 and later version.
    }
    //
    // enable DSI TX and DPHY
    //
    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 1);
    DSI->DEVICEREADY |= _VAL2FLD(DSI_DEVICEREADY_READY, 1);

    //
    // Wait for DPHY init
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_delay_us_status_change(1000, (uint32_t)&DSI->INTRSTAT, DSI_INTRSTAT_INITDONE_Msk, DSI_INTRSTAT_INITDONE_Msk))
    {
       return AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Check the low contention status bit
    //
    if ( DSI->INTRSTAT_b.LOWC == 1 )
    {
        //
        // write 1 to clear low contention
        //
        DSI->INTRSTAT_b.LOWC = 1;
    }

    if (bSendUlpsPattern)
    {
        //
        // ULPS Exit sequence
        //
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
        am_hal_delay_us(10);
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
        DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00030000);
        am_hal_delay_us(1010);
        DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;
    }
    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_init(void)
{
    //CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_ENABLE;
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    DSI->RSTENBDFE = _VAL2FLD(DSI_RSTENBDFE_ENABLE, 0);
    //DSI->DEVICEREADY = _VAL2FLD(DSI_DEVICEREADY_READY, 0);

    //
    // vdd18 enable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(true);
    }

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DBICLKDIV2EN_DISABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DBICLKSEL_DBIB_CLK, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_HFRC12, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_ENABLE, NULL);

    return am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_DSI);
}

//*****************************************************************************
//
// Turn off power and clock of DSI
//
//*****************************************************************************
uint32_t
am_hal_dsi_deinit(bool bCheckStopState)
{
    //
    // Please don't check the stop state when DSI in ULPS
    //
    if (bCheckStopState)
    {
        //
        // Wait MIPI-DSI lane 0 return stop state before deinit it.
        //
        if (AM_HAL_STATUS_TIMEOUT == am_hal_dsi_wait_stop_state(0))
        {
            //
            // Return AM_HAL_STATUS_IN_USE if DSI is working.
            //
            return AM_HAL_STATUS_IN_USE;
        }
    }

    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_OFF, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_DISABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DBICLKDIV2EN_DISABLE, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLKSEL_OFF, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_PLLCLK_DISABLE, NULL);

    //
    // vdd18 disable
    //
    if (external_vdd18_callback)
    {
        external_vdd18_callback(false);
    }

    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_DISPPHY);
    //CLKGEN->CLKCTRL_b.DISPCTRLCLKEN = CLKGEN_CLKCTRL_DISPCTRLCLKEN_DISABLE;
    return am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_DSI);
}

//*****************************************************************************
//
// Enter ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_entry(void)
{
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_LOW_POWER;
    DSI->AFETRIM0 |= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, 0x00000800); //  trim_0<11> needs to be set - contention detector disabled
    am_hal_delay_us(10);
    DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, 0x00000200); //  trim_1<9> needs to be set
    DSI->AFETRIM2 |= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, 0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be set
    DSI->AFETRIM3 |= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, 0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be set

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Exit ULPS mode
//
//*****************************************************************************
uint32_t
am_hal_dsi_ulps_exit(void)
{
    DSI->AFETRIM3 &= _VAL2FLD(DSI_AFETRIM3_AFETRIM3, ~0x00038000); //  trim_3<15>, trim_3<16>, and trim_3<17> need to be cleared
    DSI->AFETRIM2 &= _VAL2FLD(DSI_AFETRIM2_AFETRIM2, ~0x0000001C); //  trim_2<2>, trim_2<3> & trim_2<4> need to be cleared
    DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x00000200); //  trim_1<9> needs to be cleared
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_EXIT;
    am_hal_delay_us(1010);
    DSI->DEVICEREADY_b.ULPS = DSI_DEVICEREADY_ULPS_This;
    DSI->AFETRIM0 &= _VAL2FLD(DSI_AFETRIM0_AFETRIM0, ~0x00000800); //  trim_0<11> needs to be cleared - contention detector enable

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI napping
//
//*****************************************************************************
uint32_t
am_hal_dsi_napping(bool bSendUlpsPattern)
{
    if (bSendUlpsPattern)
    {
        am_hal_dsi_ulps_entry();
    }

    return am_hal_dsi_deinit(!bSendUlpsPattern);
}

//*****************************************************************************
//
// DSI wakeup
//
//*****************************************************************************
uint32_t
am_hal_dsi_wakeup(uint8_t ui8LanesNum, uint8_t ui8DBIBusWidth, uint32_t ui32FreqTrim, bool bSendUlpsPattern)
{
    am_hal_dsi_init();

    if ( am_hal_dsi_para_config(ui8LanesNum, ui8DBIBusWidth, ui32FreqTrim, bSendUlpsPattern) != 0 )
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI set return packet size (bytes)
//
//*****************************************************************************
uint32_t
am_hal_dsi_set_return_size(uint8_t ui8DataLen, bool bHS)
{
    if (APOLLO5_B0)
    {
        am_hal_dsi_pre_rw_cmd(bHS);
    }

    DSI->MAXRETPACSZE = _VAL2FLD(DSI_MAXRETPACSZE_HSLP, (uint32_t) (!bHS)) | _VAL2FLD(DSI_MAXRETPACSZE_COUNTVAL, (uint32_t)ui8DataLen);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI wait the stop state
//
//*****************************************************************************
uint32_t
am_hal_dsi_wait_stop_state(uint8_t ui8Lane)
{

    uint32_t ui32RegDSIBist = MCUCTRL->DSIBIST;
    //
    // bitfields 8-15 -> 6'b01_0011
    //
    MCUCTRL->DSIBIST = ui32RegDSIBist & ~(_VAL2FLD(MCUCTRL_DSIBIST_DSIBISTSEED, 0xFFU));
    MCUCTRL->DSIBIST |= _VAL2FLD(MCUCTRL_DSIBIST_DSIBISTSEED, 0x13U);

    uint32_t ui32RegTrim3 = DSI->AFETRIM3;

    if ( ui8Lane != 0 && ui8Lane != 1 )
    {
        //
        // Only support lane 0 and lane 1
        //
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    //
    // bitfields 22-25 -> 4'h0 for lane 0 stop state, bitfields 22-25 -> 4'h1 for lane 1 stop state.
    //
    DSI->AFETRIM3 = (ui32RegTrim3 & ~0x3C00000UL) | ((uint32_t)ui8Lane << 22);

    //
    // Wait lane 0 or lane 1 in stop state.
    //
    if ( AM_HAL_STATUS_SUCCESS != am_hal_delay_us_status_change(1000, (uint32_t)&MCUCTRL->DSIBIST, MCUCTRL_DSIBIST_DSIBISTERRRXHS_Msk, MCUCTRL_DSIBIST_DSIBISTERRRXHS_Msk) )
    {
        return AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Recover the original registers
    //
    MCUCTRL->DSIBIST = ui32RegDSIBist;
    DSI->AFETRIM3 = ui32RegTrim3;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI Pre read/write command SW workaround for LP mode
//
//*****************************************************************************
#define DSI_FREQ_TRIM_FOR_LP    ((uint32_t)AM_HAL_DSI_FREQ_TRIM_X4)
static uint32_t ui32DSIFreqTrim  = DSI_FREQ_TRIM_FOR_LP;

uint32_t
am_hal_dsi_pre_rw_cmd(bool bHS)
{
    //
    // Get the present trim value.
    //
    uint32_t ui32CurrentTrim = DSI->AFETRIM1 & 0x0000007F;

    if (DSI_FREQ_TRIM_FOR_LP != ui32CurrentTrim)
    {
        //
        // Save the original DSI frequency
        //
        ui32DSIFreqTrim = ui32CurrentTrim;
    }

    if (bHS)
    {
        if (DSI_FREQ_TRIM_FOR_LP == ui32CurrentTrim)
        {
            //
            // Recover the original DSI frequency
            //
            DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x0000007F);
            DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ui32DSIFreqTrim);
            am_hal_sysctrl_sysbus_write_flush();
            am_hal_delay_us(100);
        }
    }
    else
    {
        if (DSI_FREQ_TRIM_FOR_LP != ui32CurrentTrim)
        {
            //
            // PLL settings
            //
            DSI->AFETRIM1 &= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, ~0x0000007F);
            DSI->AFETRIM1 |= _VAL2FLD(DSI_AFETRIM1_AFETRIM1, DSI_FREQ_TRIM_FOR_LP);
            am_hal_sysctrl_sysbus_write_flush();
            am_hal_delay_us(100);
        }
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// DSI power control function
//
//*****************************************************************************
uint32_t
am_hal_dsi_power_control(am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    static am_hal_dsi_state_t sDSIState = {0};
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    bool bStatus;
    am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_DISPPHY, &bStatus);

    //
    // Decode the requested power state and update DSI operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:

            if (bStatus)
            {
                return AM_HAL_STATUS_IN_USE;
            }

            am_hal_dsi_init();

            if (bRetainState && sDSIState.bisValid)
            {
                ui32Status = am_hal_dsi_para_config(sDSIState.ui8Lanes,
                                                    sDSIState.eWidth,
                                                    sDSIState.eTrim,
                                                    true);
            }
            else
            {
                //
                // The user should invoke function am_hal_dsi_para_config() manually.
                //
                sDSIState.bisValid = false;
            }

            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:

            if (!bStatus)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Wait the stop state
            //
            ui32Status = am_hal_dsi_wait_stop_state(0);

            if (ui32Status != AM_HAL_STATUS_SUCCESS)
            {
                return ui32Status;
            }

            if (APOLLO5_B0)
            {
                //
                // Recover the DSI frequency.
                //
                am_hal_dsi_pre_rw_cmd(true);
            }

            if (bRetainState)
            {
                //
                // Store parameters
                //
                sDSIState.eTrim        = (am_hal_dsi_freq_trim_e) (DSI->AFETRIM1 & 0x7F);
                sDSIState.ui8Lanes     = (uint8_t)_FLD2VAL(DSI_DSIFUNCPRG_DATALANES, DSI->DSIFUNCPRG);
                uint8_t ui8Width = (uint8_t)_FLD2VAL(DSI_DSIFUNCPRG_REGNAME, DSI->DSIFUNCPRG);

                if (ui8Width == DSI_DSIFUNCPRG_REGNAME_16BIT1)
                {
                    sDSIState.eWidth   = AM_HAL_DSI_DBI_WIDTH_16;
                }
                else if (ui8Width == DSI_DSIFUNCPRG_REGNAME_8BIT)
                {
                    sDSIState.eWidth   = AM_HAL_DSI_DBI_WIDTH_8;
                }
                else
                {
                    return AM_HAL_STATUS_INVALID_ARG;
                }
                sDSIState.bisValid     = true;
            }
            else
            {
                //
                // Default mode.
                //
                sDSIState.bisValid     = false;
            }
            ui32Status = am_hal_dsi_napping(true);

            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return ui32Status;

} // am_hal_dsi_power_control()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
