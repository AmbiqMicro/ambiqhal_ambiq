//*****************************************************************************
//
//! @file am_hal_tpiu.c
//!
//! @brief Support functions for the Arm TPIU module
//!
//! Provides support functions for configuring the Arm TPIU module
//!
//! @addtogroup tpiu4 TPIU - Trace Port Interface Unit
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

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//
// TPIU counters
//
static bool    g_bTpiu_DebugEnabled = false;
static uint8_t g_ui8TpiuEnCnt       = 0;

//*****************************************************************************
//
// Local functions
//
//*****************************************************************************

//*****************************************************************************
//
// TPIU configuration function
//
//*****************************************************************************
uint32_t
am_hal_tpiu_config(uint32_t ui32DbgTpiuClksel,  // MCUCTRL_DBGCTRL_DBGTPIUCLKSEL_HFRC_48MHz
                   uint32_t ui32FFCR,           // 0: Disable continuous formatting (EnFCont)
                   uint32_t ui32CSPSR,          // TPI_CSPSR_CWIDTH_1BIT
                   uint32_t ui32PinProtocol,    // TPI_SPPR_TXMODE_UART
                   uint32_t ui32SWOscaler)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN

    //
    // Enable the TPIU clock source in MCU control.
    //
    MCUCTRL->DBGCTRL_b.DBGTPIUCLKSEL = ui32DbgTpiuClksel;

    //
    // Perform general debug/tracing configuration.
    //
    if ( g_bTpiu_DebugEnabled == false )
    {
        am_hal_debug_enable();
        g_bTpiu_DebugEnabled = true;
    }

    if ( ui32Status == AM_HAL_STATUS_SUCCESS )
    {

        //
        // TPIU formatter & flush control register.
        // Disable continuous formatting (EnFCont bit1).
        //
        TPI->FFCR = ui32FFCR;

        //
        // Set the Current Parallel Port Size (note - only 1 bit can be set).
        //
        TPI->CSPSR = ui32CSPSR;

        //
        // Set the scaler value.
        //
        TPI->ACPR = _VAL2FLD(TPI_ACPR_SWOSCALER, ui32SWOscaler);

        //
        // Set the Pin Protocol.
        //
        TPI->SPPR = _VAL2FLD( TPI_SPPR_TXMODE, ui32PinProtocol);
    }

    AM_CRITICAL_END

    return ui32Status;
} // am_hal_tpiu_config()

//*****************************************************************************
//
// Enable the TPIU
//
// This function enables the Arm TPIU by setting the TPIU clock source.
//
// Arm v8-M Architecture Reference Manual, DDI0553B.y section B14.4 states
// "The TPIU is not directly affected by DEMCR.TRCENA being set to 0."
//
//*****************************************************************************
uint32_t
am_hal_tpiu_enable(uint32_t ui32DeprecatedItmBaud)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN

    //
    // Bump the counter on each enable call.
    //
    g_ui8TpiuEnCnt++;

    //
    // The deprecated ItmBaud argument is an ITM-specific parameter and is only
    // maintained here for backward compatibility. On entry, it should typically
    // be set to 0. Otherwise, pass it along to the ITM HAL.
    //
    if ( (ui32DeprecatedItmBaud != AM_HAL_TPIU_BAUD_DEFAULT) &&
         (ui32DeprecatedItmBaud != 0) )
    {
        am_hal_itm_parameters_set(ui32DeprecatedItmBaud);
    }

    if ( g_bTpiu_DebugEnabled == false )
    {
        //
        // Perform general debug/tracing configuration.
        //
        ui32Status = am_hal_debug_enable();
        g_bTpiu_DebugEnabled = true;
    }

    AM_CRITICAL_END

    return ui32Status;

} // am_hal_tpiu_enable()

//*****************************************************************************
//
// Disable the TPIU
//
// This function enables the Arm TPIU by disabling the TPIU clock source.
//
//*****************************************************************************
uint32_t
am_hal_tpiu_disable(void)
{
    uint32_t ui32Status;

    AM_CRITICAL_BEGIN

    if ( g_ui8TpiuEnCnt != 0 )
    {
        --g_ui8TpiuEnCnt;
    }

    if ( g_ui8TpiuEnCnt > 0 )
    {
        //
        // TPIU needs to remain enabled for now.
        //
        ui32Status = AM_HAL_STATUS_IN_USE;
    }
    else
    {
        //
        // Invalidate any previous TPIU configuration
        //
        g_bTpiu_DebugEnabled = false;

        //
        // Shut down the general debug/tracing configuration.
        //
        ui32Status = am_hal_debug_disable();
        if ( ui32Status == AM_HAL_STATUS_IN_USE )
        {
            ui32Status = AM_HAL_STATUS_SUCCESS;
        }
    }

    AM_CRITICAL_END

    return ui32Status;

} // am_hal_tpiu_disable()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
