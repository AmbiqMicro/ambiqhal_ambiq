//*****************************************************************************
//
//! @file am_hal_reset.c
//!
//! @brief Hardware abstraction layer for the Reset Generator module.
//!
//! @addtogroup rstgen2_ap510 Reset - Reset Generator (RSTGEN)
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides hardware abstraction layer functions for the
//!          Reset Generator (RSTGEN) module on Apollo5 devices. It supports
//!          system reset control, brownout detection, watchdog reset management,
//!          and interrupt handling for system reliability and recovery.
//!
//! @section hal_reset_features Key Features
//!
//! 1. @b System @b Reset: Control system reset operations and sources.
//! 2. @b Brownout @b Detection: Monitor voltage levels and trigger resets.
//! 3. @b Watchdog @b Reset: Manage watchdog timer reset functionality.
//! 4. @b Interrupt @b Support: Interrupt handling for reset events.
//! 5. @b Status @b Monitoring: Reset status and source identification.
//!
//! @section hal_reset_functionality Functionality
//!
//! - Configure reset generator parameters
//! - Control system reset operations
//! - Handle brownout detection and response
//! - Manage watchdog reset functionality
//! - Monitor reset status and sources
//!
//! @section hal_reset_usage Usage
//!
//! 1. Configure reset generator using am_hal_reset_configure()
//! 2. Control reset operations as needed
//! 3. Handle reset interrupts and events
//! 4. Monitor reset status and sources
//! 5. Manage system recovery operations
//!
//! @section hal_reset_configuration Configuration
//!
//! - @b Reset @b Sources: Configure reset source enable/disable
//! - @b Brownout @b Levels: Set up voltage monitoring thresholds
//! - @b Watchdog @b Reset: Configure watchdog reset parameters
//! - @b Interrupts: Set up interrupt sources and handlers
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
// This is part of revision release_sdk5p1p0-366b80e084 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"

uint32_t gAmHalResetStatus = 0;

//*****************************************************************************
//
//  am_hal_reset_enable()
//  Enable and configure the Reset controller.
//
//*****************************************************************************
uint32_t
am_hal_reset_configure(am_hal_reset_configure_e eConfigure)
{
    uint32_t ui32Val;
    bool     bEnable = false;

    switch ( eConfigure )
    {
        case AM_HAL_RESET_BROWNOUT_HIGH_ENABLE:
            //
            // Apollo5 operates at 1.8v/1.9v, thus selecting
            // AM_HAL_RESET_BROWNOUT_HIGH_ENABLE is invalid.
            //
            return AM_HAL_STATUS_INVALID_ARG;

        case AM_HAL_RESET_WDT_RESET_ENABLE:
            bEnable = true;
            ui32Val = RSTGEN_CFG_WDREN_Msk;
            break;

        case AM_HAL_RESET_BROWNOUT_HIGH_DISABLE:
            bEnable = false;
            ui32Val = RSTGEN_CFG_BODHREN_Msk;
            break;

        case AM_HAL_RESET_WDT_RESET_DISABLE:
            bEnable = false;
            ui32Val = RSTGEN_CFG_WDREN_Msk;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    AM_CRITICAL_BEGIN
    if ( bEnable )
    {
        RSTGEN->CFG |= ui32Val;
    }
    else
    {
        RSTGEN->CFG &= ~ui32Val;
    }
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_configure()

//*****************************************************************************
//
//  am_hal_reset_control()
//  Perform various reset functions including assertion of software resets.
//
//*****************************************************************************
uint32_t
am_hal_reset_control(am_hal_reset_control_e eControl, void *pArgs)
{
    switch ( eControl )
    {
        case AM_HAL_RESET_CONTROL_SWPOR:
            //
            // Perform a Power On Reset level reset.
            // Write the POR key to the software POR register.
            //
            RSTGEN->SWPOR =
                   _VAL2FLD(RSTGEN_SWPOR_SWPORKEY, RSTGEN_SWPOR_SWPORKEY_KEYVALUE);

            // Spin till reset happens.
            while (1) ;
            break;

        case AM_HAL_RESET_CONTROL_SWPOI:
            //
            // Perform a Power On Initialization level reset.
            // Write the POI key to the software POI register.
            //
            RSTGEN->SWPOI =
                _VAL2FLD(RSTGEN_SWPOI_SWPOIKEY, RSTGEN_SWPOI_SWPOIKEY_KEYVALUE);

            // Spin till reset happens.
            while (1) ;
            break;

        default:
            break;
    }

    return AM_HAL_STATUS_INVALID_ARG;

} // am_hal_reset_control()

//*****************************************************************************
//
//  am_hal_reset_status_get()
//  Return status of the reset generator.
//  Application MUST call this API at least once before going to deepsleep
//  Otherwise this API will not provide correct reset status
//
//*****************************************************************************
uint32_t
am_hal_reset_status_get(am_hal_reset_status_t *psStatus)
{
    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Retrieve the reset generator status bits
    // Need to read the status only the very first time
    //
    if ( !gAmHalResetStatus )
    {
        gAmHalResetStatus = RSTGEN->STAT;
    }
    gAmHalResetStatus &= AM_HAL_RESET_STATUS_MASK;

    psStatus->eStatus           = (am_hal_reset_status_e)gAmHalResetStatus;
    psStatus->bEXTStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_EXTERNAL;
    psStatus->bPOAStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_POA;
    psStatus->bBODStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOD;
    psStatus->bSWPORStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_SWPOR;
    psStatus->bSWPOIStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_SWPOI;
    psStatus->bDBGRStat         = gAmHalResetStatus & AM_HAL_RESET_STATUS_DEBUGGER;
    psStatus->bWDTStat          = gAmHalResetStatus & AM_HAL_RESET_STATUS_WDT;
    psStatus->bBOUnregStat      = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOUNREG;
    psStatus->bBOCOREStat       = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOCORE;
    psStatus->bBOMEMStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOMEM;
    psStatus->bBOHPMEMStat      = gAmHalResetStatus & AM_HAL_RESET_STATUS_BOHPMEM;
    psStatus->bRsvd             = false;
    psStatus->bAIRCRStat        = gAmHalResetStatus & AM_HAL_RESET_STATUS_AIRCR;

    //
    // Return status.
    // If the Reset Status is 0 - this implies application did not capture the snapshot
    // before deepsleep, and hence the result is invalid
    //
    return (gAmHalResetStatus ? AM_HAL_STATUS_SUCCESS : AM_HAL_STATUS_FAIL);

} // am_hal_reset_status_get()

//*****************************************************************************
//
//  am_hal_reset_interrupt_enable()
//  Use this function to enable the reset generator interrupts.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_enable(uint32_t ui32IntMask)
{
    AM_CRITICAL_BEGIN
    RSTGEN->INTEN |= ui32IntMask;
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_enable()

//*****************************************************************************
//
//  am_hal_reset_interrupt_disable()
//  Disable selected RSTGEN Interrupts.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_disable(uint32_t ui32IntMask)
{
    AM_CRITICAL_BEGIN
    RSTGEN->INTEN &= ~ui32IntMask;
    AM_CRITICAL_END

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_disable()

//*****************************************************************************
//
//  am_hal_reset_interrupt_clear()
//  Reset generator interrupt clear
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_clear(uint32_t ui32IntMask)
{
    RSTGEN->INTEN = ui32IntMask;
    *(volatile uint32_t*)(&RSTGEN->INTSTAT);
    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_clear()

//*****************************************************************************
//
//  am_hal_reset_interrupt_status_get()
//  Get interrupt status of reset generator.
//
//*****************************************************************************
uint32_t
am_hal_reset_interrupt_status_get(bool bEnabledOnly,
                                  uint32_t *pui32IntStatus)
{
    if ( pui32IntStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Retrieve the reset generator status bits
    //
    *pui32IntStatus = RSTGEN->INTSTAT;

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_reset_interrupt_status_get()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
