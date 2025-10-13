//*****************************************************************************
//
//! @file am_hal_wdt.c
//!
//! @brief Watchdog Timer
//!
//! @addtogroup wdt_ap510 WDT - Watchdog Timer Functionality
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides watchdog timer functionality for Apollo5
//! devices, supporting system monitoring, fault detection, and automatic
//! recovery mechanisms. It enables reliable system operation through
//! configurable timeout periods, interrupt generation, and reset capabilities.
//!
//! @section hal_wdt_features Key Features
//!
//! 1. @b System @b Reset: Automatic system reset on timer expiration.
//! 2. @b Interrupt @b Support: Configurable interrupt generation before reset.
//! 3. @b Flexible @b Timing: Multiple clock sources and timing configurations.
//! 4. @b Lock @b Protection: Optional configuration locking for security.
//! 5. @b Status @b Monitoring: Real-time timer status and value reading.
//!
//! @section hal_wdt_functionality Functionality
//!
//! - Configure watchdog timer parameters and clock sources
//! - Start, stop, and restart watchdog operations
//! - Handle watchdog interrupts and status monitoring
//! - Support for configuration locking and protection
//! - Read timer values and monitor system health
//!
//! @section hal_wdt_usage Usage
//!
//! 1. Configure the watchdog using am_hal_wdt_config()
//! 2. Start the watchdog with am_hal_wdt_start()
//! 3. Service the watchdog periodically to prevent reset
//! 4. Handle watchdog interrupts if enabled
//! 5. Monitor watchdog status as needed
//!
//! @section hal_wdt_configuration Configuration
//!
//! - @b Clock @b Sources: Select from LFRC-based clock options
//! - @b Timing @b Values: Configure interrupt and reset timing
//! - @b Interrupts: Enable/disable watchdog interrupts
//! - @b Reset @b Control: Enable/disable system reset on timeout
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Configure the watchdog timer.
//
//*****************************************************************************
uint32_t
am_hal_wdt_config(am_hal_wdt_select_e eTimer, void *pvConfig)
{
    uint32_t ui32ConfigValue;
    am_hal_wdt_config_t *psConfig = pvConfig;

    //
    // Check to see if we're configuring the MCU watchdog, or one of the DSP
    // watchdogs.
    //
    if (eTimer == AM_HAL_WDT_MCU)
    {
        ui32ConfigValue = 0;

        //
        // Apply the settings from our configuration structure.
        //
        ui32ConfigValue |= _VAL2FLD(WDT_CFG_CLKSEL, psConfig->eClockSource);
        ui32ConfigValue |= _VAL2FLD(WDT_CFG_INTVAL, psConfig->ui32InterruptValue);
        ui32ConfigValue |= _VAL2FLD(WDT_CFG_RESVAL, psConfig->ui32ResetValue);

        if (psConfig->bResetEnable)
        {
            ui32ConfigValue |= _VAL2FLD(WDT_CFG_RESEN, 1);
        }

        if (psConfig->bInterruptEnable)
        {
            ui32ConfigValue |= _VAL2FLD(WDT_CFG_INTEN, 1);
        }

        //
        // Write the settings to the WDT config register.
        //
        WDT->CFG = ui32ConfigValue;

        //
        // Enabled the WDT Reset if requested.
        //
        RSTGEN->CFG_b.WDREN = psConfig->bResetEnable;

        return AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        //
        // DSP register settings are a little different.
        //
        return AM_HAL_STATUS_INVALID_ARG;
    }
}

//*****************************************************************************
//
// Enables the watchdog timer.
//
//*****************************************************************************
uint32_t
am_hal_wdt_start(am_hal_wdt_select_e eTimer, bool bLock)
{
    uint32_t ui32ClkSel;
    //
    // Enable the timer.
    //
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            ui32ClkSel = WDT->CFG_b.CLKSEL;
            if ((ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV8 )  ||
                (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV64)    ||
                (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV1K)     ||
                (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV16K))
            {
                am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_LFRC, AM_HAL_CLKMGR_USER_ID_WDT);
            }
            else if ((ui32ClkSel == WDT_CFG_CLKSEL_XTAL_HS) ||
                     (ui32ClkSel == WDT_CFG_CLKSEL_XTAL_HS_DIV2 ))
            {
                am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, AM_HAL_CLKMGR_USER_ID_WDT);
            }
            WDT->RSTRT = WDT_RSTRT_RSTRT_KEYVALUE;
            WDT->CFG_b.WDTEN = 1;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    //
    // Lock the timer if we were asked to do so.
    //
    if (bLock)
    {
        switch (eTimer)
        {
            case AM_HAL_WDT_MCU:
                WDT->LOCK = WDT_LOCK_LOCK_KEYVALUE;
                break;

            default:
                return AM_HAL_STATUS_FAIL;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disables the watchdog timer.
//
//*****************************************************************************
uint32_t
am_hal_wdt_stop(am_hal_wdt_select_e eTimer)
{
    uint32_t ui32ClkSel;
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            ui32ClkSel = WDT->CFG_b.CLKSEL;
            WDT->CFG_b.WDTEN = 0;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }
    if ( WDT->CFG_b.WDTEN == 0 )
    {
        if ( (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV8 ) ||
             (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV64)   ||
             (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV1K)    ||
             (ui32ClkSel == WDT_CFG_CLKSEL_LFRC_DIV16K))
        {
            am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_LFRC, AM_HAL_CLKMGR_USER_ID_WDT);
        }
        else if ( (ui32ClkSel == WDT_CFG_CLKSEL_XTAL_HS)    ||
                  (ui32ClkSel == WDT_CFG_CLKSEL_XTAL_HS_DIV2 ))
        {
            am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_XTAL_HS, AM_HAL_CLKMGR_USER_ID_WDT);
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Restart (pet/feed) the watchdgog
//
//*****************************************************************************
uint32_t
am_hal_wdt_restart(am_hal_wdt_select_e eTimer)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            WDT->RSTRT = WDT_RSTRT_RSTRT_KEYVALUE;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reads the watchdog timer's current value.
//
//*****************************************************************************
uint32_t
am_hal_wdt_read(am_hal_wdt_select_e eTimer, uint32_t *ui32Value)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            *ui32Value = WDT->COUNT;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Watchdog interrupt enable.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_enable(am_hal_wdt_select_e eTimer,
                            uint32_t ui32InterruptMask)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            WDT->WDTIEREN |= ui32InterruptMask;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Check to see which WDT interrupts are enabled.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_enable_get(am_hal_wdt_select_e eTimer,
                                uint32_t *pui32InterruptMask)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            *pui32InterruptMask = WDT->WDTIEREN;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable a WDT interrupt.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_disable(am_hal_wdt_select_e eTimer,
                             uint32_t ui32InterruptMask)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            WDT->WDTIEREN &= ~ui32InterruptMask;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read the WDT interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_status_get(am_hal_wdt_select_e eTimer,
                                uint32_t *pui32InterruptMask,
                                bool bEnabledOnly)
{
    uint32_t ui32Status;

    if (bEnabledOnly)
    {
        switch (eTimer)
        {
            case AM_HAL_WDT_MCU:
                ui32Status = WDT->WDTIERSTAT;
                ui32Status &= WDT->WDTIEREN;
                break;

            default:
                return AM_HAL_STATUS_FAIL;
        }

        *pui32InterruptMask = ui32Status;
    }
    else
    {
        switch (eTimer)
        {
            case AM_HAL_WDT_MCU:
                *pui32InterruptMask = WDT->WDTIERSTAT;
                break;

            default:
                return AM_HAL_STATUS_FAIL;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clears the WDT interrupt.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_clear(am_hal_wdt_select_e eTimer,
                           uint32_t ui32InterruptMask)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            WDT->WDTIERCLR = ui32InterruptMask;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }
    *(volatile uint32_t*)(&WDT->WDTIERSTAT);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Sets a WDT interrupt.
//
//*****************************************************************************
uint32_t
am_hal_wdt_interrupt_set(am_hal_wdt_select_e eTimer,
                         uint32_t ui32InterruptMask)
{
    switch (eTimer)
    {
        case AM_HAL_WDT_MCU:
            WDT->WDTIERSET = ui32InterruptMask;
            break;

        default:
            return AM_HAL_STATUS_FAIL;
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
