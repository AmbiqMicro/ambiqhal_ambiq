//*****************************************************************************
//
//! @file am_hal_vcomp.c
//!
//! @brief Functions for operating the on-chip Voltage Comparator
//!
//! @addtogroup vcomp5_ap510L Voltage Comparator (VCOMP)
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides voltage comparator functionality for Apollo5
//! devices, supporting analog voltage monitoring, threshold detection, and
//! reference voltage comparison. It enables precise analog signal processing
//! for applications requiring voltage level monitoring and control.
//!
//! @section hal_vcomp_features Key Features
//!
//! 1. @b Voltage @b Comparison: Compare input voltages against reference levels.
//! 2. @b DAC @b Reference: Configurable DAC reference voltage generation.
//! 3. @b Input @b Selection: Flexible positive and negative input selection.
//! 4. @b Interrupt @b Support: Interrupt generation on comparison events.
//! 5. @b Status @b Monitoring: Real-time comparator output status reading.
//!
//! @section hal_vcomp_functionality Functionality
//!
//! - Configure voltage comparator parameters and input selection
//! - Set up DAC reference levels for comparison
//! - Enable/disable the voltage comparator
//! - Handle comparator interrupts and status monitoring
//! - Read comparator output status in real-time
//!
//! @section hal_vcomp_usage Usage
//!
//! 1. Configure the comparator using am_hal_vcomp_config()
//! 2. Set DAC reference levels as needed
//! 3. Enable the comparator with am_hal_vcomp_enable()
//! 4. Handle comparator interrupts if enabled
//! 5. Monitor comparator status for voltage level changes
//!
//! @section hal_vcomp_configuration Configuration
//!
//! - @b Input @b Selection: Choose positive and negative input sources
//! - @b DAC @b Levels: Configure DAC reference voltage levels
//! - @b Interrupts: Enable/disable comparator interrupts
//! - @b Power @b Management: Control comparator power states
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
// This is part of revision release_sdk5_2_a_3-80ffa398f of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Configure the Voltage Comparator module.
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_config(const am_hal_vcomp_config_t *psConfig)
{
    //
    // Check if input is valid
    //
    if (((psConfig->eLevelSelect << VCOMP_CFG_LVLSEL_Pos) & ( ~ VCOMP_CFG_LVLSEL_Msk)) |
       ((psConfig->ePosInput << VCOMP_CFG_PSEL_Pos) & ( ~ VCOMP_CFG_PSEL_Msk)) |
       ((psConfig->eNegInput << VCOMP_CFG_NSEL_Pos) & ( ~ VCOMP_CFG_NSEL_Msk)))
    {
        return AM_HAL_STATUS_FAIL;
    }

    VCOMP->CFG = (psConfig->eLevelSelect << VCOMP_CFG_LVLSEL_Pos) |
                 (psConfig->ePosInput    << VCOMP_CFG_PSEL_Pos)   |
                 (psConfig->eNegInput    << VCOMP_CFG_NSEL_Pos);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set the Voltage Comparator DAC Level Select in Configuration Reg.
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_dac_level_set(am_hal_vcomp_dac_lvlsel_e eLevelSelect)
{
    //
    // Check if input is valid
    //
    if ((eLevelSelect << VCOMP_CFG_LVLSEL_Pos) & ( ~ VCOMP_CFG_LVLSEL_Msk))
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Insert the supplied level into the vcomp configuration register
    //
    VCOMP->CFG_b.LVLSEL = eLevelSelect;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read the state of the voltage comparator.
//
//*****************************************************************************
uint32_t
am_hal_vcomp_status_get(void)
{
    return VCOMP->STAT;
}

//*****************************************************************************
//
// Enable the voltage comparator.
//
//*****************************************************************************
void
am_hal_vcomp_enable(void)
{
    VCOMP->PWDKEY = 0x0;
}

//*****************************************************************************
//
// Disable the voltage comparator.
//
//*****************************************************************************
void
am_hal_vcomp_disable(void)
{
    VCOMP->PWDKEY = AM_REG_VCOMP_PWDKEY_KEYVAL;
}

//*****************************************************************************
//
// Read the state of the voltage comparator interrupt status bits.
//
//*****************************************************************************
uint32_t
am_hal_vcomp_int_status_get(bool bEnabledOnly)
{
    //
    // Read only enabled interrupt status if bEnabledOnly flag is set
    //
    if (bEnabledOnly)
    {
        uint32_t ui32RetVal = VCOMP->INTEN;
        return VCOMP->INTSTAT & ui32RetVal;
    }
    else
    {
        return VCOMP->INTSTAT;
    }
}

//*****************************************************************************
//
// Set voltage comparator interrupt to immeidately generate the interrupt of that
// corresponding bits.
// ui32Interrupt should be a combination of a logical or of
// AM_HAL_VCOMP_INT_OUTHI_SET and AM_HAL_VCOMP_INT_OUTLOW_SET
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_int_set(uint32_t ui32Interrupt)
{
    //
    // Check if input is valid
    //
    if (ui32Interrupt & ( ~ AM_HAL_VCOMP_INT_ALL_Msk))
    {
        return AM_HAL_STATUS_FAIL;
    }

    VCOMP->INTSET = ui32Interrupt;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear the state of the voltage comparator interrupt status bits.
// ui32Interrupt should be a combination of a logical or of
// AM_HAL_VCOMP_INT_OUTHI_CLR and AM_HAL_VCOMP_INT_OUTLOW_CLR
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_int_clear(uint32_t ui32Interrupt)
{
    //
    // Check if input is valid
    //
    if (ui32Interrupt & ( ~ AM_HAL_VCOMP_INT_ALL_Msk))
    {
        return AM_HAL_STATUS_FAIL;
    }

    VCOMP->INTCLR = ui32Interrupt;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable the voltage comparator interrupt bits.
// ui32Interrupt should be a combination of a logical or of
// AM_HAL_VCOMP_INT_OUTHI_EN and AM_HAL_VCOMP_INT_OUTLOW_EN
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_int_enable(uint32_t ui32Interrupt)
{
    //
    // Check if input is valid
    //
    if (ui32Interrupt & ( ~ AM_HAL_VCOMP_INT_ALL_Msk))
    {
        return AM_HAL_STATUS_FAIL;
    }

    VCOMP->INTEN |= ui32Interrupt;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Return the enabled, voltage comparator interrupt status bits.
//
//*****************************************************************************
uint32_t
am_hal_vcomp_int_enable_get(void)
{
    return VCOMP->INTEN;
}

//*****************************************************************************
//
// Disable the voltage comparator interrupt status bits.
// ui32Interrupt should be a combination of a logical or of
// AM_HAL_VCOMP_INT_OUTHI_EN and AM_HAL_VCOMP_INT_OUTLOW_EN
//
//*****************************************************************************
am_hal_status_e
am_hal_vcomp_int_disable(uint32_t ui32Interrupt)
{
    //
    // Check if input is valid
    //
    if (ui32Interrupt & ( ~ AM_HAL_VCOMP_INT_ALL_Msk))
    {
        return AM_HAL_STATUS_FAIL;
    }

    VCOMP->INTEN &= ~ui32Interrupt;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
