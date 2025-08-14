//*****************************************************************************
//
//! @file am_hal_infoc.c
//!
//! @brief Functions for INFOC functions
//!
//! @addtogroup infoc_ap510 INFOC - One-Time Programmable
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides functions for reading and writing the INFOC
//! one-time programmable memory region on Apollo5 devices, supporting secure
//! storage and retrieval of device-specific data.
//!
//! @section hal_infoc_features Key Features
//!
//! 1. @b INFOC @b Access: Read and write operations to INFOC memory region.
//! 2. @b Validation: Comprehensive offset and accessibility validation.
//! 3. @b Security: Secure access to one-time programmable memory.
//! 4. @b Error @b Handling: Proper error reporting for invalid operations.
//! 5. @b Busy @b Monitoring: Wait for programming completion and busy states.
//!
//! @section hal_infoc_functionality Functionality
//!
//! - Read INFOC words with offset validation
//! - Write INFOC words with proper programming sequences
//! - Validate INFOC accessibility and power states
//! - Handle busy states and programming completion
//! - Support for secure INFOC operations
//!
//! @section hal_infoc_usage Usage
//!
//! 1. Validate INFOC accessibility before operations
//! 2. Read INFOC words using am_hal_infoc_read_word()
//! 3. Write INFOC words using am_hal_infoc_write_word()
//! 4. Handle validation errors and busy states
//!
//! @section hal_infoc_configuration Configuration
//!
//! - @b Offset @b Validation: Ensure word-aligned access to INFOC
//! - @b Power @b States: Verify CRYPTO and OTP power states
//! - @b Busy @b Monitoring: Wait for programming completion
//! - @b Security @b Checks: Validate INFOC accessibility
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
// This is part of revision release_sdk5p1p0-366b80e084 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//
// poll on the AIB acknowledge bit
//
#define AM_HAL_INFOC_WAIT_ON_AIB_ACK_BIT()  \
    while (CRYPTO->AIBFUSEPROGCOMPLETED_b.AIBFUSEPROGCOMPLETED == 0)

//
// Check busy bit to ensure that programming has completed.
//
#define AM_HAL_OTP_WAIT_PTM_NOTBUSY()       \
    while (OTP->PTMSTAT_b.BUSY == OTP_PTMSTAT_BUSY_PTMBUSY)

//*****************************************************************************
//
//! @brief Validate the INFOC Offset and whether it is within range
//!
//! @param  offset -  word aligned offset in INFOC to be read
//!
//! @return Returns AM_HAL_STATUS_SUCCESS or OUT_OF_RANGE
//
//*****************************************************************************
static uint32_t
validate_infoc_offset(uint32_t offset)
{
    if ( offset & 0x3 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( offset > (AM_REG_OTP_INFOC_SIZE - 4) )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    return AM_HAL_STATUS_SUCCESS;

} // validate_infoc_offset()

//*****************************************************************************
//
// brief Check both that Crypto is available and that INFOC is powered up.
//
//*****************************************************************************
static uint32_t
validate_infoc_accessible(void)
{
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_OFF)   ||
        (PWRCTRL->DEVPWRSTATUS_b.PWRSTOTP    == PWRCTRL_DEVPWRSTATUS_PWRSTOTP_OFF)      ||
        (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0) )
    {
        //
        // Crypto or INFOC is not accessible
        //
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    else
    {
        //
        // Before we go, do a check for busy.
        //
        AM_HAL_OTP_WAIT_PTM_NOTBUSY();

        return AM_HAL_STATUS_SUCCESS;
    }

} // validate_infoc_accessible()

//*****************************************************************************
//
// Read INFOC word
//
// Retrieve a word of data from INFOC.
//
// Note: The caller is responsible for powering up OTP for this function.
//       The current power status of OTP can be determined via
//       am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled);
//
//       OTP can then be safely disabled after exit via:
//       am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
//       am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
//
//*****************************************************************************
uint32_t
am_hal_infoc_read_word(uint32_t offset, uint32_t *pVal)
{
    uint32_t ui32status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    ui32status = validate_infoc_offset(offset);
    if (ui32status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32status;
    }

    if ( !pVal )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif

    ui32status = validate_infoc_accessible();
    if ( ui32status != AM_HAL_STATUS_SUCCESS )
    {
        *pVal = 0x0;
        return ui32status;
    }

    *pVal = AM_REGVAL(AM_REG_OTP_INFOC_BASEADDR + offset);

    return ui32status;
} // am_hal_infoc_read_word()

//*****************************************************************************
//
// Write INFOC word
//
// Write a word to the supplied offset in the INFOC.
//
// Note: The caller is responsible for powering up OTP for this function.
//       The current power status of OTP can be determined via
//       am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bEnabled);
//
//       OTP can then be safely disabled after exit via:
//       am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
//       am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
//
//*****************************************************************************
uint32_t
am_hal_infoc_write_word(uint32_t offset, uint32_t value)
{
    uint32_t ui32status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    ui32status = validate_infoc_offset(offset);
    if (ui32status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32status;
    }
#endif

    ui32status = validate_infoc_accessible();
    if ( ui32status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32status;
    }

    AM_REGVAL(AM_REG_OTP_INFOC_BASEADDR + offset) = value;

    AM_HAL_INFOC_WAIT_ON_AIB_ACK_BIT();
    AM_HAL_OTP_WAIT_PTM_NOTBUSY();

    //
    // Read back the value to compare
    //
    if ((AM_REGVAL(AM_REG_OTP_INFOC_BASEADDR + offset) & value) != value)
    {
        ui32status = AM_HAL_STATUS_FAIL;
    }

    return ui32status;

} // am_hal_infoc_write_word()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
