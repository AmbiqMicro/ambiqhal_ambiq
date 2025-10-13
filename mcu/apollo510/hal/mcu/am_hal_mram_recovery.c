//*****************************************************************************
//
//! @file am_hal_mram_recovery.c
//!
//! @brief MRAM Recovery API.
//!
//! @addtogroup mramr_ap510 MRAM Recovery Functionality
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides MRAM (Magnetic Random Access Memory) recovery
//!          functionality for Apollo5 devices. It supports application-initiated
//!          MRAM recovery operations, status monitoring, and system reset
//!          capabilities for memory recovery and system maintenance.
//!
//! @section hal_mram_recovery_features Key Features
//!
//! 1. @b MRAM @b Recovery: Application-initiated MRAM recovery operations.
//! 2. @b Status @b Monitoring: Real-time MRAM recovery status tracking.
//! 3. @b System @b Reset: Controlled system reset for recovery operations.
//! 4. @b Key @b Validation: Secure key-based recovery authorization.
//! 5. @b Chip @b Compatibility: Support for specific chip revisions.
//!
//! @section hal_mram_recovery_functionality Functionality
//!
//! - Initialize MRAM recovery operations
//! - Monitor MRAM recovery status
//! - Handle system reset for recovery
//! - Validate recovery keys and authorization
//! - Support chip-specific recovery operations
//!
//! @section hal_mram_recovery_usage Usage
//!
//! 1. Initialize recovery using am_hal_mram_recovery_init_app_recovery()
//! 2. Monitor recovery status with am_hal_mram_recovery_read_status()
//! 3. Handle recovery operations and system reset
//! 4. Validate recovery keys and authorization
//! 5. Monitor recovery progress and completion
//!
//! @section hal_mram_recovery_configuration Configuration
//!
//! - @b Recovery @b Keys: Configure secure recovery authorization keys
//! - @b System @b Reset: Set up system reset parameters for recovery
//! - @b Chip @b Support: Configure chip-specific recovery parameters
//! - @b Status @b Monitoring: Set up recovery status tracking
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

#include "am_mcu_apollo.h"
#include "am_hal_mram_recovery.h"

//
// Application Initiated MRAM Recovery
//
uint32_t
am_hal_mram_recovery_init_app_recovery(uint32_t ui32Key, bool bReset)
{
    //
    // Check Chip Revision
    //
    if ( APOLLO5_B0 )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Check for valid key
    //
    if ( ui32Key != AM_HAL_MRAM_RECOVERY_KEY )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Write the key to the OTA Pointer register.
    //
    MCUCTRL->OTAPOINTER = ui32Key;

    //
    // Do a POI
    //
    if ( bReset )
    {
        RSTGEN->SWPOI = RSTGEN_SWPOI_SWPOIKEY_KEYVALUE;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// Read the MRAM Recovery Status
//
uint32_t
am_hal_mram_recovery_read_status(am_hal_mram_recovery_status_t *psStatus)
{

    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Mask & store the status bits.
    //
    psStatus->ui32MramRcvStatus = RSTGEN->STAT & AM_HAL_MRAM_RECOVERY_RSTGEN_STATUS_Msk;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
