//*****************************************************************************
//
//! @file am_hal_mram_recovery.c
//!
//! @brief MRAM Recovery API.
//!
//! @addtogroup mramr MRAM Recovery Functionality
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

#include "am_mcu_apollo.h"
#include "am_hal_mram_recovery.h"

//
// Application Initiated MRAM Recovery
//
uint32_t am_hal_mram_recovery_init_app_recovery(uint32_t ui32Key, bool bReset)
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
uint32_t am_hal_mram_recovery_read_status(am_hal_mram_recovery_status_t *psStatus)
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
