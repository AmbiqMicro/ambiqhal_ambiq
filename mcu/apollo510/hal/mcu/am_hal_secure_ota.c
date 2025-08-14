//*****************************************************************************
//
//! @file am_hal_secure_ota.c
//!
//! @brief Functions for secure over-the-air.
//!
//! @addtogroup secure_ota_ap510 Secure OTA Functionality
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides functions for secure over-the-air (OTA) update
//!          functionality on Apollo5 devices. It supports secure firmware updates,
//!          OTA descriptor management, and image validation for secure remote
//!          firmware updates and system maintenance.
//!
//! @section hal_secure_ota_features Key Features
//!
//! 1. @b Secure @b Updates: Secure over-the-air firmware update support.
//! 2. @b OTA @b Descriptor: Management of OTA descriptor structures.
//! 3. @b Image @b Validation: Secure image validation and verification.
//! 4. @b Flash @b Programming: Secure flash programming for updates.
//! 5. @b Status @b Monitoring: OTA status and progress monitoring.
//!
//! @section hal_secure_ota_functionality Functionality
//!
//! - Initialize OTA functionality and descriptor management
//! - Add new firmware images to OTA descriptor
//! - Validate and verify firmware images
//! - Handle secure flash programming operations
//! - Monitor OTA status and progress
//!
//! @section hal_secure_ota_usage Usage
//!
//! 1. Initialize OTA using am_hal_ota_init()
//! 2. Add firmware images with am_hal_ota_add()
//! 3. Monitor OTA status and progress
//! 4. Handle secure flash programming
//! 5. Validate firmware images as needed
//!
//! @section hal_secure_ota_configuration Configuration
//!
//! - @b Flash @b Keys: Configure secure flash programming keys
//! - @b OTA @b Descriptor: Set up OTA descriptor structures
//! - @b Image @b Validation: Configure image validation parameters
//! - @b Security: Set up secure update parameters
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
//! Local defines
//
#define FLASH_INVALID               0xFFFFFFFF

//
//! Internal OTA state information
//
typedef struct
{
    uint32_t mramSize;
    uint32_t otaDescAddr;
    uint32_t numOta;
} am_hal_secure_ota_state_t;

static am_hal_secure_ota_state_t gSOtaState;

//*****************************************************************************
//
// Initializes the OTA state. This should be called before doing any other operation
//
//*****************************************************************************
uint32_t
am_hal_ota_init(uint32_t ui32ProgramKey, am_hal_otadesc_t *pOtaDesc)
{
    am_hal_mcuctrl_device_t sDevice;
    uint32_t otaDescAddr = (uint32_t)pOtaDesc;
    uint32_t status = AM_HAL_STATUS_SUCCESS;

    //
    // Get chip specific info
    //
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);
    gSOtaState.mramSize = sDevice.ui32MRAMSize;

    // Validate the flash page
    if ( (otaDescAddr < MRAM_BASEADDR) ||
        (otaDescAddr > (MRAM_BASEADDR + gSOtaState.mramSize - sizeof(am_hal_otadesc_t))) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    // CAUTION - We do not check against protected pages
    status = am_hal_mram_main_fill(ui32ProgramKey, FLASH_INVALID, (uint32_t *)pOtaDesc, sizeof(am_hal_otadesc_t));
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        // Initialize the OTA Pointer
        MCUCTRL->OTAPOINTER = otaDescAddr;
        gSOtaState.numOta = 0;
        gSOtaState.otaDescAddr = otaDescAddr;
    }

    return status;
}

// Add a new OTA to descriptor
//*****************************************************************************
//
// Adds a new image to the OTA Descriptor.
//
//*****************************************************************************
uint32_t
am_hal_ota_add(uint32_t ui32ProgamKey, uint8_t imageMagic, uint32_t *pImage)
{
    uint32_t imageAddr = (uint32_t)pImage;
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    // Validate the Image Pointer
    if (imageAddr > (MRAM_BASEADDR + gSOtaState.mramSize) ||
        imageAddr < MRAM_BASEADDR )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if (gSOtaState.numOta == AM_HAL_SECURE_OTA_MAX_OTA)
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    imageAddr |= AM_HAL_OTA_STATUS_PENDING;
    // Program the OTA Descriptor word
    status = am_hal_mram_main_words_program(ui32ProgamKey,
                &imageAddr,
                ((uint32_t *)gSOtaState.otaDescAddr + gSOtaState.numOta++),
                1);

    if (status == AM_HAL_STATUS_SUCCESS)
    {
        // Set appropriate OTA Pointer bits
        MCUCTRL->OTAPOINTER_b.OTAVALID = 1;
    }

    return status;
}

//*****************************************************************************
//
//  Get Current OTA Descriptor state
//
//*****************************************************************************
uint32_t
am_hal_get_ota_status(uint32_t maxOta, am_hal_ota_status_t *pStatus, uint32_t *pOtaDescStatus)
{
    uint32_t numOta = 0;
    am_hal_otadesc_t *pOtaDesc = (am_hal_otadesc_t *)AM_HAL_OTA_GET_BLOB_PTR(MCUCTRL->OTAPOINTER);

    if ( ((uint32_t)pOtaDesc < MRAM_BASEADDR) ||
          (uint32_t)pOtaDesc > (MRAM_BASEADDR + gSOtaState.mramSize - sizeof(am_hal_otadesc_t)) )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
    // Fill up the return structure
    while (maxOta--)
    {
        if (pOtaDesc->upgrade[numOta] == FLASH_INVALID)
        {
            pStatus[numOta].pImage = (uint32_t *)pOtaDesc->upgrade[numOta];
            break;
        }
        else
        {
            pStatus[numOta].pImage = (uint32_t *)AM_HAL_OTA_GET_BLOB_PTR(pOtaDesc->upgrade[numOta]);
            pStatus[numOta].status = (am_hal_ota_status_e)(pOtaDesc->upgrade[numOta] & AM_HAL_OTA_STATUS_MASK);
        }
        numOta++;
    }
    *pOtaDescStatus = MCUCTRL->OTAPOINTER & AM_HAL_OTADESC_STATUS_MASK;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initializes the SBR OTA state.
//
//*****************************************************************************
uint32_t
am_hal_sbr_ota_init(uint32_t *pImage)
{
    am_hal_mcuctrl_device_t sDevice;
    uint32_t imageAddr = (uint32_t)pImage;

    //
    // Get chip specific info
    //
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);
    gSOtaState.mramSize = sDevice.ui32MRAMSize;

    // Validate the Image Pointer - It needs to be aligned to 8 bytes at minimum
    // NOTE - Currently this only supports OTA image stored in MRAM
    // We might be able to enhance it to be in SRAM in future
    if ( (imageAddr < MRAM_BASEADDR) ||
         ((imageAddr + AM_SBR_OTA_MIN_SIZE_BYTES) > (MRAM_BASEADDR + gSOtaState.mramSize)) ||
         (imageAddr & AM_HAL_OTA_STATUS_SBR_PENDING)
        )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    // Initialize the OTA Pointer
    // Least significant 3 bits need to be set to indicate it is a valid SBR OTA
    MCUCTRL->OTAPOINTER = (imageAddr | AM_HAL_OTA_STATUS_SBR_PENDING);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Get Current SBR OTA Status
//
//*****************************************************************************
uint32_t
am_hal_get_sbr_ota_status(am_hal_ota_status_t *pStatus)
{
    *((uint32_t *)pStatus) = MCUCTRL->OTAPOINTER & AM_HAL_SBR_OTA_STATUS_MASK;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
