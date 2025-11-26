//*****************************************************************************
//
//! @file am_hal_puf.c
//!
//! @brief Hardware abstraction for the PUF (Physically Unclonable Function)
//!
//! @addtogroup puf_ap510 PUF - Physically Unclonable Function
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides hardware abstraction layer functions for
//!          the device's PUF-based entropy source, RNG, and Static UID that
//!          are memory mapped into OTP. It offers initialization, entropy
//!          retrieval, unique identifier access, and power management helpers
//!          for examples and higher-level components to consume hardware
//!          entropy and device identification safely.
//!
//! @section puf_features Key Features
//!
//! 1. @b PUF-based @b RNG: Read entropy words from the OTP-mapped TRNG for
//!    cryptographically secure random number generation.
//! 2. @b Static @b UID @b Access: Read 1024-bit (32 words) unique device
//!    identifier from PUF registers for device identification, authentication,
//!    and local data protection.
//! 3. @b Power @b Management: APIs to enable/disable OTP power around
//!    entropy and UID operations.
//! 4. @b Simple @b API: Minimal blocking helpers for entropy collection and
//!    UID retrieval.
//!
//! @section puf_functionality Functionality
//!
//! - Initialize and deinitialize the OTP-based PUF peripheral.
//! - Read raw entropy words and fill caller buffers of arbitrary size.
//! - Read one or more words from the 32-word Static UID register array.
//! - Automatically manage OTP power state (only powers down if this module
//!   powered it up).
//! - Return standard HAL status codes for integration with other HAL APIs.
//!
//! @section puf_usage Usage
//!
//! Entropy (RNG) Usage:
//! 1. Call am_hal_puf_entropy_init() to power on OTP before reads.
//! 2. Call am_hal_puf_get_entropy() to retrieve entropy into a buffer.
//! 3. Call am_hal_puf_entropy_deinit() to power down OTP when finished.
//!
//! Static UID Usage:
//! 1. Call am_hal_puf_entropy_init() to power on OTP before reads.
//! 2. Call am_hal_puf_get_uid() to read UID words into a buffer.
//! 3. Call am_hal_puf_entropy_deinit() to power down OTP when finished.
//!
//! @section puf_configuration Configuration
//!
//! - No configuration is required for basic entropy or UID reads beyond
//!   powering OTP.
//! - Higher-level consumers may implement additional conditioning if required.
//! - The Static UID provides 1024 bits (32 × 32-bit words) of device-unique
//!   data suitable for fingerprinting and security applications.
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
// This is part of revision stable-c286075505 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_util.h"
#include <string.h>
#include "am_hal_puf.h"

//
// Compiler portable MIN macro in C.
//
#define MIN(a, b) ((a) < (b) ? (a) : (b))

//
// Macro to read PUF UID register by index (0..31).
// PUFUID is at offset 0x300, each word is at 0x300 + (n * 4).
//
#define GET_PUF_UID(n) (*((volatile uint32_t *)((uint32_t)&OTP->PUFUID + ((n) * 4))))

//
// Track whether OTP was enabled prior to calling am_hal_puf_entropy_init().
// If true, we must not power-down OTP in am_hal_puf_entropy_deinit().
//
static bool g_bPufOtpWasEnabled = false;

//*****************************************************************************
//
//! @brief Read a 32-bit entropy word from the OTP RNG register.
//!
//! This is a file-local helper that performs a single read from the memory
//! mapped OTP RNG register. The function is declared static inline and is not
//! exposed outside this translation unit.
//!
//! @return 32-bit raw entropy value read from OTP->RNG.
//
//*****************************************************************************
__STATIC_FORCEINLINE uint32_t
get_entropy_u32(void)
{
    return OTP->RNG;
}

//*****************************************************************************
//
//! @brief Read PUF UID data into a buffer.
//!
//! This function retrieves one or more 32-bit words from the PUF UID registers
//! (UID_PUF_000 through UID_PUF_031) and copies them into the caller's buffer.
//!
//! @param pui32Buffer - Pointer to a uint32_t array to receive the UID words.
//! @param ui32StartWord - Starting word index (0..31).
//! @param ui32NumWords - Number of consecutive words to read (1..32).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
uint32_t
am_hal_puf_get_uid(uint32_t *pui32Buffer, uint32_t ui32StartWord, uint32_t ui32NumWords)
{
    //
    // Validate input parameters.
    //
    if (pui32Buffer == NULL || ui32StartWord > 31 || ui32NumWords == 0 || (ui32StartWord + ui32NumWords) > 32)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Read the requested UID words.
    //
    for (uint32_t i = 0; i < ui32NumWords; i++)
    {
        pui32Buffer[i] = GET_PUF_UID(ui32StartWord + i);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Retrieve entropy into a buffer.
//
//*****************************************************************************
uint32_t
am_hal_puf_get_entropy(uint8_t * buffer, uint16_t length)
{
    //
	// Validate input parameters.
	//
    if (length == 0 || buffer == NULL)
    {
        return AM_HAL_STATUS_FAIL;
    }

    uint8_t *byte_buffer = buffer;
    uint8_t fail_cnt = 0;

    //
    // While the passed in length is greater than zero grab data from RNG
    // and save to output.
    //
    while ((length > 0) & (fail_cnt < 5))
    {
        uint32_t word = get_entropy_u32();
        size_t copy_length = MIN(sizeof(uint32_t), length);

        //
        // This is a failure mode where the RNG doesn't have enough randomness
        //
        if (word == 0xdeaddead)
        {
            fail_cnt++;
            continue;
        }

        memcpy(byte_buffer, &word, copy_length);
        byte_buffer += copy_length;
        length -= copy_length;
    }

    if (fail_cnt == 6)
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize the entropy peripheral (power on OTP).
//
//*****************************************************************************
uint32_t
am_hal_puf_entropy_init(void)
{
    uint32_t ui32Status = 0;
    bool bPeripheralEnabled = false;

    //
    // Check if OTP is already on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bPeripheralEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Error during read of OTP power status\n");
        return AM_HAL_STATUS_FAIL;
    }

    //
	// Record whether OTP was already enabled before we try to enable it.
	//
    g_bPufOtpWasEnabled = bPeripheralEnabled;

    //
    // Power on OTP if it is not already on.
    //
    if (!bPeripheralEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Error in power down of OTP module\n");
            return AM_HAL_STATUS_FAIL;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Deinitialize the entropy peripheral (power down OTP).
//
//*****************************************************************************
uint32_t
am_hal_puf_entropy_deinit(void)
{
    uint32_t ui32Status = 0;
    bool bPeripheralEnabled = false;

    //
    // Check if OTP is on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bPeripheralEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Error during read of OTP power status\n");
        return AM_HAL_STATUS_FAIL;
    }

    //
	// If OTP is already off, or OTP was already enabled before our init call,
    // there is nothing for us to do here — return success.
	//
    if (!bPeripheralEnabled || g_bPufOtpWasEnabled)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
	// Otherwise, OTP is on and we enabled it during init; disable it now.
	//
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Error in power down of OTP module\n");
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
