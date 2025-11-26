//*****************************************************************************
//
//! @file am_hal_puf.h
//!
//! @brief Hardware abstraction for the PUF (Physically Unclonable Function)
//!
//! @addtogroup puf_ap510 PUF - Physically Unclonable Function
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
// This is part of revision stable-c286075505 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_HAL_PUF_H
#define AM_HAL_PUF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Public PUF entropy APIs
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Read PUF UID data into a buffer.
//!
//! This function retrieves one or more 32-bit words from the PUF UID registers
//! (UID_PUF_000 through UID_PUF_031) and copies them into the caller's buffer.
//!
//! @param pui32Buffer Pointer to a uint32_t array to receive the UID words.
//! @param ui32StartWord Starting word index (0..31).
//! @param ui32NumWords Number of consecutive words to read (1..32).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, AM_HAL_STATUS_FAIL on failure.
//
//*****************************************************************************
extern uint32_t am_hal_puf_get_uid(uint32_t *pui32Buffer, uint32_t ui32StartWord, uint32_t ui32NumWords);

//*****************************************************************************
//
//! @brief Retrieve entropy into a buffer.
//!
//! @param buffer Pointer to output buffer to receive entropy bytes.
//! @param length Number of bytes requested to fill in buffer.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, AM_HAL_STATUS_FAIL on failure.
//
//*****************************************************************************
extern uint32_t am_hal_puf_get_entropy(uint8_t *buffer, uint16_t length);

//*****************************************************************************
//
//! @brief Initialize the entropy peripheral (power on OTP).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, AM_HAL_STATUS_FAIL on failure.
//
//*****************************************************************************
extern uint32_t am_hal_puf_entropy_init(void);

//*****************************************************************************
//
//! @brief Deinitialize the entropy peripheral (power down OTP).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, AM_HAL_STATUS_FAIL on failure.
//
//*****************************************************************************
extern uint32_t am_hal_puf_entropy_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_PUF_H


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
