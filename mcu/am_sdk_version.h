//*****************************************************************************
//
//! @file am_sdk_version.h
//!
//! @brief Defines SDK version.
//!
//! @addtogroup ambiqsuite Ambiqsuite SDK
//
//! @defgroup hal mcu
//! @ingroup ambiqsuite
//! @{

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
#ifndef AM_SDK_VERSION_H
#define AM_SDK_VERSION_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Macros to define HAL SDK version.
//
//*****************************************************************************
//
// Define the current HAL version.
//
#ifndef AM_HAL_VERSION_MAJ
#if defined(AM_PART_APOLLO3_API)
#define AM_HAL_VERSION_MAJ      3
#define AM_HAL_VERSION_MIN      2
#define AM_HAL_VERSION_REV      0
#elif defined(AM_PART_APOLLO4_API)
#define AM_HAL_VERSION_MAJ      4
#define AM_HAL_VERSION_MIN      5
#define AM_HAL_VERSION_REV      0
#elif defined(AM_PART_APOLLO5_API)
#define AM_HAL_VERSION_MAJ      5
#define AM_HAL_VERSION_MIN      0
#define AM_HAL_VERSION_REV      0
#else
#define AM_HAL_VERSION_MAJ      0
#define AM_HAL_VERSION_MIN      0
#define AM_HAL_VERSION_REV      0
#endif
#endif // AM_HAL_VERSION_MAJ

#if (AM_HAL_VERSION_MAJ == 0)
#error AM_HAL_VERSION_MAJ cannot be defined as 0.
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_SDK_VERSION_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
