//*****************************************************************************
//
//! @file am_util_math.c
//!
//! @brief Mathematical Utility Functions
//!
//! @addtogroup math_utils Math Utility Functions
//! @ingroup utils
//! @{
//!
//! Purpose: This module provides mathematical utility functions for
//!          embedded applications on Ambiq Micro devices. It enables
//!          efficient mathematical operations, data manipulation, and
//!          computational support for system operations. The utilities
//!          support optimized implementations of common math functions.
//!
//! @section utils_math_features Key Features
//!
//! 1. @b Fast @b Math: Optimized mathematical operations.
//! 2. @b Data @b Manipulation: Efficient data handling functions.
//! 3. @b Fixed-point: Fixed-point arithmetic support.
//! 4. @b Precision @b Control: Configurable computation accuracy.
//! 5. @b Performance: Optimized for embedded systems.
//!
//! @section utils_math_functionality Functionality
//!
//! - Perform mathematical calculations
//! - Handle data conversions
//! - Support fixed-point operations
//! - Optimize computational tasks
//! - Manage precision requirements
//!
//! @section utils_math_usage Usage
//!
//! 1. Use math functions as needed
//! 2. Select appropriate precision
//! 3. Handle data conversions
//! 4. Monitor performance impact
//!
//! @section utils_math_configuration Configuration
//!
//! - Set precision requirements
//! - Configure optimization levels
//! - Define data formats
//! - Set performance targets
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
#include "am_util_math.h"

//*****************************************************************************
//
// Converts a Binary Coded Decimal (BCD) byte to its Decimal form.
//
//*****************************************************************************
uint8_t
am_util_math_bcd_to_dec(uint8_t ui8BCDByte)
{
  return (((ui8BCDByte & 0xF0) >> 4) * 10) + (ui8BCDByte & 0x0F);
}

//*****************************************************************************
//
//Converts a Decimal byte to its Binary Coded Decimal (BCD) form.
//
//*****************************************************************************
uint8_t
am_util_math_dec_to_bcd(uint8_t ui8DecimalByte)
{
  return (((ui8DecimalByte / 10) << 4) | (ui8DecimalByte % 10));
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

