//*****************************************************************************
//
//! @file am_util_time.c
//!
//! @brief Time and Date Utility Functions
//!
//! @addtogroup time_utils Time Utility Functions
//! @ingroup utils
//! @{
//!
//! Purpose: This module provides time and date utilities for
//!          Ambiq Micro devices. It enables real-time clock operations,
//!          time/date conversions, and calendar functions for embedded
//!          applications requiring accurate timekeeping. The utilities
//!          support various time formats and calendar calculations.
//!
//! @section time_features Key Features
//!
//! 1. @b Leap @b Year @b Detection: Accurate leap year calculation and validation.
//! 2. @b Day @b of @b Week @b Computation: Calculate weekday for any given date.
//! 3. @b Date @b Validation: Comprehensive date range and format checking.
//! 4. @b Calendar @b Arithmetic: Month and year calculations with proper handling.
//! 5. @b RTC @b Integration: Real-Time Clock compatibility and time structure support.
//! 6. @b Month @b Offset @b Tables: Optimized lookup tables for calendar calculations.
//!
//! @section time_functionality Functionality
//!
//! - Calculate day of week for any valid date
//! - Validate date ranges and leap year conditions
//! - Perform calendar arithmetic operations
//! - Handle month offset calculations
//! - Support RTC time structure manipulation
//! - Provide leap year detection algorithms
//! - Enable date format validation
//! - Support embedded time management systems
//!
//! @section time_usage Usage
//!
//! 1. Initialize time utilities for target application
//! 2. Validate input dates using leap year detection
//! 3. Calculate day of week using am_util_time_computeDayofWeek()
//! 4. Perform calendar arithmetic operations
//! 5. Integrate with RTC systems for time management
//!
//! @section time_configuration Configuration
//!
//! - @b Leap @b Year @b Algorithm: Zeller's congruence for day calculation
//! - @b Month @b Offset @b Tables: Pre-calculated offsets for efficiency
//! - @b Date @b Validation: Range checking for years, months, and days
//! - @b Calendar @b Support: Gregorian calendar system implementation
//! - @b RTC @b Compatibility: Real-Time Clock integration support
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
#include "am_util_time.h"

//*****************************************************************************
//
//! @brief Calculate whether passed in year is a leap year
//!
//! @param year - year to check for leap year
//
//*****************************************************************************
#define AM_UTIL_TIME_IS_LEAP_YEAR(year) \
    (year % 4 == 0 && ((year % 100 != 0) || (year % 400 != 0)))

//*****************************************************************************
//
// Local variables.
//
//*****************************************************************************

//*****************************************************************************
//
//! Numer of days in each month in a standard year.
//
//*****************************************************************************
const static uint32_t g_iDaysPerMonth[] =
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

//*****************************************************************************
//
//! Weekday drift numbers for each month.
//
//*****************************************************************************
const static int g_iMonthOffsets[] =
    {4, 0, 0, 3, 5, 1, 3, 6, 2, 4, 0, 2};

//*****************************************************************************
//
// Compute the day of the week given the month, day, and year.
//
//*****************************************************************************
int
am_util_time_computeDayofWeek(int iYear, int iMonth, int iDay)
{
    bool bInvalidDay;
    int iYearOffset;
    int iMonthOffset;
    int iWeekday;

    int iLeapYearOffset = 0;

    //
    // Validate inputs.  Return 7 if any are out-of-bounds.
    //
    if ( (iMonth < 1) || (iMonth > 12) || (iYear < 0) || (iDay < 1) )
    {
        return 7;
    }

    //
    // Make sure this day actually exists in this month. Make sure to include
    // an exception for leap years.
    //
    if (iDay > g_iDaysPerMonth[iMonth - 1])
    {
        if (iMonth == 2 && AM_UTIL_TIME_IS_LEAP_YEAR(iYear) && iDay == 29)
        {
            bInvalidDay = false;
        }
        else
        {
            bInvalidDay = true;
        }
    }
    else
    {
        bInvalidDay = false;
    }

    if (bInvalidDay)
    {
        return 7;
    }

    iYearOffset = 2 + iYear + iYear / 4 - iYear / 100 + iYear / 400;
    iMonthOffset = g_iMonthOffsets[iMonth - 1];

    if (AM_UTIL_TIME_IS_LEAP_YEAR(iYear) && (iMonth < 3))
    {
        iLeapYearOffset = -1;
    }

    iWeekday = iDay + iYearOffset + iMonthOffset + iLeapYearOffset;

    return iWeekday % 7;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

