//*****************************************************************************
//
//! @file am_util_crypto.c
//!
//! @brief Utility functions for the crypto module.
//!
//! A few utils to help with crypto functionality
//!
//! @addtogroup crypto Crypto
//! @ingroup utils
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
#include "am_util.h"
#include "am_util_crypto.h"

//*****************************************************************************
//
// Memory management.
//
//*****************************************************************************

//
// Simple buffer for variable-size calls to malloc. Memory allocated from this
// buffer cannot be freed.
//
uint8_t g_pui8CryptoMemPool[AM_UTIL_CRYPTO_VARIABLE_POOL_SIZE];
volatile uint32_t g_ui32CryptoMEMPoolIdx = 0;

//
//! A pool of fixed-size buffers. These buffers can be freed, and are used for
//! queue entries within the LibCrypto functions.
//
typedef struct
{
    uint8_t buffer[AM_UTIL_CRYPTO_FIXED_BUFFER_SIZE];
    bool    bUsed;
}
am_buffer_pool_t;

am_buffer_pool_t am_util_crypto_fixed_pool[AM_UTIL_CRYPTO_FIXED_BUFFER_COUNT];

//
//! @name Memory stats. Useful for debugging memory issues.
//! @{
//
uint32_t g_ui32CryptoMallocCount = 0;
uint32_t g_ui32CryptoFreeCount = 0;
uint32_t g_ui32CryptoMallocMax = 0;
uint32_t g_ui32BuffersUsed = 0;
uint32_t g_ui32MaxBuffersUsed = 0;
uint32_t g_ui32CryptoMallocStats[AM_UTIL_CRYPTO_VARIABLE_POOL_SIZE];
//! @}

//*****************************************************************************
//
// Malloc implemtation tailored to LibCrypto's needs.
//
//*****************************************************************************
void *
am_util_crypto_malloc(size_t size)
{
    void *retval = NULL;
    uint32_t i;

    AM_CRITICAL_BEGIN;

    //
    // We have two memory pools to choose from. If this buffer fits in one of
    // the fixed buffers, we should prefer using that.
    //
    if (size == AM_UTIL_CRYPTO_FIXED_BUFFER_SIZE)
    {
        //
        // Search for a free buffer in our fixed buffer pool.
        //
        for (i = 0; i < AM_UTIL_CRYPTO_FIXED_BUFFER_COUNT; i++)
        {
            //
            // If we can find one, mark it used, and return a pointer to it.
            //
            if (am_util_crypto_fixed_pool[i].bUsed == false)
            {
                am_util_crypto_fixed_pool[i].bUsed = true;
                retval = &am_util_crypto_fixed_pool[i].buffer;

                //
                // Track buffer usage stats.
                //
                g_ui32BuffersUsed++;

                if (g_ui32BuffersUsed > g_ui32MaxBuffersUsed)
                {
                    g_ui32MaxBuffersUsed = g_ui32BuffersUsed;
                }

                break;
            }
        }
    }
    else if (g_ui32CryptoMEMPoolIdx + size < AM_UTIL_CRYPTO_VARIABLE_POOL_SIZE)
    {
        //
        // For other memory allocation sizes, we'll use the other pool. This
        // pool implementation is very simple, and doesn't support free().
        //
        retval = &g_pui8CryptoMemPool[g_ui32CryptoMEMPoolIdx];
        g_ui32CryptoMEMPoolIdx += size;
    }

    //
    // For diagnostic purposes only...
    //
    if (size > g_ui32CryptoMallocMax)
    {
        g_ui32CryptoMallocMax = size;
    }

    AM_CRITICAL_END;

    //
    // Error reporting routine.
    //
    if (retval == NULL)
    {
        //
        // Malloc failed.
        //
        am_util_stdio_printf("Crypto Util: Out of memory.\n");
        am_util_crypto_print_malloc_stats();

        while (1);
    }
    else
    {
        g_ui32CryptoMallocStats[size]++;
        g_ui32CryptoMallocCount++;
    }

    return retval;
}

//*****************************************************************************
//
// Calloc implementation tailored to LibCrypto's needs.
//
//*****************************************************************************
void *
am_util_crypto_calloc(size_t num, size_t size)
{
    uint32_t i;
    size_t ui32ActualSize = num * size;
    void *ptr = am_util_crypto_malloc(ui32ActualSize);

    if (ptr != NULL)
    {
        for (i = 0; i < size; i++)
        {
            ((uint8_t *)ptr)[i] = 0;
        }
    }

    return ptr;
}

//*****************************************************************************
//
// Minimal implementation of Crypto Free.
//
// Note: This implementation of free can only free memory from the fixed-size
// buffer pool. It ignores all other requests.
//
//*****************************************************************************
void
am_util_crypto_free(void *ptr)
{
    am_buffer_pool_t *pBuffer;

    //
    // Check to see if this free operation is inside of our fixed pool.
    //
    if ((uint32_t) ptr >= (uint32_t) am_util_crypto_fixed_pool &&
        (uint32_t) ptr < (uint32_t) am_util_crypto_fixed_pool +
        sizeof(am_util_crypto_fixed_pool))
    {
        //
        // Free the fixed-size buffer.
        //
        pBuffer = ptr;
        pBuffer->bUsed = false;

        //
        // Track buffer usage.
        //
        g_ui32BuffersUsed--;
    }

    //
    // Track all calls to free.
    //
    g_ui32CryptoFreeCount++;

    return;
}

//*****************************************************************************
//
// Statistics function for malloc
//
//*****************************************************************************
void
am_util_crypto_print_malloc_stats(void)
{
    uint32_t i;

    am_util_stdio_printf("Malloc ptr:   %d\n", g_ui32CryptoMEMPoolIdx);
    am_util_stdio_printf("Malloc count: %d\n", g_ui32CryptoMallocCount);
    am_util_stdio_printf("Free count:   %d\n", g_ui32CryptoFreeCount);
    am_util_stdio_printf("Malloc max:   %d\n", g_ui32CryptoMallocMax);


    am_util_stdio_printf("Malloc stats:\n");

    for (i = 0; i < AM_UTIL_CRYPTO_VARIABLE_POOL_SIZE; i++)
    {
        if (g_ui32CryptoMallocStats[i])
        {
            am_util_stdio_printf("[%3d] - %4d\n", i, g_ui32CryptoMallocStats[i]);
        }
    }

    uint32_t total_used_buffers = 0;
    // am_util_stdio_printf("Used buffers:\n");

    for (i = 0; i < AM_UTIL_CRYPTO_FIXED_BUFFER_COUNT; i++)
    {
        if (am_util_crypto_fixed_pool[i].bUsed)
        {
            // am_util_stdio_printf("%d (0x%08X) - Used\n", i, &am_util_crypto_fixed_pool[i]);
            total_used_buffers++;
        }
    }

    am_util_stdio_printf("Total fixed buffers used: %d\n", total_used_buffers);
    am_util_stdio_printf("Max buffers used: %d\n", g_ui32MaxBuffersUsed);
}

//*****************************************************************************
//
// Crypto Exit
//
//*****************************************************************************
void
am_util_crypto_exit(uint32_t exitCode)
{
    while (1);
}

//*****************************************************************************
//
// Crypto Abort
//
//*****************************************************************************
void
am_util_crypto_abort(void)
{
    while (1);
}

//*****************************************************************************
//
// Local structs for use with am_util_crypto_get_time
//
//*****************************************************************************
am_hal_rtc_time_t *g_am_util_rtc_time;
struct tm *g_gm_time;

//*****************************************************************************
//
// Changes Ambiq RTC time to gmtime
//
//*****************************************************************************
static void ambiq_time_to_gmtime_set(const am_hal_rtc_time_t *atm, struct tm *tm)
{
    tm->tm_year = atm->ui32Year;
    if (atm->ui32CenturyBit == 0)
    {
        tm->tm_year += 200;
    }
    else
    {
        tm->tm_year += 100;
    }
    tm->tm_wday = atm->ui32Weekday;
    tm->tm_mon = atm->ui32Month - 1;
    tm->tm_mday = atm->ui32DayOfMonth;
    tm->tm_hour = atm->ui32Hour;
    tm->tm_min = atm->ui32Minute;
    tm->tm_sec = atm->ui32Second;
}

//*****************************************************************************
//
// Crypto Set Ambiq RTC structure for use with am_util_crypto_get_time
//
//*****************************************************************************
void am_util_crypto_ambiq_time_structure_set(am_hal_rtc_time_t *atm)
{
    g_am_util_rtc_time = atm;
}

//*****************************************************************************
//
// Crypto Set gmtime structure for use with am_util_crypto_get_time
//
//*****************************************************************************
void am_util_crypto_gm_time_structure_set(struct tm *tm)
{
    g_gm_time = tm;
}

//*****************************************************************************
//
// Crypto Get Time for mbedtls libraries
// Calculates time in seconds since 1/1/1970 until current RTC time
//
//*****************************************************************************
time_t am_util_crypto_get_time( time_t * time )
{
    (void)time;

    am_hal_rtc_time_get(g_am_util_rtc_time);
    ambiq_time_to_gmtime_set(g_am_util_rtc_time, g_gm_time);

    //
    // Calc time in seconds since 1/1/1970 until current RTC time.
    //
    return mktime(g_gm_time);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

