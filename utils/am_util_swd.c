//*****************************************************************************
//
//! @file am_util_swd.c
//!
//! @brief Bit-bang SWD utility.
//!
//! This file provides the interface for bit-bang SWD functions.
//!
//! @addtogroup swd SWD Utility
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
#include "libswd.h"

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
libswd_ctx_t *am_util_swd_ctx;

//*****************************************************************************
//
// Static function prototypes.
//
//*****************************************************************************
void bits_to_words(uint32_t *pui32Words, uint32_t ui32Bits,
                   uint32_t ui32NumBits, bool bReverse);
void words_to_bits(uint32_t *pui32Bits, uint32_t *pui32Words,
                   uint32_t ui32NumBits, bool bReverse);
void swd_clock_wait(uint32_t ui32Delay);
void swd_clock_out(uint32_t ui32NumBits, uint32_t *pui32Data);
void swd_clock_in(uint32_t ui32NumBits, uint32_t *pui32Data);

//*****************************************************************************
//
//! @name Helpful macros.
//! @{
//
//*****************************************************************************
#define AM_UTIL_SWD_DHCSR 0xE000EDF0    // Debug Halting Control and Status Register
#define AM_UTIL_SWD_DCRSR 0xE000EDF4    // Debug Core Register Selector Register
#define AM_UTIL_SWD_DCRDR 0xE000EDF8    // Debug Core Register Data Register
#define AM_UTIL_SWD_DEMCR 0xE000EDFC    // Debug Exception and Monitor Control Register
#define AM_UTIL_SWD_AIRCR 0xE000ED0C    // Application Interrupt and Reset Control Register
//! @}

#if AM_UTIL_SWD_HALT_ON_ERROR
#define AM_UTIL_SWD_FATAL_ERROR()       while (1);
#else
#define AM_UTIL_SWD_FATAL_ERROR()
#endif

//*****************************************************************************
//
// Pin configuration macros.
//
//*****************************************************************************
#ifdef AM_PART_APOLLO3_API

#define AM_UTIL_SWD_CONFIG_SWDCLK_OUTPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWCLK_PIN, AM_UTIL_SWD_OUTPUT_CFG)

#define AM_UTIL_SWD_CONFIG_SWDIO_OUTPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWDIO_PIN, AM_UTIL_SWD_OUTPUT_CFG)

#define AM_UTIL_SWD_CONFIG_SWDIO_INPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWDIO_PIN, AM_UTIL_SWD_INPUT_CFG)

#define AM_UTIL_SWD_TRACE_PIN_CONFIG(n) \
    am_hal_gpio_pinconfig(n, g_AM_HAL_GPIO_OUTPUT)

#elif defined(AM_PART_APOLLO4_API)  || defined(AM_PART_APOLLO5_API)

am_hal_gpio_pincfg_t am_util_swd_SWDCLKCfg = AM_UTIL_SWD_OUTPUT_CFG;
am_hal_gpio_pincfg_t am_util_swd_SWDOutCfg = AM_UTIL_SWD_OUTPUT_CFG;
am_hal_gpio_pincfg_t am_util_swd_SWDInCfg = AM_UTIL_SWD_INPUT_CFG;

#define AM_UTIL_SWD_CONFIG_SWDCLK_OUTPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWCLK_PIN, am_util_swd_SWDCLKCfg)

#define AM_UTIL_SWD_CONFIG_SWDIO_OUTPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWDIO_PIN, am_util_swd_SWDOutCfg)

#define AM_UTIL_SWD_CONFIG_SWDIO_INPUT() \
    am_hal_gpio_pinconfig(AM_UTIL_SWD_SWDIO_PIN, am_util_swd_SWDInCfg)

#define AM_UTIL_SWD_TRACE_PIN_CONFIG(n) \
    am_hal_gpio_pinconfig(n, am_util_swd_SWDCLKCfg)

#endif

//*****************************************************************************
//
// Memory management.
//
//*****************************************************************************

//*****************************************************************************
//
//! @name Simple buffer for variable-size calls to malloc. Memory allocated from this
//! buffer cannot be freed.
//! @{
//
//*****************************************************************************
uint8_t g_pui8SWDMemPool[AM_UTIL_SWD_VARIABLE_POOL_SIZE];
volatile uint32_t g_ui32SWDMEMPoolIdx = 0;
//! @}

//
//! A pool of fixed-size buffers. These buffers can be freed, and are used for
//! queue entries within the LibSWD functions.
//
typedef struct
{
    uint8_t buffer[AM_UTIL_SWD_FIXED_BUFFER_SIZE];
    bool    bUsed;
}
am_buffer_pool_t;

am_buffer_pool_t am_util_swd_fixed_pool[AM_UTIL_SWD_FIXED_BUFFER_COUNT];

//*****************************************************************************
//
//! @name Memory stats. Useful for debugging memory issues.
//! @{
//
//*****************************************************************************
uint32_t g_ui32SWDMallocCount = 0;
uint32_t g_ui32SWDFreeCount = 0;
uint32_t g_ui32SWDMallocMax = 0;
uint32_t g_ui32BuffersUsed = 0;
uint32_t g_ui32MaxBuffersUsed = 0;
uint32_t g_ui32SWDMallocStats[AM_UTIL_SWD_VARIABLE_POOL_SIZE];
//! @}

//*****************************************************************************
//
// Memory management functions.
//
// These are used by LibSWD for memory allocation.
//
//*****************************************************************************

//*****************************************************************************
//
// Malloc implemtation tailored to LibSWD's needs.
//
//*****************************************************************************
void *
am_util_swd_malloc(size_t size)
{
    void *retval = NULL;
    uint32_t i;

    AM_CRITICAL_BEGIN;

    //
    // We have two memory pools to choose from. If this buffer fits in one of
    // the fixed buffers, we should prefer using that.
    //
    if (size == AM_UTIL_SWD_FIXED_BUFFER_SIZE)
    {
        //
        // Search for a free buffer in our fixed buffer pool.
        //
        for (i = 0; i < AM_UTIL_SWD_FIXED_BUFFER_COUNT; i++)
        {
            //
            // If we can find one, mark it used, and return a pointer to it.
            //
            if (am_util_swd_fixed_pool[i].bUsed == false)
            {
                am_util_swd_fixed_pool[i].bUsed = true;
                retval = &am_util_swd_fixed_pool[i].buffer;

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
    else if (g_ui32SWDMEMPoolIdx + size < AM_UTIL_SWD_VARIABLE_POOL_SIZE)
    {
        //
        // For other memory allocation sizes, we'll use the other pool. This
        // pool implementation is very simple, and doesn't support free().
        //
        retval = &g_pui8SWDMemPool[g_ui32SWDMEMPoolIdx];
        g_ui32SWDMEMPoolIdx += size;
    }

    //
    // For diagnostic purposes only...
    //
    if (size > g_ui32SWDMallocMax)
    {
        g_ui32SWDMallocMax = size;
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
        am_util_stdio_printf("SWD Util: Out of memory.\n");
        am_util_swd_print_malloc_stats();

        AM_UTIL_SWD_FATAL_ERROR();
    }
    else
    {
        g_ui32SWDMallocStats[size]++;
        g_ui32SWDMallocCount++;
    }

    return retval;
}

//*****************************************************************************
//
// Calloc implementation tailored to LibSWD's needs.
//
//*****************************************************************************
void *
am_util_swd_calloc(size_t num, size_t size)
{
    uint32_t i;
    size_t ui32ActualSize = num * size;
    void *ptr = am_util_swd_malloc(ui32ActualSize);

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
// Minimal implementation of free.
//
// Note: This implementation of free can only free memory from the fixed-size
// buffer pool. It ignores all other requests.
//
//*****************************************************************************
void
am_util_swd_free(void *ptr)
{
    am_buffer_pool_t *pBuffer;

    //
    // Check to see if this free operation is inside of our fixed pool.
    //
    if ((uint32_t) ptr >= (uint32_t) am_util_swd_fixed_pool &&
        (uint32_t) ptr < (uint32_t) am_util_swd_fixed_pool +
        sizeof(am_util_swd_fixed_pool))
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
    g_ui32SWDFreeCount++;

    return;
}

//*****************************************************************************
//
// Debug function for malloc
//
//*****************************************************************************
void
am_util_swd_print_malloc_stats(void)
{
    uint32_t i;

    am_util_stdio_printf("Malloc ptr:   %d\n", g_ui32SWDMEMPoolIdx);
    am_util_stdio_printf("Malloc count: %d\n", g_ui32SWDMallocCount);
    am_util_stdio_printf("Free count:   %d\n", g_ui32SWDFreeCount);
    am_util_stdio_printf("Malloc max:   %d\n", g_ui32SWDMallocMax);

    am_util_stdio_printf("Malloc stats:\n");

    for (i = 0; i < AM_UTIL_SWD_VARIABLE_POOL_SIZE; i++)
    {
        if (g_ui32SWDMallocStats[i])
        {
            am_util_stdio_printf("[%3d] - %4d\n", i, g_ui32SWDMallocStats[i]);
        }
    }

    uint32_t total_used_buffers = 0;
    // am_util_stdio_printf("Used buffers:\n");

    for (i = 0; i < AM_UTIL_SWD_FIXED_BUFFER_COUNT; i++)
    {
        if (am_util_swd_fixed_pool[i].bUsed)
        {
            // am_util_stdio_printf("%d (0x%08X) - Used\n", i, &am_util_swd_fixed_pool[i]);
            total_used_buffers++;
        }
    }

    am_util_stdio_printf("Total fixed buffers used: %d\n", total_used_buffers);
    am_util_stdio_printf("Max buffers used: %d\n", g_ui32MaxBuffersUsed);
}
//*****************************************************************************
//
// Usleep for LibSWD
//
//*****************************************************************************
void
am_util_swd_usleep(int microseconds)
{
#if defined(AM_PART_APOLLO4_API) || defined(AM_PART_APOLLO5_API)
    am_hal_delay_us(microseconds);
#elif defined(AM_PART_APOLLO3_API)
    am_util_delay_us(microseconds);
#else
#error "Unknown Part"
#endif
}

//*****************************************************************************
//
// SWD operations.
//
//*****************************************************************************

//*****************************************************************************
//
// Initialize the SWD interface.
//
//*****************************************************************************
uint32_t
am_util_swd_initialize(void)
{
    am_util_swd_ctx = libswd_init();

    //
    // Prepare the pins.
    //
#if IDLE_LOW
    am_hal_gpio_state_write(AM_UTIL_SWD_SWCLK_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    AM_UTIL_SWD_CONFIG_SWDCLK_OUTPUT()

    am_hal_gpio_state_write(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    AM_UTIL_SWD_CONFIG_SWDIO_OUTPUT();
#else
    am_hal_gpio_state_write(AM_UTIL_SWD_SWCLK_PIN, AM_HAL_GPIO_OUTPUT_SET);
    AM_UTIL_SWD_CONFIG_SWDCLK_OUTPUT();

    am_hal_gpio_state_write(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_OUTPUT_SET);
    AM_UTIL_SWD_CONFIG_SWDIO_OUTPUT();
#endif

    am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_SET);
    AM_UTIL_SWD_TRACE_PIN_CONFIG(7);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Deinitialize the SWD interface.
//
//*****************************************************************************
uint32_t
am_util_swd_deinitialize(void)
{
    libswd_deinit(am_util_swd_ctx);

    //
    // Reset GPIOs?
    //

    //
    // Free all memory.
    //
    g_ui32SWDMEMPoolIdx = 0;

    uint32_t i;

    for (i = 0; i < AM_UTIL_SWD_FIXED_BUFFER_COUNT; i++)
    {
        am_util_swd_fixed_pool[i].bUsed = false;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Connect to a target device.
//
//*****************************************************************************
uint32_t
am_util_swd_connect(void)
{
    uint32_t ui32ErrorCode;

    ui32ErrorCode = libswd_debug_init(am_util_swd_ctx,
                                      LIBSWD_OPERATION_EXECUTE);

    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return ui32ErrorCode;
}

//*****************************************************************************
//
// Halt the processor.
//
//*****************************************************************************
uint32_t
am_util_swd_halt(void)
{
    uint32_t ui32ErrorCode;

    ui32ErrorCode = libswd_debug_halt(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE);
    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return ui32ErrorCode;
}

//*****************************************************************************
//
// Wait for indication that the processor is halted.
//
//*****************************************************************************
bool
am_util_swd_halt_check(void)
{
    bool bHalted = 0;

    bHalted = libswd_debug_is_halted(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE);
    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return bHalted;
}

//*****************************************************************************
//
// Let the processor resume running.
//
//*****************************************************************************
uint32_t
am_util_swd_run(void)
{
    uint32_t ui32ErrorCode;

    ui32ErrorCode = libswd_debug_run(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE);
    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return ui32ErrorCode;
}

//*****************************************************************************
//
// Write to memory with 32-bit accesses.
//
//*****************************************************************************
uint32_t
am_util_swd_mem_write(uint32_t ui32Address, uint8_t *pui8Data,
                      uint32_t ui32NumBytes)
{
    int32_t i32ErrorCode;
    uint32_t ui32BytesToWrite;
    uint32_t ui32BytesRemaining;
    uint32_t ui32Offset;

    ui32BytesRemaining = ui32NumBytes;
    ui32Offset = 0;

    while (ui32BytesRemaining)
    {
        ui32BytesToWrite = (ui32BytesRemaining > AM_UTIL_SWD_WRITE_BLOCK_SIZE ?
                            AM_UTIL_SWD_WRITE_BLOCK_SIZE : ui32BytesRemaining);

        i32ErrorCode = libswd_memap_write_char(am_util_swd_ctx,
                                               LIBSWD_OPERATION_EXECUTE,
                                               ui32Address + ui32Offset,
                                               ui32BytesToWrite,
                                               (char *) &pui8Data[ui32Offset]);

        libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

        if (i32ErrorCode < 0)
        {
            AM_UTIL_SWD_FATAL_ERROR();
            return i32ErrorCode;
        }

        ui32BytesRemaining -= ui32BytesToWrite;
        ui32Offset += ui32BytesToWrite;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read from memory with 32-bit accesses.
//
//*****************************************************************************
uint32_t
am_util_swd_mem_read(uint32_t ui32Address, uint8_t *pui8Data,
                     uint32_t ui32NumBytes)
{
    int32_t i32ErrorCode;
    uint32_t ui32BytesToRead;
    uint32_t ui32BytesRemaining;
    uint32_t ui32Offset;

    ui32BytesRemaining = ui32NumBytes;
    ui32Offset = 0;

    while (ui32BytesRemaining)
    {
        ui32BytesToRead = (ui32BytesRemaining > AM_UTIL_SWD_READ_BLOCK_SIZE ?
                           AM_UTIL_SWD_READ_BLOCK_SIZE : ui32BytesRemaining);

        i32ErrorCode = libswd_memap_read_char(am_util_swd_ctx,
                                              LIBSWD_OPERATION_EXECUTE,
                                              ui32Address + ui32Offset,
                                              ui32BytesToRead,
                                              (char *) &pui8Data[ui32Offset]);

        libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

        if (i32ErrorCode < 0)
        {
            AM_UTIL_SWD_FATAL_ERROR();
            return i32ErrorCode;
        }

        ui32BytesRemaining -= ui32BytesToRead;
        ui32Offset += ui32BytesToRead;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Write to memory with 32-bit accesses.
//
//*****************************************************************************
uint32_t
am_util_swd_mem_write_words(uint32_t ui32Address, uint32_t *pui32Data,
                            uint32_t ui32NumWords)
{
    int32_t i32ErrorCode;
    uint32_t ui32WordsToWrite;
    uint32_t ui32WordsRemaining;
    uint32_t ui32Offset;

    ui32WordsRemaining = ui32NumWords;
    ui32Offset = 0;

    while (ui32WordsRemaining)
    {
        ui32WordsToWrite = (ui32WordsRemaining > AM_UTIL_SWD_WRITE_BLOCK_SIZE ?
                            AM_UTIL_SWD_WRITE_BLOCK_SIZE : ui32WordsRemaining);

        i32ErrorCode = libswd_memap_write_int_32(am_util_swd_ctx,
                                                 LIBSWD_OPERATION_EXECUTE,
                                                 ui32Address + (ui32Offset * 4),
                                                 ui32WordsToWrite,
                                                 (int *) &pui32Data[ui32Offset]);

        libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

        if (i32ErrorCode < 0)
        {
            AM_UTIL_SWD_FATAL_ERROR();
            return i32ErrorCode;
        }

        ui32WordsRemaining -= ui32WordsToWrite;
        ui32Offset += ui32WordsToWrite;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read from memory with 32-bit accesses.
//
//*****************************************************************************
uint32_t
am_util_swd_mem_read_words(uint32_t ui32Address, uint32_t *pui32Data,
                           uint32_t ui32NumWords)
{
    int32_t i32ErrorCode;
    uint32_t ui32WordsToRead;
    uint32_t ui32WordsRemaining;
    uint32_t ui32Offset;

    ui32WordsRemaining = ui32NumWords;
    ui32Offset = 0;

    while (ui32WordsRemaining)
    {
        ui32WordsToRead = (ui32WordsRemaining > AM_UTIL_SWD_READ_BLOCK_SIZE ?
                           AM_UTIL_SWD_READ_BLOCK_SIZE : ui32WordsRemaining);

        i32ErrorCode = libswd_memap_read_int_32(am_util_swd_ctx,
                                                LIBSWD_OPERATION_EXECUTE,
                                                ui32Address + (ui32Offset * 4),
                                                ui32WordsToRead,
                                                (int *) &pui32Data[ui32Offset]);

        if (i32ErrorCode < 0)
        {
            AM_UTIL_SWD_FATAL_ERROR();
            return i32ErrorCode;
        }

        libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

        ui32WordsRemaining -= ui32WordsToRead;
        ui32Offset += ui32WordsToRead;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Write a core register.
//
//*****************************************************************************
uint32_t
am_util_swd_reg_write(am_util_swd_core_reg_t eCoreReg, uint32_t ui32Value)
{
    int iResult = 0;
    int dcrsrval = eCoreReg | (1 << 16);
    int value = ui32Value;

    iResult = libswd_memap_write_int_csw(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE,
                                         AM_UTIL_SWD_DCRDR, 1, &value,
                                         LIBSWD_MEMAP_CSW_SIZE_32BIT);

    if (iResult)
    {
        return iResult;
    }

    iResult = libswd_memap_write_int(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE,
                                     AM_UTIL_SWD_DCRSR, 1, &dcrsrval);

    if (iResult)
    {
        return iResult;
    }

    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return iResult;
}

//*****************************************************************************
//
// Read a core register.
//
//*****************************************************************************
uint32_t
am_util_swd_reg_read(am_util_swd_core_reg_t eCoreReg, uint32_t *ui32Value)
{
    int iResult = 0;
    int dcrsrval = eCoreReg;
    int value = 0;

    iResult = libswd_memap_write_int_32(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE,
                                        AM_UTIL_SWD_DCRSR, 1, &dcrsrval);

    if (iResult)
    {
        return iResult;
    }

    iResult = libswd_memap_read_int_32(am_util_swd_ctx, LIBSWD_OPERATION_EXECUTE,
                                       AM_UTIL_SWD_DCRDR, 1, &value);

    if (iResult)
    {
        return iResult;
    }

    *ui32Value = value;

    libswd_cmdq_free_head(am_util_swd_ctx->cmdq);

    return iResult;
} // am_util_swd_reg_read

//*****************************************************************************
//
// LibSWD adapter functions.
//
// These functions are extern'd from the the main LibSWD libary. They translate
// between the SWD functions in the library and our specific bit-bang
// implementation.
//
//*****************************************************************************
int
libswd_drv_mosi_8(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, char *data,
                  int bits, int nLSBfirst)
{
    if (data == NULL)
    {
        return LIBSWD_ERROR_NULLPOINTER;
    }

    if (bits < 0 && bits > 8)
    {
        return LIBSWD_ERROR_PARAM;
    }

    if (nLSBfirst != 0 && nLSBfirst != 1)
    {
        return LIBSWD_ERROR_PARAM;
    }

    uint32_t pui32OutputWords[8];

    //am_util_stdio_printf("Output: 0x%08X\n", *data);
    bits_to_words(pui32OutputWords, *data, bits, nLSBfirst);
    swd_clock_out(bits, pui32OutputWords);

    return bits;
}

int
libswd_drv_mosi_32(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, int *data,
                   int bits, int nLSBfirst)
{
    if (data == NULL)
    {
        return LIBSWD_ERROR_NULLPOINTER;
    }

    if (bits < 0 && bits > 8)
    {
        return LIBSWD_ERROR_PARAM;
    }

    if (nLSBfirst != 0 && nLSBfirst != 1)
    {
        return LIBSWD_ERROR_PARAM;
    }

    uint32_t pui32OutputWords[32];

    //am_util_stdio_printf("Output: 0x%08X\n", *data);
    bits_to_words(pui32OutputWords, *data, bits, nLSBfirst);
    swd_clock_out(bits, pui32OutputWords);

    return bits;
}

int
libswd_drv_miso_8(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, char *data,
                  int bits, int nLSBfirst)
{
    if (data == NULL)
    {
        return LIBSWD_ERROR_NULLPOINTER;
    }

    if (bits < 0 && bits > 8)
    {
        return LIBSWD_ERROR_PARAM;
    }

    if (nLSBfirst != 0 && nLSBfirst != 1)
    {
        return LIBSWD_ERROR_PARAM;
    }

    uint32_t pui32InputWords[8];

    swd_clock_in(bits, pui32InputWords);
    words_to_bits((uint32_t *) data, pui32InputWords, bits, nLSBfirst);
    // am_util_stdio_printf("Input: 0x%08X\n", *data);

    return bits;
}

int
libswd_drv_miso_32(libswd_ctx_t *libswdctx, libswd_cmd_t *cmd, int *data,
                   int bits, int nLSBfirst)
{
    if (data == NULL)
    {
        return LIBSWD_ERROR_NULLPOINTER;
    }

    if (bits < 0 && bits > 8)
    {
        return LIBSWD_ERROR_PARAM;
    }

    if (nLSBfirst != 0 && nLSBfirst != 1)
    {
        return LIBSWD_ERROR_PARAM;
    }

    uint32_t pui32InputWords[32];

    swd_clock_in(bits, pui32InputWords);
    words_to_bits((uint32_t *) data, pui32InputWords, bits, nLSBfirst);
    // am_util_stdio_printf("Input: 0x%08X\n", *data);

    return bits;
}

/* This function sets interface buffers to MOSI direction.
 * Master Output Slave Input - SWD Write operation.
 * bits specify how many clock cycles must be used. */
int
libswd_drv_mosi_trn(libswd_ctx_t *libswdctx, int bits)
{
    if (bits < LIBSWD_TURNROUND_MIN_VAL && bits > LIBSWD_TURNROUND_MAX_VAL)
    {
        return LIBSWD_ERROR_TURNAROUND;
    }

    //am_hal_gpio_state_write(42, AM_HAL_GPIO_OUTPUT_SET);
    uint32_t pui32DummyWords[4] = {0, 0, 0, 0};
    swd_clock_out(bits, pui32DummyWords);

    //am_hal_gpio_state_write(42, AM_HAL_GPIO_OUTPUT_CLEAR);
    return bits;
}

int
libswd_drv_miso_trn(libswd_ctx_t *libswdctx, int bits)
{
    if (bits < LIBSWD_TURNROUND_MIN_VAL && bits > LIBSWD_TURNROUND_MAX_VAL)
    {
        return LIBSWD_ERROR_TURNAROUND;
    }

    //am_hal_gpio_state_write(42, AM_HAL_GPIO_OUTPUT_SET);
    uint32_t pui32DummyWords[4] = {0, 0, 0, 0};
    swd_clock_in(bits, pui32DummyWords);

    //am_hal_gpio_state_write(42, AM_HAL_GPIO_OUTPUT_CLEAR);
    return bits;
}

/** Set debug level according to caller's application settings.
 * \params *libswdctx swd context to work on.
 * \params loglevel caller's application log level to be converted.
 * \return LIBSWD_OK on success, of error code on failure.
 */
int
libswd_log_level_inherit(libswd_ctx_t *libswdctx, int loglevel)
{
    //if (libswdctx == NULL)
    //{
    //    // log(LOG_LEVEL_DEBUG, "libswd_log_level_inherit(): SWD Context not (yet) initialized...\n");
    //    return LIBSWD_OK;
    //}

    //libswd_loglevel_t new_swdlevel;

    //switch (loglevel)
    //{
    //    // Your code here.

    //    default:
    //        new_swdlevel = LIBSWD_LOGLEVEL_NORMAL;
    //}

    //int res = libswd_log_level_set(libswdctx, new_swdlevel);

    //if (res < 0)
    //{
    //    // Your error routine goes here...
    //    // return URJ_ERROR_SYNTAX;
    //}
    //else
    //{
    //    return LIBSWD_OK;
    //}
    //
    return LIBSWD_OK;
}

/** By default we want to use internal logging mechanisms.
 * It is possible however to use target program mechanisms to log messages.
 * In order to correctly parse variable number of arguments we need to use
 * dedicated libswd_log_internal_va() function...
 */

//char g_pcLogBuffer[1024];
//
//int
//libswd_log(libswd_ctx_t *libswdctx, libswd_loglevel_t loglevel, char *msg, ...)
//{
//    //int retval;
//    va_list ap;
//
//    AM_CRITICAL_BEGIN;
//
//    va_start(ap, msg);
//    //retval = libswd_log_internal_va(libswdctx, loglevel, msg, ap);
//    am_util_stdio_vsprintf(g_pcLogBuffer, msg, ap);
//    am_util_stdio_printf(g_pcLogBuffer);
//
//    va_end(ap);
//
//
//    AM_CRITICAL_END;
//
//    return LIBSWD_OK;
//}

#define SWDCLK_HALF_CLOCK             1 // In microseconds.
#define IDLE_LOW                      0

//*****************************************************************************
//
// Bits to words
//
//*****************************************************************************
void
bits_to_words(uint32_t *pui32Words, uint32_t ui32Bits, uint32_t ui32NumBits,
              bool bReverse)
{
    uint32_t i;

    if (bReverse == false)
    {
        for (i = 0; i < ui32NumBits; i++)
        {
            pui32Words[i] = (ui32Bits & (1 << i)) ? 1 : 0;
        }
    }
    else
    {
        for (i = 0; i < ui32NumBits; i++)
        {
            pui32Words[ui32NumBits - i - 1] = (ui32Bits & (1 << i)) ? 1 : 0;
        }
    }
}

//*****************************************************************************
//
// Words to bits.
//
//*****************************************************************************
void
words_to_bits(uint32_t *pui32Bits, uint32_t *pui32Words, uint32_t ui32NumBits,
              bool bReverse)
{
    uint32_t i;
    uint32_t ui32Index;
    uint32_t ui32Data = 0;

    for (i = 0; i < ui32NumBits; i++)
    {
        ui32Index = bReverse ? ui32NumBits - i - 1 : i;

        if (pui32Words[ui32Index])
        {
            ui32Data |= 1 << i;
        }
    }

    *pui32Bits = ui32Data;
}

//*****************************************************************************
//
// Set pin state
//
//*****************************************************************************
typedef enum
{
    SWDIO_OUTPUT_HIGH,
    SWDIO_OUTPUT_LOW,
    SWDIO_INPUT,
    SWDIO_KEEP,
}
swdio_state_e;

#define SWDCLK_HIGH     1
#define SWDCLK_LOW      0

bool g_swclk = false;
bool g_swdio = false;
bool g_input = false;

//*****************************************************************************
//
// Sets the pin state
//
//*****************************************************************************
void
set_pin_state(bool bSWCLK, swdio_state_e eSWDIO)
{
    g_swclk = bSWCLK;

    if (bSWCLK)
    {
        am_hal_gpio_state_write(AM_UTIL_SWD_SWCLK_PIN, AM_HAL_GPIO_OUTPUT_SET);
    }
    else
    {
        am_hal_gpio_state_write(AM_UTIL_SWD_SWCLK_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    }

    switch (eSWDIO)
    {
        case SWDIO_OUTPUT_HIGH:
            g_swdio = true;
            am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_SET);
            am_hal_gpio_state_write(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_OUTPUT_SET);
            break;

        case SWDIO_OUTPUT_LOW:
            g_swdio = false;
            am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_SET);
            am_hal_gpio_state_write(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
            break;

        case SWDIO_INPUT:
            g_input = true;
            am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_CLEAR);
            break;

        case SWDIO_KEEP:
            break;

        default:
            break;
    }

    return;
}

//*****************************************************************************
//
// Function for delaying to clock.
//
//*****************************************************************************
void
swd_clock_wait(uint32_t ui32Delay)
{
    am_util_delay_us(ui32Delay);
    //am_util_stdio_printf("%d, %d, %d\r\n", g_swclk, g_swdio, g_input);
}

//*****************************************************************************
//
// Clock bits out
//
//*****************************************************************************
void
swd_clock_out(uint32_t ui32NumBits, uint32_t *pui32Data)
{
    uint32_t i;
    swdio_state_e eOutputState;

    AM_UTIL_SWD_CONFIG_SWDIO_OUTPUT();

    for (i = 0; i < ui32NumBits; i++)
    {
#if IDLE_LOW
        eOutputState = pui32Data[i] ? SWDIO_OUTPUT_HIGH : SWDIO_OUTPUT_LOW;

        set_pin_state(SWDCLK_HIGH, SWDIO_KEEP);
        swd_clock_wait(SWDCLK_HALF_CLOCK);

        set_pin_state(SWDCLK_LOW, eOutputState);
        swd_clock_wait(SWDCLK_HALF_CLOCK);
#else
        eOutputState = pui32Data[i] ? SWDIO_OUTPUT_HIGH : SWDIO_OUTPUT_LOW;

        set_pin_state(SWDCLK_LOW, eOutputState);
        swd_clock_wait(SWDCLK_HALF_CLOCK);

        set_pin_state(SWDCLK_HIGH, SWDIO_KEEP);
        swd_clock_wait(SWDCLK_HALF_CLOCK);
#endif
    }
}

//*****************************************************************************
//
// Clock in SWD bits
//
//*****************************************************************************
void
swd_clock_in(uint32_t ui32NumBits, uint32_t *pui32Data)
{
    uint32_t i;

    AM_UTIL_SWD_CONFIG_SWDIO_INPUT();

    for (i = 0; i < ui32NumBits; i++)
    {
#if IDLE_LOW
        set_pin_state(SWDCLK_HIGH, SWDIO_INPUT);
        swd_clock_wait(SWDCLK_HALF_CLOCK);
        set_pin_state(SWDCLK_LOW, SWDIO_INPUT);

        //
        // Read pin.
        //
        am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_state_read(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_INPUT_READ,
                               &pui32Data[i]);
        am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_CLEAR);

        swd_clock_wait(SWDCLK_HALF_CLOCK);
#else
        set_pin_state(SWDCLK_LOW, SWDIO_INPUT);
        swd_clock_wait(SWDCLK_HALF_CLOCK);

        //
        // Read pin.
        //
        am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_state_read(AM_UTIL_SWD_SWDIO_PIN, AM_HAL_GPIO_INPUT_READ,
                               &pui32Data[i]);
        am_hal_gpio_state_write(7, AM_HAL_GPIO_OUTPUT_CLEAR);

        set_pin_state(SWDCLK_HIGH, SWDIO_INPUT);
        swd_clock_wait(SWDCLK_HALF_CLOCK);
#endif
    }

    // am_util_stdio_printf("%08X (%d)\n", pui32Data[0], 0);
    // am_util_stdio_printf("%08X (%d)\n", pui32Data[1], 1);
    // am_util_stdio_printf("%08X (%d)\n", pui32Data[2], 2);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

