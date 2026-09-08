//*****************************************************************************
//
//! @file am_hal_cc312_sha.c
//!
//! @brief Hardware abstraction for CryptoCell-312 SHA-1 / SHA-256 hashing
//!
//! @addtogroup cc312_sha_ap510L CC312 SHA - Hash Engine
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides hardware-accelerated SHA-1 and SHA-256 message
//!          digests using the CryptoCell-312 HASH engine. It mirrors the
//!          structure and DMA-coherency handling of am_hal_cc312_aes.c and
//!          am_hal_cc312_chacha.c: the running hash state and message length are
//!          loaded into the engine, data is streamed through it by DMA, and the
//!          state is read back. An mbed TLS-style update/finish API plus one-shot
//!          helpers are exposed.
//!
//! @section hal_cc312_sha_usage Usage
//!
//! 1. Initialize context using am_hal_cc312_sha_context_init()
//! 2. Add data using am_hal_cc312_sha_update() (any number of times)
//! 3. Produce the digest using am_hal_cc312_sha_finish()
//! 4. Free context using am_hal_cc312_sha_free()
//!
//! The one-shot helpers am_hal_cc312_sha1()/am_hal_cc312_sha256() perform all of the above
//! for a single buffer.
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! @name HASH engine register values (match the CC312 runtime hash driver).
//! @{
//
//*****************************************************************************
#define HW_HASH_CTL_SHA1_VAL            0x1UL   //!< HASH_CONTROL value for SHA-1.
#define HW_HASH_CTL_SHA256_VAL          0x2UL   //!< HASH_CONTROL value for SHA-256.
#define HASH_DO_PAD_VAL                 0x4UL   //!< HASH_PAD_CFG value to trigger padding.
//! @}

//
//! SHA-1 initial hash value (H0..H4).
//
static const uint32_t g_am_hal_sha1_iv[5] =
{
    0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0
};

//
//! SHA-256 initial hash value (H0..H7).
//
static const uint32_t g_am_hal_sha256_iv[8] =
{
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
};

//*****************************************************************************
//
//! @brief Number of 32-bit digest words for a given mode.
//!
//! @param mode SHA mode selector.
//!
//! @return 5 for SHA-1, 8 for SHA-256.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_sha_digest_words(am_hal_cc312_sha_mode_e mode)
{
    return (mode == AM_HAL_CC312_SHA1) ? 5U : 8U;
}

//*****************************************************************************
//
// Enable HASH and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_sha_enable_clocks(void)
{
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_HASH);
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_DMA);
}

//*****************************************************************************
//
// Disable HASH and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_sha_disable_clocks(void)
{
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_DMA);
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_HASH);
}

//*****************************************************************************
//
// Initialize SHA context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_context_init(am_hal_cc312_sha_context_t *ctx,
                              am_hal_cc312_sha_mode_e mode)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_sha_context_t));
    ctx->mode = mode;

    switch (mode)
    {
        case AM_HAL_CC312_SHA1:
            memcpy(ctx->digest, g_am_hal_sha1_iv, sizeof(g_am_hal_sha1_iv));
            break;
        case AM_HAL_CC312_SHA256:
            memcpy(ctx->digest, g_am_hal_sha256_iv, sizeof(g_am_hal_sha256_iv));
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Free (clear) SHA context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_free(am_hal_cc312_sha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_sha_context_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize HASH hardware registers for a hash pass.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_init(am_hal_cc312_sha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify a valid hash mode.
    //
    if ((ctx->mode != AM_HAL_CC312_SHA1) && (ctx->mode != AM_HAL_CC312_SHA256))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Make sure symmetric engines are ready to use.
    //
    am_hal_cc312_wait_crypto_busy();

    //
    // Clear all interrupts before starting the engine.
    //
    am_hal_cc312_clear_interrupt(0xFFFFFFFFU);

    //
    // Mask DMA interrupts which are not required.
    //
    am_hal_cc312_config_dma_interrupt_mask();

    //
    // Configure the HASH data flow and select the hash (not AES-MAC) module.
    //
    am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_HASH);
    CRYPTO->HASHPADEN     = 1;
    CRYPTO->HASHSELAESMAC = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load running hash value and length into HASH hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_load_state(am_hal_cc312_sha_context_t *ctx)
{
    uint32_t hashCtrl;

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (ctx->mode)
    {
        case AM_HAL_CC312_SHA1:
            hashCtrl = HW_HASH_CTL_SHA1_VAL;
            break;
        case AM_HAL_CC312_SHA256:
            hashCtrl = HW_HASH_CTL_SHA256_VAL;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Load the current message length being processed.
    //
    CRYPTO->HASHCURLEN0 = ctx->totalDataSizeProcessed[0];
    CRYPTO->HASHCURLEN1 = ctx->totalDataSizeProcessed[1];

    //
    // Select the hash mode, then load the running hash value. The H registers
    // can only be written once the hash mode has been selected.
    //
    CRYPTO->HASHCONTROL = hashCtrl;
    if (ctx->mode == AM_HAL_CC312_SHA256)
    {
        CRYPTO->HASHH7 = ctx->digest[7];
        CRYPTO->HASHH6 = ctx->digest[6];
        CRYPTO->HASHH5 = ctx->digest[5];
    }
    CRYPTO->HASHH4 = ctx->digest[4];
    CRYPTO->HASHH3 = ctx->digest[3];
    CRYPTO->HASHH2 = ctx->digest[2];
    CRYPTO->HASHH1 = ctx->digest[1];
    CRYPTO->HASHH0 = ctx->digest[0];

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Store running hash value and length from HASH hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_store_state(am_hal_cc312_sha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ctx->mode == AM_HAL_CC312_SHA256)
    {
        ctx->digest[7] = CRYPTO->HASHH7;
        ctx->digest[6] = CRYPTO->HASHH6;
        ctx->digest[5] = CRYPTO->HASHH5;
    }
    ctx->digest[4] = CRYPTO->HASHH4;
    ctx->digest[3] = CRYPTO->HASHH3;
    ctx->digest[2] = CRYPTO->HASHH2;
    ctx->digest[1] = CRYPTO->HASHH1;
    ctx->digest[0] = CRYPTO->HASHH0;
    ctx->totalDataSizeProcessed[0] = CRYPTO->HASHCURLEN0;
    ctx->totalDataSizeProcessed[1] = CRYPTO->HASHCURLEN1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Run one HASH-engine pass over a data block (or finalize padding).
//!
//! Loads the running state, streams @p len bytes through the engine by DMA,
//! then reads the updated state back. When @p len is zero the engine completes
//! padding for the previously fed message (DO_PAD). When @p bLast is true and
//! @p len is non-zero, hardware auto-padding finalizes the digest.
//!
//! @param ctx   Pointer to SHA context (holds state to load/store).
//! @param data  Input data buffer (ignored when @p len is zero).
//! @param len   Number of bytes to process.
//! @param bLast True when this pass completes the message (apply HW padding).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_sha_process(am_hal_cc312_sha_context_t *ctx,
                         const uint8_t *data,
                         uint32_t len,
                         bool bLast)
{
    uint32_t status;

    if (ctx == NULL || (len != 0U && data == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Enable clocks.
    //
    am_hal_cc312_sha_enable_clocks();

    status = am_hal_cc312_sha_init(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Load the running hash value and message length.
    //
    status = am_hal_cc312_sha_load_state(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    if (len == 0U)
    {
        //
        // No data: complete padding for the previously fed message.
        //
        CRYPTO->HASHPADCFG = HASH_DO_PAD_VAL;
    }
    else
    {
        //
        // Enable hardware auto-padding for the final block of the message.
        //
        if (bLast)
        {
            CRYPTO->AUTOHWPADDING = 1;
        }

        //
        // Treat buffers as secure and make the input visible to the engine.
        //
        am_hal_cc312_set_buffer_security(false, false);
        am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)data, len);

        //
        // Stream the data through the engine (kicks the DMA) and wait.
        //
        am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR,
                                    (uint32_t)(uintptr_t)data,
                                    len);
        status = am_hal_cc312_wait_interrupt(CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            goto process_exit;
        }
    }

    //
    // Capture the updated running hash value and length.
    //
    status = am_hal_cc312_sha_store_state(ctx);

process_exit:
    //
    // Restore the padding registers to their default state.
    //
    CRYPTO->HASHPADEN     = 1;
    CRYPTO->AUTOHWPADDING = 0;
    CRYPTO->HASHPADCFG    = 0;

    am_hal_cc312_sha_disable_clocks();

    return status;
}

//*****************************************************************************
//
// Feed a buffer of data into a SHA operation.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_update(am_hal_cc312_sha_context_t *ctx,
                  const uint8_t *input,
                  uint32_t ilen)
{
    uint32_t status;
    uint32_t fill;
    uint32_t full;

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ilen == 0U)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (input == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Top off any partially buffered block first.
    //
    if (ctx->blockLen > 0U)
    {
        fill = AM_HAL_SHA_BLOCK_SIZE - ctx->blockLen;
        if (fill > ilen)
        {
            fill = ilen;
        }

        memcpy(ctx->block + ctx->blockLen, input, fill);
        ctx->blockLen += fill;
        input         += fill;
        ilen          -= fill;

        if (ctx->blockLen == AM_HAL_SHA_BLOCK_SIZE)
        {
            status = am_hal_cc312_sha_process(ctx, ctx->block,
                                              AM_HAL_SHA_BLOCK_SIZE, false);
            if (status != AM_HAL_STATUS_SUCCESS)
            {
                return status;
            }
            ctx->blockLen = 0U;
        }
    }

    //
    // Process as many whole blocks as possible directly from the input.
    //
    if (ilen >= AM_HAL_SHA_BLOCK_SIZE)
    {
        full = ilen & ~(uint32_t)(AM_HAL_SHA_BLOCK_SIZE - 1U);
        status = am_hal_cc312_sha_process(ctx, input, full, false);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
        input += full;
        ilen  -= full;
    }

    //
    // Buffer any remaining tail bytes for the next update/finish.
    //
    if (ilen > 0U)
    {
        memcpy(ctx->block, input, ilen);
        ctx->blockLen = ilen;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Complete a SHA operation and write out the digest.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha_finish(am_hal_cc312_sha_context_t *ctx,
                  uint8_t *output)
{
    uint32_t status;
    uint32_t words;
    uint32_t i;
    uint32_t r;

    if (ctx == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Process the final buffered block with hardware padding. When nothing is
    // buffered (empty message or exact block multiple) this performs DO_PAD.
    //
    status = am_hal_cc312_sha_process(ctx, ctx->block, ctx->blockLen, true);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Serialize the digest. Each HASH_Hx word holds a digest word; emit it
    // most-significant byte first to produce the standard big-endian digest
    // byte order.
    //
    words = am_hal_cc312_sha_digest_words(ctx->mode);
    for (i = 0; i < words; i++)
    {
        r = ctx->digest[i];
        output[(i * 4U) + 0U] = (uint8_t)(r >> 24);
        output[(i * 4U) + 1U] = (uint8_t)(r >> 16);
        output[(i * 4U) + 2U] = (uint8_t)(r >> 8);
        output[(i * 4U) + 3U] = (uint8_t)(r);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// One-shot SHA-1 of a single buffer.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha1(const uint8_t *input,
            uint32_t ilen,
            uint8_t output[AM_HAL_SHA1_DIGEST_SIZE])
{
    uint32_t status;
    //
    // am_hal_cc312_sha_context_t is more strictly aligned (32 bytes) than the
    // stack (8 bytes). Declare it static to avoid the compiler error.
    //
    static am_hal_cc312_sha_context_t ctx;
    //
    // Full-width scratch buffer: am_hal_cc312_sha_finish() emits a mode-sized
    // digest, so give it a worst-case (SHA-256) buffer and copy out the SHA-1
    // portion. This keeps the write provably in-bounds for the compiler.
    //
    uint8_t digest[AM_HAL_SHA256_DIGEST_SIZE];

    status = am_hal_cc312_sha_context_init(&ctx, AM_HAL_CC312_SHA1);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_sha_update(&ctx, input, ilen);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_sha_finish(&ctx, digest);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    memcpy(output, digest, AM_HAL_SHA1_DIGEST_SIZE);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// One-shot SHA-256 of a single buffer.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha256(const uint8_t *input,
              uint32_t ilen,
              uint8_t output[AM_HAL_SHA256_DIGEST_SIZE])
{
    uint32_t status;
    //
    // am_hal_cc312_sha_context_t is more strictly aligned (32 bytes) than the
    // stack (8 bytes). Declare it static to avoid the compiler error.
    //
    static am_hal_cc312_sha_context_t ctx;

    status = am_hal_cc312_sha_context_init(&ctx, AM_HAL_CC312_SHA256);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_sha_update(&ctx, input, ilen);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    return am_hal_cc312_sha_finish(&ctx, output);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
