//*****************************************************************************
//
//! @file am_hal_cc312_hmac.c
//!
//! @brief Hardware abstraction for CryptoCell-312 HMAC
//!
//! HMAC is not a hardware mode. It is the FIPS 198-1 construction
//! H((K ^ opad) || H((K ^ ipad) || message)) layered on the HASH engine, so
//! every hash pass here runs on the CC312 and only the key padding and the
//! XORs are done in software.
//!
//! @addtogroup cc312_hmac_ap510L CC312 HMAC - Keyed Hashing
//! @ingroup apollo510L_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @name HMAC key padding constants (FIPS 198-1).
//! @{
//
//*****************************************************************************
#define AM_HAL_HMAC_IPAD_BYTE           0x36U   //!< Inner padding byte.
#define AM_HAL_HMAC_OPAD_BYTE           0x5CU   //!< Outer padding byte.
//! @}

//*****************************************************************************
//
//! @brief Digest width in bytes for a given mode.
//!
//! @param mode SHA mode selector.
//!
//! @return Digest size in bytes, or 0 for an unsupported mode.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_hmac_digest_size(am_hal_cc312_sha_mode_e mode)
{
    uint32_t size;

    switch (mode)
    {
        case AM_HAL_CC312_SHA1:
            size = AM_HAL_SHA1_DIGEST_SIZE;
            break;
        case AM_HAL_CC312_SHA224:
            size = AM_HAL_SHA224_DIGEST_SIZE;
            break;
        case AM_HAL_CC312_SHA256:
            size = AM_HAL_SHA256_DIGEST_SIZE;
            break;
        default:
            size = 0;
            break;
    }

    return size;
}

//*****************************************************************************
//
// Start an HMAC operation.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hmac_start(am_hal_cc312_hmac_context_t *ctx,
                        am_hal_cc312_sha_mode_e mode,
                        const uint8_t *key,
                        uint32_t keyLen)
{
    uint32_t status;
    uint32_t i;
    uint8_t keyBlock[AM_HAL_SHA_BLOCK_SIZE];

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (keyLen != 0 && key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (am_hal_cc312_hmac_digest_size(mode) == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_hmac_context_t));
    ctx->mode = mode;

    memset(keyBlock, 0, sizeof(keyBlock));

    //
    // A key wider than the block is replaced by its digest; anything else is
    // zero padded out to the block size.
    //
    if (keyLen > AM_HAL_SHA_BLOCK_SIZE)
    {
        status = am_hal_cc312_sha_context_init(&ctx->shaCtx, mode);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        status = am_hal_cc312_sha_update(&ctx->shaCtx, key, keyLen);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        status = am_hal_cc312_sha_finish(&ctx->shaCtx, keyBlock);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
    }
    else if (keyLen > 0)
    {
        memcpy(keyBlock, key, keyLen);
    }

    for (i = 0; i < AM_HAL_SHA_BLOCK_SIZE; i++)
    {
        ctx->ipad[i] = keyBlock[i] ^ AM_HAL_HMAC_IPAD_BYTE;
        ctx->opad[i] = keyBlock[i] ^ AM_HAL_HMAC_OPAD_BYTE;
    }

    am_hal_cc312_context_clear(keyBlock, sizeof(keyBlock));

    //
    // Begin the inner hash over the ipad block.
    //
    status = am_hal_cc312_sha_context_init(&ctx->shaCtx, mode);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    return am_hal_cc312_sha_update(&ctx->shaCtx, ctx->ipad,
                                   AM_HAL_SHA_BLOCK_SIZE);
}

//*****************************************************************************
//
// Add a message fragment to an HMAC operation.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hmac_update(am_hal_cc312_hmac_context_t *ctx,
                         const uint8_t *input,
                         uint32_t length)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (length == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (input == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return am_hal_cc312_sha_update(&ctx->shaCtx, input, length);
}

//*****************************************************************************
//
// Complete an HMAC operation and write out the MAC.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hmac_finish(am_hal_cc312_hmac_context_t *ctx,
                         uint8_t *output)
{
    uint32_t status;
    uint32_t digestSize;
    uint8_t inner[AM_HAL_SHA256_DIGEST_SIZE];

    if (ctx == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    digestSize = am_hal_cc312_hmac_digest_size(ctx->mode);
    if (digestSize == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Close the inner hash.
    //
    status = am_hal_cc312_sha_finish(&ctx->shaCtx, inner);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto finish_exit;
    }

    //
    // Outer hash over the opad block followed by the inner digest.
    //
    status = am_hal_cc312_sha_context_init(&ctx->shaCtx, ctx->mode);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto finish_exit;
    }

    status = am_hal_cc312_sha_update(&ctx->shaCtx, ctx->opad,
                                     AM_HAL_SHA_BLOCK_SIZE);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto finish_exit;
    }

    status = am_hal_cc312_sha_update(&ctx->shaCtx, inner, digestSize);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto finish_exit;
    }

    status = am_hal_cc312_sha_finish(&ctx->shaCtx, output);

finish_exit:
    am_hal_cc312_context_clear(inner, sizeof(inner));

    return status;
}

//*****************************************************************************
//
// Free (clear) an HMAC context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hmac_free(am_hal_cc312_hmac_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_hmac_context_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// HMAC of a single buffer in one call.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hmac(am_hal_cc312_sha_mode_e mode,
                  const uint8_t *key,
                  uint32_t keyLen,
                  const uint8_t *input,
                  uint32_t length,
                  uint8_t *output)
{
    uint32_t status;
    am_hal_cc312_hmac_context_t ctx;

    status = am_hal_cc312_hmac_start(&ctx, mode, key, keyLen);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto hmac_exit;
    }

    status = am_hal_cc312_hmac_update(&ctx, input, length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto hmac_exit;
    }

    status = am_hal_cc312_hmac_finish(&ctx, output);

hmac_exit:
    am_hal_cc312_hmac_free(&ctx);

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
