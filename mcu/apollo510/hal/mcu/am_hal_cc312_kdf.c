//*****************************************************************************
//
//! @file am_hal_cc312_kdf.c
//!
//! @brief Hardware abstraction for CryptoCell-312 HKDF key derivation
//!
//! HKDF, RFC 5869, is the extract-then-expand construction over HMAC. Every
//! HMAC call underneath runs on the CC312 HASH engine, so the derivation is
//! hardware backed even though the construction itself is sequencing.
//!
//! @addtogroup cc312_kdf_ap510 CC312 KDF - Key Derivation
//! @ingroup apollo510_hal
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

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

#include "am_mcu_apollo.h"

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
am_hal_cc312_kdf_digest_size(am_hal_cc312_sha_mode_e mode)
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
// HKDF-Extract.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hkdf_extract(am_hal_cc312_sha_mode_e mode,
                          const uint8_t *salt,
                          uint32_t saltLen,
                          const uint8_t *ikm,
                          uint32_t ikmLen,
                          uint8_t *prk)
{
    uint32_t digestSize;
    uint8_t zeroSalt[AM_HAL_SHA256_DIGEST_SIZE];

    if (prk == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((ikmLen != 0) && (ikm == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    digestSize = am_hal_cc312_kdf_digest_size(mode);
    if (digestSize == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // With no salt the standard substitutes a zero string as wide as the
    // digest. The salt is the HMAC key here, the keying material the message.
    //
    if ((salt == NULL) || (saltLen == 0))
    {
        memset(zeroSalt, 0, digestSize);
        return am_hal_cc312_hmac(mode, zeroSalt, digestSize,
                                 ikm, ikmLen, prk);
    }

    return am_hal_cc312_hmac(mode, salt, saltLen, ikm, ikmLen, prk);
}

//*****************************************************************************
//
// HKDF-Expand.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hkdf_expand(am_hal_cc312_sha_mode_e mode,
                         const uint8_t *prk,
                         uint32_t prkLen,
                         const uint8_t *info,
                         uint32_t infoLen,
                         uint8_t *okm,
                         uint32_t okmLen)
{
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    uint32_t digestSize;
    uint32_t produced = 0;
    uint8_t counter = 1;
    uint8_t block[AM_HAL_SHA256_DIGEST_SIZE];
    am_hal_cc312_hmac_context_t ctx;

    if ((prk == NULL) || (okm == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((infoLen != 0) && (info == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    digestSize = am_hal_cc312_kdf_digest_size(mode);
    if (digestSize == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((okmLen == 0) || (okmLen > AM_HAL_HKDF_MAX_OUTPUT_BYTES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // T(1) = HMAC(prk, info || 0x01), and every block after it folds in the
    // previous one: T(n) = HMAC(prk, T(n-1) || info || n).
    //
    while (produced < okmLen)
    {
        uint32_t chunk;

        status = am_hal_cc312_hmac_start(&ctx, mode, prk, prkLen);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            goto expand_exit;
        }

        if (counter > 1)
        {
            status = am_hal_cc312_hmac_update(&ctx, block, digestSize);
            if (status != AM_HAL_STATUS_SUCCESS)
            {
                goto expand_exit;
            }
        }

        if (infoLen != 0)
        {
            status = am_hal_cc312_hmac_update(&ctx, info, infoLen);
            if (status != AM_HAL_STATUS_SUCCESS)
            {
                goto expand_exit;
            }
        }

        status = am_hal_cc312_hmac_update(&ctx, &counter, 1);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            goto expand_exit;
        }

        status = am_hal_cc312_hmac_finish(&ctx, block);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            goto expand_exit;
        }

        chunk = okmLen - produced;
        if (chunk > digestSize)
        {
            chunk = digestSize;
        }

        memcpy(okm + produced, block, chunk);
        produced += chunk;
        counter++;
    }

expand_exit:
    am_hal_cc312_hmac_free(&ctx);
    am_hal_cc312_context_clear(block, sizeof(block));

    return status;
}

//*****************************************************************************
//
// HKDF extract and expand in one call.
//
//*****************************************************************************
uint32_t
am_hal_cc312_hkdf(am_hal_cc312_sha_mode_e mode,
                  const uint8_t *salt,
                  uint32_t saltLen,
                  const uint8_t *ikm,
                  uint32_t ikmLen,
                  const uint8_t *info,
                  uint32_t infoLen,
                  uint8_t *okm,
                  uint32_t okmLen)
{
    uint32_t status;
    uint32_t digestSize;
    uint8_t prk[AM_HAL_SHA256_DIGEST_SIZE];

    digestSize = am_hal_cc312_kdf_digest_size(mode);
    if (digestSize == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = am_hal_cc312_hkdf_extract(mode, salt, saltLen, ikm, ikmLen, prk);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto hkdf_exit;
    }

    status = am_hal_cc312_hkdf_expand(mode, prk, digestSize,
                                      info, infoLen, okm, okmLen);

hkdf_exit:
    am_hal_cc312_context_clear(prk, sizeof(prk));

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
