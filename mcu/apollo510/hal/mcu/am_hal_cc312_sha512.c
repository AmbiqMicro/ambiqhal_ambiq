//*****************************************************************************
//
//! @file am_hal_cc312_sha512.c
//!
//! @brief Software SHA-384 and SHA-512 for the CryptoCell-312 HAL
//!
//! The hash engine implements SHA-1, SHA-224 and SHA-256 and nothing wider,
//! so the larger digests have to be computed on the core. This is the only
//! software cipher in the HAL and it exists because Ed25519 is defined in
//! terms of SHA-512: without it that curve cannot be offered at all.
//!
//! A system that already links a software SHA-512 should not carry a second
//! one. Defining AM_HAL_CC312_SHA512_EXTERNAL removes everything below and
//! leaves the integrator to supply the six entry points declared in the
//! header, plus am_hal_cc312_sha512_context_t through
//! am_hal_cc312_sha512_external.h. The prototypes stay in the HAL header in
//! both cases so the two paths cannot drift and nothing is declared twice.
//!
//! FIPS 180-4 section 6.4. The round constants and both initial values are
//! the fractional parts of cube and square roots of the low primes; they were
//! generated rather than transcribed.
//!
//! @addtogroup cc312_sha512_ap510 CC312 SHA-512 - Software Digest
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

#if !defined(AM_HAL_CC312_SHA512_EXTERNAL)

//*****************************************************************************
//
//! Round constants, cube roots of the first eighty primes.
//
//*****************************************************************************
static const uint64_t g_am_hal_sha512_k[80] =
{
    0x428A2F98D728AE22ULL, 0x7137449123EF65CDULL,
    0xB5C0FBCFEC4D3B2FULL, 0xE9B5DBA58189DBBCULL,
    0x3956C25BF348B538ULL, 0x59F111F1B605D019ULL,
    0x923F82A4AF194F9BULL, 0xAB1C5ED5DA6D8118ULL,
    0xD807AA98A3030242ULL, 0x12835B0145706FBEULL,
    0x243185BE4EE4B28CULL, 0x550C7DC3D5FFB4E2ULL,
    0x72BE5D74F27B896FULL, 0x80DEB1FE3B1696B1ULL,
    0x9BDC06A725C71235ULL, 0xC19BF174CF692694ULL,
    0xE49B69C19EF14AD2ULL, 0xEFBE4786384F25E3ULL,
    0x0FC19DC68B8CD5B5ULL, 0x240CA1CC77AC9C65ULL,
    0x2DE92C6F592B0275ULL, 0x4A7484AA6EA6E483ULL,
    0x5CB0A9DCBD41FBD4ULL, 0x76F988DA831153B5ULL,
    0x983E5152EE66DFABULL, 0xA831C66D2DB43210ULL,
    0xB00327C898FB213FULL, 0xBF597FC7BEEF0EE4ULL,
    0xC6E00BF33DA88FC2ULL, 0xD5A79147930AA725ULL,
    0x06CA6351E003826FULL, 0x142929670A0E6E70ULL,
    0x27B70A8546D22FFCULL, 0x2E1B21385C26C926ULL,
    0x4D2C6DFC5AC42AEDULL, 0x53380D139D95B3DFULL,
    0x650A73548BAF63DEULL, 0x766A0ABB3C77B2A8ULL,
    0x81C2C92E47EDAEE6ULL, 0x92722C851482353BULL,
    0xA2BFE8A14CF10364ULL, 0xA81A664BBC423001ULL,
    0xC24B8B70D0F89791ULL, 0xC76C51A30654BE30ULL,
    0xD192E819D6EF5218ULL, 0xD69906245565A910ULL,
    0xF40E35855771202AULL, 0x106AA07032BBD1B8ULL,
    0x19A4C116B8D2D0C8ULL, 0x1E376C085141AB53ULL,
    0x2748774CDF8EEB99ULL, 0x34B0BCB5E19B48A8ULL,
    0x391C0CB3C5C95A63ULL, 0x4ED8AA4AE3418ACBULL,
    0x5B9CCA4F7763E373ULL, 0x682E6FF3D6B2B8A3ULL,
    0x748F82EE5DEFB2FCULL, 0x78A5636F43172F60ULL,
    0x84C87814A1F0AB72ULL, 0x8CC702081A6439ECULL,
    0x90BEFFFA23631E28ULL, 0xA4506CEBDE82BDE9ULL,
    0xBEF9A3F7B2C67915ULL, 0xC67178F2E372532BULL,
    0xCA273ECEEA26619CULL, 0xD186B8C721C0C207ULL,
    0xEADA7DD6CDE0EB1EULL, 0xF57D4F7FEE6ED178ULL,
    0x06F067AA72176FBAULL, 0x0A637DC5A2C898A6ULL,
    0x113F9804BEF90DAEULL, 0x1B710B35131C471BULL,
    0x28DB77F523047D84ULL, 0x32CAAB7B40C72493ULL,
    0x3C9EBE0A15C9BEBCULL, 0x431D67C49C100D4CULL,
    0x4CC5D4BECB3E42B6ULL, 0x597F299CFC657E2AULL,
    0x5FCB6FAB3AD6FAECULL, 0x6C44198C4A475817ULL
};

//*****************************************************************************
//
//! Initial chaining values, square roots of the primes. SHA-512 takes the
//! first eight, SHA-384 the ninth through sixteenth.
//
//*****************************************************************************
static const uint64_t g_am_hal_sha512_iv[AM_HAL_SHA512_STATE_WORDS] =
{
    0x6A09E667F3BCC908ULL, 0xBB67AE8584CAA73BULL,
    0x3C6EF372FE94F82BULL, 0xA54FF53A5F1D36F1ULL,
    0x510E527FADE682D1ULL, 0x9B05688C2B3E6C1FULL,
    0x1F83D9ABFB41BD6BULL, 0x5BE0CD19137E2179ULL
};

static const uint64_t g_am_hal_sha384_iv[AM_HAL_SHA512_STATE_WORDS] =
{
    0xCBBB9D5DC1059ED8ULL, 0x629A292A367CD507ULL,
    0x9159015A3070DD17ULL, 0x152FECD8F70E5939ULL,
    0x67332667FFC00B31ULL, 0x8EB44A8768581511ULL,
    0xDB0C2E0D64F98FA7ULL, 0x47B5481DBEFA4FA4ULL
};

//*****************************************************************************
//
//! @name Round functions, FIPS 180-4 section 4.1.3.
//! @{
//
//*****************************************************************************
#define SHA512_SHR(x, n)    ((x) >> (n))
#define SHA512_ROTR(x, n)   (SHA512_SHR(x, n) | ((x) << (64 - (n))))

#define SHA512_S0(x)  (SHA512_ROTR(x,  1) ^ SHA512_ROTR(x,  8) ^ SHA512_SHR(x, 7))
#define SHA512_S1(x)  (SHA512_ROTR(x, 19) ^ SHA512_ROTR(x, 61) ^ SHA512_SHR(x, 6))
#define SHA512_S2(x)  (SHA512_ROTR(x, 28) ^ SHA512_ROTR(x, 34) ^ SHA512_ROTR(x, 39))
#define SHA512_S3(x)  (SHA512_ROTR(x, 14) ^ SHA512_ROTR(x, 18) ^ SHA512_ROTR(x, 41))

#define SHA512_MAJ(x, y, z) (((x) & (y)) | ((z) & ((x) | (y))))
#define SHA512_CH(x, y, z)  ((z) ^ ((x) & ((y) ^ (z))))

#define SHA512_ROUND(a, b, c, d, e, f, g, h, x, k)                            \
    do                                                                        \
    {                                                                         \
        uint64_t t1 = (h) + SHA512_S3(e) + SHA512_CH(e, f, g) + (k) + (x);    \
        uint64_t t2 = SHA512_S2(a) + SHA512_MAJ(a, b, c);                     \
        (d) += t1;                                                            \
        (h) = t1 + t2;                                                        \
    }                                                                         \
    while (0)
//! @}

//*****************************************************************************
//
//! @brief Read a big-endian 64-bit word.
//
//*****************************************************************************
static uint64_t
sha512_get_be64(const uint8_t *src)
{
    uint64_t value = 0;
    uint32_t i;

    for (i = 0; i < 8; i++)
    {
        value = (value << 8) | (uint64_t)src[i];
    }

    return value;
}

//*****************************************************************************
//
//! @brief Write a big-endian 64-bit word.
//
//*****************************************************************************
static void
sha512_put_be64(uint64_t value, uint8_t *dst)
{
    uint32_t i;

    for (i = 0; i < 8; i++)
    {
        dst[i] = (uint8_t)(value >> (56 - (i * 8)));
    }
}

//*****************************************************************************
//
//! @brief Compress one 128-byte block into the chaining value.
//
//*****************************************************************************
static void
sha512_process(am_hal_cc312_sha512_context_t *ctx, const uint8_t *data)
{
    uint64_t W[80];
    uint64_t A, B, C, D, E, F, G, H;
    uint32_t i;

    for (i = 0; i < 16; i++)
    {
        W[i] = sha512_get_be64(&data[i * 8]);
    }

    for (i = 16; i < 80; i++)
    {
        W[i] = SHA512_S1(W[i - 2]) + W[i - 7] +
               SHA512_S0(W[i - 15]) + W[i - 16];
    }

    A = ctx->state[0];
    B = ctx->state[1];
    C = ctx->state[2];
    D = ctx->state[3];
    E = ctx->state[4];
    F = ctx->state[5];
    G = ctx->state[6];
    H = ctx->state[7];

    //
    // Eight rounds per pass, so the working variables rotate back to their
    // starting positions and no copying is needed between rounds.
    //
    for (i = 0; i < 80; i += 8)
    {
        SHA512_ROUND(A, B, C, D, E, F, G, H, W[i + 0], g_am_hal_sha512_k[i + 0]);
        SHA512_ROUND(H, A, B, C, D, E, F, G, W[i + 1], g_am_hal_sha512_k[i + 1]);
        SHA512_ROUND(G, H, A, B, C, D, E, F, W[i + 2], g_am_hal_sha512_k[i + 2]);
        SHA512_ROUND(F, G, H, A, B, C, D, E, W[i + 3], g_am_hal_sha512_k[i + 3]);
        SHA512_ROUND(E, F, G, H, A, B, C, D, W[i + 4], g_am_hal_sha512_k[i + 4]);
        SHA512_ROUND(D, E, F, G, H, A, B, C, W[i + 5], g_am_hal_sha512_k[i + 5]);
        SHA512_ROUND(C, D, E, F, G, H, A, B, W[i + 6], g_am_hal_sha512_k[i + 6]);
        SHA512_ROUND(B, C, D, E, F, G, H, A, W[i + 7], g_am_hal_sha512_k[i + 7]);
    }

    ctx->state[0] += A;
    ctx->state[1] += B;
    ctx->state[2] += C;
    ctx->state[3] += D;
    ctx->state[4] += E;
    ctx->state[5] += F;
    ctx->state[6] += G;
    ctx->state[7] += H;

    am_hal_cc312_context_clear(W, sizeof(W));
}

//*****************************************************************************
//
// Digest width for a mode.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512_digest_size(am_hal_cc312_sha512_mode_e mode)
{
    uint32_t size;

    switch (mode)
    {
        case AM_HAL_CC312_SHA384:
            size = AM_HAL_SHA384_DIGEST_SIZE;
            break;
        case AM_HAL_CC312_SHA512:
            size = AM_HAL_SHA512_DIGEST_SIZE;
            break;
        default:
            size = 0;
            break;
    }

    return size;
}

//*****************************************************************************
//
// Start a digest.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512_init(am_hal_cc312_sha512_context_t *ctx,
                         am_hal_cc312_sha512_mode_e mode)
{
    if ((ctx == NULL) || (am_hal_cc312_sha512_digest_size(mode) == 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));
    memcpy(ctx->state,
           (mode == AM_HAL_CC312_SHA384) ? g_am_hal_sha384_iv
                                         : g_am_hal_sha512_iv,
           sizeof(ctx->state));
    ctx->mode = mode;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Add data to a digest.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512_update(am_hal_cc312_sha512_context_t *ctx,
                           const uint8_t *input, uint32_t ilen)
{
    uint32_t fill;

    if ((ctx == NULL) || ((input == NULL) && (ilen != 0)))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ilen == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Length is counted in bytes across two words, which is what the padding
    // needs to emit a 128-bit bit count.
    //
    ctx->total[0] += (uint64_t)ilen;
    if (ctx->total[0] < (uint64_t)ilen)
    {
        ctx->total[1]++;
    }

    //
    // Top up a partial block first.
    //
    if (ctx->bufferLen != 0)
    {
        fill = AM_HAL_SHA512_BLOCK_SIZE - ctx->bufferLen;

        if (ilen < fill)
        {
            memcpy(&ctx->buffer[ctx->bufferLen], input, ilen);
            ctx->bufferLen += ilen;
            return AM_HAL_STATUS_SUCCESS;
        }

        memcpy(&ctx->buffer[ctx->bufferLen], input, fill);
        sha512_process(ctx, ctx->buffer);
        input += fill;
        ilen  -= fill;
        ctx->bufferLen = 0;
    }

    while (ilen >= AM_HAL_SHA512_BLOCK_SIZE)
    {
        sha512_process(ctx, input);
        input += AM_HAL_SHA512_BLOCK_SIZE;
        ilen  -= AM_HAL_SHA512_BLOCK_SIZE;
    }

    if (ilen != 0)
    {
        memcpy(ctx->buffer, input, ilen);
        ctx->bufferLen = ilen;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Finish a digest and read it out.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512_finish(am_hal_cc312_sha512_context_t *ctx, uint8_t *output)
{
    uint32_t status;
    uint32_t used;
    uint32_t padn;
    uint32_t words;
    uint32_t i;
    uint64_t high;
    uint64_t low;
    uint8_t  length[16];
    static const uint8_t padding[AM_HAL_SHA512_BLOCK_SIZE] = { 0x80 };

    if ((ctx == NULL) || (output == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    words = am_hal_cc312_sha512_digest_size(ctx->mode) / 8U;
    if (words == 0)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Bit count, big-endian across sixteen bytes.
    //
    high = (ctx->total[1] << 3) | (ctx->total[0] >> 61);
    low  = ctx->total[0] << 3;
    sha512_put_be64(high, &length[0]);
    sha512_put_be64(low, &length[8]);

    //
    // Pad so the length lands in the last sixteen bytes of a block.
    //
    used = ctx->bufferLen;
    padn = (used < 112U) ? (112U - used) : (240U - used);

    status = am_hal_cc312_sha512_update(ctx, padding, padn);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_sha512_update(ctx, length, sizeof(length));
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // SHA-384 is SHA-512 truncated to six words, which is why the loop is
    // bounded by the digest and not by the state.
    //
    for (i = 0; i < words; i++)
    {
        sha512_put_be64(ctx->state[i], &output[i * 8]);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Release a context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512_free(am_hal_cc312_sha512_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(*ctx));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Digest a single buffer.
//
//*****************************************************************************
uint32_t
am_hal_cc312_sha512(am_hal_cc312_sha512_mode_e mode, const uint8_t *input,
                    uint32_t ilen, uint8_t *output)
{
    uint32_t status;
    am_hal_cc312_sha512_context_t ctx;

    status = am_hal_cc312_sha512_init(&ctx, mode);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_sha512_update(&ctx, input, ilen);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_finish(&ctx, output);
    }

    (void)am_hal_cc312_sha512_free(&ctx);

    return status;
}

#endif // AM_HAL_CC312_SHA512_EXTERNAL

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
