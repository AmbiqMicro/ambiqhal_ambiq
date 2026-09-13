//*****************************************************************************
//
//! @file am_hal_cc312_srp.c
//!
//! @brief Hardware abstraction for CryptoCell-312 SRP-6a
//!
//! SRP is a protocol rather than an engine, but every operation it performs
//! is one the CryptoCell already provides: the digests run on the HASH
//! engine and the modular arithmetic on the PKA. Nothing here is software
//! cryptography beyond the byte shuffling the protocol specifies.
//!
//! The routines are deliberately individual steps rather than a session
//! object. Key generation and the protocol state machine need randomness and
//! message handling that belong above a HAL, so the caller drives the
//! exchange and calls in for each computation.
//!
//! @addtogroup cc312_srp_ap330P CC312 SRP - Secure Remote Password
//! @ingroup apollo330P_hal
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
//! @name PKA register allocation.
//! @{
//
//*****************************************************************************
#define SRP_REG_N       AM_HAL_PKA_REG_N   //!< Group modulus.
#define SRP_REG_NP      AM_HAL_PKA_REG_NP  //!< Barrett tag.
#define SRP_REG_T       2                  //!< Scratch for the tag calc.
#define SRP_REG_T1      3                  //!< Scratch for the tag calc.
#define SRP_REG_BASE    4                  //!< Base operand.
#define SRP_REG_EXP     5                  //!< Exponent operand.
#define SRP_REG_RES     6                  //!< Result.
#define SRP_REG_TMP     7                  //!< Working value.
#define SRP_REG_TMP1    8                  //!< Working value.
#define SRP_REG_COUNT   9                  //!< Registers to scrub on exit.
//! @}

//*****************************************************************************
//
//! @brief Digest width for a mode, or 0 if the mode is not supported.
//
//*****************************************************************************
static uint32_t
srp_digest_size(am_hal_cc312_sha_mode_e mode)
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
//! @brief Validate a group description.
//
//*****************************************************************************
static uint32_t
srp_check_group(const am_hal_cc312_srp_group_t *group)
{
    if ((group == NULL) || (group->pN == NULL) || (group->pG == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((group->modulusBytes == 0) ||
        ((group->modulusBytes % 4) != 0) ||
        (group->modulusBytes > AM_HAL_SRP_MAX_MODULUS_BYTES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (srp_digest_size(group->hashMode) == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // The Barrett tag needs an odd modulus, which every SRP group has.
    //
    if ((group->pN[group->modulusBytes - 1U] & 0x01U) == 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Bring the PKA up on the group modulus and derive its Barrett tag.
//
//*****************************************************************************
static uint32_t
srp_pka_start(const am_hal_cc312_srp_group_t *group)
{
    uint32_t status;
    uint32_t words = group->modulusBytes / 4U;
    uint32_t bits  = group->modulusBytes * 8U;

    status = am_hal_cc312_pka_init(bits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    am_hal_cc312_pka_copy_be_bytes_into_reg(SRP_REG_N,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            group->pN, words);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_N_BITS, bits,
                             SRP_REG_N, SRP_REG_NP, SRP_REG_T, SRP_REG_T1);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Load a big-endian value of any length into a register, right
//!        aligned so it takes the numeric value the protocol intends.
//
//*****************************************************************************
static void
srp_load_value(uint32_t reg, const uint8_t *value, uint32_t valueLen,
               uint32_t modulusBytes)
{
    uint8_t padded[AM_HAL_SRP_MAX_MODULUS_BYTES];

    memset(padded, 0, modulusBytes);

    if (valueLen > modulusBytes)
    {
        valueLen = modulusBytes;
    }

    memcpy(&padded[modulusBytes - valueLen], value, valueLen);

    am_hal_cc312_pka_copy_be_bytes_into_reg(reg,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            padded, modulusBytes / 4U);

    am_hal_cc312_context_clear(padded, modulusBytes);
}

//*****************************************************************************
//
//! @brief Release the PKA, scrubbing every register first.
//
//*****************************************************************************
static void
srp_pka_finish(void)
{
    am_hal_cc312_pka_clear_block_of_regs(SRP_REG_N, SRP_REG_COUNT);
    am_hal_cc312_pka_finish();
}

//*****************************************************************************
//
//! @brief Hash a list of buffers in one pass.
//
//*****************************************************************************
static uint32_t
srp_hash(am_hal_cc312_sha_mode_e mode,
         const uint8_t **parts,
         const uint32_t *lengths,
         uint32_t count,
         uint8_t *digest)
{
    uint32_t status;
    uint32_t i;
    am_hal_cc312_sha_context_t ctx;

    status = am_hal_cc312_sha_context_init(&ctx, mode);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    for (i = 0; i < count; i++)
    {
        if (lengths[i] == 0)
        {
            continue;
        }

        status = am_hal_cc312_sha_update(&ctx, parts[i], lengths[i]);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            (void)am_hal_cc312_sha_free(&ctx);
            return status;
        }
    }

    status = am_hal_cc312_sha_finish(&ctx, digest);
    (void)am_hal_cc312_sha_free(&ctx);

    return status;
}

//*****************************************************************************
//
// Derive the private key x = H(salt || H(user ":" password)).
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_derive_x(const am_hal_cc312_srp_group_t *group,
                          const uint8_t *salt, uint32_t saltLen,
                          const uint8_t *user, uint32_t userLen,
                          const uint8_t *password, uint32_t passLen,
                          uint8_t *x)
{
    uint32_t status;
    uint8_t inner[AM_HAL_SHA256_DIGEST_SIZE];
    static const uint8_t colon[1] = {':'};
    const uint8_t *parts[3];
    uint32_t lengths[3];

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((salt == NULL) || (user == NULL) || (password == NULL) || (x == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Inner hash binds the identity to the password, so the verifier cannot
    // be replayed against a different user name.
    //
    parts[0] = user;      lengths[0] = userLen;
    parts[1] = colon;     lengths[1] = 1;
    parts[2] = password;  lengths[2] = passLen;

    status = srp_hash(group->hashMode, parts, lengths, 3, inner);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto derive_x_exit;
    }

    parts[0] = salt;   lengths[0] = saltLen;
    parts[1] = inner;  lengths[1] = srp_digest_size(group->hashMode);

    status = srp_hash(group->hashMode, parts, lengths, 2, x);

derive_x_exit:
    am_hal_cc312_context_clear(inner, sizeof(inner));

    return status;
}

//*****************************************************************************
//
// Derive the multiplier k = H(N || PAD(g)).
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_derive_k(const am_hal_cc312_srp_group_t *group, uint8_t *k)
{
    uint32_t status;
    const uint8_t *parts[2];
    uint32_t lengths[2];

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if (k == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // g is already carried at the width of N, which is the padding the
    // protocol calls for.
    //
    parts[0] = group->pN;  lengths[0] = group->modulusBytes;
    parts[1] = group->pG;  lengths[1] = group->modulusBytes;

    return srp_hash(group->hashMode, parts, lengths, 2, k);
}

//*****************************************************************************
//
// Derive the scrambling parameter u = H(PAD(A) || PAD(B)).
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_derive_u(const am_hal_cc312_srp_group_t *group,
                          const uint8_t *pA, const uint8_t *pB, uint8_t *u)
{
    uint32_t status;
    const uint8_t *parts[2];
    uint32_t lengths[2];

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((pA == NULL) || (pB == NULL) || (u == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    parts[0] = pA;  lengths[0] = group->modulusBytes;
    parts[1] = pB;  lengths[1] = group->modulusBytes;

    return srp_hash(group->hashMode, parts, lengths, 2, u);
}

//*****************************************************************************
//
//! @brief Shared body for the two plain exponentiations: out = base^exp mod N.
//
//*****************************************************************************
static uint32_t
srp_mod_exp(const am_hal_cc312_srp_group_t *group,
            const uint8_t *base, uint32_t baseLen,
            const uint8_t *exp, uint32_t expLen,
            uint8_t *out)
{
    uint32_t status;

    status = srp_pka_start(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    srp_load_value(SRP_REG_BASE, base, baseLen, group->modulusBytes);
    srp_load_value(SRP_REG_EXP, exp, expLen, group->modulusBytes);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_RES,
                       SRP_REG_BASE, SRP_REG_EXP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(out, group->modulusBytes / 4U,
                                            SRP_REG_RES);
    srp_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Derive the password verifier v = g^x mod N.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_derive_verifier(const am_hal_cc312_srp_group_t *group,
                                 const uint8_t *x, uint32_t xLen, uint8_t *v)
{
    uint32_t status = srp_check_group(group);

    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((x == NULL) || (v == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return srp_mod_exp(group, group->pG, group->modulusBytes, x, xLen, v);
}

//*****************************************************************************
//
// Compute the client public value A = g^a mod N.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_client_public(const am_hal_cc312_srp_group_t *group,
                               const uint8_t *a, uint32_t aLen, uint8_t *pA)
{
    uint32_t status = srp_check_group(group);

    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((a == NULL) || (pA == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    return srp_mod_exp(group, group->pG, group->modulusBytes, a, aLen, pA);
}

//*****************************************************************************
//
// Compute the server public value B = (k * v + g^b) mod N.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_server_public(const am_hal_cc312_srp_group_t *group,
                               const uint8_t *k, uint32_t kLen,
                               const uint8_t *v,
                               const uint8_t *b, uint32_t bLen,
                               uint8_t *pB)
{
    uint32_t status;
    uint32_t words;

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((k == NULL) || (v == NULL) || (b == NULL) || (pB == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    words = group->modulusBytes / 4U;

    status = srp_pka_start(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // g^b mod N into the result, then fold in k * v.
    //
    srp_load_value(SRP_REG_BASE, group->pG, group->modulusBytes,
                   group->modulusBytes);
    srp_load_value(SRP_REG_EXP, b, bLen, group->modulusBytes);
    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_RES,
                       SRP_REG_BASE, SRP_REG_EXP);

    srp_load_value(SRP_REG_TMP, k, kLen, group->modulusBytes);
    srp_load_value(SRP_REG_TMP1, v, group->modulusBytes, group->modulusBytes);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_TMP,
                       SRP_REG_TMP, SRP_REG_TMP1);

    AM_HAL_PKA_MOD_ADD(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_RES,
                       SRP_REG_RES, SRP_REG_TMP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(pB, words, SRP_REG_RES);
    srp_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Compute the client shared secret S = (B - k * g^x)^(a + u * x) mod N.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_client_secret(const am_hal_cc312_srp_group_t *group,
                               const uint8_t *pB,
                               const uint8_t *k, uint32_t kLen,
                               const uint8_t *x, uint32_t xLen,
                               const uint8_t *a, uint32_t aLen,
                               const uint8_t *u, uint32_t uLen,
                               uint8_t *pS)
{
    uint32_t status;
    uint32_t words;

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((pB == NULL) || (k == NULL) || (x == NULL) ||
        (a == NULL) || (u == NULL) || (pS == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    words = group->modulusBytes / 4U;

    status = srp_pka_start(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Base: B - k * g^x mod N.
    //
    srp_load_value(SRP_REG_BASE, group->pG, group->modulusBytes,
                   group->modulusBytes);
    srp_load_value(SRP_REG_EXP, x, xLen, group->modulusBytes);
    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_TMP,
                       SRP_REG_BASE, SRP_REG_EXP);

    srp_load_value(SRP_REG_TMP1, k, kLen, group->modulusBytes);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_TMP,
                       SRP_REG_TMP, SRP_REG_TMP1);

    srp_load_value(SRP_REG_RES, pB, group->modulusBytes, group->modulusBytes);
    AM_HAL_PKA_MOD_SUB(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_BASE,
                       SRP_REG_RES, SRP_REG_TMP);

    //
    // Exponent: a + u * x, computed as a plain integer at the modulus width.
    //
    srp_load_value(SRP_REG_TMP, u, uLen, group->modulusBytes);
    srp_load_value(SRP_REG_TMP1, x, xLen, group->modulusBytes);
    AM_HAL_PKA_MUL_LOW(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, SRP_REG_EXP,
                       SRP_REG_TMP, SRP_REG_TMP1);

    srp_load_value(SRP_REG_TMP, a, aLen, group->modulusBytes);
    AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, SRP_REG_EXP,
                   SRP_REG_EXP, SRP_REG_TMP);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_RES,
                       SRP_REG_BASE, SRP_REG_EXP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(pS, words, SRP_REG_RES);
    srp_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Compute the server shared secret S = (A * v^u)^b mod N.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_server_secret(const am_hal_cc312_srp_group_t *group,
                               const uint8_t *pA, const uint8_t *v,
                               const uint8_t *u, uint32_t uLen,
                               const uint8_t *b, uint32_t bLen,
                               uint8_t *pS)
{
    uint32_t status;
    uint32_t words;

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((pA == NULL) || (v == NULL) || (u == NULL) ||
        (b == NULL) || (pS == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    words = group->modulusBytes / 4U;

    status = srp_pka_start(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Base: A * v^u mod N.
    //
    srp_load_value(SRP_REG_BASE, v, group->modulusBytes, group->modulusBytes);
    srp_load_value(SRP_REG_EXP, u, uLen, group->modulusBytes);
    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_TMP,
                       SRP_REG_BASE, SRP_REG_EXP);

    srp_load_value(SRP_REG_TMP1, pA, group->modulusBytes,
                   group->modulusBytes);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_BASE,
                       SRP_REG_TMP1, SRP_REG_TMP);

    //
    // Raise to the server private value.
    //
    srp_load_value(SRP_REG_EXP, b, bLen, group->modulusBytes);
    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, SRP_REG_RES,
                       SRP_REG_BASE, SRP_REG_EXP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(pS, words, SRP_REG_RES);
    srp_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Derive the session key from the shared secret.
//
//*****************************************************************************
uint32_t
am_hal_cc312_srp_session_key(const am_hal_cc312_srp_group_t *group,
                             const uint8_t *pS, uint8_t *pK)
{
    uint32_t status;
    uint32_t digestSize;
    uint32_t offset = 0;
    uint32_t half;
    uint32_t i;
    uint8_t  evenBuf[AM_HAL_SRP_MAX_MODULUS_BYTES / 2];
    uint8_t  oddBuf[AM_HAL_SRP_MAX_MODULUS_BYTES / 2];
    uint8_t  evenHash[AM_HAL_SHA256_DIGEST_SIZE];
    uint8_t  oddHash[AM_HAL_SHA256_DIGEST_SIZE];
    const uint8_t *parts[1];
    uint32_t lengths[1];

    status = srp_check_group(group);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    if ((pS == NULL) || (pK == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    digestSize = srp_digest_size(group->hashMode);

    //
    // Leading zero bytes are stripped, then an odd remaining length has its
    // first byte dropped so the two halves are equal.
    //
    while ((offset < group->modulusBytes) && (pS[offset] == 0))
    {
        offset++;
    }

    if (((group->modulusBytes - offset) & 1U) != 0U)
    {
        offset++;
    }

    half = (group->modulusBytes - offset) / 2U;
    if (half == 0)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    for (i = 0; i < half; i++)
    {
        evenBuf[i] = pS[offset + (i * 2U)];
        oddBuf[i]  = pS[offset + (i * 2U) + 1U];
    }

    parts[0] = evenBuf;  lengths[0] = half;
    status = srp_hash(group->hashMode, parts, lengths, 1, evenHash);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto session_key_exit;
    }

    parts[0] = oddBuf;
    status = srp_hash(group->hashMode, parts, lengths, 1, oddHash);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto session_key_exit;
    }

    //
    // Interleave the two digests a byte at a time.
    //
    for (i = 0; i < digestSize; i++)
    {
        pK[i * 2U]        = evenHash[i];
        pK[(i * 2U) + 1U] = oddHash[i];
    }

session_key_exit:
    am_hal_cc312_context_clear(evenBuf, sizeof(evenBuf));
    am_hal_cc312_context_clear(oddBuf, sizeof(oddBuf));
    am_hal_cc312_context_clear(evenHash, sizeof(evenHash));
    am_hal_cc312_context_clear(oddHash, sizeof(oddHash));

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
