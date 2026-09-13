//*****************************************************************************
//
//! @file am_hal_cc312_ecdsa.c
//!
//! @brief Hardware abstraction for CryptoCell-312 ECDSA sign / verify
//!
//! Purpose: Register-level port of the CC312 runtime ECDSA (ec_wrst_dsa.c
//! CalcSignature, and pka_ec_wrst_dsa_verify.c PkaEcdsaVerify) on top of the
//! PKA and EC HAL layers. The ephemeral key is supplied by the caller.
//!
//! @addtogroup cc312_ecdsa_ap510L CC312 ECDSA - Sign / Verify
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

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

#include "am_mcu_apollo.h"
#include <string.h>
#include <stdint.h>

#define DSA_CALC_FULL_32BIT_WORDS(bits)     (((bits) + 31U) / 32U)

#define DSA_LEN_N       AM_HAL_PKA_LEN_ID_N_BITS         // exact (L0)
#define DSA_LEN_NR      AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS // extended (L1)
#define DSA_LEN_MAX     AM_HAL_PKA_LEN_ID_MAX_BITS       // full register (L7)

//*****************************************************************************
//
//! @name Sign register allocation (CalcSignature). N=0, NP=1 reserved.
//! @{
//
//*****************************************************************************
#define S_RC        2   // C / r (ephemeral public X)
#define S_RM        3   // message representative e
#define S_REPHK     4   // ephemeral private key k
#define S_RK        5   // private key d
#define S_RKINV     6   // k^-1
#define S_RD        7   // D / s
#define S_RT        8   // temp
#define S_NP_T1     9   // calc_np temp
#define S_NP_T2     10  // calc_np temp
#define S_SIGN_REG_COUNT 11
//! @}

//*****************************************************************************
//
//! @name Verify register allocation (from pka_ec_wrst_dsa_verify_regs.h).
//! @{
//
//*****************************************************************************
#define V_R         AM_HAL_PKA_REG_N    // 0  order n (then swapped with p)
#define V_NR        AM_HAL_PKA_REG_NP   // 1  Barrett tag
#define V_F         2                   // message representative
#define V_D         3                   // signature s
#define V_H         4                   // s^-1
#define V_TMP       5
#define V_N4        8
#define V_N8        9
#define V_N12       10
#define V_EC_A      11
#define V_H1        18
#define V_H2        19
#define V_P_GX      20
#define V_P_GY      21
#define V_P_WX      22
#define V_P_WY      23
#define V_P_RX      24
#define V_P_RY      25
#define V_TMP_N     26
#define V_TMP_NP    27
#define V_C         28
#define V_VERIFY_REG_COUNT 29
//! @}

//*****************************************************************************
//
// Reverse-copy @p n bytes (big-endian byte buffer -> little-endian).
//
//*****************************************************************************
static void
dsa_reverse_memcpy(uint8_t *dst, const uint8_t *src, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
    {
        dst[i] = src[n - 1U - i];
    }
}

//*****************************************************************************
//
//! @brief Truncate a big-endian message hash to @p outSizeBits, producing a
//!        little-endian word integer (ported from EcWrstDsaTruncateMsg).
//
//*****************************************************************************
static void
dsa_truncate_msg(uint32_t *pMsgOut, const uint8_t *pMsgIn, uint32_t outSizeBits)
{
    uint32_t i, shift;
    uint32_t outSizeBytes = (outSizeBits + 7U) / 8U;
    uint32_t outSizeWords = DSA_CALC_FULL_32BIT_WORDS(outSizeBits);

    pMsgOut[outSizeWords - 1U] = 0;
    dsa_reverse_memcpy((uint8_t *)pMsgOut, pMsgIn, outSizeBytes);

    shift = (8U - (outSizeBits & 7U)) & 7U;
    if (shift)
    {
        for (i = 0; i < outSizeWords - 1U; i++)
        {
            pMsgOut[i] = (pMsgOut[i] >> shift) | (pMsgOut[i + 1U] << (32U - shift));
        }
        pMsgOut[i] = pMsgOut[i] >> shift;
    }
}

//*****************************************************************************
//
//! @brief Compute the ECDSA signature (r, s) given k*G's X (x1) and inputs.
//!        Ported from CalcSignature; all PKA work is modulo the order n.
//
//*****************************************************************************
static uint32_t
dsa_calc_signature(const am_hal_cc312_ecc_domain_t *pDomain,
                   const uint32_t *privKey,
                   const uint32_t *msgRep,
                   const uint32_t *ephemKey,
                   const uint32_t *ephemPubX,
                   uint32_t *sigR,
                   uint32_t *sigS)
{
    uint32_t status;
    uint32_t err = AM_HAL_STATUS_SUCCESS;
    uint32_t ordWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->ordSizeInBits);
    uint32_t modWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->modSizeInBits);

    status = am_hal_cc312_pka_init(pDomain->ordSizeInBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    am_hal_cc312_pka_copy_data_into_reg(AM_HAL_PKA_REG_N, DSA_LEN_NR, pDomain->ecOrd, ordWords);
    am_hal_cc312_pka_calc_np(DSA_LEN_N, pDomain->ordSizeInBits,
                             AM_HAL_PKA_REG_N, AM_HAL_PKA_REG_NP, S_NP_T1, S_NP_T2);

    am_hal_cc312_pka_copy_data_into_reg(S_RC, DSA_LEN_NR, ephemPubX, modWords);
    am_hal_cc312_pka_copy_data_into_reg(S_RM, DSA_LEN_NR, msgRep, ordWords);
    am_hal_cc312_pka_copy_data_into_reg(S_REPHK, DSA_LEN_NR, ephemKey, ordWords);
    am_hal_cc312_pka_copy_data_into_reg(S_RK, DSA_LEN_NR, privKey, ordWords);

    //
    // rEphK = k mod n; must be non-zero.
    //
    AM_HAL_PKA_DIV(DSA_LEN_MAX, S_RT, S_REPHK, AM_HAL_PKA_REG_N);
    AM_HAL_PKA_COMPARE_IM_STATUS(DSA_LEN_MAX, S_REPHK, 0, status);
    if (status == 1)
    {
        err = AM_HAL_STATUS_FAIL;
        goto end;
    }

    //
    // kinv = k^-1 mod n; must be non-zero.
    //
    AM_HAL_PKA_MOD_INV_W_EXP(S_RKINV, S_REPHK, S_RT);
    AM_HAL_PKA_COMPARE_IM_STATUS(DSA_LEN_NR, S_RKINV, 0, status);
    if (status == 1)
    {
        err = AM_HAL_STATUS_FAIL;
        goto end;
    }

    //
    // s = kinv * (e + d*r) mod n.
    //
    AM_HAL_PKA_MOD_MUL(DSA_LEN_N, S_RD, S_RK, S_RC);
    AM_HAL_PKA_REDUCE(DSA_LEN_N, S_RM, S_RM);
    AM_HAL_PKA_MOD_ADD(DSA_LEN_NR, S_RD, S_RD, S_RM);
    AM_HAL_PKA_MOD_MUL(DSA_LEN_N, S_RD, S_RD, S_RKINV);
    AM_HAL_PKA_COMPARE_IM_STATUS(DSA_LEN_NR, S_RD, 0, status);
    if (status == 1)
    {
        err = AM_HAL_STATUS_FAIL;
        goto end;
    }

    am_hal_cc312_pka_copy_data_from_reg(sigR, ordWords, S_RC);
    am_hal_cc312_pka_copy_data_from_reg(sigS, ordWords, S_RD);

end:
    am_hal_cc312_pka_clear_block_of_regs(AM_HAL_PKA_REG_N, S_SIGN_REG_COUNT);
    am_hal_cc312_pka_finish();
    return err;
}

//*****************************************************************************
//
//! @brief Core ECDSA verify on data already loaded into PKA registers.
//!        Ported from PkaEcdsaVerify. PKA_L0 = order size, PKA_L2 = mod size.
//
//*****************************************************************************
static uint32_t
dsa_verify_pka(void)
{
    uint32_t err = AM_HAL_STATUS_SUCCESS;
    uint32_t ordSizeInBits, modSizeInBits;
    uint32_t status1, status2;

    ordSizeInBits = am_hal_cc312_pka_get_reg_size(0);
    modSizeInBits = am_hal_cc312_pka_get_reg_size(2);

    //
    // 1. Check C and D are in [1, n-1]. Temporarily N = N - 1.
    //
    AM_HAL_PKA_FLIP_BIT0(DSA_LEN_NR, V_R, V_R);

    AM_HAL_PKA_SUB_IM(DSA_LEN_NR, AM_HAL_PKA_RES_DISCARD, V_C, 1);
    status1 = am_hal_cc312_pka_status_carry();
    AM_HAL_PKA_SUB(DSA_LEN_NR, AM_HAL_PKA_RES_DISCARD, V_R, V_C);
    status2 = am_hal_cc312_pka_status_carry();
    if (status1 == 0 || status2 == 0)
    {
        err = AM_HAL_STATUS_FAIL;
        goto end;
    }

    AM_HAL_PKA_SUB_IM(DSA_LEN_NR, AM_HAL_PKA_RES_DISCARD, V_D, 1);
    status1 = am_hal_cc312_pka_status_carry();
    AM_HAL_PKA_SUB(DSA_LEN_NR, AM_HAL_PKA_RES_DISCARD, V_R, V_D);
    status2 = am_hal_cc312_pka_status_carry();
    if (status1 == 0 || status2 == 0)
    {
        err = AM_HAL_STATUS_FAIL;
        goto end;
    }

    //
    // Restore N.
    //
    AM_HAL_PKA_FLIP_BIT0(DSA_LEN_NR, V_R, V_R);

    //
    // 2. h = s^-1 mod n; h1 = e*h mod n; h2 = r*h mod n.
    //
    AM_HAL_PKA_MOD_INV_W_EXP(V_H, V_D, V_TMP);
    AM_HAL_PKA_DIV(DSA_LEN_NR, V_TMP, V_F, V_R);     // F = e mod n
    AM_HAL_PKA_MOD_MUL(DSA_LEN_N, V_H1, V_F, V_H);
    AM_HAL_PKA_MOD_MUL(DSA_LEN_N, V_H2, V_C, V_H);

    //
    // Switch the PKA to field-modulus operations: swap N (order) <-> p.
    //
    AM_HAL_PKA_CLEAR(DSA_LEN_NR, AM_HAL_PKA_REG_T0);
    AM_HAL_PKA_CLEAR(DSA_LEN_NR, AM_HAL_PKA_REG_T1);
    am_hal_cc312_pka_set_reg_size(modSizeInBits, 0);    // PKA_L0 = mod size
    AM_HAL_PKA_COPY(DSA_LEN_NR, V_TMP, V_R);
    AM_HAL_PKA_COPY(DSA_LEN_NR, V_R, V_TMP_N);
    AM_HAL_PKA_COPY(DSA_LEN_NR, V_TMP_N, V_TMP);        // swap mod <-> ord
    AM_HAL_PKA_COPY(DSA_LEN_NR, V_NR, V_TMP_NP);

    //
    // Modulus multiples 4N, 8N, 12N.
    //
    AM_HAL_PKA_ADD(DSA_LEN_NR, V_N4, V_R, V_R);
    AM_HAL_PKA_ADD(DSA_LEN_NR, V_N4, V_N4, V_N4);
    AM_HAL_PKA_ADD(DSA_LEN_NR, V_N8, V_N4, V_N4);
    AM_HAL_PKA_ADD(DSA_LEN_NR, V_N12, V_N8, V_N4);

    //
    // 3. P = h1*G + h2*W (mod p).
    //
    err = am_hal_cc312_ecc_sum2_scalar_mult(V_P_RX, V_P_RY,
                                            V_H1, V_P_GX, V_P_GY,
                                            V_H2, V_P_WX, V_P_WY);
    if (err != AM_HAL_STATUS_SUCCESS)
    {
        goto end;
    }

    //
    // 4. c' = Px mod n; valid iff c' == c.
    //
    am_hal_cc312_pka_set_reg_size(ordSizeInBits, 0);    // PKA_L0 = order size
    AM_HAL_PKA_DIV(DSA_LEN_NR, V_TMP, V_P_RX, V_TMP_N); // P_RX %= n  (n now in TMP_N)
    AM_HAL_PKA_COMPARE_STATUS(DSA_LEN_NR, V_P_RX, V_C, status1);
    if (status1 != 1)
    {
        err = AM_HAL_STATUS_FAIL;
    }

end:
    return err;
}

//*****************************************************************************
//
// Public: ECDSA sign.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ecdsa_sign(const am_hal_cc312_ecc_domain_t *pDomain,
                        const uint32_t *privKey,
                        const uint32_t *ephemKey,
                        const uint8_t *hash,
                        uint32_t hashSizeBytes,
                        uint32_t *sigR,
                        uint32_t *sigS)
{
    uint32_t status;
    uint32_t ordWords, modWords, truncBits;
    uint32_t msgRep[AM_HAL_ECC_MAX_ORD_WORDS];
    uint32_t ephPubX[AM_HAL_ECC_MAX_MOD_WORDS];
    uint32_t ephPubY[AM_HAL_ECC_MAX_MOD_WORDS];

    if (pDomain == NULL || privKey == NULL || ephemKey == NULL ||
        hash == NULL || sigR == NULL || sigS == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ordWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->ordSizeInBits);
    modWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->modSizeInBits);

    //
    // Message representative e = leftmost min(hashlen, n-size) bits of the hash.
    //
    truncBits = hashSizeBytes * 8U;
    if (truncBits > pDomain->ordSizeInBits)
    {
        truncBits = pDomain->ordSizeInBits;
    }
    memset(msgRep, 0, sizeof(msgRep));
    dsa_truncate_msg(msgRep, hash, truncBits);

    //
    // Ephemeral public point (x1, y1) = k * G.
    //
    status = am_hal_cc312_ecc_scalar_mult(pDomain, ephemKey, ordWords,
                                          pDomain->ecGx, pDomain->ecGy,
                                          ephPubX, ephPubY);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Signature (r = x1 mod n implicitly, s) over the order field.
    //
    status = dsa_calc_signature(pDomain, privKey, msgRep, ephemKey,
                                ephPubX, sigR, sigS);

    (void)modWords;
    return status;
}

//*****************************************************************************
//
//! @brief Is an LE word buffer all-zero?
//
//*****************************************************************************
static bool
dsa_is_zero(const uint32_t *a, uint32_t words)
{
    uint32_t i, acc = 0;
    for (i = 0; i < words; i++)
    {
        acc |= a[i];
    }
    return (acc == 0U);
}

//*****************************************************************************
//
//! @brief Unsigned compare of LE word buffers: returns true if a < b.
//
//*****************************************************************************
static bool
dsa_lt(const uint32_t *a, const uint32_t *b, uint32_t words)
{
    int32_t i;
    for (i = (int32_t)words - 1; i >= 0; i--)
    {
        if (a[i] < b[i])
        {
            return true;
        }
        if (a[i] > b[i])
        {
            return false;
        }
    }
    return false;   // equal
}

//*****************************************************************************
//
//! @brief Draw a fresh ephemeral key k uniformly in [1, n-1] from the PUF.
//
//*****************************************************************************
static uint32_t
dsa_gen_ephemeral_key(const am_hal_cc312_ecc_domain_t *pDomain, uint32_t *k)
{
    uint32_t ordBits  = pDomain->ordSizeInBits;
    uint32_t ordWords = DSA_CALC_FULL_32BIT_WORDS(ordBits);
    uint32_t ordBytes = (ordBits + 7U) / 8U;
    uint32_t topBits  = ordBits & 31U;
    uint32_t tries;

    for (tries = 0; tries < 100U; tries++)
    {
        memset(k, 0, ordWords * 4U);
        if (am_hal_puf_get_entropy((uint8_t *)k, (uint16_t)ordBytes) != AM_HAL_STATUS_SUCCESS)
        {
            return AM_HAL_STATUS_FAIL;
        }

        //
        // Reduce to ordBits, then reject if out of [1, n-1].
        //
        if (topBits)
        {
            k[ordWords - 1U] &= (1UL << topBits) - 1U;
        }
        if (!dsa_is_zero(k, ordWords) && dsa_lt(k, pDomain->ecOrd, ordWords))
        {
            return AM_HAL_STATUS_SUCCESS;
        }
    }
    return AM_HAL_STATUS_FAIL;
}

//*****************************************************************************
//
// Public: ECDSA sign with PUF-generated ephemeral key.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ecdsa_sign_rnd(const am_hal_cc312_ecc_domain_t *pDomain,
                            const uint32_t *privKey,
                            const uint8_t *hash,
                            uint32_t hashSizeBytes,
                            uint32_t *sigR,
                            uint32_t *sigS)
{
    uint32_t status = AM_HAL_STATUS_FAIL;
    uint32_t k[AM_HAL_ECC_MAX_ORD_WORDS];
    uint32_t tries;

    if (pDomain == NULL || privKey == NULL || hash == NULL ||
        sigR == NULL || sigS == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    for (tries = 0; tries < 16U; tries++)
    {
        status = dsa_gen_ephemeral_key(pDomain, k);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            break;
        }

        status = am_hal_cc312_ecdsa_sign(pDomain, privKey, k, hash,
                                         hashSizeBytes, sigR, sigS);
        if (status == AM_HAL_STATUS_SUCCESS)
        {
            break;      // success
        }
        // Degenerate k (r==0 or s==0): draw a fresh key and retry.
    }

    memset(k, 0, sizeof(k));     // scrub the ephemeral key
    return status;
}

//*****************************************************************************
//
// Public: ECDSA verify.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ecdsa_verify(const am_hal_cc312_ecc_domain_t *pDomain,
                          const uint32_t *pubKeyX,
                          const uint32_t *pubKeyY,
                          const uint8_t *hash,
                          uint32_t hashSizeBytes,
                          const uint32_t *sigR,
                          const uint32_t *sigS)
{
    uint32_t status;
    uint32_t ordWords, modWords, truncBits, msgWords;
    uint32_t maxBits;
    uint32_t msgRep[AM_HAL_ECC_MAX_ORD_WORDS];

    if (pDomain == NULL || pubKeyX == NULL || pubKeyY == NULL ||
        hash == NULL || sigR == NULL || sigS == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ordWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->ordSizeInBits);
    modWords = DSA_CALC_FULL_32BIT_WORDS(pDomain->modSizeInBits);

    truncBits = hashSizeBytes * 8U;
    if (truncBits > pDomain->ordSizeInBits)
    {
        truncBits = pDomain->ordSizeInBits;
    }
    msgWords = DSA_CALC_FULL_32BIT_WORDS(truncBits);
    memset(msgRep, 0, sizeof(msgRep));
    dsa_truncate_msg(msgRep, hash, truncBits);

    //
    // Initialize the PKA for the larger of the order / modulus sizes, then set
    // the order size in L0 and the modulus size in L2.
    //
    maxBits = (pDomain->ordSizeInBits > pDomain->modSizeInBits)
              ? pDomain->ordSizeInBits : pDomain->modSizeInBits;
    status = am_hal_cc312_pka_init(maxBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }
    am_hal_cc312_pka_set_reg_size(pDomain->ordSizeInBits, 0);
    am_hal_cc312_pka_set_reg_size(pDomain->modSizeInBits, 2);

    //
    // Order n and its Barrett tag.
    //
    am_hal_cc312_pka_copy_data_into_reg(V_R, DSA_LEN_NR, pDomain->ecOrd, ordWords);
    am_hal_cc312_pka_calc_np(DSA_LEN_N, pDomain->ordSizeInBits,
                             V_R, V_NR, V_P_GX, V_P_GY);

    //
    // Field modulus p and its Barrett tag.
    //
    am_hal_cc312_pka_copy_data_into_reg(V_TMP_N, DSA_LEN_NR, pDomain->ecP, modWords);
    am_hal_cc312_pka_calc_np(DSA_LEN_N, pDomain->modSizeInBits,
                             V_TMP_N, V_TMP_NP, V_P_GX, V_P_GY);

    //
    // Signature, message, generator, public key, curve coefficient.
    //
    am_hal_cc312_pka_copy_data_into_reg(V_C, DSA_LEN_NR, sigR, ordWords);
    am_hal_cc312_pka_copy_data_into_reg(V_D, DSA_LEN_NR, sigS, ordWords);
    am_hal_cc312_pka_copy_data_into_reg(V_F, DSA_LEN_NR, msgRep, msgWords);
    am_hal_cc312_pka_copy_data_into_reg(V_P_GX, DSA_LEN_NR, pDomain->ecGx, modWords);
    am_hal_cc312_pka_copy_data_into_reg(V_P_GY, DSA_LEN_NR, pDomain->ecGy, modWords);
    am_hal_cc312_pka_copy_data_into_reg(V_P_WX, DSA_LEN_NR, pubKeyX, modWords);
    am_hal_cc312_pka_copy_data_into_reg(V_P_WY, DSA_LEN_NR, pubKeyY, modWords);
    am_hal_cc312_pka_copy_data_into_reg(V_EC_A, DSA_LEN_NR, pDomain->ecA, modWords);

    status = dsa_verify_pka();

    am_hal_cc312_pka_clear_block_of_regs(AM_HAL_PKA_REG_N, V_VERIFY_REG_COUNT);
    am_hal_cc312_pka_finish();

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
