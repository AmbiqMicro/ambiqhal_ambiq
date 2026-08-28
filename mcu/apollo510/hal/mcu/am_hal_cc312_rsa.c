//*****************************************************************************
//
//! @file am_hal_cc312_rsa.c
//!
//! @brief Hardware abstraction for CryptoCell-312 RSA primitives
//!
//! RSA has no dedicated engine. Both of its operations are one modular
//! exponentiation, which the PKA provides as a single opcode, so this file
//! is the PKA setup around that opcode: load the modulus, derive the Barrett
//! tag, load the base and exponent, run MODEXP and read the result back.
//!
//! Padding is deliberately absent. PKCS#1 v1.5, OAEP and PSS are encodings
//! over the primitive rather than hardware behaviour, and they belong in the
//! layer that owns the message.
//!
//! @addtogroup cc312_rsa_ap510 CC312 RSA - Modular Exponentiation
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

#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @name PKA register allocation for the exponentiation.
//!
//! N and NP are fixed by the hardware. The remaining four are the two
//! scratch registers the Barrett tag calculation needs plus the operands.
//! @{
//
//*****************************************************************************
#define RSA_REG_N       AM_HAL_PKA_REG_N     //!< Modulus.
#define RSA_REG_NP      AM_HAL_PKA_REG_NP    //!< Barrett tag.
#define RSA_REG_T       2                    //!< Scratch for the tag calc.
#define RSA_REG_T1      3                    //!< Scratch for the tag calc.
#define RSA_REG_BASE    4                    //!< Base operand.
#define RSA_REG_EXP     5                    //!< Exponent operand.
#define RSA_REG_RES     6                    //!< Result.
#define RSA_REG_COUNT   7                    //!< Registers to scrub on exit.
//! @}

//*****************************************************************************
//
// RSA primitive: result = base ^ exponent mod modulus.
//
//*****************************************************************************
uint32_t
am_hal_cc312_rsa_mod_exp(const uint8_t *modulus,
                         uint32_t modulusBytes,
                         const uint8_t *exponent,
                         const uint8_t *base,
                         uint8_t *result)
{
    uint32_t status;
    uint32_t modulusWords;
    uint32_t modulusBits;

    if ((modulus == NULL) || (exponent == NULL) ||
        (base == NULL) || (result == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // The PKA works in whole words, and the modulus has to fit the register
    // width the hardware was built with.
    //
    if ((modulusBytes == 0) || ((modulusBytes % 4) != 0) ||
        (modulusBytes > AM_HAL_RSA_MAX_MODULUS_BYTES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    modulusWords = modulusBytes / 4U;
    modulusBits  = modulusBytes * 8U;

    //
    // An even modulus has no inverse to build the Barrett tag from, and a
    // modulus of one leaves nothing to reduce against.
    //
    if ((modulus[modulusBytes - 1U] & 0x01U) == 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // am_hal_cc312_pka_init() enables the PKA clock and am_hal_cc312_pka_finish()
    // drops it again, so the clock is not managed here.
    //
    status = am_hal_cc312_pka_init(modulusBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Load the modulus and derive the Barrett tag the PKA reduces with.
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_N,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            modulus, modulusWords);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_N_BITS, modulusBits,
                             RSA_REG_N, RSA_REG_NP, RSA_REG_T, RSA_REG_T1);

    //
    // Load the operands and run the exponentiation.
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_BASE,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            base, modulusWords);
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_REG_EXP,
                                            AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                            exponent, modulusWords);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_N_BITS, RSA_REG_RES,
                       RSA_REG_BASE, RSA_REG_EXP);

    am_hal_cc312_pka_copy_reg_into_be_bytes(result, modulusWords, RSA_REG_RES);

    //
    // Scrub the operands; the private exponent passes through these.
    //
    am_hal_cc312_pka_clear_block_of_regs(RSA_REG_N, RSA_REG_COUNT);
    am_hal_cc312_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @name PKA register allocation for the CRT exponentiation.
//!
//! The modulus register is retargeted part way through: the Q phase runs
//! with N pointing at RSA_CRT_REG_Q, the P phase with N pointing at
//! RSA_CRT_REG_P, each with its own Barrett tag.
//! @{
//
//*****************************************************************************
#define RSA_CRT_REG_Q       AM_HAL_PKA_REG_N   //!< Q, and N for the Q phase.
#define RSA_CRT_REG_NPQ     AM_HAL_PKA_REG_NP  //!< Barrett tag for Q.
#define RSA_CRT_REG_P       2                  //!< P, and N for the P phase.
#define RSA_CRT_REG_NPP     3                  //!< Barrett tag for P.
#define RSA_CRT_REG_T       4                  //!< Base reduced mod Q, then Mp.
#define RSA_CRT_REG_T1      5                  //!< Base reduced mod P, then h.
#define RSA_CRT_REG_MQ      6                  //!< Mq.
#define RSA_CRT_REG_D       7                  //!< dQ, then dP.
#define RSA_CRT_REG_QINV    8                  //!< qInv.
#define RSA_CRT_REG_TMP     9                  //!< Scratch for the tag calc.
#define RSA_CRT_REG_TMP1    10                 //!< Scratch for the tag calc.
#define RSA_CRT_REG_COUNT   11                 //!< Registers to scrub on exit.
//! @}

//*****************************************************************************
//
// RSA private-key operation using the Chinese Remainder Theorem.
//
//*****************************************************************************
uint32_t
am_hal_cc312_rsa_mod_exp_crt(const uint8_t *p,
                             const uint8_t *q,
                             const uint8_t *dP,
                             const uint8_t *dQ,
                             const uint8_t *qInv,
                             uint32_t pqBytes,
                             const uint8_t *base,
                             uint8_t *result)
{
    uint32_t status;
    uint32_t pqWords;
    uint32_t pqBits;
    uint32_t modulusWords;
    uint32_t modulusBits;

    if ((p == NULL) || (q == NULL) || (dP == NULL) || (dQ == NULL) ||
        (qInv == NULL) || (base == NULL) || (result == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((pqBytes == 0) || ((pqBytes % 4) != 0) ||
        ((pqBytes * 2U) > AM_HAL_RSA_MAX_MODULUS_BYTES))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Both factors must be odd for their Barrett tags to exist.
    //
    if (((p[pqBytes - 1U] & 0x01U) == 0U) || ((q[pqBytes - 1U] & 0x01U) == 0U))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    pqWords      = pqBytes / 4U;
    pqBits       = pqBytes * 8U;
    modulusWords = pqWords * 2U;
    modulusBits  = pqBits * 2U;

    //
    // Size the PKA for the full modulus: the final recombination is done at
    // that width even though both exponentiations are half of it.
    //
    status = am_hal_cc312_pka_init(modulusBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Add the P/Q size class alongside the modulus size the init programmed.
    //
    am_hal_cc312_pka_set_reg_size(pqBits, AM_HAL_PKA_LEN_ID_PQ_BITS);
    am_hal_cc312_pka_set_reg_size(AM_HAL_PKA_WORD_SIZE_IN_BITS *
                                  AM_HAL_PKA_GET_FULL_OP_SIZE_PKA_WORDS(pqBits),
                                  AM_HAL_PKA_LEN_ID_PQ_PKA_REG_BITS);

    //
    // ---- Q phase: Mq = base ^ dQ mod Q ----
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_Q,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            q, pqWords);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_PQ_BITS, pqBits,
                             RSA_CRT_REG_Q, RSA_CRT_REG_NPQ,
                             RSA_CRT_REG_TMP, RSA_CRT_REG_TMP1);
    am_hal_cc312_pka_set_n_np_t0_t1(RSA_CRT_REG_Q, RSA_CRT_REG_NPQ,
                                    AM_HAL_PKA_REG_T0, AM_HAL_PKA_REG_T1);

    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_D,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            dQ, pqWords);
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_T,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            base, modulusWords);
    AM_HAL_PKA_COPY(AM_HAL_PKA_LEN_ID_MAX_BITS, RSA_CRT_REG_T1, RSA_CRT_REG_T);

    //
    // Reduce the base modulo Q at full width, then exponentiate at P/Q width.
    // The reduction leaves the quotient behind, which is discarded.
    //
    AM_HAL_PKA_DIV(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, RSA_CRT_REG_MQ,
                   RSA_CRT_REG_T, RSA_CRT_REG_Q);
    am_hal_cc312_pka_clear_reg_words(RSA_CRT_REG_T, pqWords);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_PQ_BITS, RSA_CRT_REG_MQ,
                       RSA_CRT_REG_T, RSA_CRT_REG_D);

    //
    // ---- P phase: Mp = base ^ dP mod P ----
    //
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_P,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            p, pqWords);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_PQ_BITS, pqBits,
                             RSA_CRT_REG_P, RSA_CRT_REG_NPP,
                             RSA_CRT_REG_TMP, RSA_CRT_REG_TMP1);
    am_hal_cc312_pka_set_n_np_t0_t1(RSA_CRT_REG_P, RSA_CRT_REG_NPP,
                                    AM_HAL_PKA_REG_T0, AM_HAL_PKA_REG_T1);

    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_D,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            dP, pqWords);
    am_hal_cc312_pka_copy_be_bytes_into_reg(RSA_CRT_REG_QINV,
                                            AM_HAL_PKA_LEN_ID_MAX_BITS,
                                            qInv, pqWords);

    AM_HAL_PKA_DIV(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, RSA_CRT_REG_T,
                   RSA_CRT_REG_T1, RSA_CRT_REG_P);
    am_hal_cc312_pka_clear_reg_words(RSA_CRT_REG_T1, pqWords);

    AM_HAL_PKA_MOD_EXP(AM_HAL_PKA_LEN_ID_PQ_BITS, RSA_CRT_REG_T,
                       RSA_CRT_REG_T1, RSA_CRT_REG_D);

    //
    // ---- Recombine: h = (Mp - Mq) * qInv mod P, result = Mq + Q * h ----
    //
    // Mq is reduced modulo P first so the subtraction stays in range when
    // Mq is the larger of the two.
    //
    AM_HAL_PKA_MOD_ADD_IM(AM_HAL_PKA_LEN_ID_PQ_BITS, RSA_CRT_REG_T1,
                          RSA_CRT_REG_MQ, 0);
    AM_HAL_PKA_MOD_SUB(AM_HAL_PKA_LEN_ID_PQ_BITS, RSA_CRT_REG_T,
                       RSA_CRT_REG_T, RSA_CRT_REG_T1);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_PQ_BITS, RSA_CRT_REG_T1,
                       RSA_CRT_REG_T, RSA_CRT_REG_QINV);

    //
    // The recombination runs at full modulus width, so clear the high words
    // the P/Q sized operations left behind before widening.
    //
    am_hal_cc312_pka_clear_reg_words(RSA_CRT_REG_T1, pqWords);
    am_hal_cc312_pka_clear_reg_words(RSA_CRT_REG_MQ, pqWords);
    am_hal_cc312_pka_clear_reg_words(RSA_CRT_REG_Q, pqWords);

    //
    // result = Q * h + Mq, computed at the modulus width.
    //
    AM_HAL_PKA_MUL_LOW(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, RSA_CRT_REG_T,
                       RSA_CRT_REG_Q, RSA_CRT_REG_T1);
    AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, RSA_CRT_REG_T,
                   RSA_CRT_REG_T, RSA_CRT_REG_MQ);

    am_hal_cc312_pka_copy_reg_into_be_bytes(result, modulusWords, RSA_CRT_REG_T);

    //
    // Scrub: the factors and both private exponents passed through here.
    //
    am_hal_cc312_pka_clear_block_of_regs(RSA_CRT_REG_Q, RSA_CRT_REG_COUNT);
    am_hal_cc312_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
