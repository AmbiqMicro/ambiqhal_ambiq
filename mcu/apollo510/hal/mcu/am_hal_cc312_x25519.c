//*****************************************************************************
//
//! @file am_hal_cc312_x25519.c
//!
//! @brief Hardware abstraction for CryptoCell-312 X25519
//!
//! Curve25519 is a Montgomery curve, so scalar multiplication is the
//! Montgomery ladder rather than the projective Weierstrass arithmetic the
//! NIST curves use. The ladder runs on the PKA and is written to take the
//! same number of operations whatever the scalar bits are, with the two
//! candidate points swapped by register index rather than by branching.
//!
//! @addtogroup cc312_x25519_ap510 CC312 X25519 - Montgomery Curve
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
//! @name PKA register allocation for the ladder.
//! @{
//
//*****************************************************************************
#define X25519_REG_N        AM_HAL_PKA_REG_N   //!< Field prime.
#define X25519_REG_NP       AM_HAL_PKA_REG_NP  //!< Barrett tag.
#define X25519_REG_T        2                  //!< Scratch.
#define X25519_REG_T1       3                  //!< Scratch.
#define X25519_REG_T2       4                  //!< Scratch.
#define X25519_REG_N4       5                  //!< 4 * prime.
#define X25519_REG_A24      6                  //!< (A + 2) / 4.
#define X25519_REG_RES      7                  //!< Result u-coordinate.
#define X25519_REG_X1       8                  //!< Input u-coordinate.
#define X25519_REG_X2       9                  //!< Ladder state.
#define X25519_REG_Z2       10                 //!< Ladder state.
#define X25519_REG_X3       11                 //!< Ladder state.
#define X25519_REG_Z3       12                 //!< Ladder state.
#define X25519_REG_COUNT    13                 //!< Registers to scrub.
//! @}

//! Field size: 2^255 - 19.
#define X25519_FIELD_BITS   255
#define X25519_FIELD_WORDS  8
//! The scalar is always used at its full width.
#define X25519_SCALAR_BITS  255

//
//! The prime 2^255 - 19, little endian by word.
//
static const uint32_t g_am_hal_x25519_p[X25519_FIELD_WORDS] =
{
    0xFFFFFFED, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF
};

//
//! (A + 2) / 4 for A = 486662, the curve constant the ladder folds in.
//
static const uint32_t g_am_hal_x25519_a24[X25519_FIELD_WORDS] =
{
    121666, 0, 0, 0, 0, 0, 0, 0
};

//
//! Base point u = 9.
//
static const uint32_t g_am_hal_x25519_base[X25519_FIELD_WORDS] =
{
    9, 0, 0, 0, 0, 0, 0, 0
};

//*****************************************************************************
//
//! @brief Swap two register indices when the condition is set.
//!
//! The ladder keeps its two candidate points in registers chosen by index,
//! so a conditional swap is an index exchange rather than a data move. The
//! exchange is written without a branch so the work does not depend on the
//! scalar bit.
//
//*****************************************************************************
static void
x25519_cond_swap(uint32_t *a, uint32_t *b, uint32_t swap)
{
    uint32_t mask = (uint32_t)0U - (swap & 1U);
    uint32_t diff = mask & (*a ^ *b);

    *a ^= diff;
    *b ^= diff;
}

//*****************************************************************************
//
// X25519 scalar multiplication.
//
//*****************************************************************************
uint32_t
am_hal_cc312_x25519_scalar_mult(const uint8_t *scalar,
                                const uint8_t *point,
                                uint8_t *result)
{
    uint32_t status;
    uint32_t k[X25519_FIELD_WORDS];
    uint32_t u[X25519_FIELD_WORDS];
    uint32_t out[X25519_FIELD_WORDS];
    uint8_t  clamped[AM_HAL_X25519_KEY_SIZE];
    uint32_t rX2 = X25519_REG_X2;
    uint32_t rZ2 = X25519_REG_Z2;
    uint32_t rX3 = X25519_REG_X3;
    uint32_t rZ3 = X25519_REG_Z3;
    uint32_t swap = 0;
    int32_t  i;
    uint32_t j;

    if ((scalar == NULL) || (point == NULL) || (result == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Clamp the scalar: clear the low three bits, clear the top bit and set
    // the one below it. This is what keeps the scalar in the right subgroup
    // and its length fixed.
    //
    memcpy(clamped, scalar, AM_HAL_X25519_KEY_SIZE);
    clamped[0]  &= 248U;
    clamped[31] &= 127U;
    clamped[31] |= 64U;

    for (j = 0; j < X25519_FIELD_WORDS; j++)
    {
        k[j] = ((uint32_t)clamped[j * 4U]) |
               ((uint32_t)clamped[(j * 4U) + 1U] << 8) |
               ((uint32_t)clamped[(j * 4U) + 2U] << 16) |
               ((uint32_t)clamped[(j * 4U) + 3U] << 24);

        u[j] = ((uint32_t)point[j * 4U]) |
               ((uint32_t)point[(j * 4U) + 1U] << 8) |
               ((uint32_t)point[(j * 4U) + 2U] << 16) |
               ((uint32_t)point[(j * 4U) + 3U] << 24);
    }

    //
    // The top bit of the u-coordinate is ignored on decoding.
    //
    u[X25519_FIELD_WORDS - 1U] &= 0x7FFFFFFFU;

    status = am_hal_cc312_pka_init(X25519_FIELD_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto scalar_mult_exit;
    }

    am_hal_cc312_pka_copy_data_into_reg(X25519_REG_N,
                                        AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                        g_am_hal_x25519_p,
                                        X25519_FIELD_WORDS);
    am_hal_cc312_pka_calc_np(AM_HAL_PKA_LEN_ID_N_BITS, X25519_FIELD_BITS,
                             X25519_REG_N, X25519_REG_NP,
                             X25519_REG_T, X25519_REG_T1);

    am_hal_cc312_pka_copy_data_into_reg(X25519_REG_A24,
                                        AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                        g_am_hal_x25519_a24,
                                        X25519_FIELD_WORDS);
    am_hal_cc312_pka_copy_data_into_reg(X25519_REG_X1,
                                        AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS,
                                        u, X25519_FIELD_WORDS);

    //
    // Start from (X2, Z2) = (1, 0) and (X3, Z3) = (u, 1).
    //
    am_hal_cc312_pka_clear_block_of_regs(rX2, 4);
    AM_HAL_PKA_SET_BIT0(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rX2, rX2);
    AM_HAL_PKA_COPY(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rX3, X25519_REG_X1);
    AM_HAL_PKA_SET_BIT0(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rZ3, rZ3);

    //
    // The ladder subtracts through 4 * p so intermediate values stay
    // positive without a reduction on every step.
    //
    AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_N4,
                   X25519_REG_N, X25519_REG_N);
    AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_N4,
                   X25519_REG_N4, X25519_REG_N4);

    for (i = (int32_t)X25519_SCALAR_BITS - 1; i >= 0; i--)
    {
        uint32_t bit = (k[i / 32] >> (i & 31)) & 1U;

        swap ^= bit;
        x25519_cond_swap(&rX2, &rX3, swap);
        x25519_cond_swap(&rZ2, &rZ3, swap);
        swap = bit;

        AM_HAL_PKA_SUB(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T2,
                       X25519_REG_N4, rZ3);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T,
                       rX3, X25519_REG_T2);
        AM_HAL_PKA_SUB(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T2,
                       X25519_REG_N4, rZ2);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T1,
                       rX2, X25519_REG_T2);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rX2, rZ2, rX2);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rZ2, rX3, rZ3);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rZ3,
                               X25519_REG_T, rX2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rZ2,
                               X25519_REG_T1, rZ2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, X25519_REG_T,
                               X25519_REG_T1, X25519_REG_T1);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, X25519_REG_T1,
                               rX2, rX2);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rX3, rZ3, rZ2);
        AM_HAL_PKA_SUB(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T2,
                       X25519_REG_N4, rZ2);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, rZ2, rZ3,
                       X25519_REG_T2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rX2,
                               X25519_REG_T1, X25519_REG_T);
        AM_HAL_PKA_SUB(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T2,
                       X25519_REG_N4, X25519_REG_T);
        AM_HAL_PKA_ADD(AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS, X25519_REG_T1,
                       X25519_REG_T1, X25519_REG_T2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rZ2, rZ2, rZ2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rX3, rX3, rX3);
        AM_HAL_PKA_MOD_MUL_ACC_NFR(AM_HAL_PKA_LEN_ID_N_BITS, X25519_REG_T,
                                   X25519_REG_A24, X25519_REG_T1,
                                   X25519_REG_T);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rZ3,
                               X25519_REG_X1, rZ2);
        AM_HAL_PKA_MOD_MUL_NFR(AM_HAL_PKA_LEN_ID_N_BITS, rZ2,
                               X25519_REG_T1, X25519_REG_T);
    }

    x25519_cond_swap(&rX2, &rX3, swap);
    x25519_cond_swap(&rZ2, &rZ3, swap);

    //
    // Convert back to an affine u-coordinate: u = X2 / Z2.
    //
    AM_HAL_PKA_MOD_INV_W_EXP(X25519_REG_T, rZ2, rX3);
    AM_HAL_PKA_MOD_MUL(AM_HAL_PKA_LEN_ID_N_BITS, X25519_REG_RES,
                       rX2, X25519_REG_T);

    am_hal_cc312_pka_copy_data_from_reg(out, X25519_FIELD_WORDS,
                                        X25519_REG_RES);

    for (j = 0; j < X25519_FIELD_WORDS; j++)
    {
        result[j * 4U]        = (uint8_t)(out[j]);
        result[(j * 4U) + 1U] = (uint8_t)(out[j] >> 8);
        result[(j * 4U) + 2U] = (uint8_t)(out[j] >> 16);
        result[(j * 4U) + 3U] = (uint8_t)(out[j] >> 24);
    }

    am_hal_cc312_pka_clear_block_of_regs(X25519_REG_N, X25519_REG_COUNT);
    am_hal_cc312_pka_finish();

    status = AM_HAL_STATUS_SUCCESS;

scalar_mult_exit:
    am_hal_cc312_context_clear(k, sizeof(k));
    am_hal_cc312_context_clear(clamped, sizeof(clamped));
    am_hal_cc312_context_clear(out, sizeof(out));

    return status;
}

//*****************************************************************************
//
// X25519 public key from a private scalar.
//
//*****************************************************************************
uint32_t
am_hal_cc312_x25519_base_mult(const uint8_t *scalar, uint8_t *result)
{
    uint8_t base[AM_HAL_X25519_POINT_SIZE];
    uint32_t j;

    if ((scalar == NULL) || (result == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(base, 0, sizeof(base));
    for (j = 0; j < 4U; j++)
    {
        base[j] = (uint8_t)(g_am_hal_x25519_base[0] >> (8U * j));
    }

    return am_hal_cc312_x25519_scalar_mult(scalar, base, result);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
