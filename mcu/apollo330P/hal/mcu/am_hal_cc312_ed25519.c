//*****************************************************************************
//
//! @file am_hal_cc312_ed25519.c
//!
//! @brief Hardware abstraction for CryptoCell-312 Ed25519
//!
//! Ed25519 works over two different moduli. The point arithmetic runs modulo
//! the field prime 2^255 - 19 and the scalars run modulo the group order L,
//! so the PKA is set up twice with a Barrett tag for each. The two never
//! share a session, which is what keeps the tags from being used against the
//! wrong modulus.
//!
//! Points are held in extended coordinates (X:Y:Z:T) with the affine point
//! being (X/Z, Y/Z) and T = XY/Z. The curve has a = -1 and its d is a
//! non-square, so the addition formula is complete: there is no doubling
//! case, no neutral case and nothing to branch on. Doubling still gets its
//! own routine because specialising the same formula to P = Q removes four
//! multiplications.
//!
//! Point add/double follow the CryptoCell Edwards PKA sequence: ordinary
//! ADD/SUB through 4*p (so intermediates stay positive) and Barrett multiply
//! without a final reduction (MOD_MUL_NFR). Modular ADD/SUB on partially
//! reduced field elements is what the earlier ladder got wrong; X25519 and
//! Arm's pka_ec_edw.c use the same N4 + NFR pattern.
//!
//! Scalar multiplication is a ladder that performs one addition and one
//! doubling per bit whatever the bit is, selecting between the two
//! accumulators by exchanging register indices rather than by branching, in
//! the same shape as the X25519 ladder.
//!
//! @addtogroup cc312_ed25519_ap330P CC312 Ed25519 - Edwards Curve Signatures
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
//! @name Field and scalar sizes.
//! @{
//
//*****************************************************************************
#define ED_FIELD_BITS       255 //!< Bit length of 2^255 - 19.
#define ED_ORDER_BITS       253 //!< Bit length of the group order L.
#define ED_WORDS            8   //!< 32-bit words in a field or scalar value.
#define ED_BYTES            32  //!< Bytes in an encoded point or scalar.
#define ED_SCALAR_BITS      255 //!< Bits the ladder walks, after clamping.
//! @}

//*****************************************************************************
//
//! @name PKA register allocation.
//!
//! Shared by the field and scalar sessions; the scalar session only uses the
//! modulus, the tag and the low temporaries.
//! @{
//
//*****************************************************************************
#define ED_REG_N        AM_HAL_PKA_REG_N    //!< Modulus in use.
#define ED_REG_NP       AM_HAL_PKA_REG_NP   //!< Barrett tag for it.
#define ED_REG_D2       2                   //!< 2 * d, the curve constant.
#define ED_REG_N4       3                   //!< 4 * p, for signed field adds.
#define ED_REG_T3       4                   //!< Temporary (Arm EDW naming).
#define ED_REG_T4       5                   //!< Temporary.
#define ED_REG_T5       6                   //!< Temporary.
#define ED_REG_T6       7                   //!< Temporary.
#define ED_REG_T1       8                   //!< Scratch (scalar path / encode).
#define ED_REG_T2       9                   //!< Scratch (scalar path / encode).
#define ED_REG_AX       10                  //!< First accumulator.
#define ED_REG_AY       11
#define ED_REG_AZ       12
#define ED_REG_AT       13
#define ED_REG_BX       14                  //!< Second accumulator.
#define ED_REG_BY       15
#define ED_REG_BZ       16
#define ED_REG_BT       17
#define ED_REG_QX       18                  //!< Input point.
#define ED_REG_QY       19
#define ED_REG_QZ       20
#define ED_REG_QT       21
#define ED_REG_COUNT    22                  //!< Registers to scrub on exit.
//! @}

//*****************************************************************************
//
//! @name Length classes.
//!
//! N_BITS is the modulus size the modular operations reduce against.
//! N_PKA_REG_BITS is the wider register size used for loads, copies and the
//! logical operations, which do not reduce.
//! @{
//
//*****************************************************************************
#define ED_LEN_N        AM_HAL_PKA_LEN_ID_N_BITS
#define ED_LEN_REG      AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS
//! @}

//*****************************************************************************
//
//! The field prime, 2^255 - 19, little endian by word.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_p[ED_WORDS] =
{
    0xFFFFFFED, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF
};

//*****************************************************************************
//
//! The curve constant d = -121665 / 121666.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_d[ED_WORDS] =
{
    0x135978A3, 0x75EB4DCA, 0x4141D8AB, 0x00700A4D,
    0x7779E898, 0x8CC74079, 0x2B6FFE73, 0x52036CEE
};

//*****************************************************************************
//
//! 2 * d, which is what the point formulas actually multiply by.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_d2[ED_WORDS] =
{
    0x26B2F159, 0xEBD69B94, 0x8283B156, 0x00E0149A,
    0xEEF3D130, 0x198E80F2, 0x56DFFCE7, 0x2406D9DC
};

//*****************************************************************************
//
//! The group order L = 2^252 + 27742317777372353535851937790883648493.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_l[ED_WORDS] =
{
    0x5CF5D3ED, 0x5812631A, 0xA2F79CD6, 0x14DEF9DE,
    0x00000000, 0x00000000, 0x00000000, 0x10000000
};

//*****************************************************************************
//
//! 2^256 mod L, used to recombine the two halves of a reduced digest.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_two256_mod_l[ED_WORDS] =
{
    0x8D98951D, 0xD6EC3174, 0x737DCF70, 0xC6EF5BF4,
    0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0x0FFFFFFF
};

//*****************************************************************************
//
//! x-coordinate of the base point.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_bx[ED_WORDS] =
{
    0x8F25D51A, 0xC9562D60, 0x9525A7B2, 0x692CC760,
    0xFDD6DC5C, 0xC0A4E231, 0xCD6E53FE, 0x216936D3
};

//*****************************************************************************
//
//! y-coordinate of the base point, 4 / 5.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_by[ED_WORDS] =
{
    0x66666658, 0x66666666, 0x66666666, 0x66666666,
    0x66666666, 0x66666666, 0x66666666, 0x66666666
};

//*****************************************************************************
//
//! A square root of -1, used when point decompression picks the wrong root.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_sqrtm1[ED_WORDS] =
{
    0x4A0EA0B0, 0xC4EE1B27, 0xAD2FE478, 0x2F431806,
    0x3DFBD7A7, 0x2B4D0099, 0x4FC1DF0B, 0x2B832480
};

//*****************************************************************************
//
//! (p - 5) / 8, the exponent that takes a fourth root on this field.
//
//*****************************************************************************
static const uint32_t g_am_hal_ed25519_pm5d8[ED_WORDS] =
{
    0xFFFFFFFD, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0FFFFFFF
};

//*****************************************************************************
//
//! @brief A point as four register indices.
//!
//! The ladder selects between accumulators by exchanging indices, so a point
//! has to be a set of names rather than a set of values.
//
//*****************************************************************************
typedef struct
{
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t t;
}
ed_point_t;

//*****************************************************************************
//
//! @brief Little-endian bytes to words.
//
//*****************************************************************************
static void
ed_bytes_to_words(uint32_t *dst, const uint8_t *src)
{
    uint32_t i;

    for (i = 0; i < ED_WORDS; i++)
    {
        dst[i] = (uint32_t)src[i * 4U] |
                 ((uint32_t)src[(i * 4U) + 1U] << 8) |
                 ((uint32_t)src[(i * 4U) + 2U] << 16) |
                 ((uint32_t)src[(i * 4U) + 3U] << 24);
    }
}

//*****************************************************************************
//
//! @brief Words to little-endian bytes.
//
//*****************************************************************************
static void
ed_words_to_bytes(uint8_t *dst, const uint32_t *src)
{
    uint32_t i;

    for (i = 0; i < ED_WORDS; i++)
    {
        dst[i * 4U]        = (uint8_t)(src[i]);
        dst[(i * 4U) + 1U] = (uint8_t)(src[i] >> 8);
        dst[(i * 4U) + 2U] = (uint8_t)(src[i] >> 16);
        dst[(i * 4U) + 3U] = (uint8_t)(src[i] >> 24);
    }
}

//*****************************************************************************
//
//! @brief Compare two little-endian word values.
//!
//! @return true if a is strictly less than b.
//
//*****************************************************************************
static bool
ed_words_less_than(const uint32_t *a, const uint32_t *b)
{
    int32_t i;

    for (i = (int32_t)ED_WORDS - 1; i >= 0; i--)
    {
        if (a[i] != b[i])
        {
            return (a[i] < b[i]);
        }
    }

    return false;
}

//*****************************************************************************
//
//! @brief Exchange two register indices when the condition is set.
//!
//! Written without a branch so the work does not depend on the scalar bit.
//
//*****************************************************************************
static void
ed_cond_swap(uint32_t *a, uint32_t *b, uint32_t swap)
{
    uint32_t mask = (uint32_t)0U - (swap & 1U);
    uint32_t diff = mask & (*a ^ *b);

    *a ^= diff;
    *b ^= diff;
}

//*****************************************************************************
//
//! @brief Exchange two whole points.
//
//*****************************************************************************
static void
ed_cond_swap_point(ed_point_t *a, ed_point_t *b, uint32_t swap)
{
    ed_cond_swap(&a->x, &b->x, swap);
    ed_cond_swap(&a->y, &b->y, swap);
    ed_cond_swap(&a->z, &b->z, swap);
    ed_cond_swap(&a->t, &b->t, swap);
}

//*****************************************************************************
//
//! @brief Bring the PKA up on a modulus and derive its Barrett tag.
//
//*****************************************************************************
static uint32_t
ed_pka_start(const uint32_t *modulus, uint32_t modulusBits)
{
    uint32_t status = am_hal_cc312_pka_init(modulusBits);

    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    am_hal_cc312_pka_copy_data_into_reg(ED_REG_N, ED_LEN_REG,
                                        modulus, ED_WORDS);
    am_hal_cc312_pka_calc_np(ED_LEN_N, modulusBits, ED_REG_N, ED_REG_NP,
                             ED_REG_T1, ED_REG_T2);

    //
    // 4 * p for the Edwards ADD/SUB sequence. Harmless on the scalar-modulus
    // session, which never reads N4.
    //
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_N4, ED_REG_N, ED_REG_N);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_N4, ED_REG_N4, ED_REG_N4);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Release the PKA, scrubbing every register first.
//
//*****************************************************************************
static void
ed_pka_finish(void)
{
    am_hal_cc312_pka_clear_block_of_regs(ED_REG_N, ED_REG_COUNT);
    am_hal_cc312_pka_finish();
}

//*****************************************************************************
//
//! @brief Double a point in extended coordinates.
//!
//! Arm PkaEcEdwDoublExtExt: ADD/SUB through 4*p and MOD_MUL_NFR. In-place is
//! safe - T is consumed before it is overwritten with E.
//
//*****************************************************************************
static void
ed_double(const ed_point_t *r, const ed_point_t *p)
{
    AM_HAL_PKA_SUB(ED_LEN_REG, ED_REG_T3, ED_REG_N4, p->x);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T3, ED_REG_T3, p->y);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T5, ED_REG_T3, ED_REG_T3);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T3, p->y, p->x);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T6, ED_REG_T3, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T3, p->t, p->t);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T3, ED_REG_T3, ED_REG_D2);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, p->z, p->z);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T4, ED_REG_T4, p->z);
    AM_HAL_PKA_SUB(ED_LEN_REG, r->t, ED_REG_N4, ED_REG_T5);
    AM_HAL_PKA_ADD(ED_LEN_REG, r->t, r->t, ED_REG_T6);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T6, ED_REG_T6, ED_REG_T5);
    AM_HAL_PKA_SUB(ED_LEN_REG, ED_REG_T5, ED_REG_N4, ED_REG_T3);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T5, ED_REG_T5, ED_REG_T4);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, ED_REG_T4, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->x, r->t, ED_REG_T5);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->y, ED_REG_T4, ED_REG_T6);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->z, ED_REG_T5, ED_REG_T4);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->t, r->t, ED_REG_T6);
}

//*****************************************************************************
//
//! @brief Add two points in extended coordinates.
//!
//! Arm PkaEcEdwAddExtExtExt. Complete for this curve. Destination may alias
//! either source; T2 is read before dest->t is overwritten with E.
//
//*****************************************************************************
static void
ed_add(const ed_point_t *r, const ed_point_t *p, const ed_point_t *q)
{
    AM_HAL_PKA_SUB(ED_LEN_REG, ED_REG_T3, ED_REG_N4, p->x);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T3, ED_REG_T3, p->y);
    AM_HAL_PKA_SUB(ED_LEN_REG, ED_REG_T4, ED_REG_N4, q->x);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, ED_REG_T4, q->y);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T5, ED_REG_T3, ED_REG_T4);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T3, p->y, p->x);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, q->y, q->x);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T6, ED_REG_T3, ED_REG_T4);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T3, p->t, q->t);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T3, ED_REG_T3, ED_REG_D2);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, p->z, p->z);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, ED_REG_T4, ED_REG_T4, q->z);
    AM_HAL_PKA_SUB(ED_LEN_REG, r->t, ED_REG_N4, ED_REG_T5);
    AM_HAL_PKA_ADD(ED_LEN_REG, r->t, r->t, ED_REG_T6);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T6, ED_REG_T6, ED_REG_T5);
    AM_HAL_PKA_SUB(ED_LEN_REG, ED_REG_T5, ED_REG_N4, ED_REG_T3);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T5, ED_REG_T5, ED_REG_T4);
    AM_HAL_PKA_ADD(ED_LEN_REG, ED_REG_T4, ED_REG_T4, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->x, r->t, ED_REG_T5);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->y, ED_REG_T4, ED_REG_T6);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->z, ED_REG_T5, ED_REG_T4);
    AM_HAL_PKA_MOD_MUL_NFR(ED_LEN_N, r->t, r->t, ED_REG_T6);
}

//*****************************************************************************
//
//! @brief Load the curve constant the point formulas need.
//
//*****************************************************************************
static void
ed_load_d2(void)
{
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_D2, ED_LEN_REG,
                                        g_am_hal_ed25519_d2, ED_WORDS);
}

//*****************************************************************************
//
//! @brief Multiply the point in Q by a scalar, leaving the result in the
//!        returned handle.
//!
//! One addition and one doubling per bit regardless of the bit; the choice
//! of which accumulator receives which is an index exchange.
//
//*****************************************************************************
static ed_point_t
ed_ladder(const uint32_t *scalar)
{
    ed_point_t a = { ED_REG_AX, ED_REG_AY, ED_REG_AZ, ED_REG_AT };
    ed_point_t b = { ED_REG_BX, ED_REG_BY, ED_REG_BZ, ED_REG_BT };
    ed_point_t q = { ED_REG_QX, ED_REG_QY, ED_REG_QZ, ED_REG_QT };
    uint32_t swap = 0;
    int32_t  i;

    //
    // The neutral element is (0 : 1 : 1 : 0).
    //
    am_hal_cc312_pka_clear_block_of_regs(ED_REG_AX, 8);
    AM_HAL_PKA_SET_BIT0(ED_LEN_REG, a.y, a.y);
    AM_HAL_PKA_SET_BIT0(ED_LEN_REG, a.z, a.z);
    AM_HAL_PKA_COPY(ED_LEN_REG, b.x, q.x);
    AM_HAL_PKA_COPY(ED_LEN_REG, b.y, q.y);
    AM_HAL_PKA_COPY(ED_LEN_REG, b.z, q.z);
    AM_HAL_PKA_COPY(ED_LEN_REG, b.t, q.t);

    for (i = (int32_t)ED_SCALAR_BITS - 1; i >= 0; i--)
    {
        uint32_t bit = (scalar[i / 32] >> (i & 31)) & 1U;

        swap ^= bit;
        ed_cond_swap_point(&a, &b, swap);
        swap = bit;

        ed_add(&b, &a, &b);
        ed_double(&a, &a);
    }

    ed_cond_swap_point(&a, &b, swap);

    return a;
}

//*****************************************************************************
//
//! @brief Encode a point the way RFC 8032 section 5.1.2 defines.
//!
//! The y-coordinate carries the low bit of x in its top bit.
//
//*****************************************************************************
static void
ed_encode(const ed_point_t *p, uint8_t *out)
{
    uint32_t words[ED_WORDS];
    uint32_t parity;

    //
    // Affine: divide through by Z. The field prime is prime, so the inverse
    // is an exponentiation and needs no separate inversion path. Barrett
    // products can sit in [0, 2p); REDUCE brings them into [0, p) before
    // the wire encoding, matching am_hal_cc312_ecc_jacobian_to_affine().
    //
    AM_HAL_PKA_MOD_INV_W_EXP(ED_REG_T2, p->z, ED_REG_T1);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T3, p->x, ED_REG_T2);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T4, p->y, ED_REG_T2);
    AM_HAL_PKA_REDUCE(ED_LEN_N, ED_REG_T3, ED_REG_T3);
    AM_HAL_PKA_REDUCE(ED_LEN_N, ED_REG_T4, ED_REG_T4);

    parity = am_hal_cc312_pka_read_word_from_reg(ED_REG_T3, 0) & 1U;

    am_hal_cc312_pka_copy_data_from_reg(words, ED_WORDS, ED_REG_T4);
    ed_words_to_bytes(out, words);
    out[ED_BYTES - 1U] &= 0x7FU;
    out[ED_BYTES - 1U] |= (uint8_t)(parity << 7);

    am_hal_cc312_context_clear(words, sizeof(words));
}

//*****************************************************************************
//
//! @brief Recover a point from its encoding, into the Q registers.
//!
//! Solves x^2 = (y^2 - 1) / (d y^2 + 1). The field prime is 5 mod 8, so the
//! candidate root is a single exponentiation; it is right up to a factor of
//! a square root of -1, which the check below corrects for.
//!
//! @return AM_HAL_STATUS_SUCCESS, or AM_HAL_STATUS_FAIL if the encoding does
//!         not name a point.
//
//*****************************************************************************
static uint32_t
ed_decompress(const uint8_t *encoded)
{
    uint32_t words[ED_WORDS];
    uint8_t  stripped[ED_BYTES];
    uint32_t sign;
    uint32_t match;
    uint32_t isZero;
    uint32_t parity;
    uint32_t status = AM_HAL_STATUS_SUCCESS;

    memcpy(stripped, encoded, ED_BYTES);
    sign = (uint32_t)(stripped[ED_BYTES - 1U] >> 7);
    stripped[ED_BYTES - 1U] &= 0x7FU;
    ed_bytes_to_words(words, stripped);

    //
    // A y-coordinate at or above the field prime is not a valid encoding.
    //
    if (!ed_words_less_than(words, g_am_hal_ed25519_p))
    {
        status = AM_HAL_STATUS_FAIL;
        goto decompress_exit;
    }

    am_hal_cc312_pka_copy_data_into_reg(ED_REG_QY, ED_LEN_REG,
                                        words, ED_WORDS);

    //
    // u = y^2 - 1, v = d*y^2 + 1.
    //
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T1, ED_REG_QY, ED_REG_QY);
    AM_HAL_PKA_MOD_SUB_IM(ED_LEN_N, ED_REG_T2, ED_REG_T1, 1);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T6, ED_LEN_REG,
                                        g_am_hal_ed25519_d, ED_WORDS);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T3, ED_REG_T1, ED_REG_T6);
    AM_HAL_PKA_MOD_ADD_IM(ED_LEN_N, ED_REG_T3, ED_REG_T3, 1);

    //
    // x = u*v^3 * (u*v^7)^((p-5)/8).
    //
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T4, ED_REG_T3, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T4, ED_REG_T4, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T5, ED_REG_T2, ED_REG_T4);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T6, ED_REG_T4, ED_REG_T4);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T6, ED_REG_T6, ED_REG_T3);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T6, ED_REG_T6, ED_REG_T2);

    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T1, ED_LEN_REG,
                                        g_am_hal_ed25519_pm5d8, ED_WORDS);
    AM_HAL_PKA_MOD_EXP(ED_LEN_N, ED_REG_T6, ED_REG_T6, ED_REG_T1);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_QX, ED_REG_T5, ED_REG_T6);

    //
    // Confirm the root: v*x^2 is either u or -u, and in the second case the
    // root is out by a square root of -1.
    //
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T4, ED_REG_QX, ED_REG_QX);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T4, ED_REG_T4, ED_REG_T3);
    AM_HAL_PKA_COMPARE_STATUS(ED_LEN_REG, ED_REG_T4, ED_REG_T2, match);

    if (match == 0U)
    {
        AM_HAL_PKA_MOD_NEG(ED_LEN_N, ED_REG_T5, ED_REG_T2);
        AM_HAL_PKA_COMPARE_STATUS(ED_LEN_REG, ED_REG_T4, ED_REG_T5, match);

        if (match == 0U)
        {
            status = AM_HAL_STATUS_FAIL;
            goto decompress_exit;
        }

        am_hal_cc312_pka_copy_data_into_reg(ED_REG_T6, ED_LEN_REG,
                                            g_am_hal_ed25519_sqrtm1, ED_WORDS);
        AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_QX, ED_REG_QX, ED_REG_T6);
    }

    //
    // The sign bit selects the root. Zero has only one root, so asking for
    // its negative is not a valid encoding.
    //
    AM_HAL_PKA_COMPARE_IM_STATUS(ED_LEN_REG, ED_REG_QX, 0, isZero);
    if ((isZero != 0U) && (sign != 0U))
    {
        status = AM_HAL_STATUS_FAIL;
        goto decompress_exit;
    }

    parity = am_hal_cc312_pka_read_word_from_reg(ED_REG_QX, 0) & 1U;
    if (parity != sign)
    {
        AM_HAL_PKA_MOD_NEG(ED_LEN_N, ED_REG_QX, ED_REG_QX);
    }

    //
    // Extended coordinates with Z = 1 and T = x*y.
    //
    AM_HAL_PKA_CLEAR(ED_LEN_REG, ED_REG_QZ);
    AM_HAL_PKA_SET_BIT0(ED_LEN_REG, ED_REG_QZ, ED_REG_QZ);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_QT, ED_REG_QX, ED_REG_QY);

decompress_exit:
    am_hal_cc312_context_clear(words, sizeof(words));
    am_hal_cc312_context_clear(stripped, sizeof(stripped));

    return status;
}

//*****************************************************************************
//
//! @brief Put the base point into the Q registers.
//
//*****************************************************************************
static void
ed_load_base(void)
{
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_QX, ED_LEN_REG,
                                        g_am_hal_ed25519_bx, ED_WORDS);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_QY, ED_LEN_REG,
                                        g_am_hal_ed25519_by, ED_WORDS);
    AM_HAL_PKA_CLEAR(ED_LEN_REG, ED_REG_QZ);
    AM_HAL_PKA_SET_BIT0(ED_LEN_REG, ED_REG_QZ, ED_REG_QZ);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_QT, ED_REG_QX, ED_REG_QY);
}

//*****************************************************************************
//
//! @brief Multiply the base point by a scalar and encode the result.
//
//*****************************************************************************
static uint32_t
ed_base_mult_encoded(const uint32_t *scalar, uint8_t *out)
{
    uint32_t status;
    ed_point_t result;

    status = ed_pka_start(g_am_hal_ed25519_p, ED_FIELD_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    ed_load_d2();
    ed_load_base();
    result = ed_ladder(scalar);
    ed_encode(&result, out);
    ed_pka_finish();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reduce a 64-byte digest modulo the group order.
//!
//! The digest is split into two 256-bit halves and each is reduced on its
//! own before they are recombined. A Barrett tag is built for operands below
//! the square of the modulus, and a 512-bit value is above that bound for a
//! 253-bit L; the halves are not.
//
//*****************************************************************************
static uint32_t
ed_scalar_reduce(const uint8_t *digest, uint8_t *out)
{
    uint32_t status;
    uint32_t lo[ED_WORDS];
    uint32_t hi[ED_WORDS];

    ed_bytes_to_words(lo, digest);
    ed_bytes_to_words(hi, &digest[ED_BYTES]);

    status = ed_pka_start(g_am_hal_ed25519_l, ED_ORDER_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto scalar_reduce_exit;
    }

    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T1, ED_LEN_REG, lo, ED_WORDS);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T2, ED_LEN_REG, hi, ED_WORDS);
    AM_HAL_PKA_REDUCE(ED_LEN_N, ED_REG_T1, ED_REG_T1);
    AM_HAL_PKA_REDUCE(ED_LEN_N, ED_REG_T2, ED_REG_T2);

    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T3, ED_LEN_REG,
                                        g_am_hal_ed25519_two256_mod_l,
                                        ED_WORDS);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T2, ED_REG_T2, ED_REG_T3);
    AM_HAL_PKA_MOD_ADD(ED_LEN_N, ED_REG_T1, ED_REG_T1, ED_REG_T2);

    am_hal_cc312_pka_copy_data_from_reg(lo, ED_WORDS, ED_REG_T1);
    ed_words_to_bytes(out, lo);
    ed_pka_finish();

    status = AM_HAL_STATUS_SUCCESS;

scalar_reduce_exit:
    am_hal_cc312_context_clear(lo, sizeof(lo));
    am_hal_cc312_context_clear(hi, sizeof(hi));

    return status;
}

//*****************************************************************************
//
//! @brief S = (r + k * a) mod L.
//!
//! The clamped secret scalar can exceed L, so it is reduced before it is
//! multiplied. It is below the square of L, which is what the tag covers.
//
//*****************************************************************************
static uint32_t
ed_scalar_muladd(const uint8_t *r, const uint8_t *k, const uint8_t *a,
                 uint8_t *out)
{
    uint32_t status;
    uint32_t words[ED_WORDS];

    status = ed_pka_start(g_am_hal_ed25519_l, ED_ORDER_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto muladd_exit;
    }

    ed_bytes_to_words(words, a);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T1, ED_LEN_REG,
                                        words, ED_WORDS);
    AM_HAL_PKA_REDUCE(ED_LEN_N, ED_REG_T1, ED_REG_T1);

    ed_bytes_to_words(words, k);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T2, ED_LEN_REG,
                                        words, ED_WORDS);
    AM_HAL_PKA_MOD_MUL(ED_LEN_N, ED_REG_T1, ED_REG_T1, ED_REG_T2);

    ed_bytes_to_words(words, r);
    am_hal_cc312_pka_copy_data_into_reg(ED_REG_T2, ED_LEN_REG,
                                        words, ED_WORDS);
    AM_HAL_PKA_MOD_ADD(ED_LEN_N, ED_REG_T1, ED_REG_T1, ED_REG_T2);

    am_hal_cc312_pka_copy_data_from_reg(words, ED_WORDS, ED_REG_T1);
    ed_words_to_bytes(out, words);
    ed_pka_finish();

    status = AM_HAL_STATUS_SUCCESS;

muladd_exit:
    am_hal_cc312_context_clear(words, sizeof(words));

    return status;
}

//*****************************************************************************
//
//! @brief Expand a seed into the secret scalar and the prefix.
//!
//! RFC 8032 section 5.1.5: the seed is hashed, the lower half clamped into a
//! scalar and the upper half kept as the per-message prefix.
//
//*****************************************************************************
static uint32_t
ed_expand_seed(const uint8_t *seed, uint8_t *scalar, uint8_t *prefix)
{
    uint32_t status;
    uint8_t digest[AM_HAL_SHA512_DIGEST_SIZE];
    uint8_t seedLocal[AM_HAL_ED25519_SEED_SIZE];

    //
    // Hash from a SRAM copy. Callers often pass a const seed in MRAM/XIP;
    // keeping the digest input in local RAM matches the rest of the HAL.
    //
    memcpy(seedLocal, seed, AM_HAL_ED25519_SEED_SIZE);

    status = am_hal_cc312_sha512(AM_HAL_CC312_SHA512, seedLocal,
                                 AM_HAL_ED25519_SEED_SIZE, digest);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto expand_exit;
    }

    memcpy(scalar, digest, ED_BYTES);
    memcpy(prefix, &digest[ED_BYTES], ED_BYTES);

    scalar[0]             &= 0xF8U;
    scalar[ED_BYTES - 1U] &= 0x7FU;
    scalar[ED_BYTES - 1U] |= 0x40U;

expand_exit:
    am_hal_cc312_context_clear(digest, sizeof(digest));
    am_hal_cc312_context_clear(seedLocal, sizeof(seedLocal));

    return status;
}

//*****************************************************************************
//
//! @brief The challenge k = H(R || A || M) reduced modulo the group order.
//
//*****************************************************************************
static uint32_t
ed_challenge(const uint8_t *rEncoded, const uint8_t *aEncoded,
             const uint8_t *message, uint32_t messageLen, uint8_t *out)
{
    uint32_t status;
    uint8_t digest[AM_HAL_SHA512_DIGEST_SIZE];
    am_hal_cc312_sha512_context_t ctx;

    status = am_hal_cc312_sha512_init(&ctx, AM_HAL_CC312_SHA512);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto challenge_exit;
    }

    status = am_hal_cc312_sha512_update(&ctx, rEncoded, ED_BYTES);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_update(&ctx, aEncoded, ED_BYTES);
    }
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_update(&ctx, message, messageLen);
    }
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_finish(&ctx, digest);
    }

    (void)am_hal_cc312_sha512_free(&ctx);

    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = ed_scalar_reduce(digest, out);
    }

challenge_exit:
    am_hal_cc312_context_clear(digest, sizeof(digest));

    return status;
}

//*****************************************************************************
//
// Derive the public key for a private seed.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ed25519_public_key(const uint8_t *seed, uint8_t *publicKey)
{
    uint32_t status;
    uint8_t  scalar[ED_BYTES];
    uint8_t  prefix[ED_BYTES];
    uint32_t words[ED_WORDS];

    if ((seed == NULL) || (publicKey == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = ed_expand_seed(seed, scalar, prefix);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto public_key_exit;
    }

    ed_bytes_to_words(words, scalar);
    status = ed_base_mult_encoded(words, publicKey);

public_key_exit:
    am_hal_cc312_context_clear(scalar, sizeof(scalar));
    am_hal_cc312_context_clear(prefix, sizeof(prefix));
    am_hal_cc312_context_clear(words, sizeof(words));

    return status;
}

//*****************************************************************************
//
// Sign a message.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ed25519_sign(const uint8_t *seed, const uint8_t *message,
                          uint32_t messageLen, uint8_t *signature)
{
    uint32_t status;
    uint8_t  scalar[ED_BYTES];
    uint8_t  prefix[ED_BYTES];
    uint8_t  publicKey[ED_BYTES];
    uint8_t  r[ED_BYTES];
    uint8_t  k[ED_BYTES];
    uint8_t  digest[AM_HAL_SHA512_DIGEST_SIZE];
    uint32_t words[ED_WORDS];
    am_hal_cc312_sha512_context_t ctx;

    if ((seed == NULL) || ((message == NULL) && (messageLen != 0)) ||
        (signature == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = ed_expand_seed(seed, scalar, prefix);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    ed_bytes_to_words(words, scalar);
    status = ed_base_mult_encoded(words, publicKey);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    //
    // r = H(prefix || M) mod L. The prefix is what makes this deterministic
    // without being predictable to anyone without the seed.
    //
    status = am_hal_cc312_sha512_init(&ctx, AM_HAL_CC312_SHA512);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    status = am_hal_cc312_sha512_update(&ctx, prefix, ED_BYTES);
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_update(&ctx, message, messageLen);
    }
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        status = am_hal_cc312_sha512_finish(&ctx, digest);
    }

    (void)am_hal_cc312_sha512_free(&ctx);

    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    status = ed_scalar_reduce(digest, r);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    //
    // R = r * B.
    //
    ed_bytes_to_words(words, r);
    status = ed_base_mult_encoded(words, signature);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    //
    // S = (r + H(R || A || M) * a) mod L.
    //
    status = ed_challenge(signature, publicKey, message, messageLen, k);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto sign_exit;
    }

    status = ed_scalar_muladd(r, k, scalar, &signature[ED_BYTES]);

sign_exit:
    am_hal_cc312_context_clear(scalar, sizeof(scalar));
    am_hal_cc312_context_clear(prefix, sizeof(prefix));
    am_hal_cc312_context_clear(r, sizeof(r));
    am_hal_cc312_context_clear(k, sizeof(k));
    am_hal_cc312_context_clear(digest, sizeof(digest));
    am_hal_cc312_context_clear(words, sizeof(words));

    return status;
}

//*****************************************************************************
//
// Verify a signature.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ed25519_verify(const uint8_t *publicKey, const uint8_t *message,
                            uint32_t messageLen, const uint8_t *signature)
{
    uint32_t status;
    uint8_t  k[ED_BYTES];
    uint8_t  lhs[ED_BYTES];
    uint8_t  rhs[ED_BYTES];
    uint32_t words[ED_WORDS];
    uint32_t sWords[ED_WORDS];
    ed_point_t result;
    ed_point_t q = { ED_REG_QX, ED_REG_QY, ED_REG_QZ, ED_REG_QT };

    if ((publicKey == NULL) || ((message == NULL) && (messageLen != 0)) ||
        (signature == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // A signature carrying S at or above the group order is malleable and is
    // rejected rather than reduced.
    //
    ed_bytes_to_words(sWords, &signature[ED_BYTES]);
    if (!ed_words_less_than(sWords, g_am_hal_ed25519_l))
    {
        status = AM_HAL_STATUS_FAIL;
        goto verify_exit;
    }

    status = ed_challenge(signature, publicKey, message, messageLen, k);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto verify_exit;
    }

    //
    // Left side: S * B.
    //
    status = ed_base_mult_encoded(sWords, lhs);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto verify_exit;
    }

    //
    // Right side: R + k * A, with both points recovered from their
    // encodings.
    //
    status = ed_pka_start(g_am_hal_ed25519_p, ED_FIELD_BITS);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto verify_exit;
    }

    ed_load_d2();

    status = ed_decompress(publicKey);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        ed_pka_finish();
        goto verify_exit;
    }

    ed_bytes_to_words(words, k);
    result = ed_ladder(words);

    //
    // Park k*A in the second accumulator rather than in a temporary: the
    // decode that follows uses every temporary as scratch, and R has to land
    // in the input registers the ladder has finished with.
    //
    AM_HAL_PKA_COPY(ED_LEN_REG, ED_REG_BX, result.x);
    AM_HAL_PKA_COPY(ED_LEN_REG, ED_REG_BY, result.y);
    AM_HAL_PKA_COPY(ED_LEN_REG, ED_REG_BZ, result.z);
    AM_HAL_PKA_COPY(ED_LEN_REG, ED_REG_BT, result.t);

    status = ed_decompress(signature);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        ed_pka_finish();
        goto verify_exit;
    }

    {
        ed_point_t b = { ED_REG_BX, ED_REG_BY, ED_REG_BZ, ED_REG_BT };

        ed_add(&b, &b, &q);
        ed_encode(&b, rhs);
    }

    ed_pka_finish();

    status = (memcmp(lhs, rhs, ED_BYTES) == 0) ? AM_HAL_STATUS_SUCCESS
                                               : AM_HAL_STATUS_FAIL;

verify_exit:
    am_hal_cc312_context_clear(k, sizeof(k));
    am_hal_cc312_context_clear(words, sizeof(words));
    am_hal_cc312_context_clear(sWords, sizeof(sWords));

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
