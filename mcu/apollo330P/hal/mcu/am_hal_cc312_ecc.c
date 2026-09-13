//*****************************************************************************
//
//! @file am_hal_cc312_ecc.c
//!
//! @brief Hardware abstraction for CryptoCell-312 elliptic-curve arithmetic
//!
//! Purpose: Register-level port of the CC312 runtime Weierstrass EC scalar
//! multiplication (pka_ec_wrst.c, pka_ec_wrst_smul_no_scap.c) on top of the
//! PKA big-integer engine. Computes R = k * P using modified-Jacobian
//! coordinates and a NAF double-and-add ladder.
//!
//! @addtogroup cc312_ecc_ap330P CC312 ECC - Curve Arithmetic
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

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

#include "am_mcu_apollo.h"
#include <string.h>
#include <stdint.h>

//*****************************************************************************
//
//! @name PKA register allocation for EC arithmetic (from pka_ec_wrst_glob_regs.h).
//! @{
//
//*****************************************************************************
#define ECC_REG_N       AM_HAL_PKA_REG_N    // 0  (modulus p / N1)
#define ECC_REG_NP      AM_HAL_PKA_REG_NP   // 1  (Barrett tag)
#define ECC_REG_T       2
#define ECC_REG_T1      3
#define ECC_REG_T2      4
#define ECC_REG_T3      5
#define ECC_REG_AQ      6
#define ECC_REG_A_NM2   7
#define ECC_REG_N4      8
#define ECC_REG_N8      9
#define ECC_REG_N12     10
#define ECC_REG_EC_A    11
#define ECC_REG_T4      12
#define ECC_REG_AAA_Z   13
//! Scalar-multiply working registers.
#define ECC_SM_TP       14
#define ECC_SM_ZR       15
#define ECC_SM_TR       16
#define ECC_SM_XP       18
#define ECC_SM_YP       19
#define ECC_SM_XR       20
#define ECC_SM_YR       21
#define ECC_SM_REG_COUNT 22
//! Double-scalar-multiply (verify) scratch registers (from dsa_verify_regs.h).
#define ECC_V_XPQ       14
#define ECC_V_YPQ       15
#define ECC_V_ZR        16
#define ECC_V_TR        17
//! @}

#define ECC_LEN_N       AM_HAL_PKA_LEN_ID_N_BITS         // exact modulus size
#define ECC_LEN_NR      AM_HAL_PKA_LEN_ID_N_PKA_REG_BITS // extended (with extra word)

#define ECC_CALC_FULL_32BIT_WORDS(bits)     (((bits) + 31U) / 32U)

//*****************************************************************************
//
// secp256r1 (NIST P-256) domain parameters, little-endian words.
//
//*****************************************************************************
static const am_hal_cc312_ecc_domain_t g_am_hal_ecc_secp256r1 =
{
    .modSizeInBits = 256,
    .ordSizeInBits = 256,
    // p
    .ecP   = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000,
              0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF},
    // a = p - 3
    .ecA   = {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000,
              0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF},
    // b
    .ecB   = {0x27D2604B, 0x3BCE3C3E, 0xCC53B0F6, 0x651D06B0,
              0x769886BC, 0xB3EBBD55, 0xAA3A93E7, 0x5AC635D8},
    // n (generator order)
    .ecOrd = {0xFC632551, 0xF3B9CAC2, 0xA7179E84, 0xBCE6FAAD,
              0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF},
    // Gx
    .ecGx  = {0xD898C296, 0xF4A13945, 0x2DEB33A0, 0x77037D81,
              0x63A440F2, 0xF8BCE6E5, 0xE12C4247, 0x6B17D1F2},
    // Gy
    .ecGy  = {0x37BF51F5, 0xCBB64068, 0x6B315ECE, 0x2BCE3357,
              0x7C0F9E16, 0x8EE7EB4A, 0xFE1A7F9B, 0x4FE342E2},
};

const am_hal_cc312_ecc_domain_t *
am_hal_cc312_ecc_domain_secp256r1(void)
{
    return &g_am_hal_ecc_secp256r1;
}

static const am_hal_cc312_ecc_domain_t g_am_hal_ecc_secp384r1 =
{
    .modSizeInBits = 384,
    .ordSizeInBits = 384,
    .ecP   = {0xFFFFFFFF, 0x00000000, 0x00000000, 0xFFFFFFFF,
              0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
    .ecA   = {0xFFFFFFFC, 0x00000000, 0x00000000, 0xFFFFFFFF,
              0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
    .ecB   = {0xD3EC2AEF, 0x2A85C8ED, 0x8A2ED19D, 0xC656398D,
              0x5013875A, 0x0314088F, 0xFE814112, 0x181D9C6E,
              0xE3F82D19, 0x988E056B, 0xE23EE7E4, 0xB3312FA7},
    .ecOrd = {0xCCC52973, 0xECEC196A, 0x48B0A77A, 0x581A0DB2,
              0xF4372DDF, 0xC7634D81, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
    .ecGx  = {0x72760AB7, 0x3A545E38, 0xBF55296C, 0x5502F25D,
              0x82542A38, 0x59F741E0, 0x8BA79B98, 0x6E1D3B62,
              0xF320AD74, 0x8EB1C71E, 0xBE8B0537, 0xAA87CA22},
    .ecGy  = {0x90EA0E5F, 0x7A431D7C, 0x1D7E819D, 0x0A60B1CE,
              0xB5F0B8C0, 0xE9DA3113, 0x289A147C, 0xF8F41DBD,
              0x9292DC29, 0x5D9E98BF, 0x96262C6F, 0x3617DE4A},
};

static const am_hal_cc312_ecc_domain_t g_am_hal_ecc_secp521r1 =
{
    .modSizeInBits = 521,
    .ordSizeInBits = 521,
    .ecP   = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0x000001FF},
    .ecA   = {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0x000001FF},
    .ecB   = {0x6B503F00, 0xEF451FD4, 0x3D2C34F1, 0x3573DF88,
              0x3BB1BF07, 0x1652C0BD, 0xEC7E937B, 0x56193951,
              0x8EF109E1, 0xB8B48991, 0x99B315F3, 0xA2DA725B,
              0xB68540EE, 0x929A21A0, 0x8E1C9A1F, 0x953EB961,
              0x00000051},
    .ecOrd = {0x91386409, 0xBB6FB71E, 0x899C47AE, 0x3BB5C9B8,
              0xF709A5D0, 0x7FCC0148, 0xBF2F966B, 0x51868783,
              0xFFFFFFFA, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
              0x000001FF},
    .ecGx  = {0xC2E5BD66, 0xF97E7E31, 0x856A429B, 0x3348B3C1,
              0xA2FFA8DE, 0xFE1DC127, 0xEFE75928, 0xA14B5E77,
              0x6B4D3DBA, 0xF828AF60, 0x053FB521, 0x9C648139,
              0x2395B442, 0x9E3ECB66, 0x0404E9CD, 0x858E06B7,
              0x000000C6},
    .ecGy  = {0x9FD16650, 0x88BE9476, 0xA272C240, 0x353C7086,
              0x3FAD0761, 0xC550B901, 0x5EF42640, 0x97EE7299,
              0x273E662C, 0x17AFBD17, 0x579B4468, 0x98F54449,
              0x2C7D1BD9, 0x5C8A5FB4, 0x9A3BC004, 0x39296A78,
              0x00000118},
};

//*****************************************************************************
//
// Return the secp384r1 domain parameters.
//
//*****************************************************************************
const am_hal_cc312_ecc_domain_t *
am_hal_cc312_ecc_domain_secp384r1(void)
{
    return &g_am_hal_ecc_secp384r1;
}

//*****************************************************************************
//
// Return the secp521r1 domain parameters.
//
//*****************************************************************************
const am_hal_cc312_ecc_domain_t *
am_hal_cc312_ecc_domain_secp521r1(void)
{
    return &g_am_hal_ecc_secp521r1;
}

//*****************************************************************************
//
// Small multi-precision helpers (little-endian word buffers).
//
//*****************************************************************************

//! Effective size of an LE word counter, in bits (0 if zero).
static uint32_t
ec_eff_size_bits(const uint32_t *p, uint32_t sizeWords)
{
    int32_t  i;
    uint32_t w, bits, mask;

    for (i = (int32_t)sizeWords - 1; i >= 0; i--)
    {
        if (p[i] != 0)
        {
            break;
        }
    }
    if (i < 0)
    {
        return 0;
    }

    w = p[i];
    bits = (uint32_t)(i + 1) * 32U;
    mask = 1UL << 31;
    while ((w & mask) == 0U)
    {
        bits--;
        mask >>= 1;
    }
    return bits;
}

//! Add @p val to an LE word counter; returns the carry out.
static uint32_t
ec_inc_counter(uint32_t *p, uint32_t val, uint32_t sizeWords)
{
    uint64_t carry = val;
    uint32_t i;

    for (i = 0; (i < sizeWords) && (carry != 0U); i++)
    {
        uint64_t s = (uint64_t)p[i] + carry;
        p[i] = (uint32_t)s;
        carry = s >> 32;
    }
    return (uint32_t)carry;
}

//! Divide an LE word vector by 2 (LSB in word 0).
static void
ec_divide_by_2(uint32_t *p, uint32_t sizeWords)
{
    uint32_t i, t;

    for (i = 0; i < sizeWords - 1U; i++)
    {
        p[i] >>= 1;
        t = p[i + 1] & 1UL;
        p[i] |= t << 31;
    }
    p[sizeWords - 1U] >>= 1;
}

//*****************************************************************************
//
// EC point operations (modified-Jacobian coordinates). Ported verbatim from
// pka_ec_wrst.c; all PKA register numbers are virtual.
//
//*****************************************************************************

//! EC point doubling: modified -> modified.
static void
ec_double_mdf2mdf(uint32_t x, uint32_t y, uint32_t z, uint32_t t,
                  uint32_t x1, uint32_t y1, uint32_t z1, uint32_t t1)
{
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T4, y1, y1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, z, ECC_REG_T4, z1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y1, y1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T4, x1, x1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T4, ECC_REG_T4, ECC_REG_T4);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T4, y, ECC_REG_T4);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, x1, x1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, ECC_REG_T2);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, t1, ECC_REG_T2);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T4, ECC_REG_N4, ECC_REG_T4);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, x, ECC_REG_T2, ECC_REG_T2, ECC_REG_T4);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T4, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T4, x, ECC_REG_T4);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T3, ECC_REG_N12, ECC_REG_T4);
    AM_HAL_PKA_ADD(ECC_LEN_NR, y, y, y);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y, y);
    AM_HAL_PKA_ADD(ECC_LEN_NR, y, y, y);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T4, y, y);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T4, ECC_REG_T4, t1);
    AM_HAL_PKA_SUB(ECC_LEN_NR, y, ECC_REG_N8, y);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, y, ECC_REG_T3, ECC_REG_T2, y);
    AM_HAL_PKA_COPY(ECC_LEN_NR, t, ECC_REG_T4);
}

//! EC point doubling: modified -> Jacobian.
static void
ec_double_mdf2jcb(uint32_t x, uint32_t y, uint32_t z,
                  uint32_t x1, uint32_t y1, uint32_t z1, uint32_t t1)
{
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T, y1, y1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, z, ECC_REG_T, z1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y1, y1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T, x1, x1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T, ECC_REG_T, ECC_REG_T);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T, y, ECC_REG_T);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, x1, x1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, ECC_REG_T2);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, t1, ECC_REG_T2);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T, ECC_REG_N4, ECC_REG_T);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, x, ECC_REG_T2, ECC_REG_T2, ECC_REG_T);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T, x, ECC_REG_T);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T3, ECC_REG_N12, ECC_REG_T);
    AM_HAL_PKA_ADD(ECC_LEN_NR, y, y, y);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y, y);
    AM_HAL_PKA_ADD(ECC_LEN_NR, y, y, y);
    AM_HAL_PKA_SUB(ECC_LEN_NR, y, ECC_REG_N8, y);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, y, ECC_REG_T3, ECC_REG_T2, y);
}

//! EC point add: Jacobian + affine -> modified.
static void
ec_add_jcb_afn2mdf(uint32_t x, uint32_t y, uint32_t z, uint32_t t,
                   uint32_t x1, uint32_t y1, uint32_t z1,
                   uint32_t x2, uint32_t y2)
{
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, z1, z1);
    AM_HAL_PKA_SUB(ECC_LEN_NR, x, ECC_REG_N12, x1);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, ECC_REG_T1, x2, t, x);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, z1, t);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, y2, t);
    AM_HAL_PKA_SUB(ECC_LEN_NR, t, ECC_REG_N4, t);
    AM_HAL_PKA_ADD(ECC_LEN_NR, t, y1, t);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, z, z1, ECC_REG_T1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, ECC_REG_T1, ECC_REG_T1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T1, ECC_REG_T1, ECC_REG_T2);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T1, ECC_REG_N4, ECC_REG_T1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, ECC_REG_T1, y1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, x, ECC_REG_T2);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, x, t, t, ECC_REG_T1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, x, ECC_REG_T2);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, y, t, ECC_REG_T2, y);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, z, z);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, t, t);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, t, ECC_REG_EC_A, t);
}

//! Convert Jacobian point to affine: p(x,y,z) -> p(x,y). Constant-time inverse.
static void
ec_jcb2afn(uint32_t x, uint32_t y, uint32_t z)
{
    AM_HAL_PKA_MOD_INV_W_EXP(ECC_REG_AQ, z, ECC_REG_A_NM2);

    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y, ECC_REG_AQ);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_AQ, ECC_REG_AQ, ECC_REG_AQ);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, x, x, ECC_REG_AQ);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, y, ECC_REG_AQ);

    AM_HAL_PKA_REDUCE(ECC_LEN_N, x, x);
    AM_HAL_PKA_REDUCE(ECC_LEN_N, y, y);
}

//! EC point add: affine + affine -> affine (used by double-scalar verify).
static void
ec_add_aff(uint32_t x, uint32_t y,
           uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
    AM_HAL_PKA_SUB(ECC_LEN_NR, x, ECC_REG_N, x1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_AAA_Z, x, x2);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T, ECC_REG_N, y2);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T, y1, ECC_REG_T);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, ECC_REG_AAA_Z, ECC_REG_AAA_Z);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T1, ECC_REG_AAA_Z, ECC_REG_T2);
    AM_HAL_PKA_SUB(ECC_LEN_NR, ECC_REG_T1, ECC_REG_N4, ECC_REG_T1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, y, ECC_REG_T1, y1);
    AM_HAL_PKA_MOD_MUL_NFR(ECC_LEN_N, ECC_REG_T2, x, ECC_REG_T2);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, x, ECC_REG_T, ECC_REG_T, ECC_REG_T1);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, x, ECC_REG_T2, x);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_T2, x, ECC_REG_T2);
    AM_HAL_PKA_MOD_MUL_ACC_NFR(ECC_LEN_N, y, ECC_REG_T, ECC_REG_T2, y);
    ec_jcb2afn(x, y, ECC_REG_AAA_Z);
}

//! Return the next most-significant bit (index @p i) of scalar register @p rX.
static uint32_t
ec_get_next_ms_bit(uint32_t rX, int32_t i, uint32_t *pW, uint32_t *pIsNew)
{
    uint32_t b;

    if (*pIsNew || ((uint32_t)i & 31U) == 31U)
    {
        *pW = am_hal_cc312_pka_read_word_from_reg(rX, (uint32_t)(i >> 5));
        if (((uint32_t)i & 31U) != 31U)
        {
            *pW <<= (31U - ((uint32_t)i & 31U));
        }
        *pIsNew = 0;
    }

    b = *pW >> 31;
    *pW <<= 1;
    return b;
}

//*****************************************************************************
//
//! @brief Simultaneous double scalar multiply R = a*P + b*Q (Strauss/Klimov).
//!
//! Register-level helper used by ECDSA verify. The modulus N/NP, curve EC_A,
//! and the multiples N4/N8/N12 must already be loaded; all operands are PKA
//! virtual register numbers. Returns AM_HAL_STATUS_FAIL on a degenerate scalar.
//
//*****************************************************************************
uint32_t
am_hal_cc312_ecc_sum2_scalar_mult(uint32_t xr, uint32_t yr,
                                  uint32_t a, uint32_t xp, uint32_t yp,
                                  uint32_t b, uint32_t xq, uint32_t yq)
{
    uint32_t wA = 0, wB = 0;
    uint32_t stat;
    int32_t  b2, i;
    uint32_t sa, sb;
    uint32_t isNewA = 1, isNewB = 1;

    AM_HAL_PKA_COMPARE_IM_STATUS(ECC_LEN_NR, a, 0, stat);
    if (stat == 1)
    {
        return AM_HAL_STATUS_FAIL;
    }
    AM_HAL_PKA_COMPARE_IM_STATUS(ECC_LEN_NR, b, 0, stat);
    if (stat == 1)
    {
        return AM_HAL_STATUS_FAIL;
    }

    sa = am_hal_cc312_pka_get_reg_effective_size_in_bits(a);
    sb = am_hal_cc312_pka_get_reg_effective_size_in_bits(b);
    i = (int32_t)((sa > sb) ? sa : sb) - 1;

    ec_add_aff(ECC_V_XPQ, ECC_V_YPQ, xp, yp, xq, yq);   // p+q

    b2 = (int32_t)ec_get_next_ms_bit(a, i, &wA, &isNewA) * 2 +
         (int32_t)ec_get_next_ms_bit(b, i, &wB, &isNewB);
    switch (b2)
    {
        case 1:
            AM_HAL_PKA_COPY(ECC_LEN_NR, xr, xq);
            AM_HAL_PKA_COPY(ECC_LEN_NR, yr, yq);
            break;
        case 2:
            AM_HAL_PKA_COPY(ECC_LEN_NR, xr, xp);
            AM_HAL_PKA_COPY(ECC_LEN_NR, yr, yp);
            break;
        case 3:
            AM_HAL_PKA_COPY(ECC_LEN_NR, xr, ECC_V_XPQ);
            AM_HAL_PKA_COPY(ECC_LEN_NR, yr, ECC_V_YPQ);
            break;
        default:
            return AM_HAL_STATUS_FAIL;
    }
    AM_HAL_PKA_SET_VAL(ECC_V_ZR, 1);
    AM_HAL_PKA_COPY(ECC_LEN_NR, ECC_V_TR, ECC_REG_EC_A);

    while (--i >= 0)
    {
        b2 = (int32_t)ec_get_next_ms_bit(a, i, &wA, &isNewA) * 2 +
             (int32_t)ec_get_next_ms_bit(b, i, &wB, &isNewB);
        if (b2 == 0)
        {
            ec_double_mdf2mdf(xr, yr, ECC_V_ZR, ECC_V_TR, xr, yr, ECC_V_ZR, ECC_V_TR);
        }
        else
        {
            ec_double_mdf2jcb(xr, yr, ECC_V_ZR, xr, yr, ECC_V_ZR, ECC_V_TR);
            switch (b2)
            {
                case 1:
                    ec_add_jcb_afn2mdf(xr, yr, ECC_V_ZR, ECC_V_TR, xr, yr, ECC_V_ZR, xq, yq);
                    break;
                case 2:
                    ec_add_jcb_afn2mdf(xr, yr, ECC_V_ZR, ECC_V_TR, xr, yr, ECC_V_ZR, xp, yp);
                    break;
                case 3:
                    ec_add_jcb_afn2mdf(xr, yr, ECC_V_ZR, ECC_V_TR, xr, yr, ECC_V_ZR, ECC_V_XPQ, ECC_V_YPQ);
                    break;
                default:
                    return AM_HAL_STATUS_FAIL;
            }
        }
    }
    ec_jcb2afn(xr, yr, ECC_V_ZR);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Build the NAF (non-adjacent form) digit string for scalar @p pK.
//!
//! Ported from PkaBuildNaf. On return *pNaf points to the first (MSB) digit
//! of a NUL-terminated string of '+', '-', '0'.
//
//*****************************************************************************
static uint32_t
ec_build_naf(char **pNaf, uint32_t *pNafSz, uint32_t *pK, uint32_t keySzBit)
{
    uint32_t wK;
    char    *p;

    if (keySzBit == 0U || (keySzBit + 2U) > *pNafSz)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pK[(keySzBit - 1U) / 32U] >> ((keySzBit - 1U) & 0x1FU)) != 1U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *pNafSz = 0;
    p = *pNaf + keySzBit + 1U;
    *p = 0;
    wK = ECC_CALC_FULL_32BIT_WORDS(keySzBit);
    pK[wK] = 0;

    while (keySzBit)
    {
        uint32_t carry, msBit;

        (*pNafSz)++;
        --p;
        if (p < *pNaf)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }

        *p = (pK[0] & 1U) ? ((pK[0] & 2U) ? '-' : '+') : '0';

        msBit = pK[wK - 1U] >> ((keySzBit % 32U) - 1U);
        if (*p == '-')
        {
            carry = ec_inc_counter(pK, 1, wK);
            if (carry)
            {
                pK[wK] = 1;
                keySzBit++;
            }
            else if ((pK[wK - 1U] >> ((keySzBit % 32U) - 1U)) > msBit)
            {
                keySzBit++;
            }
        }

        ec_divide_by_2(pK, wK + 1U);
        keySzBit--;

        wK = ECC_CALC_FULL_32BIT_WORDS(keySzBit);
    }

    *pNaf = p;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Inner scalar multiply over a prepared NAF: r = k*p (affine out).
//!        Registers N/NP/EC_A and the input point must be preloaded.
//
//*****************************************************************************
static void
ec_scalar_mult_inner(uint32_t xr, uint32_t yr, const char *k,
                     uint32_t xp, uint32_t yp)
{
    uint32_t tp = ECC_SM_TP;
    uint32_t zr = ECC_SM_ZR;
    uint32_t tr = ECC_SM_TR;

    //
    // Precompute modulus multiples 4N, 8N, 12N (used by the modified-coordinate
    // arithmetic to keep intermediates non-negative).
    //
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_N4, ECC_REG_N, ECC_REG_N);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_N4, ECC_REG_N4, ECC_REG_N4);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_N8, ECC_REG_N4, ECC_REG_N4);
    AM_HAL_PKA_ADD(ECC_LEN_NR, ECC_REG_N12, ECC_REG_N8, ECC_REG_N4);

    AM_HAL_PKA_SUB(ECC_LEN_NR, tp, ECC_REG_N4, yp);   // y of -p

    //
    // r = p (first NAF digit is '+').
    //
    AM_HAL_PKA_COPY(ECC_LEN_NR, xr, xp);
    AM_HAL_PKA_COPY(ECC_LEN_NR, yr, yp);
    AM_HAL_PKA_SET_VAL(zr, 1);
    AM_HAL_PKA_COPY(ECC_LEN_NR, tr, ECC_REG_EC_A);

    while (*++k)
    {
        if (*k == '0')
        {
            ec_double_mdf2mdf(xr, yr, zr, tr, xr, yr, zr, tr);
        }
        else
        {
            ec_double_mdf2jcb(xr, yr, zr, xr, yr, zr, tr);
            if (*k == '+')
            {
                ec_add_jcb_afn2mdf(xr, yr, zr, tr, xr, yr, zr, xp, yp);
            }
            else
            {
                ec_add_jcb_afn2mdf(xr, yr, zr, tr, xr, yr, zr, xp, tp);
            }
        }
    }

    ec_jcb2afn(xr, yr, zr);
}

//*****************************************************************************
//
// EC scalar multiplication R = scalar * P (affine).
//
//*****************************************************************************
uint32_t
am_hal_cc312_ecc_scalar_mult(const am_hal_cc312_ecc_domain_t *pDomain,
                             const uint32_t *scalar,
                             uint32_t scalarSizeWords,
                             const uint32_t *inX,
                             const uint32_t *inY,
                             uint32_t *outX,
                             uint32_t *outY)
{
    uint32_t status;
    uint32_t modSizeInBits, modSizeInWords, ordSizeInWords;
    uint32_t scalarSizeBits;
    uint32_t nafSz;
    char    *naf;
    uint32_t kt[AM_HAL_ECC_MAX_ORD_WORDS + 2];
    char     nafBuf[(AM_HAL_ECC_MAX_ORD_WORDS + 1) * 32];

    if (pDomain == NULL || scalar == NULL || inX == NULL || inY == NULL ||
        outX == NULL || outY == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    modSizeInBits  = pDomain->modSizeInBits;
    modSizeInWords = ECC_CALC_FULL_32BIT_WORDS(modSizeInBits);
    ordSizeInWords = ECC_CALC_FULL_32BIT_WORDS(pDomain->ordSizeInBits);

    scalarSizeBits = ec_eff_size_bits(scalar, scalarSizeWords);
    if (scalarSizeBits == 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Build the NAF from a zero-extended working copy of the scalar.
    //
    memset(kt, 0, sizeof(kt));
    memcpy(kt, scalar, scalarSizeWords * 4U);
    memset(nafBuf, 0, sizeof(nafBuf));
    naf   = nafBuf;
    nafSz = sizeof(nafBuf);

    status = ec_build_naf(&naf, &nafSz, kt, scalarSizeBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Initialize the PKA, load the modulus, compute the Barrett tag, and load
    // the curve coefficient and the input point.
    //
    status = am_hal_cc312_pka_init(modSizeInBits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    am_hal_cc312_pka_copy_data_into_reg(ECC_REG_N, ECC_LEN_NR, pDomain->ecP, modSizeInWords);
    am_hal_cc312_pka_calc_np(ECC_LEN_N, modSizeInBits, ECC_REG_N, ECC_REG_NP,
                             ECC_REG_T, ECC_REG_T1);
    am_hal_cc312_pka_copy_data_into_reg(ECC_REG_EC_A, ECC_LEN_NR, pDomain->ecA, modSizeInWords);
    am_hal_cc312_pka_copy_data_into_reg(ECC_SM_XP, ECC_LEN_NR, inX, modSizeInWords);
    am_hal_cc312_pka_copy_data_into_reg(ECC_SM_YP, ECC_LEN_NR, inY, modSizeInWords);

    ec_scalar_mult_inner(ECC_SM_XR, ECC_SM_YR, naf, ECC_SM_XP, ECC_SM_YP);

    am_hal_cc312_pka_copy_data_from_reg(outX, modSizeInWords, ECC_SM_XR);
    am_hal_cc312_pka_copy_data_from_reg(outY, modSizeInWords, ECC_SM_YR);

    am_hal_cc312_pka_clear_block_of_regs(ECC_REG_N, ECC_SM_REG_COUNT);
    am_hal_cc312_pka_finish();

    //
    // Scrub the scalar working copy.
    //
    memset(kt, 0, sizeof(kt));
    memset(nafBuf, 0, sizeof(nafBuf));
    (void)ordSizeInWords;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
