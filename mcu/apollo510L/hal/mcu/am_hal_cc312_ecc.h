//*****************************************************************************
//
//! @file am_hal_cc312_ecc.h
//!
//! @brief Hardware abstraction for CryptoCell-312 elliptic-curve arithmetic
//!
//! Purpose: Register-level port of the CC312 runtime Weierstrass EC scalar
//! multiplication (pka_ec_wrst*.c). Builds on the PKA big-integer engine
//! (am_hal_cc312_pka) to compute R = k * P on short-Weierstrass curves using
//! modified-Jacobian coordinates and a NAF double-and-add ladder. This is the
//! core primitive used by the ECDSA layer.
//!
//! @addtogroup cc312_ecc_ap510L CC312 ECC - Curve Arithmetic
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

#ifndef AM_HAL_CC312_ECC_H
#define AM_HAL_CC312_ECC_H

#include "am_hal_cc312_pka.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name EC sizing (sized for the largest supported Weierstrass curve).
//! @{
//
//*****************************************************************************
#define AM_HAL_ECC_MAX_MOD_WORDS    AM_HAL_PKA_MAX_EC_MOD_SIZE_32BIT_WORDS  //!< 17 (secp521r1).
#define AM_HAL_ECC_MAX_ORD_WORDS    (AM_HAL_ECC_MAX_MOD_WORDS + 1)
//! @}

//*****************************************************************************
//
//! EC domain parameters (short Weierstrass, all values little-endian words).
//
//*****************************************************************************
typedef struct
{
    uint32_t modSizeInBits;                     //!< Field prime size in bits.
    uint32_t ordSizeInBits;                     //!< Generator order size in bits.
    uint32_t ecP[AM_HAL_ECC_MAX_MOD_WORDS];     //!< Field prime p.
    uint32_t ecA[AM_HAL_ECC_MAX_MOD_WORDS];     //!< Curve coefficient a.
    uint32_t ecB[AM_HAL_ECC_MAX_MOD_WORDS];     //!< Curve coefficient b.
    uint32_t ecOrd[AM_HAL_ECC_MAX_ORD_WORDS];   //!< Generator order n.
    uint32_t ecGx[AM_HAL_ECC_MAX_MOD_WORDS];    //!< Generator X.
    uint32_t ecGy[AM_HAL_ECC_MAX_MOD_WORDS];    //!< Generator Y.
}
am_hal_cc312_ecc_domain_t;

//*****************************************************************************
//
//! @brief Return a pointer to the built-in secp256r1 (NIST P-256) domain.
//
//*****************************************************************************
extern const am_hal_cc312_ecc_domain_t *am_hal_cc312_ecc_domain_secp256r1(void);

//*****************************************************************************
//
//! @brief Return the secp384r1 domain parameters.
//!
//! @return Pointer to the static domain description.
//
//*****************************************************************************
extern const am_hal_cc312_ecc_domain_t *am_hal_cc312_ecc_domain_secp384r1(void);

//*****************************************************************************
//
//! @brief Return the secp521r1 domain parameters.
//!
//! @return Pointer to the static domain description.
//
//*****************************************************************************
extern const am_hal_cc312_ecc_domain_t *am_hal_cc312_ecc_domain_secp521r1(void);

//*****************************************************************************
//
//! @brief EC scalar multiplication R = scalar * P (affine coordinates).
//!
//! Not SCA-protected; intended for public-point operations (e.g. verify, or
//! self-test against known answers). All coordinate / scalar buffers are
//! little-endian 32-bit word arrays of the curve's modulus word size.
//!
//! @param pDomain         - EC domain.
//! @param scalar          - Scalar k (little-endian words).
//! @param scalarSizeWords - Size of @p scalar in words.
//! @param inX, inY        - Input point P coordinates.
//! @param outX, outY      - Output point R coordinates.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ecc_scalar_mult(const am_hal_cc312_ecc_domain_t *pDomain,
                                             const uint32_t *scalar,
                                             uint32_t scalarSizeWords,
                                             const uint32_t *inX,
                                             const uint32_t *inY,
                                             uint32_t *outX,
                                             uint32_t *outY);

//*****************************************************************************
//
//! @brief Simultaneous double scalar multiply R = a*P + b*Q (register-level).
//!
//! Used by ECDSA verify. The PKA must be initialized and the modulus N/NP,
//! curve coefficient EC_A, and modulus multiples N4/N8/N12 must already be
//! loaded into their registers. All arguments are PKA virtual register numbers.
//!
//! @return AM_HAL_STATUS_SUCCESS, or AM_HAL_STATUS_FAIL for a degenerate scalar.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ecc_sum2_scalar_mult(uint32_t xr, uint32_t yr,
                                                  uint32_t a, uint32_t xp, uint32_t yp,
                                                  uint32_t b, uint32_t xq, uint32_t yq);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_ECC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
