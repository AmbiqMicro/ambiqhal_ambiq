//*****************************************************************************
//
//! @file am_hal_cc312_srp.h
//!
//! @brief Hardware abstraction for CryptoCell-312 SRP-6a
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

#ifndef AM_HAL_CC312_SRP_H
#define AM_HAL_CC312_SRP_H

#include "am_hal_cc312.h"
#include "am_hal_cc312_sha.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Largest group this HAL accepts.
//!
//! Covers the 1024, 1536, 2048 and 3072 bit groups of RFC 5054. The PKA has
//! room for the nine registers these routines use at every one of those
//! widths.
//! @{
//
//*****************************************************************************
#define AM_HAL_SRP_MAX_MODULUS_BITS     3072
#define AM_HAL_SRP_MAX_MODULUS_BYTES    (AM_HAL_SRP_MAX_MODULUS_BITS / 8)
//! @}

//*****************************************************************************
//
//! @brief SRP group parameters.
//!
//! N and g are the agreed group, taken from RFC 5054 in practice. Both are
//! big-endian and g is carried at the width of N so every operation runs at
//! one size.
//
//*****************************************************************************
typedef struct
{
    const uint8_t *pN;                          //!< Modulus N, big-endian.
    const uint8_t *pG;                          //!< Generator g, N wide.
    uint32_t modulusBytes;                      //!< Width of N and g.
    am_hal_cc312_sha_mode_e hashMode;           //!< Digest for the protocol.
}
am_hal_cc312_srp_group_t;

//*****************************************************************************
//
//! @brief Derive the private key x = H(salt || H(user ":" password)).
//!
//! @param group     - Group parameters, for the digest selection.
//! @param salt      - Salt.
//! @param saltLen   - Length of the salt in bytes.
//! @param user      - User name.
//! @param userLen   - Length of the user name in bytes.
//! @param password  - Password.
//! @param passLen   - Length of the password in bytes.
//! @param x         - Output buffer, as wide as the digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_derive_x(const am_hal_cc312_srp_group_t *group,
                                          const uint8_t *salt,
                                          uint32_t saltLen,
                                          const uint8_t *user,
                                          uint32_t userLen,
                                          const uint8_t *password,
                                          uint32_t passLen,
                                          uint8_t *x);

//*****************************************************************************
//
//! @brief Derive the multiplier k = H(N || PAD(g)).
//!
//! @param group - Group parameters.
//! @param k     - Output buffer, as wide as the digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_derive_k(const am_hal_cc312_srp_group_t *group,
                                          uint8_t *k);

//*****************************************************************************
//
//! @brief Derive the scrambling parameter u = H(PAD(A) || PAD(B)).
//!
//! @param group - Group parameters.
//! @param pA    - Client public value, N wide.
//! @param pB    - Server public value, N wide.
//! @param u     - Output buffer, as wide as the digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_derive_u(const am_hal_cc312_srp_group_t *group,
                                          const uint8_t *pA,
                                          const uint8_t *pB,
                                          uint8_t *u);

//*****************************************************************************
//
//! @brief Derive the password verifier v = g^x mod N.
//!
//! @param group - Group parameters.
//! @param x     - Private key from am_hal_cc312_srp_derive_x().
//! @param xLen  - Length of x in bytes.
//! @param v     - Output buffer, N wide.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_derive_verifier(const am_hal_cc312_srp_group_t *group,
                                                 const uint8_t *x,
                                                 uint32_t xLen,
                                                 uint8_t *v);

//*****************************************************************************
//
//! @brief Compute the client public value A = g^a mod N.
//!
//! @param group - Group parameters.
//! @param a     - Client private value.
//! @param aLen  - Length of a in bytes.
//! @param pA    - Output buffer, N wide.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_client_public(const am_hal_cc312_srp_group_t *group,
                                               const uint8_t *a,
                                               uint32_t aLen,
                                               uint8_t *pA);

//*****************************************************************************
//
//! @brief Compute the server public value B = (k * v + g^b) mod N.
//!
//! @param group - Group parameters.
//! @param k     - Multiplier from am_hal_cc312_srp_derive_k().
//! @param kLen  - Length of k in bytes.
//! @param v     - Verifier, N wide.
//! @param b     - Server private value.
//! @param bLen  - Length of b in bytes.
//! @param pB    - Output buffer, N wide.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_server_public(const am_hal_cc312_srp_group_t *group,
                                               const uint8_t *k,
                                               uint32_t kLen,
                                               const uint8_t *v,
                                               const uint8_t *b,
                                               uint32_t bLen,
                                               uint8_t *pB);

//*****************************************************************************
//
//! @brief Compute the client shared secret S = (B - k * g^x)^(a + u * x) mod N.
//!
//! @param group - Group parameters.
//! @param pB    - Server public value, N wide.
//! @param k     - Multiplier, kLen bytes.
//! @param kLen  - Length of k in bytes.
//! @param x     - Private key, xLen bytes.
//! @param xLen  - Length of x in bytes.
//! @param a     - Client private value, aLen bytes.
//! @param aLen  - Length of a in bytes.
//! @param u     - Scrambling parameter, uLen bytes.
//! @param uLen  - Length of u in bytes.
//! @param pS    - Output buffer, N wide.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_client_secret(const am_hal_cc312_srp_group_t *group,
                                               const uint8_t *pB,
                                               const uint8_t *k,
                                               uint32_t kLen,
                                               const uint8_t *x,
                                               uint32_t xLen,
                                               const uint8_t *a,
                                               uint32_t aLen,
                                               const uint8_t *u,
                                               uint32_t uLen,
                                               uint8_t *pS);

//*****************************************************************************
//
//! @brief Compute the server shared secret S = (A * v^u)^b mod N.
//!
//! @param group - Group parameters.
//! @param pA    - Client public value, N wide.
//! @param v     - Verifier, N wide.
//! @param u     - Scrambling parameter, uLen bytes.
//! @param uLen  - Length of u in bytes.
//! @param b     - Server private value, bLen bytes.
//! @param bLen  - Length of b in bytes.
//! @param pS    - Output buffer, N wide.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_server_secret(const am_hal_cc312_srp_group_t *group,
                                               const uint8_t *pA,
                                               const uint8_t *v,
                                               const uint8_t *u,
                                               uint32_t uLen,
                                               const uint8_t *b,
                                               uint32_t bLen,
                                               uint8_t *pS);

//*****************************************************************************
//
//! @brief Derive the session key from the shared secret.
//!
//! Uses the interleaved hash of RFC 2945: the secret is stripped of leading
//! zeros, split into even and odd bytes, each half hashed, and the two
//! digests interleaved. The result is twice the digest width.
//!
//! @param group - Group parameters.
//! @param pS    - Shared secret, N wide.
//! @param pK    - Output buffer, twice the digest width.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_srp_session_key(const am_hal_cc312_srp_group_t *group,
                                             const uint8_t *pS,
                                             uint8_t *pK);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_SRP_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
