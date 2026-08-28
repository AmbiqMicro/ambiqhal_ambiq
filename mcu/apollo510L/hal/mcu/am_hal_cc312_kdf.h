//*****************************************************************************
//
//! @file am_hal_cc312_kdf.h
//!
//! @brief Hardware abstraction for CryptoCell-312 HKDF key derivation
//!
//! @addtogroup cc312_kdf_ap510L CC312 KDF - Key Derivation
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

#ifndef AM_HAL_CC312_KDF_H
#define AM_HAL_CC312_KDF_H

#include "am_hal_cc312.h"
#include "am_hal_cc312_sha.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Largest output one HKDF-Expand call can produce. RFC 5869 caps the output
//! at 255 hash lengths; this HAL caps it lower to keep the counter in a byte
//! and the working buffers small.
//
//*****************************************************************************
#define AM_HAL_HKDF_MAX_OUTPUT_BYTES    255

//*****************************************************************************
//
//! @brief HKDF-Extract: turn input keying material into a pseudorandom key.
//!
//! PRK = HMAC(salt, ikm). A NULL salt is treated as a string of zeros the
//! width of the digest, which is what RFC 5869 specifies for the salt-less
//! case.
//!
//! @param mode    - Digest to use: SHA-1, SHA-224 or SHA-256.
//! @param salt    - Optional salt, may be NULL.
//! @param saltLen - Length of the salt in bytes, 0 when salt is NULL.
//! @param ikm     - Input keying material.
//! @param ikmLen  - Length of the input keying material in bytes.
//! @param prk     - Output buffer, as wide as the digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hkdf_extract(am_hal_cc312_sha_mode_e mode,
                                          const uint8_t *salt,
                                          uint32_t saltLen,
                                          const uint8_t *ikm,
                                          uint32_t ikmLen,
                                          uint8_t *prk);

//*****************************************************************************
//
//! @brief HKDF-Expand: stretch a pseudorandom key into output keying material.
//!
//! Produces @p okmLen bytes as T(1) || T(2) || ... truncated to length, where
//! T(n) = HMAC(prk, T(n-1) || info || n).
//!
//! @param mode    - Digest to use, matching the one used to extract.
//! @param prk     - Pseudorandom key, normally the output of extract.
//! @param prkLen  - Length of the pseudorandom key in bytes.
//! @param info    - Optional context and application specific information.
//! @param infoLen - Length of the info in bytes, 0 when info is NULL.
//! @param okm     - Output buffer.
//! @param okmLen  - Number of bytes to produce.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hkdf_expand(am_hal_cc312_sha_mode_e mode,
                                         const uint8_t *prk,
                                         uint32_t prkLen,
                                         const uint8_t *info,
                                         uint32_t infoLen,
                                         uint8_t *okm,
                                         uint32_t okmLen);

//*****************************************************************************
//
//! @brief HKDF extract and expand in one call.
//!
//! @param mode    - Digest to use: SHA-1, SHA-224 or SHA-256.
//! @param salt    - Optional salt, may be NULL.
//! @param saltLen - Length of the salt in bytes, 0 when salt is NULL.
//! @param ikm     - Input keying material.
//! @param ikmLen  - Length of the input keying material in bytes.
//! @param info    - Optional context information, may be NULL.
//! @param infoLen - Length of the info in bytes, 0 when info is NULL.
//! @param okm     - Output buffer.
//! @param okmLen  - Number of bytes to produce.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hkdf(am_hal_cc312_sha_mode_e mode,
                                  const uint8_t *salt,
                                  uint32_t saltLen,
                                  const uint8_t *ikm,
                                  uint32_t ikmLen,
                                  const uint8_t *info,
                                  uint32_t infoLen,
                                  uint8_t *okm,
                                  uint32_t okmLen);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_KDF_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
