//*****************************************************************************
//
//! @file am_hal_cc312_rsa.h
//!
//! @brief Hardware abstraction for CryptoCell-312 RSA primitives
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

#ifndef AM_HAL_CC312_RSA_H
#define AM_HAL_CC312_RSA_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name RSA modulus limits.
//! @{
//
//*****************************************************************************
#define AM_HAL_RSA_MAX_MODULUS_BITS     2048    //!< Largest modulus supported.
#define AM_HAL_RSA_MAX_MODULUS_BYTES    (AM_HAL_RSA_MAX_MODULUS_BITS / 8)
#define AM_HAL_RSA_MAX_MODULUS_WORDS    (AM_HAL_RSA_MAX_MODULUS_BYTES / 4)
//! @}

//*****************************************************************************
//
//! @brief RSA primitive: result = base ^ exponent mod modulus.
//!
//! This is the raw modular exponentiation the PKA provides, with no padding
//! applied. It covers both RSA operations: pass the public exponent to
//! verify or encrypt, and the private exponent to sign or decrypt. Padding
//! and the PKCS#1 or OAEP or PSS encodings belong above this layer.
//!
//! All three big numbers are big-endian byte strings of exactly
//! @p modulusBytes bytes, which is how RSA keys and signatures are carried
//! on the wire. The base must be numerically smaller than the modulus.
//!
//! @param modulus      - Modulus n, big-endian.
//! @param modulusBytes - Length of the modulus in bytes.
//! @param exponent     - Exponent, big-endian, same length as the modulus.
//! @param base         - Base, big-endian, same length as the modulus.
//! @param result       - Output buffer, same length as the modulus.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_rsa_mod_exp(const uint8_t *modulus,
                                         uint32_t modulusBytes,
                                         const uint8_t *exponent,
                                         const uint8_t *base,
                                         uint8_t *result);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_RSA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
