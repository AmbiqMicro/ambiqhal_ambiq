//*****************************************************************************
//
//! @file am_hal_cc312_ed25519.h
//!
//! @brief Hardware abstraction for CryptoCell-312 Ed25519
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

#ifndef AM_HAL_CC312_ED25519_H
#define AM_HAL_CC312_ED25519_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Ed25519 sizes, RFC 8032 section 5.1.
//! @{
//
//*****************************************************************************
#define AM_HAL_ED25519_SEED_SIZE        32  //!< Private key seed.
#define AM_HAL_ED25519_PUBLIC_KEY_SIZE  32  //!< Encoded public key point.
#define AM_HAL_ED25519_SIGNATURE_SIZE   64  //!< R followed by S.
//! @}

//*****************************************************************************
//
//! @brief Derive the public key for a private seed.
//!
//! @param seed      - 32-byte private seed.
//! @param publicKey - 32-byte buffer for the encoded public point.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ed25519_public_key(const uint8_t *seed,
                                                uint8_t *publicKey);

//*****************************************************************************
//
//! @brief Sign a message.
//!
//! PureEdDSA over the whole message, so the caller passes the message itself
//! rather than a digest of it. Signing is deterministic: the same seed and
//! message always produce the same signature, and no entropy source is
//! consulted.
//!
//! @param seed      - 32-byte private seed.
//! @param message   - Message to sign.
//! @param messageLen - Length of the message in bytes.
//! @param signature - 64-byte buffer for the signature.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ed25519_sign(const uint8_t *seed,
                                          const uint8_t *message,
                                          uint32_t messageLen,
                                          uint8_t *signature);

//*****************************************************************************
//
//! @brief Verify a signature.
//!
//! @param publicKey  - 32-byte encoded public point.
//! @param message    - Message the signature covers.
//! @param messageLen - Length of the message in bytes.
//! @param signature  - 64-byte signature.
//!
//! @return AM_HAL_STATUS_SUCCESS if the signature is good,
//!         AM_HAL_STATUS_FAIL if it is not, or an error code if the inputs
//!         could not be processed.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_ed25519_verify(const uint8_t *publicKey,
                                            const uint8_t *message,
                                            uint32_t messageLen,
                                            const uint8_t *signature);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_ED25519_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
