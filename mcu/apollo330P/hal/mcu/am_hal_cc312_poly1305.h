//*****************************************************************************
//
//! @file am_hal_cc312_poly1305.h
//!
//! @brief Hardware abstraction for CryptoCell-312 Poly1305
//!
//! @addtogroup cc312_poly1305_ap330P CC312 Poly1305 - One Time Authenticator
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

#ifndef AM_HAL_CC312_POLY1305_H
#define AM_HAL_CC312_POLY1305_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Poly1305 sizes.
//! @{
//
//*****************************************************************************
#define AM_HAL_POLY1305_KEY_SIZE        32  //!< One time key r || s (bytes).
#define AM_HAL_POLY1305_MAC_SIZE        16  //!< Authenticator size (bytes).
#define AM_HAL_POLY1305_BLOCK_SIZE      16  //!< Message block size (bytes).
//! @}

//*****************************************************************************
//
//! @brief Compute a Poly1305 authenticator over a message.
//!
//! Poly1305 is not a hardware engine. It is arithmetic modulo 2^130 - 5,
//! which the PKA performs, so the accumulation runs on the public key
//! accelerator rather than the symmetric datapath.
//!
//! The key is the 32 byte one time key: the first 16 bytes are r, which is
//! clamped internally as the standard requires, and the second 16 bytes are
//! s. A key must never be reused across messages.
//!
//! @param key    - 32 byte one time key.
//! @param input  - Message to authenticate.
//! @param length - Length of the message in bytes.
//! @param mac    - 16 byte output buffer for the authenticator.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_poly1305(const uint8_t *key,
                                      const uint8_t *input,
                                      uint32_t length,
                                      uint8_t *mac);

//*****************************************************************************
//
//! @brief Compute a Poly1305 authenticator in the ChaCha20-Poly1305 layout.
//!
//! Authenticates the additional data and the ciphertext the way RFC 8439
//! specifies: each is padded to a multiple of the block size, and a final
//! block carries the two lengths as 64 bit little endian values.
//!
//! @param key     - 32 byte one time key, normally the first ChaCha20 block.
//! @param aad     - Additional authenticated data, may be NULL.
//! @param aadLen  - Length of the additional data in bytes.
//! @param input   - Ciphertext to authenticate, may be NULL.
//! @param length  - Length of the ciphertext in bytes.
//! @param mac     - 16 byte output buffer for the authenticator.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_poly1305_aead(const uint8_t *key,
                                           const uint8_t *aad,
                                           uint32_t aadLen,
                                           const uint8_t *input,
                                           uint32_t length,
                                           uint8_t *mac);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_POLY1305_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
