//*****************************************************************************
//
//! @file am_hal_cc312_hmac.h
//!
//! @brief Hardware abstraction for CryptoCell-312 HMAC
//!
//! @addtogroup cc312_hmac_ap330P CC312 HMAC - Keyed Hashing
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

#ifndef AM_HAL_CC312_HMAC_H
#define AM_HAL_CC312_HMAC_H

#include "am_hal_cc312.h"
#include "am_hal_cc312_sha.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! HMAC context. The key schedule is held as the two padded blocks the
//! algorithm requires, so a context can be reused across messages.
//
//*****************************************************************************
typedef struct
{
    am_hal_cc312_sha_context_t shaCtx;              //!< Inner hash state.
    am_hal_cc312_sha_mode_e    mode;                //!< Digest selection.
    uint8_t ipad[AM_HAL_SHA_BLOCK_SIZE];            //!< Key XOR 0x36.
    uint8_t opad[AM_HAL_SHA_BLOCK_SIZE];            //!< Key XOR 0x5c.
}
am_hal_cc312_hmac_context_t;

//*****************************************************************************
//
//! @brief Start an HMAC operation.
//!
//! Derives the padded key blocks and begins the inner hash. A key longer
//! than the hash block size is replaced by its own digest, as FIPS 198-1
//! requires.
//!
//! @param ctx     - Pointer to HMAC context structure.
//! @param mode    - Digest to key: SHA-1, SHA-224 or SHA-256.
//! @param key     - Pointer to the key.
//! @param keyLen  - Length of the key in bytes.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hmac_start(am_hal_cc312_hmac_context_t *ctx,
                                        am_hal_cc312_sha_mode_e mode,
                                        const uint8_t *key,
                                        uint32_t keyLen);

//*****************************************************************************
//
//! @brief Add a message fragment to an HMAC operation.
//!
//! @param ctx    - Pointer to HMAC context structure.
//! @param input  - Pointer to the message fragment.
//! @param length - Length of the fragment in bytes.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hmac_update(am_hal_cc312_hmac_context_t *ctx,
                                         const uint8_t *input,
                                         uint32_t length);

//*****************************************************************************
//
//! @brief Complete an HMAC operation and write out the MAC.
//!
//! @param ctx    - Pointer to HMAC context structure.
//! @param output - Buffer for the MAC, as wide as the selected digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hmac_finish(am_hal_cc312_hmac_context_t *ctx,
                                         uint8_t *output);

//*****************************************************************************
//
//! @brief Free (clear) an HMAC context.
//!
//! @param ctx - Pointer to HMAC context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hmac_free(am_hal_cc312_hmac_context_t *ctx);

//*****************************************************************************
//
//! @brief HMAC of a single buffer in one call.
//!
//! @param mode    - Digest to key: SHA-1, SHA-224 or SHA-256.
//! @param key     - Pointer to the key.
//! @param keyLen  - Length of the key in bytes.
//! @param input   - Pointer to the message.
//! @param length  - Length of the message in bytes.
//! @param output  - Buffer for the MAC, as wide as the selected digest.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_hmac(am_hal_cc312_sha_mode_e mode,
                                  const uint8_t *key,
                                  uint32_t keyLen,
                                  const uint8_t *input,
                                  uint32_t length,
                                  uint8_t *output);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_HMAC_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
