//*****************************************************************************
//
//! @file am_hal_cc312_sha512.h
//!
//! @brief Software SHA-384 and SHA-512 for the CryptoCell-312 HAL
//!
//! @addtogroup cc312_sha512_ap330P CC312 SHA-512 - Software Digest
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

#ifndef AM_HAL_CC312_SHA512_H
#define AM_HAL_CC312_SHA512_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Digest and block sizes.
//! @{
//
//*****************************************************************************
#define AM_HAL_SHA512_BLOCK_SIZE        128 //!< Data block size (bytes).
#define AM_HAL_SHA384_DIGEST_SIZE       48  //!< SHA-384 digest size (bytes).
#define AM_HAL_SHA512_DIGEST_SIZE       64  //!< SHA-512 digest size (bytes).
#define AM_HAL_SHA512_STATE_WORDS       8   //!< Chaining value size (64-bit words).
//! @}

//*****************************************************************************
//
//! @brief Digest selection.
//!
//! Kept separate from am_hal_cc312_sha_mode_e, which names the modes the hash
//! engine implements. Nothing here touches the engine.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CC312_SHA384            = 1,         //!< SHA-384.
    AM_HAL_CC312_SHA512            = 2,         //!< SHA-512.
    AM_HAL_CC312_SHA512_RESERVE32B = 0x7FFFFFFF //!< Reserved for 32-bit enum.
}
am_hal_cc312_sha512_mode_e;

#if defined(AM_HAL_CC312_SHA512_EXTERNAL)

//
// The integrator supplies the context type and the definitions below. This
// exists so a system that already links a software SHA-512 does not carry a
// second one; see am_hal_cc312_sha512.c for the contract.
//
#include "am_hal_cc312_sha512_external.h"

#else

//*****************************************************************************
//
//! @brief SHA-384 and SHA-512 context.
//
//*****************************************************************************
typedef struct
{
    uint64_t state[AM_HAL_SHA512_STATE_WORDS];  //!< Chaining value.
    uint64_t total[2];                          //!< Message length in bytes.
    uint8_t  buffer[AM_HAL_SHA512_BLOCK_SIZE];  //!< Partial-block buffer.
    uint32_t bufferLen;                         //!< Bytes buffered.
    am_hal_cc312_sha512_mode_e mode;            //!< Selected digest.
}
am_hal_cc312_sha512_context_t;

#endif // AM_HAL_CC312_SHA512_EXTERNAL

//*****************************************************************************
//
//! @brief Start a digest.
//!
//! @param ctx  - Context to initialise.
//! @param mode - AM_HAL_CC312_SHA384 or AM_HAL_CC312_SHA512.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512_init(am_hal_cc312_sha512_context_t *ctx,
                                         am_hal_cc312_sha512_mode_e mode);

//*****************************************************************************
//
//! @brief Add data to a digest.
//!
//! @param ctx   - Context from am_hal_cc312_sha512_init().
//! @param input - Data to add.
//! @param ilen  - Length of the data in bytes.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512_update(am_hal_cc312_sha512_context_t *ctx,
                                           const uint8_t *input,
                                           uint32_t ilen);

//*****************************************************************************
//
//! @brief Finish a digest and read it out.
//!
//! @param ctx    - Context from am_hal_cc312_sha512_init().
//! @param output - Buffer for the digest, as wide as the selected mode.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512_finish(am_hal_cc312_sha512_context_t *ctx,
                                           uint8_t *output);

//*****************************************************************************
//
//! @brief Release a context, scrubbing the state it holds.
//!
//! @param ctx - Context to release.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512_free(am_hal_cc312_sha512_context_t *ctx);

//*****************************************************************************
//
//! @brief Digest a single buffer.
//!
//! @param mode   - AM_HAL_CC312_SHA384 or AM_HAL_CC312_SHA512.
//! @param input  - Data to digest.
//! @param ilen   - Length of the data in bytes.
//! @param output - Buffer for the digest, as wide as the selected mode.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512(am_hal_cc312_sha512_mode_e mode,
                                    const uint8_t *input,
                                    uint32_t ilen,
                                    uint8_t *output);

//*****************************************************************************
//
//! @brief Digest width for a mode, or 0 if the mode is not one of the two.
//!
//! @param mode - Digest selection.
//!
//! @return Digest size in bytes.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha512_digest_size(am_hal_cc312_sha512_mode_e mode);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_SHA512_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
