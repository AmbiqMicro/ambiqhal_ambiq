//*****************************************************************************
//
//! @file am_hal_cc312_sha.h
//!
//! @brief Hardware abstraction for CryptoCell-312 SHA-1 / SHA-256 hashing
//!
//! @addtogroup cc312_sha_ap510L CC312 SHA - Hash Engine
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

#ifndef AM_HAL_CC312_SHA_H
#define AM_HAL_CC312_SHA_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// SHA hash constants.
//
//*****************************************************************************
#define AM_HAL_SHA_BLOCK_SIZE           64  //!< HASH data block size (bytes).
#define AM_HAL_SHA1_DIGEST_SIZE         20  //!< SHA-1 digest size (bytes).
#define AM_HAL_SHA224_DIGEST_SIZE       28  //!< SHA-224 digest size (bytes).
#define AM_HAL_SHA256_DIGEST_SIZE       32  //!< SHA-256 digest size (bytes).
#define AM_HAL_SHA_MAX_DIGEST_WORDS     8   //!< Maximum digest size (32-bit words).

//*****************************************************************************
//
// SHA operation mode.
//
//! Values match the CC312 HASH_CONTROL field encoding.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CC312_SHA1           = 1,            //!< SHA-1   (HW_HASH_CTL_SHA1_VAL).
    AM_HAL_CC312_SHA256         = 2,            //!< SHA-256 (HW_HASH_CTL_SHA256_VAL).
    AM_HAL_CC312_SHA224         = 3,            //!< SHA-224 (SHA-256 datapath, SHA-224 IV).
    AM_HAL_CC312_SHA_RESERVE32B = 0x7FFFFFFF    //!< Reserved for 32-bit enum.
}
am_hal_cc312_sha_mode_e;

//*****************************************************************************
//
// SHA context structure.
//
//! Declared with DCACHE-line alignment so the partial-block buffer never shares
//! a cache line with unrelated data across a DMA operation (see
//! am_hal_cc312_sha_process()).
//
//*****************************************************************************
//! SHA context data used by CC312 HAL.
typedef struct
{
    uint32_t digest[AM_HAL_SHA_MAX_DIGEST_WORDS];   //!< Running hash state (raw HASH_Hx words).
    uint32_t totalDataSizeProcessed[2];             //!< Current length tracked by HW (HASH_CUR_LEN).
    uint8_t  block[AM_HAL_SHA_BLOCK_SIZE];          //!< Partial-block buffer for streaming.
    uint32_t blockLen;                              //!< Bytes currently buffered in block[].
    am_hal_cc312_sha_mode_e mode;                   //!< Selected SHA mode.
} __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)))
am_hal_cc312_sha_context_t;

//*****************************************************************************
//
//! @brief Enable HASH and DMA clocks required for CC312 hash operations.
//
//*****************************************************************************
extern void am_hal_cc312_sha_enable_clocks(void);

//*****************************************************************************
//
//! @brief Disable HASH and DMA clocks used by CC312 hash operations.
//
//*****************************************************************************
extern void am_hal_cc312_sha_disable_clocks(void);

//*****************************************************************************
//
//! @brief Initialize a SHA context and select the hash mode.
//!
//! Loads the appropriate initial hash value and resets the running length.
//!
//! @param ctx  - Pointer to SHA context structure.
//! @param mode - AM_HAL_CC312_SHA1 or AM_HAL_CC312_SHA256.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_context_init(am_hal_cc312_sha_context_t *ctx,
                                              am_hal_cc312_sha_mode_e mode);

//*****************************************************************************
//
//! @brief Free (clear) a SHA context.
//!
//! @param ctx - Pointer to SHA context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_free(am_hal_cc312_sha_context_t *ctx);

//*****************************************************************************
//
//! @brief Initialize HASH hardware registers for a hash pass.
//!
//! @param ctx - Pointer to SHA context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_init(am_hal_cc312_sha_context_t *ctx);

//*****************************************************************************
//
//! @brief Load running hash value and length from context into HASH hardware.
//!
//! @param ctx - Pointer to SHA context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_load_state(am_hal_cc312_sha_context_t *ctx);

//*****************************************************************************
//
//! @brief Store running hash value and length from HASH hardware into context.
//!
//! @param ctx - Pointer to SHA context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_store_state(am_hal_cc312_sha_context_t *ctx);

//*****************************************************************************
//
//! @brief Feed a buffer of data into a SHA operation.
//!
//! May be called any number of times with arbitrary length buffers between
//! am_hal_cc312_sha_context_init() and am_hal_cc312_sha_finish().
//!
//! @param ctx   - Pointer to SHA context (previously initialized).
//! @param input - Input data buffer.
//! @param ilen  - Number of bytes in @p input.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_update(am_hal_cc312_sha_context_t *ctx,
                                  const uint8_t *input,
                                  uint32_t ilen);

//*****************************************************************************
//
//! @brief Complete a SHA operation and write out the digest.
//!
//! @param ctx    - Pointer to SHA context (previously initialized).
//! @param output - Digest output buffer (20 bytes for SHA-1, 32 for SHA-256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha_finish(am_hal_cc312_sha_context_t *ctx,
                                  uint8_t *output);

//*****************************************************************************
//
//! @brief One-shot SHA-1 of a single buffer.
//!
//! @param input  - Input data buffer.
//! @param ilen   - Number of bytes in @p input.
//! @param output - 20-byte digest output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha1(const uint8_t *input,
                            uint32_t ilen,
                            uint8_t output[AM_HAL_SHA1_DIGEST_SIZE]);

//*****************************************************************************
//
//! @brief One-shot SHA-256 of a single buffer.
//!
//! @param input  - Input data buffer.
//! @param ilen   - Number of bytes in @p input.
//! @param output - 32-byte digest output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_sha256(const uint8_t *input,
                              uint32_t ilen,
                              uint8_t output[AM_HAL_SHA256_DIGEST_SIZE]);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_SHA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
