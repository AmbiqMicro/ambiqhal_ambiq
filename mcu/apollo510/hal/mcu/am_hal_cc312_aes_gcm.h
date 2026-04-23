//*****************************************************************************
//
//! @file am_hal_cc312_aes_gcm.h
//!
//! @brief Hardware abstraction for CryptoCell-312 AES-GCM mode
//!
//! @addtogroup cc312_aes_gcm_ap510 CC312 AES-GCM - Authenticated Encryption
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

#ifndef AM_HAL_CC312_AES_GCM_H
#define AM_HAL_CC312_AES_GCM_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// AES-GCM constants.
//
//*****************************************************************************
#define AM_HAL_AES_GCM_BLOCK_SIZE           16  //!< GCM block size (bytes).
#define AM_HAL_AES_GCM_TAG_MIN_SIZE         4   //!< Minimum tag size (bytes).
#define AM_HAL_AES_GCM_TAG_MAX_SIZE         16  //!< Maximum tag size (bytes).
#define AM_HAL_AES_GCM_IV_96_BIT_SIZE       12  //!< Standard IV size (96 bits).
#define AM_HAL_AES_GCM_H_SIZE               16  //!< H subkey size (bytes).
#define AM_HAL_AES_GCM_J0_SIZE              16  //!< J0 counter size (bytes).

//
// Data size limits.
//
#define AM_HAL_AES_GCM_DATA_MAX_SIZE        0xFFFF  //!< Max data size (64KB-1).
#define AM_HAL_AES_GCM_IV_MAX_SIZE          0xFFFF  //!< Max IV size (64KB-1).
#define AM_HAL_AES_GCM_AAD_MAX_SIZE         0xFFFF  //!< Max AAD size (64KB-1).

//
// GCM hardware helper constants.
//
#define AM_HAL_CC312_AES_GCM_GHASH_INIT_SET_VAL  0x1UL  //!< GHASH init value.
#define AM_HAL_CC312_AES_GCM_HASH_SEL_HASH_MOD   0x1UL  //!< HASH select value.
#define AM_HAL_CC312_AES_GCM_GHASH_SEL_GHASH_MOD 0x1UL  //!< GHASH select value.
#define AM_HAL_CC312_AES_GCM_HASH_XOR_DATA_VAL   0x0UL  //!< HASH XOR DIN value.
#define AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(val)                                                \
    ((((val) & 0x000000FFU) << 24) | (((val) & 0x0000FF00U) << 8) |                         \
     (((val) & 0x00FF0000U) >> 8) | (((val) & 0xFF000000U) >> 24))
#define AM_HAL_CC312_AES_GCM_BITMASK(n) (((n) < 32U) ? ((1UL << (n)) - 1UL) : 0xFFFFFFFFUL)

//*****************************************************************************
//
// AES-GCM operation direction.
//
//*****************************************************************************
#define AM_HAL_AES_GCM_ENCRYPT              1  //!< GCM encryption mode.
#define AM_HAL_AES_GCM_DECRYPT              0  //!< GCM decryption mode.

//*****************************************************************************
//
// AES-GCM process modes (internal state machine).
//
//*****************************************************************************
typedef enum
{
    AM_HAL_AES_GCM_CALC_H            = 0,  //!< Calculate H (GHASH subkey).
    AM_HAL_AES_GCM_CALC_J0_PHASE1    = 1,  //!< Calculate J0 - first phase.
    AM_HAL_AES_GCM_CALC_J0_PHASE2    = 2,  //!< Calculate J0 - second phase.
    AM_HAL_AES_GCM_PROCESS_AAD       = 3,  //!< GHASH AAD processing.
    AM_HAL_AES_GCM_PROCESS_DATA      = 4,  //!< GCTR encrypt/decrypt + GHASH.
    AM_HAL_AES_GCM_PROCESS_LEN       = 5,  //!< GHASH Len(A) || Len(C).
    AM_HAL_AES_GCM_GCTR_FINAL        = 6,  //!< GCTR final (tag generation).
    AM_HAL_AES_GCM_PROCESS_RESERVE   = 0x7FFFFFFF
}
am_hal_cc312_aes_gcm_process_mode_e;

//*****************************************************************************
//
// AES-GCM context structure.
//
//*****************************************************************************
//! AES-GCM context data used by CC312 HAL.
typedef struct
{
    uint32_t keyBuf[8];         //!< AES key buffer (max 256-bit = 8 words).
    uint32_t H[4];              //!< GHASH subkey H (128-bit).
    uint32_t J0[4];             //!< Initial counter J0 (128-bit).
    uint32_t tempBuf[4];        //!< Temporary buffer (128-bit).
    uint32_t aesCntrBuf[4];     //!< AES counter buffer (128-bit).
    uint32_t ghashResBuf[4];    //!< GHASH result buffer (128-bit).
    uint8_t  preTagBuf[16];     //!< Pre-tag buffer (before final GCTR).
    bool     j0Inc32Done;       //!< J0 Inc32 completion flag.
    uint8_t  keySizeId;         //!< Key size: 0=128, 1=192, 2=256.
    uint8_t  dir;               //!< Direction: 0=decrypt, 1=encrypt.
    uint8_t  tagSize;           //!< Tag size in bytes (4-16).
    uint8_t  reserved[1];       //!< Alignment padding.
    am_hal_cc312_aes_gcm_process_mode_e processMode;  //!< Current process mode.
    uint32_t dataSize;          //!< Plaintext/ciphertext size.
    uint32_t ivSize;            //!< IV size.
    uint32_t aadSize;           //!< AAD size.
} __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)))
am_hal_cc312_aes_gcm_context_t;

//*****************************************************************************
//
//! @brief Load GHASH subkey H into GHASH hardware key registers.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_load_ghash_subkey(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Load GHASH IV/state from context into hardware registers.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_load_ghash_iv(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Store GHASH IV/state from hardware back into context buffers.
//!
//! @param ctx       - Pointer to AES-GCM context structure.
//! @param storeInJ0 - When true, also copy GHASH state into J0.
//
//*****************************************************************************
extern void am_hal_aes_gcm_store_ghash_iv(am_hal_cc312_aes_gcm_context_t *ctx, bool storeInJ0);

//*****************************************************************************
//
//! @brief Load AES key from context into hardware key registers.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_load_key(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Load AES counter state from context into hardware counter registers.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_load_counter(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Store AES counter state from hardware into context.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_store_counter(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Increment the low 32 bits of J0 (Inc32) into the active counter state.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_inc32_j0(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Configure AES/GHASH hardware for the current GCM process phase.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_gcm_init_hw(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Execute one GCM processing phase via CC312 DMA path.
//!
//! @param ctx    - Pointer to AES-GCM context structure.
//! @param input  - Pointer to input buffer.
//! @param output - Pointer to output buffer (or NULL for hash-only phases).
//! @param length - Number of bytes to process.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_gcm_process(am_hal_cc312_aes_gcm_context_t *ctx,
                                       const uint8_t *input,
                                       uint8_t *output,
                                       uint32_t length);

//*****************************************************************************
//
//! @brief Initialize GCM context with key material and key size.
//!
//! @param ctx      - Pointer to AES-GCM context structure.
//! @param key      - Pointer to key bytes.
//! @param key_bits - Key size in bits (128, 192, or 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_gcm_context_init(am_hal_cc312_aes_gcm_context_t *ctx,
                                                  const uint8_t *key,
                                                  uint32_t key_bits);


//*****************************************************************************
//
//! @brief Initialize AES-GCM context.
//!
//! This function initializes an AES-GCM context structure to a known state.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_init(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Free (clear) AES-GCM context.
//!
//! This function securely clears an AES-GCM context structure.
//!
//! @param ctx - Pointer to AES-GCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_gcm_free(am_hal_cc312_aes_gcm_context_t *ctx);

//*****************************************************************************
//
//! @brief Set AES-GCM encryption/decryption key.
//!
//! This function sets the key for AES-GCM operations. The same function
//! is used for both encryption and decryption (GCM uses AES in encryption
//! mode for both directions).
//!
//! @param ctx     - Pointer to AES-GCM context structure.
//! @param key     - Pointer to key data.
//! @param keybits - Key size in bits (128, 192, or 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_gcm_setkey(am_hal_cc312_aes_gcm_context_t *ctx,
                                      const uint8_t *key,
                                      uint32_t keybits);

//*****************************************************************************
//
//! @brief AES-GCM authenticated encryption.
//!
//! This function performs AES-GCM authenticated encryption in a single call.
//! It encrypts the input data and generates an authentication tag.
//!
//! @param ctx      - Pointer to AES-GCM context (must be initialized with key).
//! @param length   - Length of input/output data in bytes.
//! @param iv       - Pointer to IV (nonce). Typically 12 bytes (96 bits).
//! @param iv_len   - IV length in bytes (must be > 0).
//! @param aad      - Pointer to additional authenticated data (AAD). Can be NULL if aad_len is 0.
//! @param aad_len  - AAD length in bytes.
//! @param input    - Pointer to plaintext input data.
//! @param output   - Pointer to ciphertext output buffer.
//! @param tag_len  - Desired tag length in bytes (4-16, typically 16).
//! @param tag      - Pointer to output buffer for authentication tag.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_gcm_crypt_and_tag(am_hal_cc312_aes_gcm_context_t *ctx,
                                             int mode,
                                             uint32_t length,
                                             const uint8_t *iv,
                                             uint32_t iv_len,
                                             const uint8_t *aad,
                                             uint32_t aad_len,
                                             const uint8_t *input,
                                             uint8_t *output,
                                             uint32_t tag_len,
                                             uint8_t *tag);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_AES_GCM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
