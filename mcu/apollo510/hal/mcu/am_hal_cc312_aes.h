//*****************************************************************************
//
//! @file am_hal_cc312_aes.h
//!
//! @brief Hardware abstraction for CryptoCell-312 AES cipher modes
//!
//! @addtogroup cc312_aes_ap510 CC312 AES - Cipher Modes
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

#ifndef AM_HAL_CC312_AES_H
#define AM_HAL_CC312_AES_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// AES cipher constants.
//
//*****************************************************************************
#define AES_BLOCK_SIZE                  16  //!< AES block size (bytes).
#define AES_IV_SIZE                     16  //!< AES IV size (bytes).
#define AES_IV_SIZE_WORDS               (AES_IV_SIZE >> 2)        //!< IV size in 32-bit words.
#define AES_256_BIT_KEY_SIZE            32  //!< AES-256 key size (bytes).
#define AES_256_BIT_KEY_SIZE_WORDS      (AES_256_BIT_KEY_SIZE >> 2)  //!< Key size in 32-bit words.

//*****************************************************************************
//
// AES operation direction.
//
//*****************************************************************************
#define AM_HAL_AES_DECRYPT              0   //!< AES decryption mode.
#define AM_HAL_AES_ENCRYPT              1   //!< AES encryption mode.

//*****************************************************************************
//
// AES cipher modes (hardware-supported).
//
//*****************************************************************************
typedef enum
{
    CIPHER_NULL_MODE  = -1,             //!< No cipher mode selected.
    CIPHER_ECB        = 0,              //!< Electronic Codebook mode.
    CIPHER_CTR        = 2,              //!< Counter mode.
    CIPHER_OFB        = 6,              //!< Output Feedback mode.
    CIPHER_RESERVE32B = 0x7FFFFFFF      //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_mode_e;

//*****************************************************************************
//
// AES block position indicator.
//
//*****************************************************************************
typedef enum
{
    FIRST_BLOCK          = 0,           //!< First block in multi-block operation.
    MIDDLE_BLOCK         = 1,           //!< Middle block in multi-block operation.
    LAST_BLOCK           = 2,           //!< Last block in multi-block operation.
    RESERVE32B_BLOCK     = 0x7FFFFFFF   //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_block_e;

//*****************************************************************************
//
// AES buffer address type.
//
//*****************************************************************************
typedef enum
{
    SRAM_ADDR        = 0,               //!< SRAM address.
    DLLI_ADDR        = 1,               //!< Direct LLI address.
    ADDR_RESERVE32B  = 0x7FFFFFFF       //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_addr_e;

//*****************************************************************************
//
// AES operation direction (internal).
//
//*****************************************************************************
typedef enum
{
    CRYPTO_DIRECTION_ENCRYPT          = 0,  //!< Encryption direction.
    CRYPTO_DIRECTION_DECRYPT          = 1,  //!< Decryption direction.
    CRYPTO_DIRECTION_NUM_OF_ENC_MODES = 2,  //!< Number of directions.
    CRYPTO_DIRECTION_RESERVE32B       = 0x7FFFFFFF  //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_direction_e;

//*****************************************************************************
//
// AES key sizes.
//
//*****************************************************************************
typedef enum
{
    KEY_SIZE_128_BIT       = 0,         //!< 128-bit key (16 bytes).
    KEY_SIZE_192_BIT       = 1,         //!< 192-bit key (24 bytes).
    KEY_SIZE_256_BIT       = 2,         //!< 256-bit key (32 bytes).
    KEY_SIZE_ID_RESERVE32B = 0x7FFFFFFF //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_key_size_e;

//*****************************************************************************
//
// AES key source selection.
//
//*****************************************************************************
typedef enum
{
    RKEK_KEY     = 0,                   //!< Root KEK (Key Encryption Key).
    USER_KEY     = 1,                   //!< User-provided key.
    KCP_KEY      = 2,                   //!< KCP key.
    KCE_KEY      = 3,                   //!< KCE key.
    KPICV_KEY    = 4,                   //!< KPICV key.
    KCEICV_KEY   = 5,                   //!< KCEICV key.
    RTL_KEY      = 6,                   //!< RTL key.
    END_OF_KEYS  = 0x7FFFFFFF           //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_key_type_e;

//*****************************************************************************
//
// AES padding type.
//
//*****************************************************************************
typedef enum
{
    CRYPTO_PADDING_NONE      = 0,       //!< No padding.
    CRYPTO_PADDING_PKCS7     = 1,       //!< PKCS#7 padding.
    CRYPTO_PADDING_RESERVE32B = 0x7FFFFFFF  //!< Reserved for 32-bit enum.
}
am_hal_cc312_aes_padding_e;

//*****************************************************************************
//
// AES context structure.
//
//*****************************************************************************
//! AES context data used by CC312 HAL.
typedef struct
{
    uint32_t ivBuf[AES_IV_SIZE_WORDS];           //!< IV buffer (CTR uses counter).
    uint32_t keyBuf[AES_256_BIT_KEY_SIZE_WORDS]; //!< AES key buffer (max 256-bit).
    am_hal_cc312_aes_key_size_e keySizeId;       //!< Key size identifier.
    am_hal_cc312_aes_mode_e mode;                //!< AES mode selection.
    am_hal_cc312_aes_direction_e dir;            //!< Encrypt/decrypt direction.
    am_hal_cc312_aes_key_type_e cryptoKey;       //!< Key source selection.
    am_hal_cc312_aes_padding_e padType;          //!< Padding type.
    am_hal_cc312_aes_block_e dataBlockType;      //!< Block position selector.
    am_hal_cc312_aes_addr_e inputDataAddrType;   //!< Input buffer address type.
    am_hal_cc312_aes_addr_e outputDataAddrType;  //!< Output buffer address type.
    uint8_t tempBuff[AES_BLOCK_SIZE];            //!< Temporary buffer.
} __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)))
am_hal_cc312_aes_context_t;

//*****************************************************************************
//
// AES-XTS context structure (software implementation).
//
//*****************************************************************************
//! AES-XTS context (software implementation using ECB hardware acceleration).
typedef struct
{
    am_hal_cc312_aes_context_t crypt;            //!< Context for data encryption/decryption.
    am_hal_cc312_aes_context_t tweak;            //!< Context for tweak encryption (always encrypt).
}
am_hal_cc312_aes_xts_context_t;

//*****************************************************************************
//
//! @brief Enable AES and DMA clocks required for CC312 AES operations.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_cc312_aes_enable_clocks(void);

//*****************************************************************************
//
//! @brief Disable AES and DMA clocks used by CC312 AES operations.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_cc312_aes_disable_clocks(void);

//*****************************************************************************
//
//! @brief Initialize AES context.
//!
//! This function initializes an AES context structure to a known state.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_context_init(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief Free (clear) AES context.
//!
//! This function securely clears an AES context structure.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_free(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief Set AES encryption key.
//!
//! This function sets the key for AES encryption operations.
//!
//! @param ctx      - Pointer to AES context structure.
//! @param key      - Pointer to key data.
//! @param key_bits - Key size in bits (128, 192, or 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_setkey_enc(am_hal_cc312_aes_context_t *ctx,
                                            const uint8_t *key,
                                            uint32_t key_bits);

//*****************************************************************************
//
//! @brief Set AES decryption key.
//!
//! This function sets the key for AES decryption operations.
//!
//! @param ctx      - Pointer to AES context structure.
//! @param key      - Pointer to key data.
//! @param key_bits - Key size in bits (128, 192, or 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_setkey_dec(am_hal_cc312_aes_context_t *ctx,
                                            const uint8_t *key,
                                            uint32_t key_bits);

//*****************************************************************************
//
//! @brief Initialize AES hardware registers for current context.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_init(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief Load context key material into AES hardware key registers.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_load_key(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief Load IV/counter state from context into AES hardware registers.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_load_iv(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief Store IV/counter state from AES hardware registers into context.
//!
//! @param ctx - Pointer to AES context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_store_iv(am_hal_cc312_aes_context_t *ctx);

//*****************************************************************************
//
//! @brief AES-ECB block encryption/decryption.
//!
//! This function performs AES-ECB (Electronic Codebook) mode encryption
//! or decryption on a single 16-byte block.
//!
//! @param ctx    - Pointer to AES context (must be initialized with key).
//! @param mode   - Operation mode (AM_HAL_AES_ENCRYPT or AM_HAL_AES_DECRYPT).
//! @param input  - Pointer to 16-byte input block.
//! @param output - Pointer to 16-byte output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_crypt_ecb(am_hal_cc312_aes_context_t *ctx,
                                    int32_t mode,
                                    const uint8_t input[AES_BLOCK_SIZE],
                                    uint8_t output[AES_BLOCK_SIZE]);

//*****************************************************************************
//
//! @brief Set data buffer information for DMA transfers.
//!
//! @param input        - Pointer to input buffer.
//! @param input_info   - Pointer to input buffer info structure to populate.
//! @param output       - Pointer to output buffer.
//! @param output_info  - Pointer to output buffer info structure to populate.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_aes_set_data_buffers_info(const uint8_t *input,
                                                       am_hal_cc312_buffer_t *input_info,
                                                       const uint8_t *output,
                                                       am_hal_cc312_buffer_t *output_info);

//*****************************************************************************
//
//! @brief AES-CTR buffer encryption/decryption.
//!
//! This function performs AES-CTR (Counter) mode encryption or decryption.
//! CTR mode converts a block cipher into a stream cipher by encrypting
//! successive counter values.
//!
//! @param ctx           - Pointer to AES context (must be initialized with key).
//! @param length        - Length of input/output data in bytes.
//! @param nc_off        - Pointer to nonce/counter offset (must be 0).
//! @param nonce_counter - Pointer to 16-byte nonce/counter (IV).
//! @param input         - Pointer to input data.
//! @param output        - Pointer to output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_crypt_ctr(am_hal_cc312_aes_context_t *ctx,
                                    uint32_t length,
                                    uint32_t *nc_off,
                                    uint8_t nonce_counter[AES_BLOCK_SIZE],
                                    const uint8_t *input,
                                    uint8_t *output);

//*****************************************************************************
//
//! @brief AES-OFB buffer encryption/decryption.
//!
//! This function performs AES-OFB (Output Feedback) mode encryption or
//! decryption. OFB mode converts a block cipher into a stream cipher by
//! repeatedly encrypting an IV.
//!
//! @param ctx     - Pointer to AES context (must be initialized with key).
//! @param length  - Length of input/output data in bytes.
//! @param iv_off  - Pointer to IV offset (must be 0).
//! @param iv      - Pointer to 16-byte IV (initialization vector).
//! @param input   - Pointer to input data.
//! @param output  - Pointer to output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_crypt_ofb(am_hal_cc312_aes_context_t *ctx,
                                    uint32_t length,
                                    uint32_t *iv_off,
                                    uint8_t iv[AES_BLOCK_SIZE],
                                    const uint8_t *input,
                                    uint8_t *output);

//*****************************************************************************
//
//! @brief Initialize AES-XTS context.
//!
//! This function initializes an AES-XTS context structure. XTS mode is
//! implemented in software using ECB hardware acceleration.
//!
//! @param ctx - Pointer to AES-XTS context structure.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_aes_xts_init(am_hal_cc312_aes_xts_context_t *ctx);

//*****************************************************************************
//
//! @brief Free (clear) AES-XTS context.
//!
//! This function securely clears an AES-XTS context structure.
//!
//! @param ctx - Pointer to AES-XTS context structure.
//!
//! @return None.
//
//*****************************************************************************
extern void am_hal_aes_xts_free(am_hal_cc312_aes_xts_context_t *ctx);

//*****************************************************************************
//
//! @brief Set AES-XTS encryption key.
//!
//! This function sets the key for AES-XTS encryption. XTS requires a
//! double-length key (256 or 512 bits) which is split into two keys.
//!
//! @param ctx     - Pointer to AES-XTS context structure.
//! @param key     - Pointer to double-length key data.
//! @param keybits - Key size in bits (256 or 512).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_xts_setkey_enc(am_hal_cc312_aes_xts_context_t *ctx,
                                          const uint8_t *key,
                                          uint32_t keybits);

//*****************************************************************************
//
//! @brief Set AES-XTS decryption key.
//!
//! This function sets the key for AES-XTS decryption. XTS requires a
//! double-length key (256 or 512 bits) which is split into two keys.
//!
//! @param ctx     - Pointer to AES-XTS context structure.
//! @param key     - Pointer to double-length key data.
//! @param keybits - Key size in bits (256 or 512).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_xts_setkey_dec(am_hal_cc312_aes_xts_context_t *ctx,
                                          const uint8_t *key,
                                          uint32_t keybits);

//*****************************************************************************
//
//! @brief AES-XTS buffer encryption/decryption.
//!
//! This function performs AES-XTS encryption or decryption. XTS mode is
//! designed for disk/flash encryption and implements ciphertext stealing
//! to handle non-block-aligned data.
//!
//! Note: This is a software implementation using hardware-accelerated ECB.
//!
//! @param ctx       - Pointer to AES-XTS context (must be initialized with key).
//! @param mode      - Operation mode (AM_HAL_AES_ENCRYPT or AM_HAL_AES_DECRYPT).
//! @param length    - Length of input/output data in bytes (minimum 16).
//! @param data_unit - Pointer to 16-byte data unit identifier (sector number).
//! @param input     - Pointer to input data.
//! @param output    - Pointer to output buffer.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_crypt_xts(am_hal_cc312_aes_xts_context_t *ctx,
                                     int32_t mode,
                                     uint32_t length,
                                     const uint8_t data_unit[AES_BLOCK_SIZE],
                                     const uint8_t *input,
                                     uint8_t *output);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_AES_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
