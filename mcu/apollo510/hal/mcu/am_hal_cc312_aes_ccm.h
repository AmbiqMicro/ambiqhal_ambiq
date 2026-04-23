//*****************************************************************************
//
//! @file am_hal_cc312_aes_ccm.h
//!
//! @brief Hardware abstraction for CryptoCell-312 AES-CCM mode
//!
//! @addtogroup cc312_aes_ccm_ap510 CC312 AES-CCM - Authenticated Encryption
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

#ifndef AM_HAL_CC312_AES_CCM_H
#define AM_HAL_CC312_AES_CCM_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// AES-CCM constants.
//
//*****************************************************************************
#define AM_HAL_AES_CCM_BLOCK_SIZE           16  //!< CCM block size (bytes).
#define AM_HAL_AES_CCM_IV_SIZE              16  //!< IV size (bytes).

//
// Nonce (N) size constraints.
//
#define AM_HAL_AES_CCM_NONCE_MIN_SIZE       7   //!< Minimum nonce size (bytes).
#define AM_HAL_AES_CCM_NONCE_MAX_SIZE       13  //!< Maximum nonce size (bytes).
#define AM_HAL_AES_CCM_STAR_NONCE_SIZE      13  //!< CCM* standard nonce size.

//
// Tag (T) size constraints.
//
#define AM_HAL_AES_CCM_TAG_MIN_SIZE         4   //!< Minimum tag size (bytes).
#define AM_HAL_AES_CCM_TAG_MAX_SIZE         16  //!< Maximum tag size (bytes).

//
// CCM hardware helper constants.
//
#define AM_HAL_CC312_AES_CCM_CMAC_INIT_VAL  0x1UL  //!< CMAC initialization value.
#define AM_HAL_CC312_DATA_BUFFER_IS_SECURE  0U     //!< Secure buffer attribute.

//*****************************************************************************
//
// AES-CCM operation modes.
//
//*****************************************************************************
#define AM_HAL_AES_CCM_MODE_CCM             0   //!< Standard CCM mode.
#define AM_HAL_AES_CCM_MODE_STAR            1   //!< CCM* mode (802.15.4).

//*****************************************************************************
//
// AES-CCM operation direction.
//
//*****************************************************************************
#define AM_HAL_AES_CCM_ENCRYPT              0   //!< CCM encryption mode.
#define AM_HAL_AES_CCM_DECRYPT              1   //!< CCM decryption mode.

//*****************************************************************************
//
// AES-CCM internal cipher modes (for tunneling).
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CCM_MODE_CBC_MAC = 3,    //!< CBC-MAC for authentication.
    AM_HAL_CCM_MODE_CTR     = 2,    //!< CTR for encryption/decryption.
    AM_HAL_CCM_MODE_CCMPE   = 9,    //!< CCM Parallel Encrypt (tunnel mode).
    AM_HAL_CCM_MODE_CCMPD   = 10,   //!< CCM Parallel Decrypt (tunnel mode).
    AM_HAL_CCM_MODE_RESERVE = 0x7FFFFFFF
}
am_hal_cc312_aes_ccm_mode_e;

//*****************************************************************************
//
// AES-CCM context structure.
//
//*****************************************************************************
//! AES-CCM context data used by CC312 HAL.
typedef struct
{
    uint32_t ivBuf[4];              //!< IV buffer for CBC-MAC (128-bit).
    uint32_t keyBuf[8];             //!< AES key buffer (max 256-bit = 8 words).
    uint32_t ctrStateBuf[4];        //!< AES counter state buffer (128-bit).
    uint8_t  tempBuff[16];          //!< Scratch buffer for internal use.
    am_hal_cc312_aes_ccm_mode_e mode;   //!< Current cipher mode.
    uint8_t  keySizeId;             //!< Key size: 0=128, 1=192, 2=256.
    uint8_t  dir;                   //!< Direction: 0=encrypt, 1=decrypt.
    uint8_t  sizeOfN;               //!< Nonce size (7-13 bytes).
    uint8_t  sizeOfT;               //!< Tag/MAC size (4-16 bytes).
} __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)))
am_hal_cc312_aes_ccm_context_t;

//*****************************************************************************
//
//! @brief Initialize AES-CCM context.
//!
//! This function initializes an AES-CCM context structure to a known state.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_init(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Free (clear) AES-CCM context.
//!
//! This function securely clears an AES-CCM context structure.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_free(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Set AES-CCM encryption/decryption key.
//!
//! This function sets the key for AES-CCM operations. The same key is used
//! for both CBC-MAC authentication and CTR mode encryption/decryption.
//!
//! @param ctx     - Pointer to AES-CCM context structure.
//! @param key     - Pointer to key data.
//! @param keybits - Key size in bits (128, 192, or 256).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_setkey(am_hal_cc312_aes_ccm_context_t *ctx,
                                      const uint8_t *key,
                                      uint32_t keybits);

//*****************************************************************************
//
//! @brief AES-CCM authenticated encryption.
//!
//! This function performs AES-CCM authenticated encryption in a single call.
//! It encrypts the input data and generates an authentication tag.
//!
//! @param ctx     - Pointer to AES-CCM context (must be initialized with key).
//! @param length  - Length of input/output data in bytes.
//! @param iv      - Pointer to nonce/IV. Size determined by iv_len (7-13 bytes).
//! @param iv_len  - Nonce/IV length in bytes (7-13, typically 13 for BLE/Zigbee).
//! @param aad     - Pointer to additional authenticated data (AAD). Can be NULL if aad_len is 0.
//! @param aad_len - AAD length in bytes.
//! @param input   - Pointer to plaintext input data.
//! @param output  - Pointer to ciphertext output buffer.
//! @param tag     - Pointer to output buffer for authentication tag.
//! @param tag_len - Desired tag length in bytes (4-16, must be even for standard CCM).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_encrypt_and_tag(am_hal_cc312_aes_ccm_context_t *ctx,
                                               uint32_t length,
                                               const uint8_t *iv,
                                               uint32_t iv_len,
                                               const uint8_t *aad,
                                               uint32_t aad_len,
                                               const uint8_t *input,
                                               uint8_t *output,
                                               uint8_t *tag,
                                               uint32_t tag_len);

//*****************************************************************************
//
//! @brief AES-CCM authenticated decryption with verification.
//!
//! This function performs AES-CCM authenticated decryption and verifies
//! the authentication tag. If the tag is invalid, the function returns
//! an error and the output should not be trusted.
//!
//! @param ctx     - Pointer to AES-CCM context (must be initialized with key).
//! @param length  - Length of input/output data in bytes.
//! @param iv      - Pointer to nonce/IV. Size determined by iv_len (7-13 bytes).
//! @param iv_len  - Nonce/IV length in bytes (7-13, typically 13 for BLE/Zigbee).
//! @param aad     - Pointer to additional authenticated data (AAD). Can be NULL if aad_len is 0.
//! @param aad_len - AAD length in bytes.
//! @param input   - Pointer to ciphertext input data.
//! @param output  - Pointer to plaintext output buffer.
//! @param tag     - Pointer to authentication tag to verify.
//! @param tag_len - Tag length in bytes (4-16, must be even for standard CCM).
//!
//! @return AM_HAL_STATUS_SUCCESS if tag is valid, error code otherwise.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_auth_decrypt(am_hal_cc312_aes_ccm_context_t *ctx,
                                            uint32_t length,
                                            const uint8_t *iv,
                                            uint32_t iv_len,
                                            const uint8_t *aad,
                                            uint32_t aad_len,
                                            const uint8_t *input,
                                            uint8_t *output,
                                            const uint8_t *tag,
                                            uint32_t tag_len);

//*****************************************************************************
//
//! @brief AES-CCM* authenticated encryption (802.15.4).
//!
//! This function performs AES-CCM* authenticated encryption. CCM* is a variant
//! of CCM used in IEEE 802.15.4 that allows tag_len = 0 (encryption only).
//!
//! @param ctx     - Pointer to AES-CCM context (must be initialized with key).
//! @param length  - Length of input/output data in bytes.
//! @param iv      - Pointer to 13-byte nonce (802.15.4 format).
//! @param iv_len  - Nonce length in bytes (must be 13).
//! @param aad     - Pointer to additional authenticated data (AAD). Can be NULL if aad_len is 0.
//! @param aad_len - AAD length in bytes.
//! @param input   - Pointer to plaintext input data.
//! @param output  - Pointer to ciphertext output buffer.
//! @param tag     - Pointer to output buffer for authentication tag.
//! @param tag_len - Desired tag length (0, 4, 8, or 16 bytes).
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_star_encrypt_and_tag(am_hal_cc312_aes_ccm_context_t *ctx,
                                                    uint32_t length,
                                                    const uint8_t *iv,
                                                    uint32_t iv_len,
                                                    const uint8_t *aad,
                                                    uint32_t aad_len,
                                                    const uint8_t *input,
                                                    uint8_t *output,
                                                    uint8_t *tag,
                                                    uint32_t tag_len);

//*****************************************************************************
//
//! @brief AES-CCM* authenticated decryption with verification (802.15.4).
//!
//! This function performs AES-CCM* authenticated decryption and verifies
//! the authentication tag.
//!
//! @param ctx     - Pointer to AES-CCM context (must be initialized with key).
//! @param length  - Length of input/output data in bytes.
//! @param iv      - Pointer to 13-byte nonce (802.15.4 format).
//! @param iv_len  - Nonce length in bytes (must be 13).
//! @param aad     - Pointer to additional authenticated data (AAD). Can be NULL if aad_len is 0.
//! @param aad_len - AAD length in bytes.
//! @param input   - Pointer to ciphertext input data.
//! @param output  - Pointer to plaintext output buffer.
//! @param tag     - Pointer to authentication tag to verify.
//! @param tag_len - Tag length (0, 4, 8, or 16 bytes).
//!
//! @return AM_HAL_STATUS_SUCCESS if tag is valid, error code otherwise.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_star_auth_decrypt(am_hal_cc312_aes_ccm_context_t *ctx,
                                                 uint32_t length,
                                                 const uint8_t *iv,
                                                 uint32_t iv_len,
                                                 const uint8_t *aad,
                                                 uint32_t aad_len,
                                                 const uint8_t *input,
                                                 uint8_t *output,
                                                 const uint8_t *tag,
                                                 uint32_t tag_len);

//*****************************************************************************
//
//! @brief Copy bytes from source to destination in reverse order.
//!
//! @param dst  - Pointer to destination buffer.
//! @param src  - Pointer to source buffer.
//! @param size - Number of bytes to copy.
//
//*****************************************************************************
extern void am_hal_aes_ccm_reverse_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size);

//*****************************************************************************
//
//! @brief Convert CCM* tag size into the 802.15.4 security-level field.
//!
//! @param sizeOfT         - CCM* tag size in bytes.
//! @param pSecurityLevel  - Pointer to output security-level field.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_get_security_level(uint8_t sizeOfT, uint8_t *pSecurityLevel);

//*****************************************************************************
//
//! @brief Load CCM key material into AES key registers.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_load_key(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Load CCM IV state into hardware IV registers.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_load_iv(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Store CCM IV state from hardware back into context.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_store_iv(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Load CCM counter state into hardware counter registers.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_load_ctr(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Store CCM counter state from hardware back into context.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//
//*****************************************************************************
extern void am_hal_aes_ccm_store_ctr(am_hal_cc312_aes_ccm_context_t *ctx);

//*****************************************************************************
//
//! @brief Initialize CC312 hardware for the current CCM operation mode.
//!
//! @param ctx - Pointer to AES-CCM context structure.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_aes_ccm_init_hw(am_hal_cc312_aes_ccm_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_AES_CCM_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
