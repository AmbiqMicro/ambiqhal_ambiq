//*****************************************************************************
//
//! @file am_hal_cc312_aes_ccm.c
//!
//! @brief Hardware abstraction for CryptoCell-312 AES-CCM mode
//!
//! @addtogroup cc312_aes_ccm_ap510 CC312 AES-CCM - Authenticated Encryption
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides comprehensive hardware-accelerated AES-CCM
//!          authenticated encryption operations using the CryptoCell-312
//!          hardware accelerator. AES-CCM combines CBC-MAC authentication
//!          with CTR mode encryption for secure communications.
//!
//! @section hal_cc312_aes_ccm_features Key Features
//!
//! 1. @b Authenticated @b Encryption: Combined encryption and authentication in one pass.
//! 2. @b Hardware @b Tunneling: Efficient parallel processing of CBC-MAC and CTR.
//! 3. @b IoT @b Protocols: Optimized for BLE, Zigbee, Thread, Matter, IEEE 802.15.4.
//! 4. @b CCM @b Star: Support for CCM* variant (tag size = 0 for encryption only).
//! 5. @b Flexible @b Parameters: Configurable nonce, AAD, and tag sizes.
//!
//! @section hal_cc312_aes_ccm_functionality Functionality
//!
//! - Initialize CCM context and set encryption keys
//! - Perform authenticated encryption with tag generation
//! - Execute authenticated decryption with tag verification
//! - Support CCM and CCM* (802.15.4) variants
//! - Handle additional authenticated data (AAD)
//! - Provide flexible nonce and tag size configuration
//!
//! @section hal_cc312_aes_ccm_usage Usage
//!
//! 1. Initialize context using am_hal_aes_ccm_init()
//! 2. Set key using am_hal_aes_ccm_setkey()
//! 3. Encrypt using am_hal_aes_ccm_encrypt_and_tag()
//! 4. Decrypt using am_hal_aes_ccm_auth_decrypt()
//! 5. Free context using am_hal_aes_ccm_free()
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include <string.h>
#include <stdint.h>

//*****************************************************************************
//
// CCM* security levels (IEEE 802.15.4-2011, Table 58).
//
//*****************************************************************************
#define AESCCM_STAR_SECURITY_LEVEL_ENC          4   //!< Encryption only.
#define AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_32   5   //!< Enc + 32-bit MIC.
#define AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_64   6   //!< Enc + 64-bit MIC.
#define AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_128  7   //!< Enc + 128-bit MIC.

//*****************************************************************************
//
// Helper macros.
//
//*****************************************************************************
#define MIN(a, b) ((a) < (b) ? (a) : (b))
//*****************************************************************************
//
//! @brief Initialize DMA buffer metadata for a CCM data pointer.
//!
//! @param buffer Pointer to raw data buffer.
//! @param info   Pointer to destination metadata structure.
//
//*****************************************************************************
static void
aes_ccm_init_buffer(const uint8_t *buffer,
                    am_hal_cc312_buffer_t *info)
{
    info->ui32DataAddr = (uint32_t)buffer;
    info->ui8NonSecure = AM_HAL_CC312_DATA_BUFFER_IS_SECURE;
}

//*****************************************************************************
//
//! @brief Execute one CCM hardware DMA processing step.
//!
//! @param ctx    Pointer to CCM context.
//! @param input  Pointer to input data.
//! @param output Pointer to output data (can be NULL for MAC-only paths).
//! @param length Number of bytes to process.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_process(am_hal_cc312_aes_ccm_context_t *ctx,
                             const uint8_t *input,
                             uint8_t *output,
                             uint32_t length)
{
    uint32_t status;
    am_hal_cc312_buffer_t input_info;
    am_hal_cc312_buffer_t output_info;

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Enable clocks.
    //
    am_hal_cc312_aes_enable_clocks();

    //
    // Initialize hardware.
    //
    status = am_hal_aes_ccm_init_hw(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Load key (both KEY_0 and KEY_1).
    //
    am_hal_aes_ccm_load_key(ctx);

    //
    // Initialize buffer info.
    //
    aes_ccm_init_buffer(input, &input_info);
    aes_ccm_init_buffer(output, &output_info);

    //
    // Configure buffer security attributes.
    //
    am_hal_cc312_set_buffer_security(input_info.ui8NonSecure, output_info.ui8NonSecure);

    //
    // Load CTR state for CTR/CCMPE/CCMPD modes.
    //
    if (ctx->mode != AM_HAL_CCM_MODE_CBC_MAC)
    {
        am_hal_aes_ccm_load_ctr(ctx);

        am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)output_info.ui32DataAddr, length);

        //
        // Configure destination for CTR output.
        //
        am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
                                         output_info.ui32DataAddr,
                                         length);
    }

    //
    // Load IV for CBC-MAC/CCMPE/CCMPD modes.
    //
    if (ctx->mode != AM_HAL_CCM_MODE_CTR)
    {
        am_hal_aes_ccm_load_iv(ctx);

        //
        // Initiate CMAC sub-key calculation.
        //
        CRYPTO->AESCMACINIT = AM_HAL_CC312_AES_CCM_CMAC_INIT_VAL;

        //
        // Set remaining bytes.
        //
        CRYPTO->AESREMAININGBYTES = length;
    }

    am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)input_info.ui32DataAddr,
                                               length);

    //
    // Configure source address and size.
    //
    am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR,
                                input_info.ui32DataAddr,
                                length);

    //
    // Wait for DMA completion.
    //
    status = am_hal_cc312_wait_interrupt(CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Store CTR state for CTR/CCMPE/CCMPD modes.
    //
    if (ctx->mode != AM_HAL_CCM_MODE_CBC_MAC)
    {
        am_hal_aes_ccm_store_ctr(ctx);
    }

    //
    // Store IV state for CBC-MAC/CCMPE/CCMPD modes.
    //
    if (ctx->mode != AM_HAL_CCM_MODE_CTR)
    {
        am_hal_aes_ccm_store_iv(ctx);
    }

    //
    // Perform DLLI-mode dummy read flush after state capture.
    //
    status = am_hal_cc312_flush_dummy_dlli(output_info.ui32DataAddr, length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    am_hal_cc312_cache_invalidate_region((const void *)(uintptr_t)output_info.ui32DataAddr, length);

process_exit:
    //
    // Disable clocks.
    //
    am_hal_cc312_aes_disable_clocks();

    return status;
}

//*****************************************************************************
//
//! @brief Initialize CCM/CCM* state, format B0, and initialize counter state.
//!
//! @param ctx      Pointer to CCM context.
//! @param dir      Operation direction (encrypt/decrypt).
//! @param aad_len  Additional authenticated data length in bytes.
//! @param data_len Payload length in bytes.
//! @param nonce    Pointer to nonce bytes.
//! @param nonce_len Nonce size in bytes.
//! @param tag_len  Requested tag length in bytes.
//! @param ccm_mode CCM mode selector (CCM or CCM*).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_init(am_hal_cc312_aes_ccm_context_t *ctx,
                uint8_t dir,
                uint32_t aad_len,
                uint32_t data_len,
                const uint8_t *nonce,
                uint8_t nonce_len,
                uint8_t tag_len,
                uint32_t ccm_mode)
{
    uint32_t status;
    uint8_t ctrStateBuf[AM_HAL_AES_CCM_BLOCK_SIZE];
    uint8_t qFieldSize = 15 - nonce_len;
    uint8_t *tempBuff;
    uint8_t securityLevelField = 0;

    //
    // Validate direction.
    //
    if (dir >= 2)  // CRYPTO_DIRECTION_NUM_OF_ENC_MODES
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Validate nonce.
    //
    if (nonce == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Validate tag size based on mode.
    //
    if (ccm_mode == AM_HAL_AES_CCM_MODE_CCM)
    {
        //
        // Standard CCM: tag must be even and in range [4, 16].
        //
        if ((tag_len < 4) || (tag_len > 16) || ((tag_len & 1) != 0))
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
    }
    else if (ccm_mode == AM_HAL_AES_CCM_MODE_STAR)
    {
        //
        // CCM*: validate security level.
        //
        status = am_hal_aes_ccm_get_security_level(tag_len, &securityLevelField);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // CCM* nonce must be 13 bytes.
        //
        if (nonce_len != AM_HAL_AES_CCM_STAR_NONCE_SIZE)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }

        //
        // Verify security level in nonce.
        //
        if (nonce[AM_HAL_AES_CCM_STAR_NONCE_SIZE - 1] != securityLevelField)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
    }
    else
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Validate Q field size (2 <= q <= 8).
    //
    if ((qFieldSize < 2) || (qFieldSize > 8))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify data size fits in Q field.
    //
    if ((qFieldSize < 4) && ((data_len >> (qFieldSize * 8)) > 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Validate nonce size.
    //
    if (nonce_len < AM_HAL_AES_CCM_NONCE_MIN_SIZE ||
        nonce_len >= AM_HAL_AES_CCM_BLOCK_SIZE)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Set context parameters.
    //
    ctx->mode = AM_HAL_CCM_MODE_CBC_MAC;
    ctx->sizeOfN = nonce_len;
    ctx->sizeOfT = tag_len;
    ctx->dir = dir;

    //
    // Clear buffers.
    //
    am_hal_cc312_context_clear(ctx->ivBuf, sizeof(ctx->ivBuf));
    am_hal_cc312_context_clear(ctx->ctrStateBuf, sizeof(ctx->ctrStateBuf));
    am_hal_cc312_context_clear(ctx->tempBuff, sizeof(ctx->tempBuff));

    //
    // Format first block B0 for CBC-MAC.
    //
    tempBuff = ctx->tempBuff;

    //
    // Set Adata flag (bit 6).
    //
    if (aad_len > 0)
    {
        tempBuff[0] = 1 << 6;
    }

    //
    // Set (t-2)/2 in bits [5:3] and (q-1) in bits [2:0].
    //
    tempBuff[0] |= ((tag_len - 2) / 2) << 3;
    tempBuff[0] |= (qFieldSize - 1);

    //
    // Copy nonce (N) into B0.
    //
    memcpy(tempBuff + 1, nonce, nonce_len);

    //
    // Copy data length (Q) into B0 in big-endian format.
    //
    am_hal_aes_ccm_reverse_memcpy(tempBuff + 16 - MIN(qFieldSize, 4),
                              (uint8_t *)&data_len,
                              MIN(qFieldSize, 4));

    //
    // Process B0 block with CBC-MAC.
    //
    status = aes_ccm_process(ctx, tempBuff, NULL, AM_HAL_AES_CCM_BLOCK_SIZE);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Set up initial counter value for CTR mode.
    //
    am_hal_cc312_context_clear(ctrStateBuf, sizeof(ctrStateBuf));

    //
    // Flags byte = (q-1).
    //
    ctrStateBuf[0] = qFieldSize - 1;

    //
    // Copy nonce.
    //
    memcpy(ctrStateBuf + 1, nonce, nonce_len);

    //
    // Set counter i = 1 (i = 0 is reserved for MAC encryption).
    //
    ctrStateBuf[15] = 1;

    memcpy(ctx->ctrStateBuf, ctrStateBuf, AM_HAL_AES_CCM_BLOCK_SIZE);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Process additional authenticated data for CCM/CCM*.
//!
//! @param ctx     Pointer to CCM context.
//! @param aad     Pointer to AAD buffer.
//! @param aad_len AAD length in bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_process_aad(am_hal_cc312_aes_ccm_context_t *ctx,
                    const uint8_t *aad,
                    uint32_t aad_len)
{
    uint32_t status;
    uint32_t firstBlockRemSize = 0;
    uint8_t Asize = 0;
    uint8_t *tempBuff;

    if (aad == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    tempBuff = ctx->tempBuff;

    //
    // Clear temp buffer.
    //
    am_hal_cc312_context_clear(tempBuff, AM_HAL_AES_CCM_BLOCK_SIZE);

    //
    // Encode AAD length.
    // Note: aad_len is uint32_t, so it's already <= 2^32-1.
    //
    if (aad_len >= 0xff00)
    {
        //
        // ASize >= 2^16-2^8: use 6 bytes (0xff||0xfe||ASize).
        //
        tempBuff[0] = 0xff;
        tempBuff[1] = 0xfe;
        am_hal_aes_ccm_reverse_memcpy(tempBuff + 2, (uint8_t *)&aad_len, 4);
        Asize = 6;
    }
    else if (aad_len > 0)
    {
        //
        // 0 < ASize < 2^16-2^8: use 2 bytes.
        //
        am_hal_aes_ccm_reverse_memcpy(tempBuff, (uint8_t *)&aad_len, 2);
        Asize = 2;
    }

    //
    // Calculate remaining space in first block.
    //
    firstBlockRemSize = AM_HAL_AES_CCM_BLOCK_SIZE - Asize;

    //
    // Copy first part of AAD into temp buffer.
    //
    if (aad_len > 0)
    {
        uint32_t copySize = MIN(firstBlockRemSize, aad_len);
        memcpy(tempBuff + Asize, aad, copySize);

        //
        // Process first block.
        //
        status = aes_ccm_process(ctx, tempBuff, NULL, AM_HAL_AES_CCM_BLOCK_SIZE);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // Process remaining AAD if any.
        //
        if (aad_len > firstBlockRemSize)
        {
            uint32_t remainingAad = aad_len - firstBlockRemSize;
            status = aes_ccm_process(ctx, aad + firstBlockRemSize,
                                                  NULL, remainingAad);
            if (status != AM_HAL_STATUS_SUCCESS)
            {
                return status;
            }
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Process plaintext/ciphertext payload for CCM data phase.
//!
//! @param ctx    Pointer to CCM context.
//! @param input  Pointer to input payload.
//! @param output Pointer to output payload.
//! @param length Payload length in bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_process_data(am_hal_cc312_aes_ccm_context_t *ctx,
                        const uint8_t *input,
                        uint8_t *output,
                        uint32_t length)
{
    uint32_t status;

    //
    // Validate buffer overlap.
    // In-place operation (input == output) is OK.
    // Input > output is OK (read before overwrite).
    // Input < output is NOT OK (would overwrite next input block).
    //
    if ((input < output) && (input + length > output))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Use hardware tunneling for parallel CTR + CBC-MAC.
    //
    if (ctx->dir == AM_HAL_AES_CCM_DECRYPT)
    {
        ctx->mode = AM_HAL_CCM_MODE_CCMPD;
    }
    else
    {
        ctx->mode = AM_HAL_CCM_MODE_CCMPE;
    }

    status = aes_ccm_process(ctx, input, output, length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Finalize CCM operation by generating or verifying the authentication tag.
//!
//! @param ctx     Pointer to CCM context.
//! @param tag     Pointer to tag buffer.
//! @param tag_len Pointer to tag length (in/out).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_finish(am_hal_cc312_aes_ccm_context_t *ctx,
               uint8_t *tag,
               uint32_t *tag_len)
{
    uint32_t status;
    uint8_t *tempBuff;
    uint8_t qFieldSize;
    uint32_t i;

    if (tag_len == NULL || tag == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    qFieldSize = 15 - ctx->sizeOfN;

    //
    // Set operation to CTR mode.
    //
    ctx->mode = AM_HAL_CCM_MODE_CTR;

    //
    // Set CTR value = CTR0 (zero the Q field).
    //
    am_hal_cc312_context_clear((uint8_t *)ctx->ctrStateBuf + AM_HAL_AES_CCM_BLOCK_SIZE - qFieldSize,
                               qFieldSize);

    if (ctx->dir == AM_HAL_AES_CCM_ENCRYPT)
    {
        tempBuff = ctx->tempBuff;
        am_hal_cc312_context_clear(tempBuff, sizeof(ctx->tempBuff));

        //
        // Encrypt MAC with CTR0.
        //
        status = aes_ccm_process(ctx, (uint8_t *)ctx->ivBuf,
                                              tempBuff, AM_HAL_AES_CCM_BLOCK_SIZE);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // Copy tag (truncated to requested size).
        //
        memcpy(tag, tempBuff, ctx->sizeOfT);
    }
    else
    {
        //
        // Decrypt mode: decrypt tag and verify.
        //
        tempBuff = ctx->tempBuff;
        am_hal_cc312_context_clear(tempBuff, sizeof(ctx->tempBuff));

        //
        // Copy encrypted MAC to temp buffer (padded with zeros).
        //
        memcpy(tempBuff, tag, ctx->sizeOfT);

        //
        // Decrypt MAC.
        //
        status = aes_ccm_process(ctx, tempBuff, tempBuff,
                                              AM_HAL_AES_CCM_BLOCK_SIZE);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // Constant-time comparison of calculated vs decrypted MAC.
        //
        uint32_t diff = 0;
        for (i = 0; i < ctx->sizeOfT; i++)
        {
            diff |= ((uint8_t *)ctx->ivBuf)[i] ^ tempBuff[i];
        }

        if (diff != 0)
        {
            return AM_HAL_STATUS_FAIL;
        }
    }

    *tag_len = ctx->sizeOfT;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Shared authenticated encryption/decryption flow for CCM and CCM*.
//!
//! @param ctx      Pointer to CCM context.
//! @param length   Payload length in bytes.
//! @param iv       Pointer to nonce/IV.
//! @param iv_len   Nonce/IV length in bytes.
//! @param aad      Pointer to additional authenticated data.
//! @param aad_len  Additional authenticated data length in bytes.
//! @param input    Pointer to input payload.
//! @param output   Pointer to output payload.
//! @param tag      Pointer to authentication tag buffer.
//! @param tag_len  Tag length in bytes.
//! @param dir      Operation direction (encrypt/decrypt).
//! @param ccm_mode CCM mode selector (CCM or CCM*).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_ccm_auth_crypt(am_hal_cc312_aes_ccm_context_t *ctx,
                       uint32_t length,
                       const uint8_t *iv,
                      uint32_t iv_len,
                      const uint8_t *aad,
                      uint32_t aad_len,
                      const uint8_t *input,
                      uint8_t *output,
                      uint8_t *tag,
                      uint32_t tag_len,
                      uint8_t dir,
                      uint32_t ccm_mode)
{
    uint32_t status;

    //
    // Validate context.
    //
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Initialize CCM operation.
    //
    status = aes_ccm_init(ctx, dir, aad_len, length, iv, iv_len, tag_len, ccm_mode);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Process AAD if present.
    //
    if (aad_len != 0)
    {
        status = aes_ccm_process_aad(ctx, aad, aad_len);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
    }

    //
    // Process data if present.
    //
    if (length != 0)
    {
        status = aes_ccm_process_data(ctx, input, output, length);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
    }

    //
    // Finalize and generate/verify tag.
    //
    status = aes_ccm_finish(ctx, tag, &tag_len);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        //
        // On failure, clear output.
        //
        am_hal_cc312_context_clear(output, length);
    }

    return status;
}


//*****************************************************************************
//
// Reverse byte order memory copy (for big-endian formatting).
//
//*****************************************************************************
void
am_hal_aes_ccm_reverse_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size)
{
    uint32_t i;
    for (i = 0; i < size; i++)
    {
        dst[i] = src[size - 1 - i];
    }
}

//*****************************************************************************
//
// Get CCM* security level field from tag size.
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_get_security_level(uint8_t sizeOfT, uint8_t *pSecurityLevel)
{
    if (pSecurityLevel == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // CCM* security levels (IEEE 802.15.4-2011, Table 58).
    // Only security levels with encryption (1XX) are supported.
    //
    switch (sizeOfT)
    {
        case 0:
            *pSecurityLevel = AESCCM_STAR_SECURITY_LEVEL_ENC;
            break;
        case 4:
            *pSecurityLevel = AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_32;
            break;
        case 8:
            *pSecurityLevel = AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_64;
            break;
        case 16:
            *pSecurityLevel = AESCCM_STAR_SECURITY_LEVEL_ENC_MIC_128;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize buffer information structure.
//
//*****************************************************************************

//*****************************************************************************
//
// Load AES key to hardware (both KEY_0 and KEY_1 for tunneling).
//
//*****************************************************************************
void
am_hal_aes_ccm_load_key(am_hal_cc312_aes_ccm_context_t *ctx)
{
    //
    // Load KEY_0 [127:0].
    //
    CRYPTO->AESKEY00 = ctx->keyBuf[0];
    CRYPTO->AESKEY01 = ctx->keyBuf[1];
    CRYPTO->AESKEY02 = ctx->keyBuf[2];
    CRYPTO->AESKEY03 = ctx->keyBuf[3];

    //
    // Load KEY_1 [127:0] (same as KEY_0).
    //
    CRYPTO->AESKEY10 = ctx->keyBuf[0];
    CRYPTO->AESKEY11 = ctx->keyBuf[1];
    CRYPTO->AESKEY12 = ctx->keyBuf[2];
    CRYPTO->AESKEY13 = ctx->keyBuf[3];

    if (ctx->keySizeId >= 1)  // 192 or 256-bit
    {
        //
        // Load KEY_0 [191:128].
        //
        CRYPTO->AESKEY04 = ctx->keyBuf[4];
        CRYPTO->AESKEY05 = ctx->keyBuf[5];

        //
        // Load KEY_1 [191:128].
        //
        CRYPTO->AESKEY14 = ctx->keyBuf[4];
        CRYPTO->AESKEY15 = ctx->keyBuf[5];
    }

    if (ctx->keySizeId == 2)  // 256-bit
    {
        //
        // Load KEY_0 [255:192].
        //
        CRYPTO->AESKEY06 = ctx->keyBuf[6];
        CRYPTO->AESKEY07 = ctx->keyBuf[7];

        //
        // Load KEY_1 [255:192].
        //
        CRYPTO->AESKEY16 = ctx->keyBuf[6];
        CRYPTO->AESKEY17 = ctx->keyBuf[7];
    }
}

//*****************************************************************************
//
// Load IV to hardware.
//
//*****************************************************************************
void
am_hal_aes_ccm_load_iv(am_hal_cc312_aes_ccm_context_t *ctx)
{
    if (ctx->mode == AM_HAL_CCM_MODE_CBC_MAC)
    {
        //
        // CBC-MAC uses AES_IV_0.
        //
        CRYPTO->AESIV00 = ctx->ivBuf[0];
        CRYPTO->AESIV01 = ctx->ivBuf[1];
        CRYPTO->AESIV02 = ctx->ivBuf[2];
        CRYPTO->AESIV03 = ctx->ivBuf[3];
    }
    else
    {
        //
        // Tunnel modes use AES_IV_1.
        //
        CRYPTO->AESIV10 = ctx->ivBuf[0];
        CRYPTO->AESIV11 = ctx->ivBuf[1];
        CRYPTO->AESIV12 = ctx->ivBuf[2];
        CRYPTO->AESIV13 = ctx->ivBuf[3];
    }
}

//*****************************************************************************
//
// Store IV from hardware.
//
//*****************************************************************************
void
am_hal_aes_ccm_store_iv(am_hal_cc312_aes_ccm_context_t *ctx)
{
    if (ctx->mode == AM_HAL_CCM_MODE_CBC_MAC)
    {
        //
        // CBC-MAC uses AES_IV_0.
        //
        ctx->ivBuf[0] = CRYPTO->AESIV00;
        ctx->ivBuf[1] = CRYPTO->AESIV01;
        ctx->ivBuf[2] = CRYPTO->AESIV02;
        ctx->ivBuf[3] = CRYPTO->AESIV03;
    }
    else
    {
        //
        // Tunnel modes use AES_IV_1.
        //
        ctx->ivBuf[0] = CRYPTO->AESIV10;
        ctx->ivBuf[1] = CRYPTO->AESIV11;
        ctx->ivBuf[2] = CRYPTO->AESIV12;
        ctx->ivBuf[3] = CRYPTO->AESIV13;
    }
}

//*****************************************************************************
//
// Load counter to hardware.
//
//*****************************************************************************
void
am_hal_aes_ccm_load_ctr(am_hal_cc312_aes_ccm_context_t *ctx)
{
    CRYPTO->AESCTR00 = ctx->ctrStateBuf[0];
    CRYPTO->AESCTR01 = ctx->ctrStateBuf[1];
    CRYPTO->AESCTR02 = ctx->ctrStateBuf[2];
    CRYPTO->AESCTR03 = ctx->ctrStateBuf[3];
}

//*****************************************************************************
//
// Store counter from hardware.
//
//*****************************************************************************
void
am_hal_aes_ccm_store_ctr(am_hal_cc312_aes_ccm_context_t *ctx)
{
    ctx->ctrStateBuf[0] = CRYPTO->AESCTR00;
    ctx->ctrStateBuf[1] = CRYPTO->AESCTR01;
    ctx->ctrStateBuf[2] = CRYPTO->AESCTR02;
    ctx->ctrStateBuf[3] = CRYPTO->AESCTR03;
}

//*****************************************************************************
//
// Initialize AES-CCM hardware registers.
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_init_hw(am_hal_cc312_aes_ccm_context_t *ctx)
{
    uint32_t aesControl = 0;

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify valid direction.
    //
    if ((ctx->dir != AM_HAL_AES_CCM_ENCRYPT) &&
        (ctx->dir != AM_HAL_AES_CCM_DECRYPT))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Configure AES control based on mode.
    //
    switch (ctx->mode)
    {
        case AM_HAL_CCM_MODE_CBC_MAC:
            aesControl |= CRYPTO_AESCONTROL_DECKEY0_ENCRYPT;  // Always encrypt for MAC
            aesControl |= (AM_HAL_CCM_MODE_CBC_MAC << CRYPTO_AESCONTROL_MODEKEY0_Pos);
            break;

        case AM_HAL_CCM_MODE_CTR:
            aesControl |= (ctx->dir << CRYPTO_AESCONTROL_DECKEY0_Pos);
            aesControl |= (AM_HAL_CCM_MODE_CTR << CRYPTO_AESCONTROL_MODEKEY0_Pos);
            break;

        case AM_HAL_CCM_MODE_CCMPE:
            //
            // Parallel encryption: CTR (KEY_0) + CBC-MAC (KEY_1).
            //
            aesControl |= (ctx->dir << CRYPTO_AESCONTROL_DECKEY0_Pos);
            aesControl |= (AM_HAL_CCM_MODE_CTR << CRYPTO_AESCONTROL_MODEKEY0_Pos);
            aesControl |= (AM_HAL_CCM_MODE_CBC_MAC << CRYPTO_AESCONTROL_MODEKEY1_Pos);
            aesControl |= CRYPTO_AESCONTROL_AESTUNNELISON_Msk;
            aesControl |= CRYPTO_AESCONTROL_AESTUNB1USESPADDEDDATAIN_Msk;
            aesControl |= CRYPTO_AESCONTROL_AESTUNNEL0ENCRYPT_Msk;
            aesControl |= CRYPTO_AESCONTROL_AESOUTPUTMIDTUNNELDATA_Msk;
            break;

        case AM_HAL_CCM_MODE_CCMPD:
            //
            // Parallel decryption: CTR (KEY_0) + CBC-MAC (KEY_1).
            //
            aesControl |= (ctx->dir << CRYPTO_AESCONTROL_DECKEY0_Pos);
            aesControl |= (AM_HAL_CCM_MODE_CTR << CRYPTO_AESCONTROL_MODEKEY0_Pos);
            aesControl |= (AM_HAL_CCM_MODE_CBC_MAC << CRYPTO_AESCONTROL_MODEKEY1_Pos);
            aesControl |= CRYPTO_AESCONTROL_AESTUNNELISON_Msk;
            aesControl |= CRYPTO_AESCONTROL_AESOUTPUTMIDTUNNELDATA_Msk;
            aesControl |= CRYPTO_AESCONTROL_AESTUNNELB1PADEN_Msk;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Set key size for both KEY_0 and KEY_1.
    //
    aesControl |= (ctx->keySizeId << CRYPTO_AESCONTROL_NKKEY0_Pos);
    aesControl |= (ctx->keySizeId << CRYPTO_AESCONTROL_NKKEY1_Pos);

    //
    // Make sure symmetric engines are ready.
    //
    am_hal_cc312_wait_crypto_busy();

    //
    // Clear all interrupts.
    //
    am_hal_cc312_clear_interrupt(0xFFFFFFFFU);

    //
    // Mask DMA interrupts which are not required.
    //
    am_hal_cc312_config_dma_interrupt_mask();

    //
    // Configure DIN-AES-DOUT.
    //
    am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_AES);

    //
    // Zero AES remaining bytes.
    //
    CRYPTO->AESREMAININGBYTES = 0;

    //
    // Configure AES control register.
    //
    CRYPTO->AESCONTROL = aesControl;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Process CCM operation (call hardware DMA).
//
//*****************************************************************************

//*****************************************************************************
//
// Initialize CCM operation (format and process B0 block).
//
//*****************************************************************************

//*****************************************************************************
//
// Process AAD (additional authenticated data).
//
//*****************************************************************************

//*****************************************************************************
//
// Process plaintext/ciphertext data.
//
//*****************************************************************************

//*****************************************************************************
//
// Finalize CCM operation (encrypt/decrypt and verify MAC).
//
//*****************************************************************************

//*****************************************************************************
//
// Main CCM authenticated encryption/decryption.
//
//*****************************************************************************

//*****************************************************************************
//
// Initialize AES-CCM context.
//
//*****************************************************************************
void
am_hal_aes_ccm_init(am_hal_cc312_aes_ccm_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_ccm_context_t));
}

//*****************************************************************************
//
// Free (clear) AES-CCM context.
//
//*****************************************************************************
void
am_hal_aes_ccm_free(am_hal_cc312_aes_ccm_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_ccm_context_t));
}

//*****************************************************************************
//
// Set AES-CCM key.
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_setkey(am_hal_cc312_aes_ccm_context_t *ctx,
                      const uint8_t *key,
                      uint32_t keybits)
{
    uint32_t status;
    uint32_t keySizeId;

    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = am_hal_cc312_validate_key_size(keybits, &keySizeId);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    ctx->keySizeId = (uint8_t)keySizeId;

    memcpy(ctx->keyBuf, key, keybits / 8);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// AES-CCM authenticated encryption.
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_encrypt_and_tag(am_hal_cc312_aes_ccm_context_t *ctx,
                               uint32_t length,
                               const uint8_t *iv,
                               uint32_t iv_len,
                               const uint8_t *aad,
                               uint32_t aad_len,
                               const uint8_t *input,
                               uint8_t *output,
                               uint8_t *tag,
                               uint32_t tag_len)
{
    return aes_ccm_auth_crypt(ctx, length, iv, iv_len, aad, aad_len,
                                 input, output, tag, tag_len,
                                 AM_HAL_AES_CCM_ENCRYPT,
                                 AM_HAL_AES_CCM_MODE_CCM);
}

//*****************************************************************************
//
// AES-CCM authenticated decryption with verification.
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_auth_decrypt(am_hal_cc312_aes_ccm_context_t *ctx,
                            uint32_t length,
                            const uint8_t *iv,
                            uint32_t iv_len,
                            const uint8_t *aad,
                            uint32_t aad_len,
                            const uint8_t *input,
                            uint8_t *output,
                            const uint8_t *tag,
                            uint32_t tag_len)
{
    uint8_t localMacBuf[AM_HAL_AES_CCM_BLOCK_SIZE] __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)));

    if (tag_len > AM_HAL_AES_CCM_BLOCK_SIZE)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (tag == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Copy tag to local buffer for verification.
    //
    memcpy(localMacBuf, tag, tag_len);

    return aes_ccm_auth_crypt(ctx, length, iv, iv_len, aad, aad_len,
                                 input, output, localMacBuf, tag_len,
                                 AM_HAL_AES_CCM_DECRYPT,
                                 AM_HAL_AES_CCM_MODE_CCM);
}

//*****************************************************************************
//
// AES-CCM* authenticated encryption (802.15.4).
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_star_encrypt_and_tag(am_hal_cc312_aes_ccm_context_t *ctx,
                                    uint32_t length,
                                    const uint8_t *iv,
                                    uint32_t iv_len,
                                    const uint8_t *aad,
                                    uint32_t aad_len,
                                    const uint8_t *input,
                                    uint8_t *output,
                                    uint8_t *tag,
                                    uint32_t tag_len)
{
    return aes_ccm_auth_crypt(ctx, length, iv, iv_len, aad, aad_len,
                                 input, output, tag, tag_len,
                                 AM_HAL_AES_CCM_ENCRYPT,
                                 AM_HAL_AES_CCM_MODE_STAR);
}

//*****************************************************************************
//
// AES-CCM* authenticated decryption with verification (802.15.4).
//
//*****************************************************************************
uint32_t
am_hal_aes_ccm_star_auth_decrypt(am_hal_cc312_aes_ccm_context_t *ctx,
                                 uint32_t length,
                                 const uint8_t *iv,
                                 uint32_t iv_len,
                                 const uint8_t *aad,
                                 uint32_t aad_len,
                                 const uint8_t *input,
                                 uint8_t *output,
                                 const uint8_t *tag,
                                 uint32_t tag_len)
{
    uint8_t localMacBuf[AM_HAL_AES_CCM_BLOCK_SIZE] __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)));

    if (tag_len > AM_HAL_AES_CCM_BLOCK_SIZE)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (tag == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Copy tag to local buffer for verification.
    //
    memcpy(localMacBuf, tag, tag_len);

    return aes_ccm_auth_crypt(ctx, length, iv, iv_len, aad, aad_len,
                                 input, output, localMacBuf, tag_len,
                                 AM_HAL_AES_CCM_DECRYPT,
                                 AM_HAL_AES_CCM_MODE_STAR);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
