//*****************************************************************************
//
//! @file am_hal_cc312_aes_gcm.c
//!
//! @brief Hardware abstraction for CryptoCell-312 AES-GCM mode
//!
//! @addtogroup cc312_aes_gcm_ap510 CC312 AES-GCM - Authenticated Encryption
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides comprehensive hardware-accelerated AES-GCM
//!          authenticated encryption operations using the CryptoCell-312
//!          hardware accelerator. AES-GCM combines AES-CTR encryption with
//!          GHASH authentication for high-performance secure communications.
//!
//! @section hal_cc312_aes_gcm_features Key Features
//!
//! 1. @b Authenticated @b Encryption: Combined encryption and authentication using GHASH.
//! 2. @b Parallel @b Processing: CTR mode enables parallelizable encryption.
//! 3. @b TLS @b Support: Primary mode for TLS 1.2/1.3 and IPsec.
//! 4. @b Flexible @b IV: Support for standard 96-bit and arbitrary-length IVs.
//! 5. @b High @b Performance: Hardware-accelerated GHASH and AES-CTR operations.
//!
//! @section hal_cc312_aes_gcm_functionality Functionality
//!
//! - Initialize GCM context and set encryption keys
//! - Perform authenticated encryption with tag generation
//! - Execute authenticated decryption with tag verification
//! - Support arbitrary-length initialization vectors
//! - Handle additional authenticated data (AAD)
//! - Provide flexible tag size configuration (4-16 bytes)
//!
//! @section hal_cc312_aes_gcm_usage Usage
//!
//! 1. Initialize context using am_hal_aes_gcm_init()
//! 2. Set key using am_hal_aes_gcm_setkey()
//! 3. Encrypt using am_hal_aes_gcm_crypt_and_tag() with mode AM_HAL_AES_GCM_ENCRYPT
//! 4. Decrypt using am_hal_aes_gcm_crypt_and_tag() with mode AM_HAL_AES_GCM_DECRYPT
//! 5. Free context using am_hal_aes_gcm_free()
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

static uint32_t
am_hal_aes_gcm_auth_diff(const uint8_t *lhs, const uint8_t *rhs, uint32_t len)
{
    uint32_t diff = 0U;
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        diff |= ((uint32_t)lhs[i] ^ (uint32_t)rhs[i]);
    }

    return diff;
}

//*****************************************************************************
//
//! @brief Compute the GHASH subkey H by encrypting an all-zero block.
//!
//! @param ctx Pointer to AES-GCM context.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_calc_h(am_hal_cc312_aes_gcm_context_t *ctx)
{
    uint32_t status;

    // Set process mode
    ctx->processMode = AM_HAL_AES_GCM_CALC_H;

    // Process zero block to generate H
    am_hal_cc312_context_clear(ctx->tempBuf, 16);
    status = am_hal_aes_gcm_process(ctx, (uint8_t *)ctx->tempBuf, (uint8_t *)ctx->H, 16);

    return status;
}

//*****************************************************************************
//
//! @brief Compute initial counter J0 from the provided IV.
//!
//! @param ctx Pointer to AES-GCM context.
//! @param iv  Pointer to IV bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_calc_j0(am_hal_cc312_aes_gcm_context_t *ctx, const uint8_t *iv)
{
    uint32_t status;

    if (ctx->ivSize == 12)  // 96-bit IV
    {
        // Concatenate IV||0(31)||1
        memcpy(ctx->J0, iv, 12);
        ctx->J0[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(0x00000001);
    }
    else  // Non-standard IV size
    {
        //
        // Phase 1: GHASH over IV
        //
        am_hal_cc312_context_clear(ctx->ghashResBuf, 16);
        ctx->processMode = AM_HAL_AES_GCM_CALC_J0_PHASE1;
        status = am_hal_aes_gcm_process(ctx, iv, NULL, ctx->ivSize);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // Phase 2: GHASH over len(IV)
        //
        am_hal_cc312_context_clear(ctx->tempBuf, 16);
        ctx->tempBuf[3] = (ctx->ivSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
        ctx->tempBuf[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[3]);

        ctx->processMode = AM_HAL_AES_GCM_CALC_J0_PHASE2;
        status = am_hal_aes_gcm_process(ctx, (uint8_t *)ctx->tempBuf, NULL, 16);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Process additional authenticated data into GHASH state.
//!
//! @param ctx Pointer to AES-GCM context.
//! @param aad Pointer to AAD bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_process_aad(am_hal_cc312_aes_gcm_context_t *ctx, const uint8_t *aad)
{
    uint32_t status;

    // Clear Ghash result buffer
    am_hal_cc312_context_clear(ctx->ghashResBuf, 16);

    if (ctx->aadSize == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    // Set process mode to 'Process_A'
    ctx->processMode = AM_HAL_AES_GCM_PROCESS_AAD;

    // Calculate GHASH(A)
    status = am_hal_aes_gcm_process(ctx, aad, NULL, ctx->aadSize);

    return status;
}

//*****************************************************************************
//
//! @brief Process plaintext/ciphertext payload for GCM data phase.
//!
//! @param ctx    Pointer to AES-GCM context.
//! @param input  Pointer to input payload.
//! @param output Pointer to output payload.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_process_cipher(am_hal_cc312_aes_gcm_context_t *ctx,
                       const uint8_t *input,
                       uint8_t *output)
{
    uint32_t status;

    if (ctx->dataSize == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    // Set process mode to 'Process_DataIn'
    ctx->processMode = AM_HAL_AES_GCM_PROCESS_DATA;

    status = am_hal_aes_gcm_process(ctx, input, output, ctx->dataSize);

    return status;
}

//*****************************************************************************
//
//! @brief Process encoded AAD/data length block into GHASH state.
//!
//! @param ctx Pointer to AES-GCM context.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_process_len(am_hal_cc312_aes_gcm_context_t *ctx)
{
    uint32_t status;

    // Build buffer: len(A) || len(C)
    ctx->tempBuf[1] = (ctx->aadSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
    ctx->tempBuf[1] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[1]);
    ctx->tempBuf[0] = 0;
    ctx->tempBuf[3] = (ctx->dataSize << 3) & AM_HAL_CC312_AES_GCM_BITMASK(32);
    ctx->tempBuf[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->tempBuf[3]);
    ctx->tempBuf[2] = 0;

    // Set process mode to 'Process_LenA_LenC'
    ctx->processMode = AM_HAL_AES_GCM_PROCESS_LEN;

    status = am_hal_aes_gcm_process(ctx, (uint8_t *)ctx->tempBuf, NULL, 16);

    return status;
}

//*****************************************************************************
//
//! @brief Generate (or verify) the final GCM authentication tag.
//!
//! @param ctx Pointer to AES-GCM context.
//! @param tag Pointer to tag buffer (output for encrypt, input for decrypt).
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
aes_gcm_finish(am_hal_cc312_aes_gcm_context_t *ctx, uint8_t *tag)
{
    uint32_t status;

    // Set process mode to 'Process_GctrFinal'
    ctx->processMode = AM_HAL_AES_GCM_GCTR_FINAL;

    status = am_hal_aes_gcm_process(ctx, (uint8_t *)ctx->tempBuf, ctx->preTagBuf, 16);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    // Copy tag (possibly truncated)
    if (ctx->dir == AM_HAL_AES_GCM_ENCRYPT)
    {
        memcpy(tag, ctx->preTagBuf, ctx->tagSize);
    }
    else  // Decrypt: compare tags
    {
        if (am_hal_aes_gcm_auth_diff(ctx->preTagBuf, tag, ctx->tagSize) != 0U)
        {
            return AM_HAL_STATUS_FAIL;  // Auth failed
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load GHASH subkey (H) to hardware.
// Reference: loadAesGcmGhashSubkey() in aesgcm_driver.c line 28-37
//
//*****************************************************************************
void
am_hal_aes_gcm_load_ghash_subkey(am_hal_cc312_aes_gcm_context_t *ctx)
{
    CRYPTO->GHASHSUBKEY00 = ctx->H[0];
    CRYPTO->GHASHSUBKEY01 = ctx->H[1];
    CRYPTO->GHASHSUBKEY02 = ctx->H[2];
    CRYPTO->GHASHSUBKEY03 = ctx->H[3];
}

//*****************************************************************************
//
// Load AES key to hardware.
// Reference: loadAesGcmKey() in aesgcm_driver.c line 38-59
//
//*****************************************************************************
void
am_hal_aes_gcm_load_key(am_hal_cc312_aes_gcm_context_t *ctx)
{
    CRYPTO->AESKEY00 = ctx->keyBuf[0];
    CRYPTO->AESKEY01 = ctx->keyBuf[1];
    CRYPTO->AESKEY02 = ctx->keyBuf[2];
    CRYPTO->AESKEY03 = ctx->keyBuf[3];

    if (ctx->keySizeId > 0)  // 192 or 256-bit
    {
        CRYPTO->AESKEY04 = ctx->keyBuf[4];
        CRYPTO->AESKEY05 = ctx->keyBuf[5];
    }

    if (ctx->keySizeId > 1)  // 256-bit
    {
        CRYPTO->AESKEY06 = ctx->keyBuf[6];
        CRYPTO->AESKEY07 = ctx->keyBuf[7];
    }
}

//*****************************************************************************
//
// Load counter to hardware.
// Reference: loadAesGcmCntr() in aesgcm_driver.c line 79-88
//
//*****************************************************************************
void
am_hal_aes_gcm_load_counter(am_hal_cc312_aes_gcm_context_t *ctx)
{
    CRYPTO->AESCTR00 = ctx->aesCntrBuf[0];
    CRYPTO->AESCTR01 = ctx->aesCntrBuf[1];
    CRYPTO->AESCTR02 = ctx->aesCntrBuf[2];
    CRYPTO->AESCTR03 = ctx->aesCntrBuf[3];
}

//*****************************************************************************
//
// Store counter from hardware.
// Reference: storeAesGcmCntr() in aesgcm_driver.c line 90-100
//
//*****************************************************************************
void
am_hal_aes_gcm_store_counter(am_hal_cc312_aes_gcm_context_t *ctx)
{
    ctx->aesCntrBuf[0] = CRYPTO->AESCTR00;
    ctx->aesCntrBuf[1] = CRYPTO->AESCTR01;
    ctx->aesCntrBuf[2] = CRYPTO->AESCTR02;
    ctx->aesCntrBuf[3] = CRYPTO->AESCTR03;
}

//*****************************************************************************
//
// Load GHASH IV to hardware.
// Reference: loadAesGcmGhashIV() in aesgcm_driver.c line 102-110
//
//*****************************************************************************
void
am_hal_aes_gcm_load_ghash_iv(am_hal_cc312_aes_gcm_context_t *ctx)
{
    CRYPTO->GHASHIV00 = ctx->ghashResBuf[0];
    CRYPTO->GHASHIV01 = ctx->ghashResBuf[1];
    CRYPTO->GHASHIV02 = ctx->ghashResBuf[2];
    CRYPTO->GHASHIV03 = ctx->ghashResBuf[3];
}

//*****************************************************************************
//
// Store GHASH IV from hardware.
// Reference: storeAesGcmGhashIV() in aesgcm_driver.c line 112-126
//
//*****************************************************************************
void
am_hal_aes_gcm_store_ghash_iv(am_hal_cc312_aes_gcm_context_t *ctx, bool storeInJ0)
{
    ctx->ghashResBuf[0] = CRYPTO->GHASHIV00;
    ctx->ghashResBuf[1] = CRYPTO->GHASHIV01;
    ctx->ghashResBuf[2] = CRYPTO->GHASHIV02;
    ctx->ghashResBuf[3] = CRYPTO->GHASHIV03;

    if (storeInJ0)
    {
        memcpy(ctx->J0, ctx->ghashResBuf, 16);
    }
}

//*****************************************************************************
//
// Increment J0 counter by 1.
// Reference: inc32AesGcmJ0() in aesgcm_driver.c line 61-77
//
//*****************************************************************************
void
am_hal_aes_gcm_inc32_j0(am_hal_cc312_aes_gcm_context_t *ctx)
{
    uint32_t tmpWord = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(ctx->J0[3]);
    
    // Check overflow and increment by 1
    if (tmpWord != 0xFFFFFFFF)
    {
        tmpWord++;
    }
    else
    {
        tmpWord = 0;
    }
    
    // Copy J0 to aesCntrBuf and update the incremented value
    memcpy(ctx->aesCntrBuf, ctx->J0, 16);
    ctx->aesCntrBuf[3] = AM_HAL_CC312_AES_GCM_SWAP_ENDIAN(tmpWord);
}

//*****************************************************************************
//
// Initialize hardware for GCM operation.
// Reference: InitAesGcm() in aesgcm_driver.c line 135-298
//
//*****************************************************************************
uint32_t
am_hal_aes_gcm_init_hw(am_hal_cc312_aes_gcm_context_t *ctx)
{
    uint32_t auxReg = 0;

    // Wait for crypto engine to be ready
    am_hal_cc312_wait_crypto_busy();

    // Clear AES_REMAINING_BYTES
    CRYPTO->AESREMAININGBYTES = 0x00;

    // Configure hardware based on process mode
    switch (ctx->processMode)
    {
        case AM_HAL_AES_GCM_CALC_H:
            // Set cryptographic flow to AES
            am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_AES);
            
            // Set Direction: Enc, Mode: ECB
            auxReg = 0;
            auxReg |= CRYPTO_AESCONTROL_DECKEY0_ENCRYPT;
            auxReg |= (CIPHER_ECB << CRYPTO_AESCONTROL_MODEKEY0_Pos);  // ECB mode
            auxReg |= (ctx->keySizeId << CRYPTO_AESCONTROL_NKKEY0_Pos);
            CRYPTO->AESCONTROL = auxReg;

            // Load Key
            am_hal_aes_gcm_load_key(ctx);

            // Set to 0x10 (16 bytes)
            CRYPTO->AESREMAININGBYTES = 16;
            break;

        case AM_HAL_AES_GCM_CALC_J0_PHASE1:
        case AM_HAL_AES_GCM_PROCESS_AAD:
            // Set the GHASH Engine to be ready to a new GHASH operation
            CRYPTO->GHASHINIT = AM_HAL_CC312_AES_GCM_GHASH_INIT_SET_VAL;
            // Falls through

        case AM_HAL_AES_GCM_CALC_J0_PHASE2:
            // Set cryptographic flow to HASH
            am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_HASH);
            
            // Set H as the GHASH Key
            am_hal_aes_gcm_load_ghash_subkey(ctx);
            
            // Set '0' as the GHASH IV
            am_hal_aes_gcm_load_ghash_iv(ctx);
            
            // Set Hash select module and GHASH select module
            auxReg = 0;
            auxReg |= (AM_HAL_CC312_AES_GCM_HASH_SEL_HASH_MOD << CRYPTO_HASHSELAESMAC_HASHSELAESMAC_Pos);
            auxReg |= (AM_HAL_CC312_AES_GCM_GHASH_SEL_GHASH_MOD << CRYPTO_HASHSELAESMAC_GHASHSEL_Pos);
            CRYPTO->HASHSELAESMAC = auxReg;
            
            // Set '0' as the GHASH IV (HASH_XOR_DIN)
            CRYPTO->HASHXORDIN = AM_HAL_CC312_AES_GCM_HASH_XOR_DATA_VAL;
            break;

        case AM_HAL_AES_GCM_PROCESS_DATA:
            // Set cryptographic flow according to crypto direction
            if (ctx->dir == AM_HAL_AES_GCM_ENCRYPT)
            {
                am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_AES_TO_HASH_AND_DOUT);
            }
            else
            {
                am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_AES_AND_HASH);
            }
            
            // AES Configuration: Direction: Enc, Mode: CTR
            auxReg = 0;
            auxReg |= CRYPTO_AESCONTROL_DECKEY0_ENCRYPT;  // Always encrypt for CTR
            auxReg |= (CIPHER_CTR << CRYPTO_AESCONTROL_MODEKEY0_Pos);  // CTR mode
            auxReg |= (ctx->keySizeId << CRYPTO_AESCONTROL_NKKEY0_Pos);
            CRYPTO->AESCONTROL = auxReg;

            // Increment J0 if not done yet
            if (!ctx->j0Inc32Done)
            {
                am_hal_aes_gcm_inc32_j0(ctx);
                ctx->j0Inc32Done = true;
            }

            // Load Counter
            am_hal_aes_gcm_load_counter(ctx);

            // Load Key
            am_hal_aes_gcm_load_key(ctx);

            // Set to data size
            CRYPTO->AESREMAININGBYTES = ctx->dataSize;

            // GHASH Configuration: Set H as the GHASH Key
            am_hal_aes_gcm_load_ghash_subkey(ctx);
            
            // Set former GHASH result as the new GHASH_IV value
            am_hal_aes_gcm_load_ghash_iv(ctx);

            // Set Hash select module and GHASH select module
            auxReg = 0;
            auxReg |= (AM_HAL_CC312_AES_GCM_HASH_SEL_HASH_MOD << CRYPTO_HASHSELAESMAC_HASHSELAESMAC_Pos);
            auxReg |= (AM_HAL_CC312_AES_GCM_GHASH_SEL_GHASH_MOD << CRYPTO_HASHSELAESMAC_GHASHSEL_Pos);
            CRYPTO->HASHSELAESMAC = auxReg;

            // Set '0' as the GHASH IV (HASH_XOR_DIN)
            CRYPTO->HASHXORDIN = AM_HAL_CC312_AES_GCM_HASH_XOR_DATA_VAL;
            break;

        case AM_HAL_AES_GCM_PROCESS_LEN:
            // Set cryptographic flow to HASH
            am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_HASH);
            
            // Set H as the GHASH Key
            am_hal_aes_gcm_load_ghash_subkey(ctx);
            
            // Set former GHASH result as the new GHASH_IV value
            am_hal_aes_gcm_load_ghash_iv(ctx);

            // Set Hash select module and GHASH select module
            auxReg = 0;
            auxReg |= (AM_HAL_CC312_AES_GCM_HASH_SEL_HASH_MOD << CRYPTO_HASHSELAESMAC_HASHSELAESMAC_Pos);
            auxReg |= (AM_HAL_CC312_AES_GCM_GHASH_SEL_GHASH_MOD << CRYPTO_HASHSELAESMAC_GHASHSEL_Pos);
            CRYPTO->HASHSELAESMAC = auxReg;

            // Set '0' as the GHASH IV (HASH_XOR_DIN)
            CRYPTO->HASHXORDIN = AM_HAL_CC312_AES_GCM_HASH_XOR_DATA_VAL;
            break;

        case AM_HAL_AES_GCM_GCTR_FINAL:
            // Set cryptographic flow to AES
            am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_AES);
            
            // AES Configuration: Direction: Enc, Mode: CTR
            auxReg = 0;
            auxReg |= CRYPTO_AESCONTROL_DECKEY0_ENCRYPT;
            auxReg |= (CIPHER_CTR << CRYPTO_AESCONTROL_MODEKEY0_Pos);  // CTR mode
            auxReg |= (ctx->keySizeId << CRYPTO_AESCONTROL_NKKEY0_Pos);
            CRYPTO->AESCONTROL = auxReg;

            // Load Counter (J0)
            memcpy(ctx->aesCntrBuf, ctx->J0, 16);
            am_hal_aes_gcm_load_counter(ctx);

            // Load Key
            am_hal_aes_gcm_load_key(ctx);

            // Copy GHASH result to the Input address (tempBuf)
            memcpy(ctx->tempBuf, ctx->ghashResBuf, 16);

            // Set to GHASH Output size (16 bytes)
            CRYPTO->AESREMAININGBYTES = 16;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    // Wait for busy, clear interrupts, mask interrupts
    am_hal_cc312_wait_crypto_busy();
    CRYPTO->HOSTRGFICR = 0xFFFFFFFFUL;
    
    // Mask unused DMA interrupts
    if ((ctx->processMode == AM_HAL_AES_GCM_CALC_H) ||
        (ctx->processMode == AM_HAL_AES_GCM_PROCESS_DATA) ||
        (ctx->processMode == AM_HAL_AES_GCM_GCTR_FINAL))
    {
        am_hal_cc312_config_dma_interrupt_mask();
    }
    else
    {
        am_hal_cc312_mask_all_interrupts();
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Process GCM operation (DMA).
// Reference: ProcessAesGcm() in aesgcm_driver.c line 300-465
//
//*****************************************************************************
uint32_t
am_hal_aes_gcm_process(am_hal_cc312_aes_gcm_context_t *ctx,
            const uint8_t *input,
            uint8_t *output,
            uint32_t length)
{
    uint32_t status;

    // Initialize hardware for this process mode
    status = am_hal_aes_gcm_init_hw(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    // Configure DMA destination (only for operations with output)
    if ((ctx->processMode == AM_HAL_AES_GCM_CALC_H) ||
        (ctx->processMode == AM_HAL_AES_GCM_PROCESS_DATA) ||
        (ctx->processMode == AM_HAL_AES_GCM_GCTR_FINAL))
    {
        am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)output, length);
        am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
                                         (uint32_t)output,
                                         length);
    }

    am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)input, length);

    // Configure DMA source and kick operation
    am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR,
                                (uint32_t)input,
                                length);

    // Wait for DMA completion interrupt
    status = am_hal_cc312_wait_interrupt(HOST_IRR_SYM_DMA_COMPLETED_BIT);

    // Post-processing: store results based on mode
    switch (ctx->processMode)
    {
        case AM_HAL_AES_GCM_CALC_H:
            // H is already in output buffer (ctx->H)
            break;

        case AM_HAL_AES_GCM_CALC_J0_PHASE1:
            am_hal_aes_gcm_store_ghash_iv(ctx, false);
            break;

        case AM_HAL_AES_GCM_CALC_J0_PHASE2:
            am_hal_aes_gcm_store_ghash_iv(ctx, true);  // Store in J0
            break;

        case AM_HAL_AES_GCM_PROCESS_AAD:
            am_hal_aes_gcm_store_ghash_iv(ctx, false);
            break;

        case AM_HAL_AES_GCM_PROCESS_DATA:
            am_hal_aes_gcm_store_counter(ctx);
            am_hal_aes_gcm_store_ghash_iv(ctx, false);
            break;

        case AM_HAL_AES_GCM_PROCESS_LEN:
            am_hal_aes_gcm_store_ghash_iv(ctx, false);
            break;

        case AM_HAL_AES_GCM_GCTR_FINAL:
            // preTagBuf already filled by DMA
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Perform DLLI-mode dummy read flush after state capture.
    //
    if (output != NULL)
    {
        status = am_hal_cc312_flush_dummy_dlli((uint32_t)output, length);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return AM_HAL_STATUS_TIMEOUT;
        }

        am_hal_cc312_cache_invalidate_region((const void *)(uintptr_t)output, length);
    }

    return status;
}

//*****************************************************************************
//
// Initialize GCM context.
// Reference: mbedtls_gcm_setkey() in gcm_alt.c
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_gcm_context_init(am_hal_cc312_aes_gcm_context_t *ctx,
                                   const uint8_t *key,
                                   uint32_t key_bits)
{
    uint32_t status;
    uint32_t keySizeId;

    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Clear context
    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_gcm_context_t));

    // Set key size
    status = am_hal_cc312_validate_key_size(key_bits, &keySizeId);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    ctx->keySizeId = (uint8_t)keySizeId;

    // Copy key
    memcpy(ctx->keyBuf, key, key_bits / 8);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Combined GCM operation.
// Reference: gcm_crypt_and_tag() in gcm_alt.c line 458-532
//
//*****************************************************************************
uint32_t
am_hal_aes_gcm_crypt_and_tag(am_hal_cc312_aes_gcm_context_t *ctx,
                              int mode,
                              uint32_t length,
                              const uint8_t *iv,
                              uint32_t iv_len,
                              const uint8_t *aad,
                              uint32_t aad_len,
                              const uint8_t *input,
                              uint8_t *output,
                              uint32_t tag_len,
                              uint8_t *tag)
{
    uint32_t status;

    // Set parameters
    ctx->dir = mode;
    ctx->dataSize = length;
    ctx->ivSize = iv_len;
    ctx->aadSize = aad_len;
    ctx->tagSize = tag_len;
    ctx->j0Inc32Done = false;

    // Enable clocks
    am_hal_cc312_aes_enable_clocks();
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_HASH);

    // Step 1: Calculate H
    status = aes_gcm_calc_h(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto cleanup;
    }

    // Step 2: Calculate J0
    status = aes_gcm_calc_j0(ctx, iv);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto cleanup;
    }

    // Step 3: Process AAD
    status = aes_gcm_process_aad(ctx, aad);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto cleanup;
    }

    // Step 4: Process cipher data
    status = aes_gcm_process_cipher(ctx, input, output);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto cleanup;
    }

    // Step 5: Process Len(A) || Len(C)
    status = aes_gcm_process_len(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto cleanup;
    }

    // Step 6: Generate/verify tag
    status = aes_gcm_finish(ctx, tag);

cleanup:
    if ((mode == AM_HAL_AES_GCM_DECRYPT) && (status != AM_HAL_STATUS_SUCCESS) &&
        (output != NULL) && (length != 0U))
    {
        am_hal_cc312_context_clear(output, length);
    }

    // Disable clocks
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_HASH);
    am_hal_cc312_aes_disable_clocks();

    return status;
}

//*****************************************************************************
//
// Initialize GCM context (compatibility wrapper).
//
//*****************************************************************************
void
am_hal_aes_gcm_init(am_hal_cc312_aes_gcm_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_gcm_context_t));
}

//*****************************************************************************
//
// Set key (compatibility wrapper).
//
//*****************************************************************************
uint32_t
am_hal_aes_gcm_setkey(am_hal_cc312_aes_gcm_context_t *ctx,
                      const uint8_t *key,
                      uint32_t key_bits)
{
    return am_hal_cc312_aes_gcm_context_init(ctx, key, key_bits);
}

//*****************************************************************************
//
// Free context (compatibility wrapper - no-op since we don't allocate).
//
//*****************************************************************************
void
am_hal_aes_gcm_free(am_hal_cc312_aes_gcm_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_gcm_context_t));
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
