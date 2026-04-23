//*****************************************************************************
//
//! @file am_hal_cc312_aes.c
//!
//! @brief Hardware abstraction for CryptoCell-312 AES cipher modes
//!
//! @addtogroup cc312_aes_ap510 CC312 AES - Cipher Modes
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides comprehensive hardware-accelerated AES
//!          encryption and decryption operations using the CryptoCell-312
//!          hardware accelerator. It supports multiple cipher modes including
//!          ECB, CTR, OFB (hardware), and XTS (software implementation).
//!
//! @section hal_cc312_aes_features Key Features
//!
//! 1. @b ECB @b Mode: Electronic Codebook basic block cipher (hardware).
//! 2. @b CTR @b Mode: Counter mode stream cipher with parallel processing (hardware).
//! 3. @b OFB @b Mode: Output Feedback stream cipher (hardware).
//! 4. @b XTS @b Mode: Disk encryption with tweak support (software).
//! 5. @b Key @b Sizes: Support for 128-bit, 192-bit, and 256-bit keys.
//!
//! @section hal_cc312_aes_functionality Functionality
//!
//! - Initialize AES context and set encryption/decryption keys
//! - Perform ECB mode encryption/decryption operations
//! - Execute CTR mode stream cipher operations
//! - Implement OFB mode stream cipher operations
//! - Provide XTS mode for disk encryption applications
//! - Support multiple key sizes (128/192/256-bit)
//!
//! @section hal_cc312_aes_usage Usage
//!
//! 1. Initialize context using am_hal_cc312_aes_context_init()
//! 2. Set key using am_hal_cc312_aes_setkey_enc() or am_hal_cc312_aes_setkey_dec()
//! 3. Call appropriate cipher mode function (ECB, CTR, OFB, or XTS)
//! 4. Free context using am_hal_cc312_aes_free()
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
//! @name Register configuration values.
//! @{
//
//*****************************************************************************
#define DATA_BUFFER_IS_SECURE           0       //!< Buffer security attribute.
#define INPUT_DATA_BUFFER               1       //!< Input buffer flag.
#define OUTPUT_DATA_BUFFER              0       //!< Output buffer flag.
//! @}

//*****************************************************************************
//
//! @name XTS mode endianness conversion macros (for 64-bit values).
//! @{
//
//*****************************************************************************
#define AM_HAL_GET_UINT64_LE(n,b,i)                         \
{                                                           \
    (n) = ( (uint64_t) (b)[(i) + 7] << 56 )                 \
        | ( (uint64_t) (b)[(i) + 6] << 48 )                 \
        | ( (uint64_t) (b)[(i) + 5] << 40 )                 \
        | ( (uint64_t) (b)[(i) + 4] << 32 )                 \
        | ( (uint64_t) (b)[(i) + 3] << 24 )                 \
        | ( (uint64_t) (b)[(i) + 2] << 16 )                 \
        | ( (uint64_t) (b)[(i) + 1] <<  8 )                 \
        | ( (uint64_t) (b)[(i)    ]       );                \
}

#define AM_HAL_PUT_UINT64_LE(n,b,i)                         \
{                                                           \
    (b)[(i) + 7] = (uint8_t) ( (n) >> 56 );                 \
    (b)[(i) + 6] = (uint8_t) ( (n) >> 48 );                 \
    (b)[(i) + 5] = (uint8_t) ( (n) >> 40 );                 \
    (b)[(i) + 4] = (uint8_t) ( (n) >> 32 );                 \
    (b)[(i) + 3] = (uint8_t) ( (n) >> 24 );                 \
    (b)[(i) + 2] = (uint8_t) ( (n) >> 16 );                 \
    (b)[(i) + 1] = (uint8_t) ( (n) >>  8 );                 \
    (b)[(i)    ] = (uint8_t) ( (n)       );                 \
}
//! @}

//*****************************************************************************
//
//! @brief Internal helper to set AES key material and direction in context.
//!
//! @param ctx      Pointer to AES context.
//! @param key      Pointer to raw AES key bytes.
//! @param key_bits AES key size in bits (128/192/256).
//! @param dir      Requested encrypt/decrypt direction.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_aes_setkey(am_hal_cc312_aes_context_t *ctx,
                        const uint8_t *key,
                        uint32_t key_bits,
                        am_hal_cc312_aes_direction_e dir)
{
    uint32_t status;

    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    uint32_t keySizeId;

    ctx->dir = dir;
    ctx->cryptoKey = USER_KEY;

    status = am_hal_cc312_validate_key_size(key_bits, &keySizeId);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    ctx->keySizeId = (am_hal_cc312_aes_key_size_e)keySizeId;

    memcpy(ctx->keyBuf, key, key_bits / 8);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Initialize DMA buffer metadata for a data pointer.
//!
//! @param buffer Pointer to data buffer.
//! @param info   Pointer to destination metadata structure.
//
//*****************************************************************************
static void
am_hal_cc312_aes_init_buffer(const uint8_t *buffer,
                             am_hal_cc312_buffer_t *info)
{
    info->ui32DataAddr = (uint32_t)buffer;
    info->ui8NonSecure = DATA_BUFFER_IS_SECURE;
}

//*****************************************************************************
//
//! @brief Execute one AES DMA transfer operation using the configured context.
//!
//! @param ctx         Pointer to AES context structure.
//! @param input_info  Pointer to input buffer metadata.
//! @param output_info Pointer to output buffer metadata.
//! @param length      Number of bytes to process.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_aes_process(am_hal_cc312_aes_context_t *ctx,
                         am_hal_cc312_buffer_t *input_info,
                         am_hal_cc312_buffer_t *output_info,
                         uint32_t length)
{
    uint32_t status;
    uint32_t irr_val = 0;

    //
    // Check input parameters.
    //
    if (ctx == NULL || input_info == NULL || output_info == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Enable clocks.
    //
    am_hal_cc312_aes_enable_clocks();

    status = am_hal_cc312_aes_init(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    status = am_hal_cc312_aes_load_key(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    status = am_hal_cc312_aes_load_iv(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Configure the HW with the correct data buffer attributes.
    //
    am_hal_cc312_set_buffer_security(input_info->ui8NonSecure, output_info->ui8NonSecure);

    //
    // Configure destination address and size.
    //
    am_hal_cc312_set_dma_destination((am_hal_cc312_dma_addr_type_e)ctx->outputDataAddrType,
                                     output_info->ui32DataAddr,
                                     length);

    am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)input_info->ui32DataAddr,
                                               length);
    am_hal_cc312_cache_clean_invalidate_region((const void *)(uintptr_t)output_info->ui32DataAddr,
                                               length);

    //
    // Configure source address and size.
    //
    am_hal_cc312_set_dma_source((am_hal_cc312_dma_addr_type_e)ctx->inputDataAddrType,
                                input_info->ui32DataAddr,
                                length);

    //
    // Set DMA completion bit in IRR and wait.
    //
    irr_val |= CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
    status = am_hal_cc312_wait_interrupt(irr_val);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Store IV state.
    //
    status = am_hal_cc312_aes_store_iv(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // At least one block of data processed.
    //
    ctx->dataBlockType = MIDDLE_BLOCK;

    //
    // Perform DLLI-mode dummy read flush after state capture.
    //
    if (ctx->outputDataAddrType == DLLI_ADDR)
    {
        status = am_hal_cc312_flush_dummy_dlli(output_info->ui32DataAddr, length);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            goto process_exit;
        }

        am_hal_cc312_cache_invalidate_region((const void *)(uintptr_t)output_info->ui32DataAddr,
                                             length);
    }

process_exit:
    am_hal_cc312_aes_disable_clocks();
    return status;
}

//*****************************************************************************
//
//! @brief Shared implementation for AES-CTR and AES-OFB operations.
//!
//! @param ctx    Pointer to AES context.
//! @param mode   Cipher mode (CTR or OFB).
//! @param length Number of bytes to process.
//! @param iv_off Pointer to stream offset (must be zero if provided).
//! @param iv     Pointer to 16-byte IV/counter buffer.
//! @param input  Pointer to input data.
//! @param output Pointer to output data.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
am_hal_aes_crypt_ctr_ofb(am_hal_cc312_aes_context_t *ctx,
                         am_hal_cc312_aes_mode_e mode,
                         uint32_t length,
                         uint32_t *iv_off,
                         uint8_t iv[AES_BLOCK_SIZE],
                         const uint8_t *input,
                         uint8_t *output)
{
    uint32_t status;
    am_hal_cc312_buffer_t input_info;
    am_hal_cc312_buffer_t output_info;

    //
    // Input validation.
    //
    if (length == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (ctx == NULL || iv == NULL || input == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((iv_off != NULL) && (*iv_off != 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Set mode and copy IV to context.
    //
    ctx->mode = mode;
    memcpy(ctx->ivBuf, iv, AES_BLOCK_SIZE);

    status = am_hal_cc312_aes_set_data_buffers_info(input, &input_info,
                                                    output, &output_info);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_aes_process(ctx, &input_info, &output_info, length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Copy updated IV back.
    //
    memcpy(iv, ctx->ivBuf, AES_BLOCK_SIZE);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Multiply a 128-bit value by x in GF(2^128) for XTS tweak updates.
//!
//! @param r Output 16-byte result buffer.
//! @param x Input 16-byte value.
//
//*****************************************************************************
static void
am_hal_gf128mul_x_ble(uint8_t r[16], const uint8_t x[16])
{
    uint64_t a, b, ra, rb;

    AM_HAL_GET_UINT64_LE(a, x, 0);
    AM_HAL_GET_UINT64_LE(b, x, 8);

    ra = (a << 1) ^ 0x0087 >> (8 - ((b >> 63) << 3));
    rb = (a >> 63) | (b << 1);

    AM_HAL_PUT_UINT64_LE(ra, r, 0);
    AM_HAL_PUT_UINT64_LE(rb, r, 8);
}

//*****************************************************************************
//
//! @brief Split an XTS double-length key into data and tweak keys.
//!
//! @param key      Pointer to concatenated XTS key material.
//! @param keybits  Total XTS key size in bits (256 or 512).
//! @param key1     Output pointer to first half key.
//! @param key1bits Output first half key size in bits.
//! @param key2     Output pointer to second half key.
//! @param key2bits Output second half key size in bits.
//!
//! @return AM_HAL_STATUS_SUCCESS on success, error code otherwise.
//
//*****************************************************************************
static uint32_t
am_hal_aes_xts_decode_keys(const uint8_t *key,
                           uint32_t keybits,
                           const uint8_t **key1,
                           uint32_t *key1bits,
                           const uint8_t **key2,
                           uint32_t *key2bits)
{
    const uint32_t half_keybits = keybits / 2;
    const uint32_t half_keybytes = half_keybits / 8;

    switch (keybits)
    {
        case 256:
        case 512:
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    *key1bits = half_keybits;
    *key2bits = half_keybits;
    *key1 = &key[0];
    *key2 = &key[half_keybytes];

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize AES hardware registers.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_init(am_hal_cc312_aes_context_t *ctx)
{
    am_hal_cc312_aes_direction_e dir;

    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify supported mode.
    //
    switch (ctx->mode)
    {
        case CIPHER_ECB:
        case CIPHER_CTR:
        case CIPHER_OFB:
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify valid direction.
    //
    if ((ctx->dir != CRYPTO_DIRECTION_ENCRYPT) &&
        (ctx->dir != CRYPTO_DIRECTION_DECRYPT))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify valid address types.
    //
    if ((ctx->inputDataAddrType != SRAM_ADDR) &&
        (ctx->inputDataAddrType != DLLI_ADDR))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((ctx->outputDataAddrType != SRAM_ADDR) &&
        (ctx->outputDataAddrType != DLLI_ADDR))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Make sure symmetric engines are ready to use.
    //
    am_hal_cc312_wait_crypto_busy();

    //
    // Clear all interrupts before starting the engine.
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
    // Configure AES direction.
    //
    dir = ctx->dir;

    //
    // Configure AES control register.
    //
    CRYPTO->AESCONTROL = 0;
    CRYPTO->AESCONTROL_b.DECKEY0 = dir;
    CRYPTO->AESCONTROL_b.MODEKEY0 = ctx->mode;
    CRYPTO->AESCONTROL_b.NKKEY0 = ctx->keySizeId;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load AES key to hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_load_key(am_hal_cc312_aes_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (ctx->cryptoKey != USER_KEY)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Load key [127:0].
    //
    CRYPTO->AESKEY00 = ctx->keyBuf[0];
    CRYPTO->AESKEY01 = ctx->keyBuf[1];
    CRYPTO->AESKEY02 = ctx->keyBuf[2];
    CRYPTO->AESKEY03 = ctx->keyBuf[3];

    if (ctx->keySizeId == KEY_SIZE_192_BIT || ctx->keySizeId == KEY_SIZE_256_BIT)
    {
        //
        // Load key [191:128].
        //
        CRYPTO->AESKEY04 = ctx->keyBuf[4];
        CRYPTO->AESKEY05 = ctx->keyBuf[5];
    }

    if (ctx->keySizeId == KEY_SIZE_256_BIT)
    {
        //
        // Load key [255:192].
        //
        CRYPTO->AESKEY06 = ctx->keyBuf[6];
        CRYPTO->AESKEY07 = ctx->keyBuf[7];
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load IV to hardware (CTR/OFB modes).
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_load_iv(am_hal_cc312_aes_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (ctx->mode)
    {
        case CIPHER_CTR:
        case CIPHER_OFB:
            CRYPTO->AESCTR00 = ctx->ivBuf[0];
            CRYPTO->AESCTR01 = ctx->ivBuf[1];
            CRYPTO->AESCTR02 = ctx->ivBuf[2];
            CRYPTO->AESCTR03 = ctx->ivBuf[3];
            break;
        case CIPHER_ECB:
            //
            // ECB doesn't use IV.
            //
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Store IV from hardware (CTR/OFB modes).
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_store_iv(am_hal_cc312_aes_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (ctx->mode)
    {
        case CIPHER_CTR:
        case CIPHER_OFB:
            ctx->ivBuf[0] = CRYPTO->AESCTR00;
            ctx->ivBuf[1] = CRYPTO->AESCTR01;
            ctx->ivBuf[2] = CRYPTO->AESCTR02;
            ctx->ivBuf[3] = CRYPTO->AESCTR03;
            break;
        case CIPHER_ECB:
            //
            // ECB doesn't use IV.
            //
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Set data buffer information for DMA.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_set_data_buffers_info(const uint8_t *input,
                                       am_hal_cc312_buffer_t *input_info,
                                       const uint8_t *output,
                                       am_hal_cc312_buffer_t *output_info)
{
    if (input_info == NULL || output_info == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_aes_init_buffer(input, input_info);
    am_hal_cc312_aes_init_buffer(output, output_info);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable AES and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_aes_enable_clocks(void)
{
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_AES);
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_DMA);
}

//*****************************************************************************
//
// Disable AES and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_aes_disable_clocks(void)
{
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_DMA);
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_AES);
}

//*****************************************************************************
//
// Initialize AES context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_context_init(am_hal_cc312_aes_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ctx->padType = CRYPTO_PADDING_NONE;
    ctx->dataBlockType = FIRST_BLOCK;
    ctx->inputDataAddrType = DLLI_ADDR;
    ctx->outputDataAddrType = DLLI_ADDR;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Free (clear) AES context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_free(am_hal_cc312_aes_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_aes_context_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set AES encryption key.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_setkey_enc(am_hal_cc312_aes_context_t *ctx,
                            const uint8_t *key,
                            uint32_t key_bits)
{
    return am_hal_cc312_aes_setkey(ctx, key, key_bits, CRYPTO_DIRECTION_ENCRYPT);
}

//*****************************************************************************
//
// Set AES decryption key.
//
//*****************************************************************************
uint32_t
am_hal_cc312_aes_setkey_dec(am_hal_cc312_aes_context_t *ctx,
                            const uint8_t *key,
                            uint32_t key_bits)
{
    return am_hal_cc312_aes_setkey(ctx, key, key_bits, CRYPTO_DIRECTION_DECRYPT);
}

//*****************************************************************************
//
// AES-ECB block encryption/decryption.
//
//*****************************************************************************
uint32_t
am_hal_aes_crypt_ecb(am_hal_cc312_aes_context_t *ctx,
                     int32_t mode,
                     const uint8_t input[AES_BLOCK_SIZE],
                     uint8_t output[AES_BLOCK_SIZE])
{
    uint32_t status;
    am_hal_cc312_buffer_t input_info;
    am_hal_cc312_buffer_t output_info;

    if (ctx == NULL || input == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (mode != AM_HAL_AES_DECRYPT && mode != AM_HAL_AES_ENCRYPT)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((mode == AM_HAL_AES_ENCRYPT && ctx->dir != CRYPTO_DIRECTION_ENCRYPT) ||
        (mode == AM_HAL_AES_DECRYPT && ctx->dir != CRYPTO_DIRECTION_DECRYPT))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ctx->mode = CIPHER_ECB;

    status = am_hal_cc312_aes_set_data_buffers_info(input, &input_info,
                                                    output, &output_info);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    return am_hal_cc312_aes_process(ctx, &input_info, &output_info, AES_BLOCK_SIZE);
}


//*****************************************************************************
//
// AES-CTR buffer encryption/decryption.
//
//*****************************************************************************
uint32_t
am_hal_aes_crypt_ctr(am_hal_cc312_aes_context_t *ctx,
                     uint32_t length,
                     uint32_t *nc_off,
                     uint8_t nonce_counter[AES_BLOCK_SIZE],
                     const uint8_t *input,
                     uint8_t *output)
{
    return am_hal_aes_crypt_ctr_ofb(ctx, CIPHER_CTR, length, nc_off,
                                    nonce_counter, input, output);
}

//*****************************************************************************
//
// AES-OFB buffer encryption/decryption.
//
//*****************************************************************************
uint32_t
am_hal_aes_crypt_ofb(am_hal_cc312_aes_context_t *ctx,
                     uint32_t length,
                     uint32_t *iv_off,
                     uint8_t iv[AES_BLOCK_SIZE],
                     const uint8_t *input,
                     uint8_t *output)
{
    return am_hal_aes_crypt_ctr_ofb(ctx, CIPHER_OFB, length, iv_off,
                                    iv, input, output);
}

//*****************************************************************************
//
// AES-XTS software implementation using ECB hardware acceleration.
//
//*****************************************************************************



//*****************************************************************************
//
// Initialize XTS context.
//
//*****************************************************************************
void
am_hal_aes_xts_init(am_hal_cc312_aes_xts_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_aes_context_init(&ctx->crypt);
    am_hal_cc312_aes_context_init(&ctx->tweak);
}

//*****************************************************************************
//
// Free (clear) XTS context.
//
//*****************************************************************************
void
am_hal_aes_xts_free(am_hal_cc312_aes_xts_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    am_hal_cc312_aes_free(&ctx->crypt);
    am_hal_cc312_aes_free(&ctx->tweak);
}

//*****************************************************************************
//
// Set XTS encryption key.
//
//*****************************************************************************
uint32_t
am_hal_aes_xts_setkey_enc(am_hal_cc312_aes_xts_context_t *ctx,
                          const uint8_t *key,
                          uint32_t keybits)
{
    uint32_t status;
    const uint8_t *key1, *key2;
    uint32_t key1bits, key2bits;

    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = am_hal_aes_xts_decode_keys(key, keybits, &key1, &key1bits,
                                        &key2, &key2bits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Set the tweak key (always encrypt).
    //
    status = am_hal_cc312_aes_setkey_enc(&ctx->tweak, key2, key2bits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Set crypt key for encryption.
    //
    return am_hal_cc312_aes_setkey_enc(&ctx->crypt, key1, key1bits);
}

//*****************************************************************************
//
// Set XTS decryption key.
//
//*****************************************************************************
uint32_t
am_hal_aes_xts_setkey_dec(am_hal_cc312_aes_xts_context_t *ctx,
                          const uint8_t *key,
                          uint32_t keybits)
{
    uint32_t status;
    const uint8_t *key1, *key2;
    uint32_t key1bits, key2bits;

    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    status = am_hal_aes_xts_decode_keys(key, keybits, &key1, &key1bits,
                                        &key2, &key2bits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Set the tweak key (always encrypt).
    //
    status = am_hal_cc312_aes_setkey_enc(&ctx->tweak, key2, key2bits);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Set crypt key for decryption.
    //
    return am_hal_cc312_aes_setkey_dec(&ctx->crypt, key1, key1bits);
}

//*****************************************************************************
//
// AES-XTS buffer encryption/decryption.
//
//*****************************************************************************
uint32_t
am_hal_aes_crypt_xts(am_hal_cc312_aes_xts_context_t *ctx,
                     int32_t mode,
                     uint32_t length,
                     const uint8_t data_unit[AES_BLOCK_SIZE],
                     const uint8_t *input,
                     uint8_t *output)
{
    uint32_t status;
    uint32_t blocks = length / 16;
    uint32_t leftover = length % 16;
    uint8_t tweak[16];
    uint8_t prev_tweak[16];
    uint8_t tmp[16];
    uint32_t i;

    if (ctx == NULL || data_unit == NULL || input == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (mode != AM_HAL_AES_ENCRYPT && mode != AM_HAL_AES_DECRYPT)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Data units must be at least 16 bytes long.
    //
    if (length < 16)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // NIST SP 800-38E disallows data units larger than 2^20 blocks.
    //
    if (length > (1 << 20) * 16)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Compute the tweak by encrypting the data unit.
    //
    status = am_hal_aes_crypt_ecb(&ctx->tweak, AM_HAL_AES_ENCRYPT, data_unit, tweak);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    while (blocks--)
    {
        if (leftover && (mode == AM_HAL_AES_DECRYPT) && blocks == 0)
        {
            //
            // Last block in decrypt with leftover bytes.
            // Save current tweak for leftovers, update for this block.
            //
            memcpy(prev_tweak, tweak, sizeof(tweak));
            am_hal_gf128mul_x_ble(tweak, tweak);
        }

        //
        // XOR input with tweak.
        //
        for (i = 0; i < 16; i++)
        {
            tmp[i] = input[i] ^ tweak[i];
        }

        //
        // Encrypt/decrypt the block.
        //
        status = am_hal_aes_crypt_ecb(&ctx->crypt, mode, tmp, tmp);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // XOR output with tweak.
        //
        for (i = 0; i < 16; i++)
        {
            output[i] = tmp[i] ^ tweak[i];
        }

        //
        // Update tweak for next block (multiply by x in GF(2^128)).
        //
        am_hal_gf128mul_x_ble(tweak, tweak);

        output += 16;
        input += 16;
    }

    if (leftover)
    {
        //
        // Ciphertext stealing for partial final block.
        //
        uint8_t *t = (mode == AM_HAL_AES_DECRYPT) ? prev_tweak : tweak;
        uint8_t *prev_output = output - 16;

        //
        // Copy ciphertext from previous block and input for final round.
        //
        for (i = 0; i < leftover; i++)
        {
            output[i] = prev_output[i];
            tmp[i] = input[i] ^ t[i];
        }

        //
        // Copy remaining ciphertext from previous block.
        //
        for (; i < 16; i++)
        {
            tmp[i] = prev_output[i] ^ t[i];
        }

        //
        // Encrypt/decrypt.
        //
        status = am_hal_aes_crypt_ecb(&ctx->crypt, mode, tmp, tmp);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }

        //
        // Write result back to previous block.
        //
        for (i = 0; i < 16; i++)
        {
            prev_output[i] = tmp[i] ^ t[i];
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
