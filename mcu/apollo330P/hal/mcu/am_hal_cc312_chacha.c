//*****************************************************************************
//
//! @file am_hal_cc312_chacha.c
//!
//! @brief Hardware abstraction for CryptoCell-312 ChaCha20 stream cipher
//!
//! @addtogroup cc312_chacha_ap330P CC312 ChaCha - Stream Cipher
//! @ingroup apollo330P_hal
//! @{
//!
//! Purpose: This module provides hardware-accelerated ChaCha20 stream cipher
//!          operations using the CryptoCell-312 hardware accelerator. It mirrors
//!          the structure and DMA-coherency handling of am_hal_cc312_aes.c.
//!
//! @section hal_cc312_chacha_features Key Features
//!
//! 1. @b ChaCha20: 20-round ChaCha stream cipher (256-bit key).
//! 2. @b Nonce: 64-bit (64-bit block counter) or 96-bit (32-bit counter, RFC 8439).
//! 3. @b DMA: 32-byte aligned buffers with DCACHE clean/invalidate and a DLLI
//!    dummy-read flush, identical to the AES cipher path.
//!
//! @section hal_cc312_chacha_usage Usage
//!
//! 1. Initialize context using am_hal_cc312_chacha_context_init()
//! 2. Set key using am_hal_cc312_chacha_setkey()
//! 3. Set nonce/counter using am_hal_cc312_chacha_set_nonce()
//! 4. Encrypt/decrypt using am_hal_chacha_crypt()
//! 5. Free context using am_hal_cc312_chacha_free()
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

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

//
//! CRYPTOCTL data-flow selector for the DIN-CHACHA-DOUT path. The ChaCha mode
//! is not enumerated in CRYPTO_CRYPTOCTL_MODE_Enum; the CC312 value is 0x10.
//
#define CHACHA_DATA_FLOW_DIN_CHACHA_DOUT 0x10UL
//! @}

//*****************************************************************************
//
//! @brief Validate a DMA endpoint against the CC312 transfer contract.
//!
//! @param addr_type Address type configured for this endpoint.
//! @param addr      Buffer address.
//! @param length    Number of bytes to process.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_chacha_check_buffer(am_hal_cc312_dma_addr_type_e addr_type,
                                 uint32_t addr,
                                 uint32_t length)
{
    if (addr_type != AM_HAL_CC312_DMA_DLLI_ADDR)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (length >= AM_HAL_CC312_DLLI_MAX_BUFF_SIZE)
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if ((addr & (AM_HAL_CC312_DMA_ALIGNMENT - 1U)) != 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

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
am_hal_cc312_chacha_init_buffer(const uint8_t *buffer,
                                am_hal_cc312_buffer_t *info)
{
    info->ui32DataAddr = (uint32_t)buffer;
    info->ui8NonSecure = DATA_BUFFER_IS_SECURE;
}

//*****************************************************************************
//
//! @brief Write the ChaCha control register for the current context.
//!
//! @param ctx Pointer to ChaCha context structure.
//
//*****************************************************************************
void
am_hal_cc312_chacha_set_control(am_hal_cc312_chacha_context_t *ctx)
{
    uint32_t ctrl = CRYPTO_CHACHACONTROLREG_INITFROMHOST_Msk;

    //
    // ChaCha (not Salsa), 20 rounds, 256-bit key, host-initialized state.
    // Select 96-bit IV layout when requested (RFC 8439).
    //
    if (ctx->nonceSize == AM_HAL_CHACHA_NONCE_SIZE_96)
    {
        ctrl |= CRYPTO_CHACHACONTROLREG_USEIV96BIT_Msk;
    }

    CRYPTO->CHACHACONTROLREG = ctrl;
}

//*****************************************************************************
//
//! @brief Execute one ChaCha DMA transfer using the configured context.
//!
//! @param ctx         Pointer to ChaCha context structure.
//! @param input_info  Pointer to input buffer metadata.
//! @param output_info Pointer to output buffer metadata.
//! @param length      Number of bytes to process.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
static uint32_t
am_hal_cc312_chacha_process(am_hal_cc312_chacha_context_t *ctx,
                            am_hal_cc312_buffer_t *input_info,
                            am_hal_cc312_buffer_t *output_info,
                            uint32_t length)
{
    uint32_t status;
    uint32_t irr_val = 0;

    if (ctx == NULL || input_info == NULL || output_info == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Reject transfers the DMA cannot express, and buffers that would make the
    // cache maintenance below touch memory outside the caller's allocation.
    // Checked before the engine is clocked so nothing needs unwinding.
    //
    status = am_hal_cc312_chacha_check_buffer(ctx->inputDataAddrType,
                                              input_info->ui32DataAddr,
                                              length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    status = am_hal_cc312_chacha_check_buffer(ctx->outputDataAddrType,
                                              output_info->ui32DataAddr,
                                              length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Enable clocks.
    //
    am_hal_cc312_chacha_enable_clocks();

    status = am_hal_cc312_chacha_init(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Load counter/nonce state and key, then configure the ChaCha mode.
    //
    status = am_hal_cc312_chacha_load_state(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    status = am_hal_cc312_chacha_load_key(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    am_hal_cc312_chacha_set_control(ctx);

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
    // Configure source address and size (kicks the DMA).
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
    // Capture the updated counter/nonce state.
    //
    status = am_hal_cc312_chacha_store_state(ctx);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto process_exit;
    }

    //
    // Perform DLLI-mode dummy read flush after state capture.
    //
    if (ctx->outputDataAddrType == AM_HAL_CC312_DMA_DLLI_ADDR)
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
    am_hal_cc312_chacha_disable_clocks();
    return status;
}

//*****************************************************************************
//
// Enable ChaCha and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_chacha_enable_clocks(void)
{
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_CHACHA);
    am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_DMA);
}

//*****************************************************************************
//
// Disable ChaCha and DMA clocks.
//
//*****************************************************************************
void
am_hal_cc312_chacha_disable_clocks(void)
{
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_DMA);
    am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_CHACHA);
}

//*****************************************************************************
//
// Initialize ChaCha context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_context_init(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_chacha_context_t));

    ctx->dir = AM_HAL_CHACHA_ENCRYPT;
    ctx->nonceSize = AM_HAL_CHACHA_NONCE_SIZE_96;
    ctx->inputDataAddrType = AM_HAL_CC312_DMA_DLLI_ADDR;
    ctx->outputDataAddrType = AM_HAL_CC312_DMA_DLLI_ADDR;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Free (clear) ChaCha context.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_free(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_context_clear(ctx, sizeof(am_hal_cc312_chacha_context_t));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set ChaCha key.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_setkey(am_hal_cc312_chacha_context_t *ctx,
                           const uint8_t *key,
                           uint32_t key_bits)
{
    if (ctx == NULL || key == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // CC312 ChaCha20 uses a fixed 256-bit key.
    //
    if (key_bits != (CHACHA_KEY_SIZE * 8))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memcpy(ctx->keyBuf, key, CHACHA_KEY_SIZE);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set ChaCha nonce and initial block counter.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_set_nonce(am_hal_cc312_chacha_context_t *ctx,
                              const uint8_t *nonce,
                              am_hal_cc312_chacha_nonce_size_e nonce_size,
                              uint32_t initial_counter)
{
    if (ctx == NULL || nonce == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(ctx->nonceBuf, 0, sizeof(ctx->nonceBuf));

    switch (nonce_size)
    {
        case AM_HAL_CHACHA_NONCE_SIZE_64:
            memcpy(ctx->nonceBuf, nonce, CHACHA_NONCE_64_SIZE);
            break;
        case AM_HAL_CHACHA_NONCE_SIZE_96:
            memcpy(ctx->nonceBuf, nonce, CHACHA_NONCE_96_SIZE);
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    ctx->nonceSize = nonce_size;
    ctx->blockCounterLsb = initial_counter;
    ctx->blockCounterMsb = 0;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initialize ChaCha hardware registers.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_init(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify valid nonce size.
    //
    if ((ctx->nonceSize != AM_HAL_CHACHA_NONCE_SIZE_64) &&
        (ctx->nonceSize != AM_HAL_CHACHA_NONCE_SIZE_96))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Verify valid address types.
    //
    if ((ctx->inputDataAddrType != AM_HAL_CC312_DMA_SRAM_ADDR) &&
        (ctx->inputDataAddrType != AM_HAL_CC312_DMA_DLLI_ADDR))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((ctx->outputDataAddrType != AM_HAL_CC312_DMA_SRAM_ADDR) &&
        (ctx->outputDataAddrType != AM_HAL_CC312_DMA_DLLI_ADDR))
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
    // Configure DIN-CHACHA-DOUT.
    //
    am_hal_cc312_set_data_flow(CHACHA_DATA_FLOW_DIN_CHACHA_DOUT);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load ChaCha key to hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_load_key(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    CRYPTO->CHACHAKEY0 = ctx->keyBuf[0];
    CRYPTO->CHACHAKEY1 = ctx->keyBuf[1];
    CRYPTO->CHACHAKEY2 = ctx->keyBuf[2];
    CRYPTO->CHACHAKEY3 = ctx->keyBuf[3];
    CRYPTO->CHACHAKEY4 = ctx->keyBuf[4];
    CRYPTO->CHACHAKEY5 = ctx->keyBuf[5];
    CRYPTO->CHACHAKEY6 = ctx->keyBuf[6];
    CRYPTO->CHACHAKEY7 = ctx->keyBuf[7];

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Load ChaCha nonce/counter state to hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_load_state(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (ctx->nonceSize)
    {
        case AM_HAL_CHACHA_NONCE_SIZE_64:
            //
            // 64-bit nonce: 64-bit block counter, 2 nonce words.
            //
            CRYPTO->CHACHABLOCKCNTMSB = ctx->blockCounterMsb;
            CRYPTO->CHACHAIV0 = ctx->nonceBuf[0];
            CRYPTO->CHACHAIV1 = ctx->nonceBuf[1];
            break;
        case AM_HAL_CHACHA_NONCE_SIZE_96:
            //
            // 96-bit nonce: 32-bit block counter, 3 nonce words.
            //
            CRYPTO->CHACHABLOCKCNTMSB = ctx->nonceBuf[0];
            CRYPTO->CHACHAIV0 = ctx->nonceBuf[1];
            CRYPTO->CHACHAIV1 = ctx->nonceBuf[2];
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    CRYPTO->CHACHABLOCKCNTLSB = ctx->blockCounterLsb;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Store ChaCha nonce/counter state from hardware.
//
//*****************************************************************************
uint32_t
am_hal_cc312_chacha_store_state(am_hal_cc312_chacha_context_t *ctx)
{
    if (ctx == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (ctx->nonceSize)
    {
        case AM_HAL_CHACHA_NONCE_SIZE_64:
            ctx->blockCounterMsb = CRYPTO->CHACHABLOCKCNTMSB;
            ctx->nonceBuf[0] = CRYPTO->CHACHAIV0;
            ctx->nonceBuf[1] = CRYPTO->CHACHAIV1;
            break;
        case AM_HAL_CHACHA_NONCE_SIZE_96:
            ctx->nonceBuf[0] = CRYPTO->CHACHABLOCKCNTMSB;
            ctx->nonceBuf[1] = CRYPTO->CHACHAIV0;
            ctx->nonceBuf[2] = CRYPTO->CHACHAIV1;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    ctx->blockCounterLsb = CRYPTO->CHACHABLOCKCNTLSB;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// ChaCha20 buffer encryption/decryption.
//
//*****************************************************************************
uint32_t
am_hal_chacha_crypt(am_hal_cc312_chacha_context_t *ctx,
                    uint32_t length,
                    const uint8_t *input,
                    uint8_t *output)
{
    am_hal_cc312_buffer_t input_info;
    am_hal_cc312_buffer_t output_info;

    //
    // Processing zero bytes is a no-op.
    //
    if (length == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    if (ctx == NULL || input == NULL || output == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_chacha_init_buffer(input, &input_info);
    am_hal_cc312_chacha_init_buffer(output, &output_info);

    return am_hal_cc312_chacha_process(ctx, &input_info, &output_info, length);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
