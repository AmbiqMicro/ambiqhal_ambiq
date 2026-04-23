//*****************************************************************************
//
//! @file am_hal_cc312.c
//!
//! @brief Hardware abstraction for the CryptoCell-312
//!
//! @addtogroup cc312_ap510 CryptoCell-312 Functionality
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module provides comprehensive hardware abstraction for the
//!          CryptoCell-312 cryptographic accelerator in Apollo5 devices.
//!          It includes common helper functions for register access, clock
//!          control, interrupt handling, and DMA operations.
//!
//! @section hal_cc312_features Key Features
//!
//! 1. @b Clock @b Control: Enable/disable clocks for AES, HASH, PKA, and DMA engines.
//! 2. @b Interrupt @b Management: Wait for and clear CryptoCell interrupts.
//! 3. @b Status @b Monitoring: Check busy status and operation completion.
//! 4. @b DMA @b Configuration: Set up DMA source/destination addresses.
//! 5. @b Context @b Management: Clear and initialize cryptographic contexts.
//!
//! @section hal_cc312_functionality Functionality
//!
//! - Initialize and configure CryptoCell-312 hardware
//! - Control clock enables for cryptographic engines
//! - Manage interrupts and status monitoring
//! - Configure DMA operations for data transfer
//! - Provide helper functions for register access
//!
//! @section hal_cc312_usage Usage
//!
//! 1. Enable required clocks using am_hal_cc312_clock_enable()
//! 2. Configure DMA source and destination addresses
//! 3. Initiate cryptographic operation
//! 4. Wait for completion using am_hal_cc312_wait_interrupt()
//! 5. Disable clocks using am_hal_cc312_clock_disable()
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
#include <stdbool.h>

//*****************************************************************************
//
// Clock register lookup table.
//
//*****************************************************************************
static volatile uint32_t* const g_am_hal_cc312_clk_regs[] =
{
    &CRYPTO->AESCLKENABLE,   // AM_HAL_CC312_CLK_AES
    &CRYPTO->HASHCLKENABLE,  // AM_HAL_CC312_CLK_HASH
    &CRYPTO->PKACLKENABLE,   // AM_HAL_CC312_CLK_PKA
    &CRYPTO->DMACLKENABLE,   // AM_HAL_CC312_CLK_DMA
};

//*****************************************************************************
//
// Enable a CryptoCell-312 clock.
//
//*****************************************************************************
void
am_hal_cc312_clock_enable(am_hal_cc312_clk_type_t clk_type)
{
    if (clk_type < (sizeof(g_am_hal_cc312_clk_regs) / sizeof(g_am_hal_cc312_clk_regs[0])))
    {
        *g_am_hal_cc312_clk_regs[clk_type] = SET_CLOCK_ENABLE;
    }
}

//*****************************************************************************
//
// Disable a CryptoCell-312 clock.
//
//*****************************************************************************
void
am_hal_cc312_clock_disable(am_hal_cc312_clk_type_t clk_type)
{
    if (clk_type < (sizeof(g_am_hal_cc312_clk_regs) / sizeof(g_am_hal_cc312_clk_regs[0])))
    {
        *g_am_hal_cc312_clk_regs[clk_type] = SET_CLOCK_DISABLE;
    }
}

//*****************************************************************************
//
// Check if a CryptoCell-312 clock is enabled.
//
//*****************************************************************************
bool
am_hal_cc312_clock_is_enabled(am_hal_cc312_clk_type_t clk_type)
{
    if (clk_type < (sizeof(g_am_hal_cc312_clk_regs) / sizeof(g_am_hal_cc312_clk_regs[0])))
    {
        return (*g_am_hal_cc312_clk_regs[clk_type] & 0x1UL) != 0;
    }
    return false;
}

//*****************************************************************************
//
// Wait for a CC312 interrupt.
//
//*****************************************************************************
int
am_hal_cc312_wait_interrupt(uint32_t irr_mask)
{
    uint32_t irr_val = 0;
    uint32_t error = AM_HAL_STATUS_SUCCESS;

    //
    // busy wait upon IRR signal.
    //
    do
    {
        irr_val = CRYPTO->HOSTRGFIRR;

        //
        // check APB bus error from HOST.
        //
        if (irr_val & CRYPTO_HOSTRGFIRR_AHBERRINT_Msk)
        {
            error = AM_HAL_STATUS_HW_ERR;

            //
            // set data for clearing bus error.
            //
            irr_mask |= CRYPTO_HOSTRGFICR_AXIERRCLEAR_Msk;
            break;
        }
    } while (!(irr_val & irr_mask));

    //
    // clear interrupt.
    //
    CRYPTO->HOSTRGFICR = irr_mask;

    return error;
}

//*****************************************************************************
//
// Set buffer security attributes for DMA operations.
//
//*****************************************************************************
void
am_hal_cc312_set_buffer_security(bool input_nonsecure, bool output_nonsecure)
{
    uint32_t reg_val = 0;

    if (input_nonsecure)
    {
        reg_val |= CRYPTO_AHBMHNONSEC_AHBREADHNONSEC_Msk;
    }

    if (output_nonsecure)
    {
        reg_val |= CRYPTO_AHBMHNONSEC_AHBWRITEHNONSEC_Msk;
    }

    CRYPTO->AHBMHNONSEC = reg_val;
}

//*****************************************************************************
//
// Configure interrupt mask for DMA completion.
//
//*****************************************************************************
void
am_hal_cc312_config_dma_interrupt_mask(void)
{
    uint32_t imr_val = CRYPTO->HOSTRGFIMR;
    
    imr_val |= (CRYPTO_HOSTRGFIMR_SRAMTODINMASK_Msk | 
                CRYPTO_HOSTRGFIMR_DOUTTOSRAMMASK_Msk |
                CRYPTO_HOSTRGFIMR_MEMTODINMASK_Msk | 
                CRYPTO_HOSTRGFIMR_DOUTTOMEMMASK_Msk);
    imr_val &= ~CRYPTO_HOSTRGFIMR_SYMDMACOMPLETEDMASK_Msk;
    
    CRYPTO->HOSTRGFIMR = imr_val;
}

//*****************************************************************************
//
// Configure DMA source address and length.
//
//*****************************************************************************
void
am_hal_cc312_set_dma_source(am_hal_cc312_dma_addr_type_e addr_type,
                            uint32_t address,
                            uint32_t length)
{
    if (addr_type == AM_HAL_CC312_DMA_DLLI_ADDR)
    {
        CRYPTO->SRCLLIWORD0 = address;
        CRYPTO->SRCLLIWORD1 = length;
    }
    else
    {
        CRYPTO->SRAMSRCADDR = address;
        CRYPTO->DINSRAMBYTESLEN = length;
    }
}

//*****************************************************************************
//
// Configure DMA destination address and length.
//
//*****************************************************************************
void
am_hal_cc312_set_dma_destination(am_hal_cc312_dma_addr_type_e addr_type,
                                 uint32_t address,
                                 uint32_t length)
{
    if (addr_type == AM_HAL_CC312_DMA_DLLI_ADDR)
    {
        CRYPTO->DSTLLIWORD0 = address;
        CRYPTO->DSTLLIWORD1 = length;
    }
    else
    {
        CRYPTO->SRAMDESTADDR = address;
        CRYPTO->DOUTSRAMBYTESLEN = length;
    }
}

//*****************************************************************************
//
// Range-based dcache clean+invalidate helper.
//
//*****************************************************************************
void
am_hal_cc312_cache_clean_invalidate_region(const void *addr, uint32_t length)
{
    am_hal_cachectrl_range_t range = {0};
    uint32_t cache_len;

    if (addr == NULL || length == 0U)
    {
        return;
    }

    cache_len = (length < AM_HAL_CC312_DMA_ALIGNMENT) ? AM_HAL_CC312_DMA_ALIGNMENT : length;
    range.ui32StartAddr = (uint32_t)addr;
    range.ui32Size = cache_len;

    (void)am_hal_cachectrl_dcache_invalidate(&range, true);
}

//*****************************************************************************
//
// Range-based dcache invalidate helper.
//
//*****************************************************************************
void
am_hal_cc312_cache_invalidate_region(const void *addr, uint32_t length)
{
    am_hal_cachectrl_range_t range = {0};
    uint32_t cache_len;

    if (addr == NULL || length == 0U)
    {
        return;
    }

    cache_len = (length < AM_HAL_CC312_DMA_ALIGNMENT) ? AM_HAL_CC312_DMA_ALIGNMENT : length;
    range.ui32StartAddr = (uint32_t)addr;
    range.ui32Size = cache_len;

    (void)am_hal_cachectrl_dcache_invalidate(&range, false);
}

//
// Local aligned sink for DMA-read flushes.
//
static uint8_t cc312_dummy_scratch[AM_HAL_CC312_DMA_ALIGNMENT] __attribute__((aligned(AM_HAL_CC312_DMA_ALIGNMENT)));

//*****************************************************************************
//
// Perform a tiny DMA read from the DLLI source tail into dummy SRAM.
//
//*****************************************************************************
uint32_t
am_hal_cc312_flush_dummy_dlli(uint32_t src_addr, uint32_t length)
{
    register uint32_t irr_val = CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;

    if (src_addr == 0U || length == 0U)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_cc312_clear_interrupt(0xFFFFFFFFUL);
    am_hal_cc312_config_dma_interrupt_mask();
    am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
                                     (uint32_t)cc312_dummy_scratch,
                                     AM_HAL_CC312_DMA_ALIGNMENT);
    am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR, src_addr, AM_HAL_CC312_DMA_ALIGNMENT);

    return am_hal_cc312_wait_interrupt(irr_val);
}

//*****************************************************************************
//
//! @brief Validate AES key size and set key size ID.
//!
//! Generic function to validate AES key size (128/192/256 bits) and set the
//! corresponding key size ID (0/1/2) used by the CC312 hardware.
//!
//! @param key_bits  - Key size in bits (must be 128, 192, or 256).
//! @param keySizeId - Pointer to store the key size ID (0=128, 1=192, 2=256).
//!
//! @return AM_HAL_STATUS_SUCCESS if valid key size.
//! @return AM_HAL_STATUS_INVALID_ARG if invalid key size.
//
//*****************************************************************************
uint32_t
am_hal_cc312_validate_key_size(uint32_t key_bits, uint32_t *keySizeId)
{
    if (keySizeId == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (key_bits)
    {
        case 128:
            *keySizeId = 0;
            break;
        case 192:
            *keySizeId = 1;
            break;
        case 256:
            *keySizeId = 2;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
