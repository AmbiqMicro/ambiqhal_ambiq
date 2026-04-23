//*****************************************************************************
//
//! @file am_hal_cc312.h
//!
//! @brief Hardware abstraction for the CryptoCell-312
//!
//! @addtogroup cc312_ap510 CryptoCell-312 Functionality
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

#ifndef AM_HAL_CC312_H
#define AM_HAL_CC312_H

#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// CryptoCell-312 clock types.
//
//*****************************************************************************
//
//! Clock type selection for generic clock control functions.
//!
//! Usage example:
//! @code
//!     // Enable AES and DMA clocks for AES operation
//!     am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_AES);
//!     am_hal_cc312_clock_enable(AM_HAL_CC312_CLK_DMA);
//!
//!     // Perform AES operations...
//!
//!     // Disable clocks when done
//!     am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_AES);
//!     am_hal_cc312_clock_disable(AM_HAL_CC312_CLK_DMA);
//! @endcode
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CC312_CLK_AES  = 0,  //!< AES engine clock.
    AM_HAL_CC312_CLK_HASH = 1,  //!< HASH engine clock.
    AM_HAL_CC312_CLK_PKA  = 2,  //!< PKA engine clock.
    AM_HAL_CC312_CLK_DMA  = 3,  //!< DMA engine clock.
}
am_hal_cc312_clk_type_t;

//*****************************************************************************
//
//! DMA address types for source and destination.
//
//*****************************************************************************
typedef enum
{
    AM_HAL_CC312_DMA_SRAM_ADDR = 0,  //!< Use SRAM address registers.
    AM_HAL_CC312_DMA_DLLI_ADDR = 1,  //!< Use DLLI (descriptor) address registers.
}
am_hal_cc312_dma_addr_type_e;

//*****************************************************************************
//
//! DMA buffer information structure.
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32DataAddr;      //!< Buffer address.
    uint8_t ui8NonSecure;       //!< Non-secure attribute.
}
am_hal_cc312_buffer_t;

//*****************************************************************************
//
// Register offsets from CryptoCell-312 base.
//
//*****************************************************************************
#define CC_AES_CLK_ENABLE               (0x810UL)
#define CC_HASH_CLK_ENABLE              (0x818UL)
#define CC_PKA_CLK_ENABLE               (0x81CUL)
#define CC_DMA_CLK_ENABLE               (0x820UL)
#define CC_CRYPTO_CTL                   (0x900UL)
#define CC_CRYPTO_BUSY                  (0x910UL)
#define CC_HOST_IRR                     (0xA00UL)
#define CC_HOST_IMR                     (0xA04UL)
#define CC_HOST_ICR                     (0xA08UL)
#define CC_AHBM_HNONSEC                 (0xC00UL)

// AES registers
#define CC_AES_CONTROL                  (0x400UL)
#define CC_AES_KEY_0_0                  (0x410UL)
#define CC_AES_KEY_0_1                  (0x414UL)
#define CC_AES_KEY_0_2                  (0x418UL)
#define CC_AES_KEY_0_3                  (0x41CUL)
#define CC_AES_KEY_0_4                  (0x420UL)
#define CC_AES_KEY_0_5                  (0x424UL)
#define CC_AES_KEY_0_6                  (0x428UL)
#define CC_AES_KEY_0_7                  (0x42CUL)
#define CC_AES_IV_0_0                   (0x440UL)
#define CC_AES_IV_0_1                   (0x444UL)
#define CC_AES_IV_0_2                   (0x448UL)
#define CC_AES_IV_0_3                   (0x44CUL)
#define CC_AES_REMAINING_BYTES          (0x4C0UL)

// DMA registers
#define CC_SRC_LLI_WORD0                (0x800UL)
#define CC_SRC_LLI_WORD1                (0x804UL)
#define CC_DST_LLI_WORD0                (0x808UL)
#define CC_DST_LLI_WORD1                (0x80CUL)

// Register values
#define SET_CLOCK_ENABLE                0x1UL
#define SET_CLOCK_DISABLE               0x0UL
#define CONFIG_DIN_AES_DOUT_VAL         0x1UL
#define AES_DRV_OK                      0

// AES Control register fields
#define AES_CONTROL_DEC_KEY0_SHIFT      0
#define AES_CONTROL_MODE_KEY0_SHIFT     2
#define AES_CONTROL_NK_KEY0_SHIFT       12

// IRR bits
#define HOST_IRR_SYM_DMA_COMPLETED_BIT  (CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk)

// Timeout for hardware operations
#define AM_HAL_CC_HAL_WAIT_TIMEOUT             100000UL
// Minimum alignment for CC312 DMA source/destination buffers.
#define AM_HAL_CC312_DMA_ALIGNMENT             32U

//*****************************************************************************
//
// Helper functions
//
//*****************************************************************************

//*****************************************************************************
//
// Wait for crypto engine to be ready.
//
//*****************************************************************************
static inline void
am_hal_cc312_wait_crypto_busy(void)
{
    register uint32_t timeout = AM_HAL_CC_HAL_WAIT_TIMEOUT;

    //
    // Wait for crypto engine to be ready
    //
    while (CRYPTO->CRYPTOBUSY_b.CRYPTOBUSY && timeout--) { }
}

//*****************************************************************************
//
// Clear specified CC312 interrupt(s).
//
//*****************************************************************************
static inline void
am_hal_cc312_clear_interrupt(uint32_t mask)
{
    CRYPTO->HOSTRGFICR = mask;
}

//*****************************************************************************
//
// Mask all CC312 interrupts.
//
//*****************************************************************************
static inline void
am_hal_cc312_mask_all_interrupts(void)
{
    CRYPTO->HOSTRGFIMR = 0xFFFFFFFFUL;
}

//*****************************************************************************
//
// Set CC312 data flow configuration.
//
//*****************************************************************************
static inline void
am_hal_cc312_set_data_flow(uint32_t flow_config)
{
    CRYPTO->CRYPTOCTL = flow_config;
}

//*****************************************************************************
//
// Clear/initialize a crypto context structure.
//
//*****************************************************************************
static inline void
am_hal_cc312_context_clear(void *ctx, uint32_t size)
{
    if (ctx != NULL)
    {
        volatile uint8_t *p = (volatile uint8_t *)ctx;

        while (size-- != 0U)
        {
            *p++ = 0U;
        }
    }
}

//*****************************************************************************
//
//! @brief Wait for a CryptoCell-312 interrupt.
//!
//! This function waits for the specified interrupt(s) to be set in the
//! CryptoCell-312 interrupt status register (HOST_IRR). It polls the
//! interrupt register until the masked interrupt is signaled or timeout occurs.
//!
//! @param irr_mask - Interrupt mask to wait for (HOST_IRR_* bits).
//!
//! @return AM_HAL_STATUS_SUCCESS if interrupt occurred.
//! @return AM_HAL_STATUS_TIMEOUT if timeout occurred.
//
//*****************************************************************************
extern int am_hal_cc312_wait_interrupt(uint32_t irr_mask);

//*****************************************************************************
//
//! @brief Enable a CryptoCell-312 clock.
//!
//! This function enables the specified clock for a CryptoCell-312 engine
//! (AES, HASH, PKA, or DMA).
//!
//! @param clk_type - Clock type to enable (AM_HAL_CC312_CLK_*).
//
//*****************************************************************************
extern void am_hal_cc312_clock_enable(am_hal_cc312_clk_type_t clk_type);

//*****************************************************************************
//
//! @brief Disable a CryptoCell-312 clock.
//!
//! This function disables the specified clock for a CryptoCell-312 engine
//! (AES, HASH, PKA, or DMA).
//!
//! @param clk_type - Clock type to disable (AM_HAL_CC312_CLK_*).
//
//*****************************************************************************
extern void am_hal_cc312_clock_disable(am_hal_cc312_clk_type_t clk_type);

//*****************************************************************************
//
//! @brief Check if a CryptoCell-312 clock is enabled.
//!
//! This function checks the enable status of the specified clock for a
//! CryptoCell-312 engine (AES, HASH, PKA, or DMA).
//!
//! @param clk_type - Clock type to check (AM_HAL_CC312_CLK_*).
//!
//! @return true if the clock is enabled, false otherwise.
//
//*****************************************************************************
extern bool am_hal_cc312_clock_is_enabled(am_hal_cc312_clk_type_t clk_type);

//*****************************************************************************
//
//! @brief Set buffer security attributes for DMA operations.
//!
//! Configures the AHBMHNONSEC register to set read/write security attributes
//! for input and output buffers used in DMA operations.
//!
//! @param input_nonsecure - true if input buffer is non-secure, false if secure
//! @param output_nonsecure - true if output buffer is non-secure, false if secure
//
//*****************************************************************************
extern void am_hal_cc312_set_buffer_security(bool input_nonsecure,
                                              bool output_nonsecure);

//*****************************************************************************
//
//! @brief Configure interrupt mask for DMA completion.
//!
//! Configures the HOSTRGFIMR register to mask all interrupts except the
//! SYM_DMA_COMPLETED interrupt. This is the standard configuration for
//! waiting on DMA operations.
//
//*****************************************************************************
extern void am_hal_cc312_config_dma_interrupt_mask(void);

//*****************************************************************************
//
//! @brief Configure DMA source address and length.
//!
//! Sets up the source address and data length for a DMA operation.
//! Supports both DLLI (descriptor-based) and SRAM addressing modes.
//!
//! @param addr_type - Address type (SRAM or DLLI)
//! @param address - Source buffer address
//! @param length - Data length in bytes
//
//*****************************************************************************
extern void am_hal_cc312_set_dma_source(am_hal_cc312_dma_addr_type_e addr_type,
                                        uint32_t address,
                                        uint32_t length);

//*****************************************************************************
//
//! @brief Configure DMA destination address and length.
//!
//! Sets up the destination address and data length for a DMA operation.
//! Supports both DLLI (descriptor-based) and SRAM addressing modes.
//!
//! @param addr_type - Address type (SRAM or DLLI)
//! @param address - Destination buffer address
//! @param length - Data length in bytes
//
//*****************************************************************************
extern void am_hal_cc312_set_dma_destination(am_hal_cc312_dma_addr_type_e addr_type,
                                              uint32_t address,
                                              uint32_t length);

//*****************************************************************************
//
//! @brief Clean+invalidate data cache for a specific memory range.
//!
//! @param addr   - Start address of range.
//! @param length - Range length in bytes.
//
//*****************************************************************************
extern void am_hal_cc312_cache_clean_invalidate_region(const void *addr, uint32_t length);

//*****************************************************************************
//
//! @brief Invalidate data cache for a specific memory range.
//!
//! @param addr   - Start address of range.
//! @param length - Range length in bytes.
//
//*****************************************************************************
extern void am_hal_cc312_cache_invalidate_region(const void *addr, uint32_t length);

//*****************************************************************************
//
//! @brief Perform dummy DLLI read to flush data cache.
//!
//! This function performs a dummy DLLI-mode DMA read operation to ensure
//! that data written by the CryptoCell-312 is properly flushed from the
//! data cache to memory. Required after DMA operations in DLLI mode.
//!
//! @param src_addr - Source address to read from.
//! @param length - Length of data in bytes.
//!
//! @return AM_HAL_STATUS_SUCCESS on success.
//! @return AM_HAL_STATUS_TIMEOUT on timeout.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_flush_dummy_dlli(uint32_t src_addr, uint32_t length);

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
extern uint32_t am_hal_cc312_validate_key_size(uint32_t key_bits,
                                                uint32_t *keySizeId);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
