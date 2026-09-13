//*****************************************************************************
//
//! @file am_hal_cc312_bypass.c
//!
//! @brief Hardware abstraction for the CryptoCell-312 bypass DMA
//!
//! The bypass data flow wires DIN directly to DOUT with no engine in
//! between, which turns the CryptoCell DMA into a copy engine. The sequence
//! is the same as any other symmetric operation minus the key and mode
//! setup: select the flow, point the DMA at both buffers and wait for the
//! completion interrupt.
//!
//! @addtogroup cc312_bypass_ap510 CC312 Bypass - DMA Copy
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Redirect libc memcpy/memset to volatile helpers in am_hal_cc312.h.
#define AM_HAL_CC312_C

#include "am_mcu_apollo.h"

//! Buffer security attribute, matching the symmetric drivers.
#define BYPASS_BUFFER_IS_SECURE         0

//*****************************************************************************
//
// Copy a buffer through the CryptoCell DMA with no transformation.
//
//*****************************************************************************
uint32_t
am_hal_cc312_bypass_copy(const uint8_t *input,
                         uint8_t *output,
                         uint32_t length)
{
    uint32_t status;
    uint32_t irr_val = 0;

    if ((input == NULL) || (output == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (length == 0)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    am_hal_cc312_aes_enable_clocks();

    //
    // Make sure the symmetric engines are free before taking the datapath.
    //
    am_hal_cc312_wait_crypto_busy();

    am_hal_cc312_clear_interrupt(0xFFFFFFFFU);
    am_hal_cc312_config_dma_interrupt_mask();

    //
    // Route DIN straight to DOUT.
    //
    am_hal_cc312_set_data_flow(CRYPTO_CRYPTOCTL_MODE_BYPASS);

    am_hal_cc312_set_buffer_security(BYPASS_BUFFER_IS_SECURE,
                                     BYPASS_BUFFER_IS_SECURE);

    am_hal_cc312_set_dma_destination(AM_HAL_CC312_DMA_DLLI_ADDR,
                                     (uint32_t)output, length);

    am_hal_cc312_cache_clean_invalidate_region((const void *)input, length);
    am_hal_cc312_cache_clean_invalidate_region((const void *)output, length);

    am_hal_cc312_set_dma_source(AM_HAL_CC312_DMA_DLLI_ADDR,
                                (uint32_t)input, length);

    irr_val |= CRYPTO_HOSTRGFIRR_SYMDMACOMPLETED_Msk;
    status = am_hal_cc312_wait_interrupt(irr_val);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto bypass_exit;
    }

    status = am_hal_cc312_flush_dummy_dlli((uint32_t)output, length);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        goto bypass_exit;
    }

    am_hal_cc312_cache_invalidate_region((const void *)output, length);

bypass_exit:
    am_hal_cc312_aes_disable_clocks();

    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
