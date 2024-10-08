//*****************************************************************************
//
//! @file am_hal_pdm.c
//!
//! @brief HAL implementation for the PDM module.
//!
//! @addtogroup pdm3p PDM - Pulse Density Modulated Functions
//! @ingroup apollo3p_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2024, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_2_0-dd5f40c14b of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// PDM magic number for handle verification.
//
//*****************************************************************************
#define AM_HAL_MAGIC_PDM                0xF956E2

#define AM_HAL_PDM_HANDLE_VALID(h)                                            \
    ((h) &&                                                                   \
     ((am_hal_handle_prefix_t *)(h))->s.bInit &&                              \
     (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_PDM))

//*****************************************************************************
//
// Convenience macro for passing errors.
//
//*****************************************************************************
#define RETURN_ON_ERROR(x)                                                    \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        return (x);                                                           \
    };

//*****************************************************************************
//
// Abbreviation for validating handles and returning errors.
//
//*****************************************************************************
#ifndef AM_HAL_DISABLE_API_VALIDATION

#define AM_HAL_PDM_HANDLE_CHECK(h)                                            \
    if (!AM_HAL_PDM_HANDLE_VALID(h))                                          \
    {                                                                         \
        return AM_HAL_STATUS_INVALID_HANDLE;                                  \
    }

#else

#define AM_HAL_PDM_HANDLE_CHECK(h)

#endif // AM_HAL_DISABLE_API_VALIDATION

//*****************************************************************************
//
// Helper macros for delays.
//
//*****************************************************************************
#define delay_ms(ms)                                                          \
    if (1)                                                                    \
    {                                                                         \
        am_hal_clkgen_status_t sClkGenStatus;                                 \
        am_hal_clkgen_status_get(&sClkGenStatus);                             \
        am_hal_flash_delay((ms) * (sClkGenStatus.ui32SysclkFreq / 3000));     \
    }

#define delay_us(us)                                                          \
    if (1)                                                                    \
    {                                                                         \
        am_hal_clkgen_status_t sClkGenStatus;                                 \
        am_hal_clkgen_status_get(&sClkGenStatus);                             \
        am_hal_flash_delay((us) * (sClkGenStatus.ui32SysclkFreq / 3000000));  \
    }

//*****************************************************************************
//
// Structure for handling PDM register state information for power up/down
//
//*****************************************************************************
typedef struct
{
    bool bValid;
}
am_hal_pdm_register_state_t;

//*****************************************************************************
//
// Structure for handling PDM HAL state information.
//
//*****************************************************************************
typedef struct
{
    am_hal_handle_prefix_t prefix; //!< Don't move, This has to be first in this structure
    uint32_t ui32XfrCount;  //!< used for dma restart
    uint32_t ui32Module;
    uint32_t ui32BufferPtr;
    uint32_t ui32BufferPong;
    uint32_t ui32BufferPing;
    uint32_t ui32Threshold;
    am_hal_pdm_register_state_t sRegState;

}
am_hal_pdm_state_t;

//*****************************************************************************
//
// State structure for each module.
//
//*****************************************************************************
am_hal_pdm_state_t g_am_hal_pdm_states[AM_REG_PDM_NUM_MODULES];

//*****************************************************************************
//
// Static function definitions.
//
//*****************************************************************************
static uint32_t find_dma_threshold(uint32_t ui32TotalCount);

//*****************************************************************************
//
// Initialization function.
//
//*****************************************************************************
uint32_t
am_hal_pdm_initialize(uint32_t ui32Module, void **ppHandle)
{
    //
    // Check that the request module is in range.
    //
    if ( ui32Module >= AM_REG_PDM_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if (!ppHandle)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if (g_am_hal_pdm_states[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Initialize the handle.
    //
    g_am_hal_pdm_states[ui32Module].prefix.s.bInit = true;
    g_am_hal_pdm_states[ui32Module].prefix.s.magic = AM_HAL_MAGIC_PDM;
    g_am_hal_pdm_states[ui32Module].ui32Module = ui32Module;
    g_am_hal_pdm_states[ui32Module].sRegState.bValid = false;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_am_hal_pdm_states[ui32Module];

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// De-Initialization function.
//
//*****************************************************************************
uint32_t
am_hal_pdm_deinitialize(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *)pHandle;

    //
    // Reset the handle.
    //
    pState->prefix.s.bInit = false;
    pState->prefix.s.magic = 0;
    pState->ui32Module = 0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Power control function.
//
//*****************************************************************************
uint32_t
am_hal_pdm_power_control(void *pHandle,
                         am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    am_hal_pwrctrl_periph_e ePDMPowerModule = ((am_hal_pwrctrl_periph_e)
                                               (AM_HAL_PWRCTRL_PERIPH_PDM +
                                                ui32Module));

    //
    // Decode the requested power state and update PDM operation accordingly.
    //
    switch (ePowerState)
    {
        //
        // Turn on the PDM.
        //
        case AM_HAL_SYSCTRL_WAKE:
            //
            // Make sure we don't try to restore an invalid state.
            //
            if (bRetainState && !pState->sRegState.bValid)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable(ePDMPowerModule);

            if (bRetainState)
            {
                //
                // Restore PDM registers
                //
                AM_CRITICAL_BEGIN;

                pState->sRegState.bValid = false;

                AM_CRITICAL_END;
            }
            break;

        //
        // Turn off the PDM.
        //
        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if (bRetainState)
            {
                AM_CRITICAL_BEGIN;

                pState->sRegState.bValid = true;

                AM_CRITICAL_END;
            }

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable(ePDMPowerModule);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_configure(void *pHandle, const am_hal_pdm_config_t *psConfig)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Apply the config structure settings to the PCFG register.
    //
    uint32_t ui32ResMask = ~(PDM_PCFG_LRSWAP_Msk | PDM_PCFG_PGARIGHT_Msk | PDM_PCFG_PGALEFT_Msk |
        PDM_PCFG_MCLKDIV_Msk | PDM_PCFG_SINCRATE_Msk | PDM_PCFG_ADCHPD_Msk | PDM_PCFG_HPCUTOFF_Msk |
        PDM_PCFG_CYCLES_Msk | PDM_PCFG_SOFTMUTE_Msk | PDM_PCFG_PDMCOREEN_Msk);


    uint32_t ui32Temp = PDMn(ui32Module)->PCFG & ui32ResMask;

    ui32Temp |= _VAL2FLD(PDM_PCFG_SOFTMUTE, psConfig->bSoftMute)
                | _VAL2FLD( PDM_PCFG_CYCLES, psConfig->ui32GainChangeDelay)
                | _VAL2FLD( PDM_PCFG_HPCUTOFF, psConfig->ui32HighPassCutoff)
                | _VAL2FLD( PDM_PCFG_ADCHPD, psConfig->bHighPassEnable)
                | _VAL2FLD( PDM_PCFG_SINCRATE, psConfig->ui32DecimationRate)
                | _VAL2FLD( PDM_PCFG_MCLKDIV, psConfig->eClkDivider)
                | _VAL2FLD( PDM_PCFG_PGALEFT, psConfig->eLeftGain)
                | _VAL2FLD( PDM_PCFG_PGARIGHT, psConfig->eRightGain)
                | _VAL2FLD( PDM_PCFG_LRSWAP, psConfig->bLRSwap);


    if (!psConfig->bDoNotStartPdm)
    {
        //
        // Set the PDM Core enable bit to enable PDM to PCM conversions.
        //
        ui32Temp |=  _VAL2FLD(PDM_PCFG_PDMCOREEN, 1);
    }

    PDMn(ui32Module)->PCFG = ui32Temp;

    //
    // Program the "voice" registers.
    //
    ui32ResMask = ~(
        PDM_VCFG_PDMCLKEN_Msk | PDM_VCFG_IOCLKEN_Msk | PDM_VCFG_RSTB_Msk |
        PDM_VCFG_CHSET_Msk | PDM_VCFG_PCMPACK_Msk | PDM_VCFG_SELAP_Msk | PDM_VCFG_DMICKDEL_Msk |
        PDM_VCFG_BCLKINV_Msk | PDM_VCFG_I2SEN_Msk | PDM_VCFG_PDMCLKSEL_Msk);

    ui32Temp = PDMn(ui32Module)->VCFG & ui32ResMask;
    ui32Temp |=
        _VAL2FLD(PDM_VCFG_PDMCLKEN, PDM_VCFG_PDMCLKEN_DIS) |
        _VAL2FLD(PDM_VCFG_IOCLKEN, PDM_VCFG_IOCLKEN_DIS) |
        _VAL2FLD(PDM_VCFG_RSTB, PDM_VCFG_RSTB_RESET) |
        _VAL2FLD(PDM_VCFG_CHSET, psConfig->ePCMChannels) |
        _VAL2FLD(PDM_VCFG_PCMPACK, psConfig->bDataPacking) |
        _VAL2FLD(PDM_VCFG_SELAP, psConfig->ePDMClkSource) |
        _VAL2FLD(PDM_VCFG_DMICKDEL, psConfig->bPDMSampleDelay) |
        _VAL2FLD(PDM_VCFG_BCLKINV, psConfig->bInvertI2SBCLK) |
        _VAL2FLD(PDM_VCFG_I2SEN, psConfig->bI2SEnable) |
        _VAL2FLD(PDM_VCFG_PDMCLKSEL, psConfig->ePDMClkSpeed);

    PDMn(ui32Module)->VCFG = ui32Temp;

    delay_us(100);

    PDMn(ui32Module)->VCFG_b.RSTB = PDM_VCFG_RSTB_NORM;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_enable(void *pHandle)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle)
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->VCFG_b.IOCLKEN = PDM_VCFG_IOCLKEN_EN;
    PDMn(ui32Module)->VCFG_b.PDMCLKEN = PDM_VCFG_PDMCLKEN_EN;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_disable(void *pHandle)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle)
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->VCFG_b.IOCLKEN = PDM_VCFG_IOCLKEN_DIS;
    PDMn(ui32Module)->VCFG_b.PDMCLKEN = PDM_VCFG_PDMCLKEN_DIS;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// set the PDM FIFO Threshold value
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_threshold_setup(void *pHandle, uint32_t value)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOTHR = value;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// Get the DMA Bufffer.
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_get_buffer(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;

    uint32_t ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong)? pState->ui32BufferPing: pState->ui32BufferPong;

    return ui32BufferPtr;
}

//*****************************************************************************
//
// Given the total number of bytes in a DMA transaction, find a reasonable
// threshold setting.
//
//*****************************************************************************
static uint32_t
find_dma_threshold(uint32_t ui32TotalCount)
{
    //
    // Start with a threshold value of 24, and search downward for values that
    // fit our criteria.
    //
    uint32_t ui32Threshold;
    uint32_t ui32Minimum = AM_HAL_PDM_DMA_THRESHOLD_MIN;

    for ( ui32Threshold = 24; ui32Threshold >= ui32Minimum; ui32Threshold -= 4 )
    {
        //
        // With our loop parameters, we've already guaranteed that the
        // threshold will be no higher than 24, and that it will be divisible
        // by 4. The only remaining requirement is that ui32TotalCount must
        // also be divisible by the threshold.
        //
        if ((ui32TotalCount % ui32Threshold) == 0)
        {
            break;
        }
    }

    //
    // If we found an appropriate value, we'll return it here. Otherwise, we
    // will return zero.
    //
    if (ui32Threshold < ui32Minimum)
    {
        ui32Threshold = 0;
    }

    return ui32Threshold;
}

//*****************************************************************************
//
// Sets up, but doesn't start, a DMA transaction
//
//*****************************************************************************
uint32_t
am_hal_dma_param_setup(void *pHandle, am_hal_pdm_transfer_t *pDmaCfg)
{

    AM_HAL_PDM_HANDLE_CHECK(pHandle)

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check for DMA to/from DTCM.
    //
    if ( (pDmaCfg->ui32TargetAddr >= AM_HAL_FLASH_DTCM_START) &&
         (pDmaCfg->ui32TargetAddr <= AM_HAL_FLASH_DTCM_END) )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }
#endif



    pDmaCfg->ui32BufferWithDataAddr = 0;

    pState->ui32BufferPtr = pDmaCfg->ui32TargetAddr;
    pState->ui32BufferPing = pDmaCfg->ui32TargetAddr;
    pState->ui32BufferPong = pDmaCfg->ui32TargetAddrReverse;
    pState->ui32XfrCount   = pDmaCfg->ui32TotalCount;

    //
    // Find an appropriate threshold size for this transfer.
    //
    pState->ui32Threshold = find_dma_threshold(pDmaCfg->ui32TotalCount);

    //
    // If we didn't find a threshold that will work, throw an error.
    //
    if (pState->ui32Threshold == 0)
    {
        return AM_HAL_PDM_STATUS_BAD_TOTALCOUNT;
    }

    return AM_HAL_STATUS_SUCCESS;

}

//*****************************************************************************
//
// Starts a DMA transaction from the PDM directly to SRAM using previous parameters
//
//*****************************************************************************
uint32_t
am_hal_dma_restart(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOTHR = pState->ui32Threshold;

    //
    // Configure DMA.
    //
    PDMn(ui32Module)->DMACFG = 0;
    //
    //! flush the fifo
    //
    PDMn(ui32Module)->FIFOFLUSH = 1;

    //
    // Configure DMA.
    //
    PDMn(ui32Module)->DMACFG_b.DMAPRI = PDM_DMACFG_DMAPRI_HIGH;
    PDMn(ui32Module)->DMACFG_b.DMADIR = PDM_DMACFG_DMADIR_P2M;
    PDMn(ui32Module)->DMATOTCOUNT = pState->ui32XfrCount;
    PDMn(ui32Module)->DMATARGADDR = pState->ui32BufferPtr;
    PDMn(ui32Module)->DMASTAT = 0;

    //
    // Make sure the trigger is set for threshold.
    //
    PDMn(ui32Module)->DMATRIGEN_b.DTHR = 1;

    //
    // clear all interrupts
    //
    PDMn(ui32Module)->INTCLR = PDMn(ui32Module)->INTSTAT;

    //
    // Enable DMA
    //
    PDMn(ui32Module)->DMACFG_b.DMAEN = PDM_DMACFG_DMAEN_EN;

    //
    // Set the PDM Core enable bit to enable PDM to PCM conversions.
    //
    PDMn(ui32Module)->PCFG_b.PDMCOREEN = PDM_PCFG_PDMCOREEN_EN;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Starts a DMA transaction from the PDM directly to SRAM
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_start(void *pHandle, am_hal_pdm_transfer_t *pDmaCfg)
{
    uint32_t ui32Status = am_hal_dma_param_setup(pHandle, pDmaCfg);
    if (ui32Status)
    {
        return ui32Status;
    }

    return am_hal_dma_restart(pHandle);
}


//*****************************************************************************
//
// Flush the PDM FIFO
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_flush(void *pHandle)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOFLUSH = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Enable PDM passthrough to the I2S slave.
//
//*****************************************************************************
uint32_t
am_hal_pdm_i2s_enable(void *pHandle)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;

    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->VCFG_b.I2SEN = PDM_VCFG_I2SEN_EN;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Disable PDM passthrough to the I2S slave.
//
//*****************************************************************************
uint32_t
am_hal_pdm_i2s_disable(void *pHandle)
{
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->VCFG_b.I2SEN = PDM_VCFG_I2SEN_DIS;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// PDM interrupt service assistant, for dma transfers
//
//*****************************************************************************
uint32_t am_hal_pdm_interrupt_service(void *pHandle,
                                      am_hal_interupt_service_msg_t *ptIsrData,
                                      am_hal_pdm_transfer_t *pTransferCfg)
{

    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    
    uint32_t ui32Module = pState->ui32Module;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    PDM_Type *pPdm  = PDMn(ui32Module);

    uint32_t ui32IntStatus =  pPdm->INTSTAT;
    ui32IntStatus &= pPdm->INTEN;  // apply interrupt mask
    ptIsrData->isrStat = ui32IntStatus;
    ptIsrData->dmaStat = pPdm->DMASTAT;

    if (ui32IntStatus & AM_HAL_PDM_INT_OVF)
    {


        uint32_t dmaCfg = pPdm->DMACFG;
        // stop dma
        pPdm->DMACFG_b.DMAEN = 0;

        //
        // fifo overflow
        //
        pPdm->FIFOFLUSH = 1;

        //
        // clear overflow status
        //

        pPdm->INTCLR_b.OVF = 1;

        pPdm->DMACFG = dmaCfg;

        ui32Status = AM_HAL_STATUS_FAIL;
    } // fifo overflow

    if ( ui32IntStatus & AM_HAL_PDM_INT_DERR)
    {
        //
        // dma error
        //
        uint32_t dmaCfg = pPdm->DMACFG;
        //
        // stop dma
        //
        pPdm->DMACFG_b.DMAEN = 0;

        pPdm->DMASTAT = ptIsrData->dmaStat & 0x01;

        pPdm->INTCLR_b.DERR = 1;  // clear interrupt

        pPdm->DMACFG = dmaCfg;

        ui32Status = AM_HAL_STATUS_FAIL;
    } // dma error

    if ( ui32IntStatus & AM_HAL_PDM_INT_UNDFL)
    {
        pPdm->INTCLR_b.UNDFL = 1;

    } // underflow

    if (ui32IntStatus & AM_HAL_PDM_INT_DCMP)
    {
        //
        // dma complete
        //

        //
        // disable dma first
        //
        pPdm->DMACFG_b.DMAEN = PDM_DMACFG_DMAEN_DIS;

        //
        // swap the buffer.
        //
        uint32_t currBuffer = pState->ui32BufferPtr;
        pTransferCfg->ui32BufferWithDataAddr = pState->ui32BufferPtr;  // save address of buffer that has data

        pState->ui32BufferPtr = (currBuffer == pState->ui32BufferPong) ? pState->ui32BufferPing: pState->ui32BufferPong;

        pPdm->DMATARGADDR = pState->ui32BufferPtr ;
        pPdm->DMATOTCOUNT = pTransferCfg->ui32TotalCount; // reset dma count

        pPdm->INTCLR_b.DCMP = 1; // clear interrupt

        if ( pTransferCfg->bRestartDMA)
        {
            //
            // want continuous transfers, re-enable DMA
            //
            pPdm->DMACFG_b.DMAEN = PDM_DMACFG_DMAEN_EN;
        }
        else 
        {
            //
            // disable DMA (done above) and streaming
            //
            pPdm->PCFG_b.PDMCOREEN = 0 ;
        }

    } // dma complete


    return ui32Status;
}

//*****************************************************************************
//
// Interrupt enable.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_enable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTEN |= ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt disable.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTEN &= ~ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt clear.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->INTCLR = ui32IntMask;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_pdm_interrupt_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // If requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32Status = PDMn(ui32Module)->INTSTAT;
        *pui32Status = ui32Status & PDMn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = PDMn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read the FIFO.
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_data_read(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->FIFOREAD;
}

//*****************************************************************************
//
// Read the FIFOs.
//
//*****************************************************************************
uint32_t am_hal_pdm_fifo_data_reads(void *pHandle, uint8_t* buffer, uint32_t size)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    uint32_t fiforead;

    uint8_t buf_size = 0;
    for ( uint32_t i = 0; i < size; i++ )
    {
        {
            fiforead = PDMn(ui32Module)->FIFOREAD; //left/right channel
            buffer[buf_size++] = fiforead & 0xFF;
            buffer[buf_size++] = (fiforead & 0xFF00)>>8;
            buffer[buf_size++] = (fiforead & 0x00FF0000)>>16;
        }
    }

    return 0;
}

//*****************************************************************************
//
// am_hal_pdm_fifo_count_get
//
//*****************************************************************************
uint32_t am_hal_pdm_fifo_count_get(void *pHandle)
{
    // this is not available on apollo3
    return 0;
}

//*****************************************************************************
//
// am_hal_pdm_dma_state
//
//*****************************************************************************
uint32_t am_hal_pdm_dma_state(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->DMASTAT;
}

//*****************************************************************************
//
// disable PDM DMA
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_disable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // disable dma interrupts
    //
    PDMn(ui32Module)->INTEN &= ~(AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP);

    //
    // clear interrupts
    //
    PDMn(ui32Module)->INTCLR = (AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP);

    PDMn(ui32Module)->DMATOTCOUNT = 0;


    return AM_HAL_STATUS_SUCCESS;
}

// End Doxygen group.
//! @}
//
//*****************************************************************************
