//*****************************************************************************
//
//! @file am_hal_pdm.c
//!
//! @brief HAL implementation for the PDM module.
//!
//! @addtogroup pdm PDM - Pulse Density Modulation
//! @ingroup apollo510L_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
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
// This is part of revision release_sdk5_2_a_0-438c93f352 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "mcu/am_hal_crm_private.h"

//*****************************************************************************
//
//! @name PDM magic number for handle verification
//! @{
//
//*****************************************************************************
#define AM_HAL_MAGIC_PDM                0xF956E2

#define AM_HAL_PDM_HANDLE_VALID(h)                                            \
    ((h) &&                                                                   \
    (((am_hal_handle_prefix_t *)(h))->s.bInit) &&                             \
    (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_PDM))
//! @}

//*****************************************************************************
//
//! Abbreviation for validating handles and returning errors
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
//! @def USE_PDM_TWO_STAGE_DMA
//! @brief Enable the two-stage DMA request pipeline feature.
//!
//! This feature is added to ease SW ISR latency requirement to reload next
//! DMA request when the current DMA request is completed.
//!
//! To disable the 2-stage DMA feature, undefine this macro. The user needs to
//! ensure that the response time of the DMACPL ISR is less than the time it
//! takes for the PDM FIFO (typically with a size of 16 words) to become filled,
//! or else data loss may occur.
//!
//! To enable the 2-stage DMA feature, define ths macro. The user has a longer
//! response time for the DMACPL ISR, which is approximately equal to the time
//! it takes to fill the entire DMA buffer.
//
//*****************************************************************************
#ifndef AM_HAL_DISABLE_PDM_TWO_STAGE_DMA
#define USE_PDM_TWO_STAGE_DMA
#else
#undef USE_PDM_TWO_STAGE_DMA
#endif

//*****************************************************************************
//
// Extract CRM clock source and clock divider ratio.
//
//*****************************************************************************
#define EXTRACT_PDM_CRM_CLKSRC(x)   (((x) & AM_HAL_PDM_CRM_CLKSRC_MSK) >> AM_HAL_PDM_CRM_CLKSRC_POS)
#define EXTRACT_PDM_CRM_CLKDIV(x)   (((x) & AM_HAL_PDM_CRM_CLKDIV_MSK) >> AM_HAL_PDM_CRM_CLKDIV_POS)

//*****************************************************************************
//
//! Is Register State Valid
//
//*****************************************************************************
typedef struct
{
    bool bValid;
}
am_hal_pdm_register_state_t;

//*****************************************************************************
//
//! Structure for handling PDM HAL state information.
//
//*****************************************************************************
typedef struct
{
    am_hal_handle_prefix_t prefix;
    am_hal_pdm_register_state_t sRegState;
    uint32_t ui32Module;

    //
    //! DMA transaction Tranfer Control Buffer.
    //
    uint32_t            ui32BufferPing;
    uint32_t            ui32BufferPong;
    uint32_t            ui32BufferPtr;
    uint32_t            ui32BufferSizeBytes;

    //
    //! Selected Clock Source
    //
    am_hal_pdm_clkspd_e eClockSel;
    bool                bModuleEnabledBeforePowerdown;
}
am_hal_pdm_state_t;

//*****************************************************************************
//
//! Structure for handling PDM register state information for power up/down
//
//*****************************************************************************
am_hal_pdm_state_t g_PDMhandles[AM_REG_PDM_NUM_MODULES];

//*****************************************************************************
//
//! @brief [Internal] Get the CLKMGR ID from the clock selection
//!
//! @param eClocksel - Clock selection in am_hal_pdm_clkspd_e.
//!
//! @return clockId  - Clock ID in clock manager.
//
//*****************************************************************************
static am_hal_clkmgr_clock_id_e
pdm_clock_id_get(am_hal_pdm_clkspd_e eClocksel)
{
    switch (eClocksel)
    {
        case AM_HAL_PDM_CLK_PLL:
        case AM_HAL_PDM_CLK_PLL_DIV16:
        case AM_HAL_PDM_CLK_PLL_DIV48:
        case AM_HAL_PDM_CLK_PLL_FOUT4:
            return AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV;

        case AM_HAL_PDM_CLK_HFXTAL:
        case AM_HAL_PDM_CLK_HFXTAL_DIV2:
        case AM_HAL_PDM_CLK_HFXTAL_DIV4:
            return AM_HAL_CLKMGR_CLK_ID_XTAL_HS;

        case AM_HAL_PDM_CLK_HFRC_24MHZ:
        case AM_HAL_PDM_CLK_HFRC_1P5MHZ:
        case AM_HAL_PDM_CLK_HFRC_0P5MHZ:
            return AM_HAL_CLKMGR_CLK_ID_HFRC;

        case AM_HAL_PDM_CLK_EXTREF:
            return AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK;

        case AM_HAL_PDM_CLK_OFF:
        default:
            return AM_HAL_CLKMGR_CLK_ID_MAX;
    }
}

//*****************************************************************************
//
//! @brief [Internal] Set PDM moudule clock
//!
//! @param ui32Module - PDM instance index.
//! @param eClocksel  - Clock selection in am_hal_pdm_clkspd_e.
//!
//! @return status    - generic or interface specific status.
//
//*****************************************************************************
static uint32_t
pdm_clock_set(uint32_t ui32Module, am_hal_pdm_clkspd_e eClocksel)
{
    uint32_t ui32Status;
    am_hal_clkmgr_user_id_e eUserID = AM_HAL_CLKMGR_USER_ID_PDM0;
    // Array to store the clock ID for each suceessful request.
    static am_hal_clkmgr_clock_id_e eStoredClockId[AM_REG_PDM_NUM_MODULES] = {AM_HAL_CLKMGR_CLK_ID_MAX};

    if (eClocksel != AM_HAL_PDM_CLK_OFF)
    {
        //
        // Request clock from the clock manager.
        //
        am_hal_clkmgr_clock_id_e eClockID = pdm_clock_id_get(eClocksel);
        ui32Status = am_hal_clkmgr_clock_request(eClockID, eUserID);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        //
        // Set and enable clock via CRM.
        //
        ui32Status = am_hal_crm_clock_config_PDM0CLK(EXTRACT_PDM_CRM_CLKSRC(eClocksel), EXTRACT_PDM_CRM_CLKDIV(eClocksel));
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_hal_clkmgr_clock_release(eClockID, eUserID);
            return ui32Status;
        }

        ui32Status = am_hal_crm_control_PDM0CLK_CLOCK_SET(true);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_hal_clkmgr_clock_release(eClockID, eUserID);
            return ui32Status;
        }

        //
        // Store the clock ID if everything is successful.
        //
        eStoredClockId[ui32Module] = eClockID;
    }
    else
    {
        //
        // Gate clock.
        //
        am_hal_crm_control_PDM0CLK_CLOCK_SET(false);

        //
        // Release the clock if it was previously requested.
        //
        am_hal_clkmgr_clock_release(eStoredClockId[ui32Module], eUserID);
        eStoredClockId[ui32Module] = AM_HAL_CLKMGR_CLK_ID_MAX;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// am_hal_pdm_initialize
//
//*****************************************************************************
uint32_t
am_hal_pdm_initialize(uint32_t ui32Module, void **ppHandle)
{
    //
    // Check that the request module is in range.
    //
    if (ui32Module >= AM_REG_PDM_NUM_MODULES)
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
    if (g_PDMhandles[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    //
    // Initialize the handle.
    //
    g_PDMhandles[ui32Module].prefix.s.bInit = true;
    g_PDMhandles[ui32Module].prefix.s.magic = AM_HAL_MAGIC_PDM;
    g_PDMhandles[ui32Module].ui32Module = ui32Module;
    g_PDMhandles[ui32Module].sRegState.bValid = false;
    g_PDMhandles[ui32Module].bModuleEnabledBeforePowerdown = false;

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_PDMhandles[ui32Module];

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
    pState->bModuleEnabledBeforePowerdown = false;

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

    uint32_t ui32Status;
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    am_hal_pwrctrl_periph_e ePDMPowerModule =
        (am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_PDM0 + ui32Module);

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

            //
            // Enable APB clock.
            //
            am_hal_crm_control_PDM0_CLOCK_SET(true);

            if (bRetainState)
            {
                //
                // Restore PDM registers
                //
                AM_CRITICAL_BEGIN;

                if (pState->bModuleEnabledBeforePowerdown)
                {
                    ui32Status = am_hal_pdm_enable(pHandle);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        am_hal_pwrctrl_periph_disable(ePDMPowerModule);
                        return ui32Status;
                    }
                }

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

            if (pState->prefix.s.bEnable == true)
            {
                //
                // Record enabled state and disable PDM.
                //
                pState->bModuleEnabledBeforePowerdown = true;
                am_hal_pdm_disable(pHandle);
            }

            //
            // Disable APB clock.
            //
            am_hal_crm_control_PDM0_CLOCK_SET(false);

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
am_hal_pdm_configure(void *pHandle, am_hal_pdm_config_t *psConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Apply the config structure settings to the CORECFG0 register.
    //
    PDMn(ui32Module)->CORECFG0_b.LRSWAP = psConfig->bLRSwap;
    PDMn(ui32Module)->CORECFG0_b.SOFTMUTE = psConfig->bSoftMute;
    // Set number of PDMA_CKO cycles during gain setting changes or soft mute
    PDMn(ui32Module)->CORECFG0_b.SCYCLES = psConfig->ui32GainChangeDelay;

    PDMn(ui32Module)->CORECFG0_b.HPGAIN = psConfig->ui32HighPassCutoff;
    PDMn(ui32Module)->CORECFG0_b.ADCHPD = psConfig->bHighPassEnable;

    PDMn(ui32Module)->CORECFG0_b.PGAL = psConfig->eLeftGain;
    PDMn(ui32Module)->CORECFG0_b.PGAR = psConfig->eRightGain;

    //
    // Program the "CORECFG1_b" registers.
    //
    PDMn(ui32Module)->CORECFG1_b.PCMCHSET = psConfig->ePCMChannels;
    // PDMA_CKO clock phase delay in terms of PDMCLK period to internal sampler
    PDMn(ui32Module)->CORECFG1_b.CKODLY = psConfig->bPDMSampleDelay;
    // Fine grain step size for smooth PGA or Softmute attenuation transition
    PDMn(ui32Module)->CORECFG1_b.SELSTEP = psConfig->eStepSize;

    //
    // Set the PDM Control register.
    //
    // PDMn(ui32Module)->CTRL_b.CLKEN = 0;
    // PDMn(ui32Module)->CTRL_b.PCMPACK = psConfig->bDataPacking;
    PDMn(ui32Module)->CTRL_b.CLKGATEMODE = 1;

    bool bI2sOn = false;
    switch (psConfig->eDataFlowDirection)
    {
        case AM_HAL_PDM_DATA_FLOW_TO_MEMORY:
            PDMn(ui32Module)->CORECFG1_b.I2SON = 0;
            PDMn(ui32Module)->CORECFG1_b.PCMON = 1;
            bI2sOn = false;
            break;
        case AM_HAL_PDM_DATA_FLOW_TO_I2S:
            PDMn(ui32Module)->CORECFG1_b.I2SON = 1;
            PDMn(ui32Module)->CORECFG1_b.PCMON = 0;
            bI2sOn = true;
            break;
        case AM_HAL_PDM_DATA_FLOW_TO_BOTH:
            PDMn(ui32Module)->CORECFG1_b.I2SON = 1;
            PDMn(ui32Module)->CORECFG1_b.PCMON = 1;
            bI2sOn = true;
            break;
    }

    if (bI2sOn)
    {
        if (psConfig->bI2sMaster)
        {
            PDMn(ui32Module)->CTRL_b.I2SMASTERMODE = 1;

            if (psConfig->ui32DecimationRate == 32)
            {
                PDMn(ui32Module)->CTRL_b.I2SNUMBCLKSINHALFWDCLK = 0;
            }
            else if (psConfig->ui32DecimationRate == 24)
            {
                PDMn(ui32Module)->CTRL_b.I2SNUMBCLKSINHALFWDCLK = 1;
            }
            else
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
        }
        else
        {
            PDMn(ui32Module)->CTRL_b.I2SMASTERMODE = 0;
        }
    }

    if (psConfig->eWordWidth == AM_HAL_PDM_DATA_WORD_WIDTH_16BITS)
    {
        PDMn(ui32Module)->DMACFG_b.DMAPACKED = 1;
    }
    else
    {
        PDMn(ui32Module)->DMACFG_b.DMAPACKED = 0;
    }

    pState->eClockSel = psConfig->ePDMClkSpeed;

    PDMn(ui32Module)->CORECFG1_b.CASES = 0;
    PDMn(ui32Module)->CORECFG0_b.MCLKDIV = psConfig->ePDMAClkOutDivder;
    PDMn(ui32Module)->CORECFG1_b.DIVMCLKQ = psConfig->eClkDivider;

    bool bHpOn;
    if (bI2sOn == true)
    {
        bHpOn = false;
    }
    else
    {
        bHpOn = ((psConfig->ui32DecimationRate == 25) || (psConfig->ui32DecimationRate == 50)) ? false : true;
    }

    PDMn(ui32Module)->CORECFG1_b.HPON     = (bHpOn == true) ? 1 : 0;
    PDMn(ui32Module)->CORECFG0_b.SINCRATE = (bHpOn == true) ? (psConfig->ui32DecimationRate) : (psConfig->ui32DecimationRate * 2);

    PDMn(ui32Module)->CTRL_b.RSTB = 0;

    am_hal_delay_us(100);

    PDMn(ui32Module)->CTRL_b.RSTB = 1;

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
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    uint32_t ui32Status;
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Request HFRC for APBDMA.
    //
    ui32Status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_PDM0);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Set PDM module clock.
    //
    ui32Status = pdm_clock_set(ui32Module, pState->eClockSel);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Reset PDM module
    //
    PDMn(ui32Module)->CTRL_b.RSTB = 0;
    am_hal_delay_us(100);
    PDMn(ui32Module)->CTRL_b.RSTB = 1;

    //
    // Enable PDM Module
    //
    PDMn(ui32Module)->CTRL_b.EN = 1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Reset the PDM.
//
//*****************************************************************************
uint32_t
am_hal_pdm_reset(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->CTRL_b.RSTB = 1;

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
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Release HFRC for APBDMA.
    //
    am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, AM_HAL_CLKMGR_USER_ID_PDM0);

    //
    // Gate PDM module clock.
    //
    pdm_clock_set(ui32Module, AM_HAL_PDM_CLK_OFF);

    //
    // Disable PDM
    //
    PDMn(ui32Module)->CTRL_b.EN = 0;

    return AM_HAL_STATUS_SUCCESS;
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

    for (ui32Threshold = 24; ui32Threshold >= ui32Minimum; ui32Threshold -= 4)
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
// Starts a DMA transaction from the PDM directly to SRAM
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_start(void *pHandle, am_hal_pdm_transfer_t *pTransferCfg)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Configure Pingpong Buffer
    //
    pState->ui32BufferPtr  = \
    pState->ui32BufferPing = pTransferCfg->ui32TargetAddr;
    pState->ui32BufferPong = pTransferCfg->ui32TargetAddrReverse;
    pState->ui32BufferSizeBytes = pTransferCfg->ui32TotalCount;

    //
    // Find an appropriate threshold size for this transfer.
    //
    uint32_t ui32Threshold = find_dma_threshold(pTransferCfg->ui32TotalCount);

    //
    // If we didn't find a threshold that will work, throw an error.
    //
    if (ui32Threshold == 0)
    {
        return AM_HAL_PDM_STATUS_BAD_TOTALCOUNT;
    }

    PDMn(ui32Module)->FIFOTHR = ui32Threshold;

    //
    // Reset DMA status, write 1 to clear.
    //
    PDMn(ui32Module)->DMASTAT = 0xFFFFFFFF;

    //
    // Enable DMA
    //
    PDMn(ui32Module)->DMACFG_b.DMAEN = PDM0_DMACFG_DMAEN_EN;

    //
    // Configure DMA.
    //
    // If using 2-stage DMA, need to enable the NEXTDMAEN bit first. After writing
    // the address and size into the DMATARGADDRNEXT and DMATOTCOUNTNEXT registers
    // and enabling the DMAENNEXTCTRL bit, the hardware will automatically load the
    // contents of the DMATARGADDRNEXT and DMATOTCOUNTNEXT registers into the DMATARGADDR
    // and DMATOTCOUNT registers. Once the loading is complete, the DMAENNEXTCTRL
    // bit will be automatically cleared.
    // Note: automatic loading occurs only when the DMAEN bit is set to 1 and
    // the DMAENNEXTCTRL bit is not equal to 0.
    //
    #ifdef USE_PDM_TWO_STAGE_DMA
    PDMn(ui32Module)->DMACFG_b.NEXTDMAEN = 1;
    PDMn(ui32Module)->DMATARGADDRNEXT = pState->ui32BufferPing;
    PDMn(ui32Module)->DMATOTCOUNTNEXT = pState->ui32BufferSizeBytes;
    PDMn(ui32Module)->DMAENNEXTCTRL = 1;
    #else
    PDMn(ui32Module)->DMATARGADDR = pState->ui32BufferPing;
    PDMn(ui32Module)->DMATOTCOUNT = pState->ui32BufferSizeBytes;
    #endif

    //
    // Make sure the trigger is set for threshold.
    //
    PDMn(ui32Module)->DMATRIGEN_b.DTHR = 1;


    //
    // If a pong buffer is provided, and the USE_PDM_TWO_STAGE_DMA macro is defined,
    // configure the second DMA buffer immediately after the configuration of the
    // first DMA buffer is completed.
    //
    #ifdef USE_PDM_TWO_STAGE_DMA
    // "Pong = 0xFFFFFFFF" means that only the ping buffer works, otherwise, set the pong buffer to the next stage of DMA.
    if (pState->ui32BufferPong != 0xFFFFFFFF)
    {
        am_hal_delay_us(10);

        if (PDMn(ui32Module)->DMAENNEXTCTRL == 0)
        {
            pState->ui32BufferPtr = pState->ui32BufferPong;
            PDMn(ui32Module)->DMATARGADDRNEXT = pState->ui32BufferPong;
            PDMn(ui32Module)->DMATOTCOUNTNEXT = pState->ui32BufferSizeBytes;
            PDMn(ui32Module)->DMAENNEXTCTRL = 1;
        }
        else
        {
            return AM_HAL_STATUS_HW_ERR;
        }
    }
    #endif

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Stop DMA transfer.
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_stop(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Disable DMA.
    //
    PDMn(ui32Module)->DMACFG_b.DMAEN = PDM0_DMACFG_DMAEN_DIS;

    //
    // Clear TOTCOUNT register.
    //
    #ifdef USE_PDM_TWO_STAGE_DMA
    PDMn(ui32Module)->DMATOTCOUNTNEXT = 0;
    PDMn(ui32Module)->DMATOTCOUNT     = 0;
    #else
    PDMn(ui32Module)->DMATOTCOUNT     = 0;
    #endif

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Flush the PDM FIFO
//
//*****************************************************************************
uint32_t
am_hal_pdm_fifo_flush(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_PDM_HANDLE_CHECK(pHandle);

    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    PDMn(ui32Module)->FIFOFLUSH = 1;

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
// PDM interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_pdm_interrupt_service(void *pHandle, uint32_t ui32IntMask, am_hal_pdm_transfer_t *pTransferCfg)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    if (ui32IntMask & AM_HAL_PDM_INT_OVF)
    {
        am_hal_pdm_fifo_flush(pHandle);
    }

    if ((ui32IntMask & AM_HAL_PDM_INT_DCMP) && (pState->ui32BufferPong != 0xFFFFFFFF))
    {
        #ifdef USE_PDM_TWO_STAGE_DMA
        PDMn(ui32Module)->DMASTAT_b.DMACPL = 1;
        if (PDMn(ui32Module)->DMAENNEXTCTRL == 0)
        {
            PDMn(ui32Module)->DMATARGADDRNEXT = pState->ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong) ? pState->ui32BufferPing : pState->ui32BufferPong;
            PDMn(ui32Module)->DMATOTCOUNTNEXT = pState->ui32BufferSizeBytes;
            PDMn(ui32Module)->DMAENNEXTCTRL = 1;
        }
        else
        {
            return AM_HAL_STATUS_HW_ERR;
        }
        #else // !USE_PDM_TWO_STAGE_DMA
        PDMn(ui32Module)->DMATARGADDR = pState->ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong) ? pState->ui32BufferPing : pState->ui32BufferPong;
        PDMn(ui32Module)->DMATOTCOUNT = pState->ui32BufferSizeBytes;
        #endif // !USE_PDM_TWO_STAGE_DMA
    }


    return AM_HAL_STATUS_SUCCESS;
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
    *(volatile uint32_t*)(&PDMn(ui32Module)->INTSTAT);
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
    if (bEnabledOnly)
    {
        *pui32Status = PDMn(ui32Module)->INTSTAT;
        *pui32Status &= PDMn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = PDMn(ui32Module)->INTSTAT;
    }

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
    uint32_t ui32BufferPtr;
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;

    //
    // "Pong = 0xFFFFFFFF" means that only the ping buffer works, just return the ping buffer.
    //
    if (pState->ui32BufferPong == 0xFFFFFFFF)
    {
        ui32BufferPtr = pState->ui32BufferPing;
    }
    else
    {
        #ifdef USE_PDM_TWO_STAGE_DMA
        ui32BufferPtr = pState->ui32BufferPtr;
        #else
        ui32BufferPtr = (pState->ui32BufferPtr == pState->ui32BufferPong) ? pState->ui32BufferPing : pState->ui32BufferPong;
        #endif
    }

    return ui32BufferPtr;
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
    uint32_t buf_size = 0;

    for (uint32_t i = 0; i < size; i++)
    {
        fiforead = PDMn(ui32Module)->FIFOREAD; //left/right channel
        buffer[buf_size++] =  fiforead & 0xFF;
        buffer[buf_size++] = (fiforead & 0xFF00)>>8;
        buffer[buf_size++] = (fiforead & 0x00FF0000)>>16;
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
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    return PDMn(ui32Module)->FIFOCNT;
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
// am_hal_pdm_dma_transfer_continue
//
//*****************************************************************************
uint32_t
am_hal_pdm_dma_transfer_continue(void *pHandle, am_hal_pdm_transfer_t *pTransferCfg)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // The continue() API doesn't apply to the ping-pong mechanism.
    //
    if (pState->ui32BufferPong != 0xFFFFFFFF)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    #ifdef USE_PDM_TWO_STAGE_DMA
    if (PDMn(ui32Module)->DMAENNEXTCTRL == 0)
    {
        PDMn(ui32Module)->DMATARGADDRNEXT = pState->ui32BufferPtr       = pTransferCfg->ui32TargetAddr;
        PDMn(ui32Module)->DMATOTCOUNTNEXT = pState->ui32BufferSizeBytes = pTransferCfg->ui32TotalCount;
        PDMn(ui32Module)->DMAENNEXTCTRL = 1;
    }
    else
    {
        return AM_HAL_STATUS_HW_ERR;
    }
    #else // !USE_PDM_TWO_STAGE_DMA
    PDMn(ui32Module)->DMATARGADDR = pState->ui32BufferPtr       = pTransferCfg->ui32TargetAddr;
    PDMn(ui32Module)->DMATOTCOUNT = pState->ui32BufferSizeBytes = pTransferCfg->ui32TotalCount;
    #endif // !USE_PDM_TWO_STAGE_DMA

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
