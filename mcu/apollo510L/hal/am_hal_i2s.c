//*****************************************************************************
//
//! @file am_hal_i2s.c
//!
//! @brief HAL implementation for the I2S module.
//!
//! @addtogroup i2s4_ap510L I2S - Inter-IC Sound
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides functions for configuring, controlling, and
//! managing the Inter-IC Sound (I2S) peripheral on Apollo5 devices. It supports
//! audio data transfer, clock configuration, DMA, and interrupt handling for
//! digital audio applications.
//!
//! @section hal_i2s_features Key Features
//!
//! 1. @b Audio @b Data @b Transmission: Support for I2S audio data transfer.
//! 2. @b DMA @b Support: High-speed DMA-based audio streaming.
//! 3. @b Clock @b Configuration: Flexible clock source and divider settings.
//! 4. @b Format @b Support: Multiple I2S data formats and word lengths.
//! 5. @b Interrupt @b Handling: Comprehensive interrupt management for I2S events.
//!
//! @section hal_i2s_functionality Functionality
//!
//! - Initialize and configure the I2S peripheral
//! - Set up audio data transmission and reception
//! - Configure and manage DMA transfers for audio streaming
//! - Handle I2S interrupts and status monitoring
//! - Support for various I2S data formats and clocking options
//!
//! @section hal_i2s_usage Usage
//!
//! 1. Initialize I2S using am_hal_i2s_initialize()
//! 2. Configure I2S parameters and clock settings
//! 3. Set up DMA or interrupt-driven audio streaming
//! 4. Start and stop audio data transfer as needed
//! 5. Handle I2S events and monitor status
//!
//! @section hal_i2s_configuration Configuration
//!
//! - @b Clock @b Source: Select and configure I2S clock source
//! - @b Data @b Format: Set up I2S data format and word length
//! - @b DMA @b Settings: Configure DMA transfer parameters
//! - @b Interrupts: Set up interrupt sources and handlers
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
// This is part of revision release_sdk5_2_a_1-29944d3085 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "am_mcu_apollo.h"
#include "mcu/am_hal_crm_private.h"

//*****************************************************************************
//
//! @name I2S magic number macros for handle verification.
//! @{
//
//*****************************************************************************
#define AM_HAL_MAGIC_I2S            0x125125
#define AM_HAL_I2S_HANDLE_VALID(h)                                            \
    ((h) &&                                                                   \
    ((am_hal_handle_prefix_t *)(h))->s.bInit &&                               \
    (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_I2S))
//! @}

//*****************************************************************************
//
//! Abbreviation for validating handles and returning errors
//
//*****************************************************************************
#ifndef AM_HAL_DISABLE_API_VALIDATION
#define AM_HAL_I2S_CHK_HANDLE(h)                                              \
    if (!AM_HAL_I2S_HANDLE_VALID(h))                                          \
    {                                                                         \
        return AM_HAL_STATUS_INVALID_HANDLE;                                  \
    }
#else
#define AM_HAL_I2S_CHK_HANDLE(h)
#endif // AM_HAL_DISABLE_API_VALIDATION

//*****************************************************************************
//
//! @def USE_I2S_TWO_STAGE_DMA
//! @brief Enable the two-stage DMA request pipeline feature.
//!
//! This feature is added to ease SW ISR latency requirement to reload next
//! DMA request when the current DMA request is completed.
//!
//! To disable the 2-stage DMA feature, undefine this macro. The user needs to
//! ensure that the response time of the DMACPL ISR is less than the time it
//! takes for the I2S FIFO (typically with a size of 64 words) to become filled,
//! or else data loss may occur.
//!
//! To enable the 2-stage DMA feature, define ths macro. The user has a longer
//! response time for the DMACPL ISR, which is approximately equal to the time
//! it takes to fill the entire DMA buffer.
//
//*****************************************************************************
#ifndef AM_HAL_DISABLE_I2S_TWO_STAGE_DMA
#define USE_I2S_TWO_STAGE_DMA
#else
#undef  USE_I2S_TWO_STAGE_DMA
#endif

//*****************************************************************************
//
// Extract the mux selection from a given clock.
//
//*****************************************************************************
#define EXTRACT_CRM_MUX(x)     ((x & AM_HAL_I2S_CLK_CRM_MUX_MSK) >> AM_HAL_I2S_CLK_CRM_MUX_POS)
#define EXTRACT_NCO_MUX(x)     ((x & AM_HAL_I2S_CLK_NCO_MUX_MSK) >> AM_HAL_I2S_CLK_NCO_MUX_POS)

//*****************************************************************************
//
//! Structure for I2S registers.
//
//*****************************************************************************
typedef struct
{
    bool        bValid;

    uint32_t    regI2SCTL;
    uint32_t    regI2SDATACFG;
    uint32_t    regI2SIOCFG;
    uint32_t    regAMQCFG;
    uint32_t    regI2SCLKCFG;
    uint32_t    regIPBIRPT;
    uint32_t    regRXUPPERLIMIT;
    uint32_t    regTXLOWERLIMIT;
    uint32_t    regINTEN;
    uint32_t    regI2SDMACFG;
    uint32_t    regINTDIV;
    uint32_t    regFRACDIV;
} am_hal_i2s_register_state_t;

//*****************************************************************************
//
//! Structure for handling I2S HAL state information.
//
//*****************************************************************************
typedef struct
{
    am_hal_handle_prefix_t           prefix;

    uint32_t                         ui32Module;

    am_hal_i2s_register_state_t      sRegState;
    //
    // DMA transaction Tranfer Control Buffer.
    //
    uint32_t                ui32RxBufferPing;
    uint32_t                ui32RxBufferPong;
    uint32_t                ui32TxBufferPing;
    uint32_t                ui32TxBufferPong;
    //
    // Current DMA buffer ptr.
    //
    uint32_t                ui32RxBufferPtr;
    uint32_t                ui32TxBufferPtr;
    //
    // DMA buffer size in bytes
    //
    uint32_t                ui32RxBufferSizeBytes;
    uint32_t                ui32TxBufferSizeBytes;

    am_hal_i2s_clksel_e     eClockSel;
    uint32_t                ui32ClockDivideRatio;

    am_hal_i2s_mclkout_sel_e eMclkout;
    uint32_t                ui32MclkoutDiv;
    bool                    bInternalMclkRequired;
    bool                    bModuleEnabledBeforePowerdown;

    //! Store application's settings.
    am_hal_i2s_data_format_t sDataFormat;
    am_hal_i2s_io_signal_t   sIoConfig;
}am_hal_i2s_state_t;

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
am_hal_i2s_state_t          g_I2Shandles[AM_REG_I2S_NUM_MODULES];

//*****************************************************************************
//
//! @brief [Internal] Get the CLKMGR ID from the clock selection
//!
//! @param ui32Clocksel - Clock selection in am_hal_i2s_clksel_e or am_hal_i2s_mclkout_sel_e.
//! @param bIsMclkout   - Specifies if ui32Clocksel is am_hal_i2s_mclkout_sel_e, true for yes.
//!
//! @return clockId     - Clock ID in clock manager.
//
//*****************************************************************************
static am_hal_clkmgr_clock_id_e
i2s_clock_id_get(uint32_t ui32Clocksel, bool bIsMclkout)
{
    if (bIsMclkout == false)
    {
        switch ((am_hal_i2s_clksel_e)ui32Clocksel)
        {
            case AM_HAL_I2S_CLKSEL_HFRC_48MHz:
            case AM_HAL_I2S_CLKSEL_NCO_HFRC_96MHz:
                return AM_HAL_CLKMGR_CLK_ID_HFRC;

            case AM_HAL_I2S_CLKSEL_XTHS:
            case AM_HAL_I2S_CLKSEL_NCO_XTHS:
                return AM_HAL_CLKMGR_CLK_ID_XTAL_HS;

            case AM_HAL_I2S_CLKSEL_PLL_POSTDIV:
            case AM_HAL_I2S_CLKSEL_PLL_FOUT3:
            case AM_HAL_I2S_CLKSEL_PLL_FOUT4:
            case AM_HAL_I2S_CLKSEL_NCO_PLL_POSTDIV:
                return AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV;

            case AM_HAL_I2S_CLKSEL_EXTREF:
            case AM_HAL_I2S_CLKSEL_NCO_EXTREF:
                return AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK;

            case AM_HAL_I2S_CLKSEL_OFF:
            default:
                return AM_HAL_CLKMGR_CLK_ID_MAX;
        }
    }
    else
    {
        switch ((am_hal_i2s_mclkout_sel_e)ui32Clocksel)
        {
            case AM_HAL_I2S_MCLKOUT_SEL_HFRC48:
                return AM_HAL_CLKMGR_CLK_ID_HFRC;

            case AM_HAL_I2S_MCLKOUT_SEL_XTAL:
                return AM_HAL_CLKMGR_CLK_ID_XTAL_HS;

            case AM_HAL_I2S_MCLKOUT_SEL_PLLPOSTDIV:
            case AM_HAL_I2S_MCLKOUT_SEL_PLLFOUT3:
            case AM_HAL_I2S_MCLKOUT_SEL_PLLFOUT4:
                return AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV;

            case AM_HAL_I2S_MCLKOUT_SEL_EXTREF:
                return AM_HAL_CLKMGR_CLK_ID_EXTREF_CLK;

            case AM_HAL_I2S_MCLKOUT_SEL_OFF:
            default:
                return AM_HAL_CLKMGR_CLK_ID_MAX;
        }
    }
}

//*****************************************************************************
//
//! @brief [Internal] Set I2S moudule clock
//!
//! @param ui32Module           - I2S instance index.
//! @param eClocksel            - Clock selection in am_hal_i2s_clksel_e.
//! @param ui32ClockDivideRatio - Clock divider ratio for the given clock.
//!
//! This function configures the I2S module clock, the frequency of moudle clock
//! equals to the selected clock frequency divided by the clock divider ratio.
//!
//! @return status              - generic or interface specific status.
//
//*****************************************************************************
static uint32_t
i2s_clock_set(uint32_t ui32Module, am_hal_i2s_clksel_e eClocksel, uint32_t ui32ClockDivideRatio)
{
    uint32_t ui32Status;
    am_hal_clkmgr_user_id_e eUserID = AM_HAL_CLKMGR_USER_ID_I2S0;
    // Array to store the clock ID for each suceessful request.
    static am_hal_clkmgr_clock_id_e eStoredClockId[AM_REG_I2S_NUM_MODULES] = {AM_HAL_CLKMGR_CLK_ID_MAX};

    if (eClocksel != AM_HAL_I2S_CLKSEL_OFF)
    {
        bool bUseNcoClock = (EXTRACT_NCO_MUX(eClocksel) == AM_HAL_I2S_CLK_NCO_ENABLE) ? true : false;

        //
        // Request clock from the clock manager.
        //
        am_hal_clkmgr_clock_id_e eClockID = i2s_clock_id_get(eClocksel, false);
        ui32Status = am_hal_clkmgr_clock_request(eClockID, eUserID);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        if (bUseNcoClock)
        {
            //
            // Select NCO clock as I2S clock source
            //
            I2Sn(0)->AMQCFG_b.MCLKSRC = 1;

            //
            // Set and enable clock via CRM.
            //
            ui32Status = am_hal_crm_clock_config_I2S0REFCLK((am_hal_crm_i2srefclk_clksel_e)EXTRACT_CRM_MUX(eClocksel), ui32ClockDivideRatio - 1);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_hal_clkmgr_clock_release(eClockID, eUserID);
                return ui32Status;
            }

            ui32Status = am_hal_crm_control_I2S0REFCLK_CLOCK_SET(true);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_hal_clkmgr_clock_release(eClockID, eUserID);
                return ui32Status;
            }
        }
        else
        {
            //
            // Select MCLK as I2S clock source
            //
            I2Sn(0)->AMQCFG_b.MCLKSRC = 0;

            //
            // Set and enable clock via CRM.
            //
            ui32Status = am_hal_crm_clock_config_I2S0MCLK((am_hal_crm_i2smclk_clksel_e)EXTRACT_CRM_MUX(eClocksel), ui32ClockDivideRatio - 1);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_hal_clkmgr_clock_release(eClockID, eUserID);
                return ui32Status;
            }

            ui32Status = am_hal_crm_control_I2S0MCLK_CLOCK_SET(true);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                am_hal_clkmgr_clock_release(eClockID, eUserID);
                return ui32Status;
            }
        }

        //
        // Store the clock ID if everything is successful.
        //
        eStoredClockId[ui32Module] = eClockID;
    }
    else
    {
        //
        // Gate MCLK and NCO clock.
        //
        am_hal_crm_control_I2S0MCLK_CLOCK_SET(false);
        am_hal_crm_control_I2S0REFCLK_CLOCK_SET(false);

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
//! @brief [Internal] Set I2S MCLKOUT
//!
//! @param ui32Module           - I2S instance index.
//! @param eClocksel            - Clock selection in am_hal_i2s_mclkout_sel_e.
//! @param ui32ClockDivideRatio - Clock divider ratio for the given clock.
//!
//! This function configures the I2S MCLKOUT, the frequency of MCLKOUT equals
//! to the selected clock frequency divided by the clock divider ratio.
//! The MCLKOUT clock is an independent signal and does not cross the I2S module.
//! It is used to drive external devices like a codec.
//!
//! @return status              - generic or interface specific status.
//
//*****************************************************************************
static uint32_t
i2s_mclkout_set(uint32_t ui32Module, am_hal_i2s_mclkout_sel_e eClocksel, uint32_t ui32ClockDivideRatio)
{
    uint32_t ui32Status;
    am_hal_clkmgr_user_id_e eUserID = AM_HAL_CLKMGR_USER_ID_I2S0;
    // Array to store the clock ID for each suceessful request.
    static am_hal_clkmgr_clock_id_e eStoredClockId[AM_REG_I2S_NUM_MODULES] = {AM_HAL_CLKMGR_CLK_ID_MAX};

    if (eClocksel != AM_HAL_I2S_MCLKOUT_SEL_OFF)
    {
        //
        // Request clock from the clock manager.
        //
        am_hal_clkmgr_clock_id_e eClockID = i2s_clock_id_get(eClocksel, true);
        ui32Status = am_hal_clkmgr_clock_request(eClockID, eUserID);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        //
        // Set and enable MCLKOUT via CRM.
        //
        ui32Status = am_hal_crm_clock_config_I2S0MCLKOUT((am_hal_crm_i2smclkout_clksel_e)EXTRACT_CRM_MUX(eClocksel), ui32ClockDivideRatio - 1);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_hal_clkmgr_clock_release(eClockID, eUserID);
            return ui32Status;
        }

        ui32Status = am_hal_crm_control_I2S0MCLKOUT_CLOCK_SET(true);
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
        // Gate MCLKOUT.
        //
        am_hal_crm_control_I2S0MCLKOUT_CLOCK_SET(false);

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
// I2S initialization function
//
//*****************************************************************************
uint32_t
am_hal_i2s_initialize(uint32_t ui32Module, void **ppHandle)
{
    //
    // Compile time check to ensure ENTRY_SIZE macros are defined correctly
    // incorrect definition will cause divide by 0 error at build time
    //
#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the module number
    //
    if (ui32Module >= AM_REG_I2S_NUM_MODULES)
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    if (ppHandle == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if (g_I2Shandles[ui32Module].prefix.s.bInit)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    g_I2Shandles[ui32Module].prefix.s.bInit = true;
    g_I2Shandles[ui32Module].prefix.s.bEnable = false;
    g_I2Shandles[ui32Module].prefix.s.magic = AM_HAL_MAGIC_I2S;
    //
    // Initialize the handle.
    //
    g_I2Shandles[ui32Module].ui32Module = ui32Module;
    g_I2Shandles[ui32Module].bInternalMclkRequired = false;
    g_I2Shandles[ui32Module].bModuleEnabledBeforePowerdown = false;
    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_I2Shandles[ui32Module];
    //
    // Return the status
    //
    return AM_HAL_STATUS_SUCCESS;
} // am_hal_i2s_initialize()

//*****************************************************************************
//
// am_hal_i2s_deinitialize
//
//*****************************************************************************
uint32_t
am_hal_i2s_deinitialize(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *)pHandle;

    //
    // Reset the handle.
    //
    pState->prefix.s.bInit = false;
    pState->prefix.s.magic = 0;
    pState->ui32Module = 0;
    pState->bInternalMclkRequired = false;
    pState->bModuleEnabledBeforePowerdown = false;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S control function
// This function allows advanced settings
//
//*****************************************************************************
uint32_t am_hal_i2s_control(void *pHandle, am_hal_i2s_request_e eReq, void *pArgs)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;

    switch (eReq)
    {
        case AM_HAL_I2S_REQ_INTSET:
            I2Sn(ui32Module)->INTSET = *((uint32_t *)pArgs);
            break;
        case AM_HAL_I2S_REQ_INTCLR:
            I2Sn(ui32Module)->INTCLR = *((uint32_t *)pArgs);
            break;
        case AM_HAL_I2S_REQ_TXFIFOCNT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT;
            break;
        case AM_HAL_I2S_REQ_READ_RXUPPERLIMIT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->RXUPPERLIMIT;
            break;
        case AM_HAL_I2S_REQ_READ_TXLOWERLIMIT:
            *((uint32_t*)pArgs) = I2Sn(ui32Module)->TXLOWERLIMIT;
            break;
        case AM_HAL_I2S_REQ_WRITE_RXUPPERLIMIT:
            I2Sn(ui32Module)->RXUPPERLIMIT = *((uint32_t*)pArgs);
            break;
        case AM_HAL_I2S_REQ_WRITE_TXLOWERLIMIT:
            I2Sn(ui32Module)->TXLOWERLIMIT = *((uint32_t*)pArgs);
            break;

        case AM_HAL_I2S_REQ_SET_CH_NUM_FOR_MONO:
            if (pArgs == NULL)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }

            // Only 1 or 2 channels are allowed for mono mode.
            uint32_t ui32ChannelNumbersForMono;

            ui32ChannelNumbersForMono = *((uint32_t*)pArgs);
            if ((ui32ChannelNumbersForMono != 1) && (ui32ChannelNumbersForMono != 2))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }

            am_hal_i2s_data_format_t* pI2SData;
            am_hal_i2s_io_signal_t* pIoConfig;
            uint32_t ui32FramePeriod;

            pI2SData = &(pState->sDataFormat);
            pIoConfig = &(pState->sIoConfig);
            ui32FramePeriod = ui32ChannelNumbersForMono * ui32I2sWordLength[pI2SData->eChannelLenPhase1];

            if ((pI2SData->ePhase == AM_HAL_I2S_DATA_PHASE_SINGLE) && (pI2SData->ui32ChannelNumbersPhase1 == 1))
            {
                I2Sn(ui32Module)->I2SIOCFG_b.FPER = ui32FramePeriod - 1;
                if (pIoConfig->sFsyncPulseCfg.eFsyncPulseType == AM_HAL_I2S_FSYNC_PULSE_HALF_FRAME_PERIOD)
                {
                    I2Sn(ui32Module)->I2SIOCFG_b.FWID = ui32FramePeriod / 2 - 1;
                }
                else if (pIoConfig->sFsyncPulseCfg.eFsyncPulseType == AM_HAL_I2S_FSYNC_PULSE_CUSTOM)
                {
                    if (pIoConfig->sFsyncPulseCfg.ui32FsyncPulseWidth >= ui32FramePeriod)
                    {
                        return AM_HAL_STATUS_INVALID_ARG;
                    }
                }
            }
            else
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            break;

        case AM_HAL_I2S_REQ_MAX:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S configuration function.
//
//*****************************************************************************
uint32_t
am_hal_i2s_configure(void *pHandle, am_hal_i2s_config_t *psConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t status = AM_HAL_STATUS_SUCCESS;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Validate the parameters
    //
    if ((psConfig == NULL) || (psConfig->eData == NULL) || (psConfig->eIO == NULL))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Configure not allowed in Enabled state
    //
    if (pState->prefix.s.bEnable)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    am_hal_i2s_data_format_t* pI2SData = psConfig->eData;
    am_hal_i2s_io_signal_t* pI2SIOCfg  = psConfig->eIO;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ((pI2SData->ui32ChannelNumbersPhase1 > 8) || (pI2SData->ui32ChannelNumbersPhase2 > 8) || \
        (pI2SData->ui32ChannelNumbersPhase1 == 0) || (pI2SData->ui32ChannelNumbersPhase1 + pI2SData->ui32ChannelNumbersPhase2 > 8) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_8BITS) && (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_16BITS) && \
        (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_24BITS) && (pI2SData->eSampleLenPhase1 != AM_HAL_I2S_SAMPLE_LENGTH_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_8BITS) && (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_16BITS) && \
        (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_24BITS) && (pI2SData->eSampleLenPhase2 != AM_HAL_I2S_SAMPLE_LENGTH_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_8BITS) && (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_16BITS) && \
        (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_24BITS) && (pI2SData->eChannelLenPhase1 != AM_HAL_I2S_FRAME_WDLEN_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ((pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_8BITS) && (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_16BITS) && \
        (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_24BITS) && (pI2SData->eChannelLenPhase2 != AM_HAL_I2S_FRAME_WDLEN_32BITS) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Store data format information from the application.
    //
    pState->sDataFormat.ePhase                   = pI2SData->ePhase;
    pState->sDataFormat.ui32ChannelNumbersPhase1 = pI2SData->ui32ChannelNumbersPhase1;
    pState->sDataFormat.ui32ChannelNumbersPhase2 = pI2SData->ui32ChannelNumbersPhase2;
    pState->sDataFormat.eChannelLenPhase1        = pI2SData->eChannelLenPhase1;
    pState->sDataFormat.eChannelLenPhase2        = pI2SData->eChannelLenPhase2;
    pState->sDataFormat.eDataDelay               = pI2SData->eDataDelay;
    pState->sDataFormat.eSampleLenPhase1         = pI2SData->eSampleLenPhase1;
    pState->sDataFormat.eSampleLenPhase2         = pI2SData->eSampleLenPhase2;
    pState->sDataFormat.eDataJust                = pI2SData->eDataJust;

    //
    // Store IO configuration from the application.
    //
    pState->sIoConfig.sFsyncPulseCfg.eFsyncPulseType = pI2SIOCfg->sFsyncPulseCfg.eFsyncPulseType;
    pState->sIoConfig.sFsyncPulseCfg.ui32FsyncPulseWidth = pI2SIOCfg->sFsyncPulseCfg.ui32FsyncPulseWidth;

    uint32_t ui32FramePeriod = 0;
    uint32_t ui32FsyncPulseWidth = 0;

    //
    // Reset the serial receiver or transmitter by asserting bits RXRST and/or TXRST in the I2SCTL register
    //
    I2Sn(ui32Module)->I2SCTL = _VAL2FLD(I2S0_I2SCTL_RXRST, 1) | _VAL2FLD(I2S0_I2SCTL_TXRST, 1);
    am_hal_delay_us(200);

    //
    // I2S IPB clocks and IO signals
    //
    uint32_t ui32OEN = (psConfig->eXfer == AM_HAL_I2S_XFER_TX || psConfig->eXfer == AM_HAL_I2S_XFER_RXTX) ? 1 : 0;

    if (pI2SData->ePhase == AM_HAL_I2S_DATA_PHASE_SINGLE)
    {
        //
        // In mono mode, frame period (in units of sclk) can be set to once or twice of the word width.
        //
        if (pI2SData->ui32ChannelNumbersPhase1 == 1)
        {
            ui32FramePeriod = AM_HAL_I2S_CH_NUM_FOR_MONO * ui32I2sWordLength[pI2SData->eChannelLenPhase1];
        }
        else
        {
            ui32FramePeriod = (pI2SData->ui32ChannelNumbersPhase1 * ui32I2sWordLength[pI2SData->eChannelLenPhase1]);
        }
    }
    else
    {
        ui32FramePeriod = ((pI2SData->ui32ChannelNumbersPhase1 * ui32I2sWordLength[pI2SData->eChannelLenPhase1]) +  \
                           (pI2SData->ui32ChannelNumbersPhase2 * ui32I2sWordLength[pI2SData->eChannelLenPhase2]) );
    }

    switch (pI2SIOCfg->sFsyncPulseCfg.eFsyncPulseType)
    {
        case AM_HAL_I2S_FSYNC_PULSE_ONE_SUBFRAME:
            ui32FsyncPulseWidth = ui32I2sWordLength[pI2SData->eChannelLenPhase1];
            break;
        case AM_HAL_I2S_FSYNC_PULSE_ONE_BIT_CLOCK:
            ui32FsyncPulseWidth = 1;
            break;
        case AM_HAL_I2S_FSYNC_PULSE_HALF_FRAME_PERIOD:
            ui32FsyncPulseWidth = ui32FramePeriod / 2;
            break;
        case AM_HAL_I2S_FSYNC_PULSE_CUSTOM:
            ui32FsyncPulseWidth = pI2SIOCfg->sFsyncPulseCfg.ui32FsyncPulseWidth;
            //
            // ui32FsyncPulseWidth should be in the range of [0, ui32FramePeriod]
            //
            if ((ui32FsyncPulseWidth == 0) || (ui32FsyncPulseWidth >= ui32FramePeriod))
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    I2Sn(ui32Module)->I2SIOCFG =
         _VAL2FLD(I2S0_I2SIOCFG_OEN,  ui32OEN)              | /*Output enable for SDATA output */
         _VAL2FLD(I2S0_I2SIOCFG_FPER, ui32FramePeriod - 1)  | /*specifying a frame period of 2x WDLEN1 cycles*/
         _VAL2FLD(I2S0_I2SIOCFG_FSP,  pI2SIOCfg->eFyncCpol) |
         _VAL2FLD(I2S0_I2SIOCFG_PRTX, pI2SIOCfg->eTxCpol)   |
         _VAL2FLD(I2S0_I2SIOCFG_MSL,  psConfig->eMode)      |
         _VAL2FLD(I2S0_I2SIOCFG_PRx,  pI2SIOCfg->eRxCpol)   |
         _VAL2FLD(I2S0_I2SIOCFG_FWID, ui32FsyncPulseWidth - 1);

    //
    // Set the audio data format.
    //
    I2Sn(ui32Module)->I2SDATACFG =
         _VAL2FLD(I2S0_I2SDATACFG_SSZ1,    pI2SData->eSampleLenPhase1)             |
         _VAL2FLD(I2S0_I2SDATACFG_JUST,    pI2SData->eDataJust)                    |
         _VAL2FLD(I2S0_I2SDATACFG_WDLEN1,  pI2SData->eChannelLenPhase1)            |
         _VAL2FLD(I2S0_I2SDATACFG_FRLEN1,  (pI2SData->ui32ChannelNumbersPhase1-1)) |
         _VAL2FLD(I2S0_I2SDATACFG_SSZ2,    pI2SData->eSampleLenPhase2)             |
         _VAL2FLD(I2S0_I2SDATACFG_DATADLY, pI2SData->eDataDelay)                   |
         _VAL2FLD(I2S0_I2SDATACFG_WDLEN2,  pI2SData->eChannelLenPhase2)            |
         _VAL2FLD(I2S0_I2SDATACFG_FRLEN2,  (pI2SData->ui32ChannelNumbersPhase2-1)) |
         _VAL2FLD(I2S0_I2SDATACFG_PH,      pI2SData->ePhase);

    //
    // IPB issue fix.
    //
    I2Sn(ui32Module)->AMQCFG_b.FIX8B16B = 1;

    //
    // Clock related settings.
    //
    pState->bInternalMclkRequired = (psConfig->eMode == AM_HAL_I2S_IO_MODE_MASTER) ? true : false;

    if (pState->bInternalMclkRequired)
    {
        //
        // Not a vaild clock source.
        //
        if (psConfig->eClock == AM_HAL_I2S_CLKSEL_OFF)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }

        pState->eClockSel = psConfig->eClock;
        pState->ui32ClockDivideRatio = (psConfig->eDiv3 == 0) ? (psConfig->ui32ClockDivideRatio) : (psConfig->ui32ClockDivideRatio * 3);

        bool bUseNcoClock = (EXTRACT_NCO_MUX(psConfig->eClock) == AM_HAL_I2S_CLK_NCO_ENABLE) ? true : false;
        if (bUseNcoClock)
        {
            //
            // Extract the integer part of the divisor.
            //
            uint32_t ui32I = (uint32_t)(psConfig->f64NcoDiv);
            //
            // Extract the fractional part of the divisor, multiply 2^32 and round the result.
            //
            uint64_t ui64Q = (uint64_t)((psConfig->f64NcoDiv - (double)ui32I) * 0x100000000ULL + 0.5);
            //
            // If the fractional part is saturated, add one to the integer part.
            //
            if (ui64Q == 0x100000000ULL)
            {
                ui32I = ui32I + 1;
                ui64Q = 0;
            }
            //
            // nco_ref_clk frequency must be at least 4x the desired nco_mclk frequency.
            //
            if (ui32I < 4)
            {
                return AM_HAL_STATUS_INVALID_ARG;
            }

            I2Sn(ui32Module)->INTDIV  = ui32I;
            I2Sn(ui32Module)->FRACDIV = (uint32_t)ui64Q;
        }

    }
    else
    {
        pState->eClockSel = AM_HAL_I2S_CLKSEL_OFF;
    }

    pState->eMclkout = psConfig->eMclkout;
    pState->ui32MclkoutDiv = psConfig->ui32MclkoutDiv;

    //
    // RXTX DMA limit: FIFO 50 percent full
    //
    I2Sn(ui32Module)->RXUPPERLIMIT = 0x20;
    I2Sn(ui32Module)->TXLOWERLIMIT = 0x20;

    //
    // Return the status.
    //
    return status;
} // am_hal_i2s_configure()

//*****************************************************************************
//
// I2S DMA nonblocking transfer function
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_start(void *pHandle,  am_hal_i2s_config_t *pConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Poll Transmit FIFO Status register to
    // prevent the Transmit FIFO from becoming empty.
    //
    uint32_t ui32Fifocnt  = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT;
    uint32_t ui32Fifofull = I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOFULL;
    if ( ui32Fifocnt || ui32Fifofull )
    {
        return AM_HAL_STATUS_FAIL;
    }
    //
    // Enable the transmission of serial audio.
    // TXRST & RXRST must be cleared in advance!!!
    //
    if ( pConfig->eXfer == AM_HAL_I2S_XFER_RX )
    {
        I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL = _VAL2FLD(I2S0_I2SCTL_RXEN, 1);
    }
    else if ( pConfig->eXfer == AM_HAL_I2S_XFER_TX )
    {
        I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL = _VAL2FLD(I2S0_I2SCTL_TXEN, 1);
    }
    else if ( pConfig->eXfer == AM_HAL_I2S_XFER_RXTX )
    {
        I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 0x1;
        I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 0x1;
        I2Sn(ui32Module)->I2SCTL = _VAL2FLD(I2S0_I2SCTL_RXEN, 1) | _VAL2FLD(I2S0_I2SCTL_TXEN, 1);
    }

    #ifdef USE_I2S_TWO_STAGE_DMA
    //
    // "Pong = 0xFFFFFFFF" means that only the ping buffer works, otherwise, set the pong buffer to the next stage of DMA.
    //
    if ((pConfig->eXfer == AM_HAL_I2S_XFER_RX) && (pState->ui32RxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPtr = pState->ui32RxBufferPong;
        I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk;
    }
    else if ((pConfig->eXfer == AM_HAL_I2S_XFER_TX) && (pState->ui32TxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPtr = pState->ui32TxBufferPong;
        I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
    }
    else if ((pConfig->eXfer == AM_HAL_I2S_XFER_RXTX) && (pState->ui32TxBufferPong != 0xFFFFFFFF) && (pState->ui32RxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPtr = pState->ui32TxBufferPong;
        I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPtr = pState->ui32RxBufferPong;
        I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk | I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
    }
    #endif

    return ui32Status;
}

//*****************************************************************************
//
// am_hal_i2s_dma_transfer_continue
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_continue(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    pState->ui32RxBufferSizeBytes = pTransferCfg->ui32RxTotalCount * 4;
    pState->ui32TxBufferSizeBytes = pTransferCfg->ui32TxTotalCount * 4;
    //
    // Once completed, software must first write the DMACFG register to 0.
    //
    I2Sn(ui32Module)->DMACFG = 0x0;
    #ifdef USE_I2S_TWO_STAGE_DMA
    I2Sn(ui32Module)->DMACFG_b.NEXTDMAEN = 1;
    #endif
    //
    // Clear dma status.
    //
    I2Sn(ui32Module)->RXDMASTAT = 0x0;
    I2Sn(ui32Module)->TXDMASTAT = 0x0;
    //
    // High Priority (service immediately)
    //
    I2Sn(ui32Module)->DMACFG_b.RXDMAPRI = 0x1;
    I2Sn(ui32Module)->DMACFG_b.TXDMAPRI = 0x1;

    switch(psConfig->eXfer)
    {
        case AM_HAL_I2S_XFER_RX:
            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPtr = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPtr = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->RXDMATOTCNT = pState->ui32RxBufferSizeBytes >> 2;
            #endif
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 1;
            break;

        case AM_HAL_I2S_XFER_TX:
            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPtr = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPtr = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->TXDMATOTCNT = pState->ui32TxBufferSizeBytes >> 2;
            #endif
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 1;
            break;

        case AM_HAL_I2S_XFER_RXTX:
            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPtr = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPtr = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk | I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPtr = pTransferCfg->ui32TxTargetAddr;
            I2Sn(ui32Module)->TXDMATOTCNT = pTransferCfg->ui32TxTotalCount;
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPtr = pTransferCfg->ui32RxTargetAddr;
            I2Sn(ui32Module)->RXDMATOTCNT = pTransferCfg->ui32RxTotalCount;
            #endif
            I2Sn(ui32Module)->DMACFG_b.RXDMAEN = 1;
            I2Sn(ui32Module)->DMACFG_b.TXDMAEN = 1;
            break;
    }

    return ui32Status;
}

//*****************************************************************************
//
// Power control function.
//
//*****************************************************************************
uint32_t
am_hal_i2s_power_control(void *pHandle,
                         am_hal_sysctrl_power_state_e ePowerState,
                         bool bRetainState)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t ui32Status;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    am_hal_pwrctrl_periph_e eI2SPowerModule = ((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I2S0 + ui32Module));

    //
    // Decode the requested power state and update I2S operation accordingly.
    //
    switch (ePowerState)
    {
        //
        // Turn on the I2S.
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
            am_hal_pwrctrl_periph_enable(eI2SPowerModule);

            //
            // Enable APB clock.
            //
            ui32Status = am_hal_crm_control_I2S0_CLOCK_SET(true);
            if (AM_HAL_STATUS_SUCCESS != ui32Status)
            {
                return ui32Status;
            }

            if (bRetainState)
            {
                //
                // Restore I2S registers
                //
                I2Sn(ui32Module)->I2SCTL       = pState->sRegState.regI2SCTL;
                I2Sn(ui32Module)->I2SDATACFG   = pState->sRegState.regI2SDATACFG;
                I2Sn(ui32Module)->I2SIOCFG     = pState->sRegState.regI2SIOCFG;
                I2Sn(ui32Module)->AMQCFG       = pState->sRegState.regAMQCFG;
                I2Sn(ui32Module)->CLKCFG       = pState->sRegState.regI2SCLKCFG;
                I2Sn(ui32Module)->IPBIRPT      = pState->sRegState.regIPBIRPT;
                I2Sn(ui32Module)->RXUPPERLIMIT = pState->sRegState.regRXUPPERLIMIT;
                I2Sn(ui32Module)->TXLOWERLIMIT = pState->sRegState.regTXLOWERLIMIT;
                I2Sn(ui32Module)->INTEN        = pState->sRegState.regINTEN;
                I2Sn(ui32Module)->DMACFG       = pState->sRegState.regI2SDMACFG;
                I2Sn(ui32Module)->INTDIV       = pState->sRegState.regINTDIV;
                I2Sn(ui32Module)->FRACDIV      = pState->sRegState.regFRACDIV;

                if (pState->bModuleEnabledBeforePowerdown)
                {
                    ui32Status = am_hal_i2s_enable(pHandle);
                    if (AM_HAL_STATUS_SUCCESS != ui32Status)
                    {
                        am_hal_pwrctrl_periph_disable(eI2SPowerModule);
                        return ui32Status;
                    }
                }

                pState->sRegState.bValid = false;
            }
            break;
        //
        // Turn off the I2S.
        //
        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if (bRetainState)
            {
                //
                // Save I2S Registers
                //
                pState->sRegState.regI2SCTL       = I2Sn(ui32Module)->I2SCTL & (I2S0_I2SCTL_RXEN_Msk | I2S0_I2SCTL_TXEN_Msk);
                pState->sRegState.regI2SDATACFG   = I2Sn(ui32Module)->I2SDATACFG;
                pState->sRegState.regI2SIOCFG     = I2Sn(ui32Module)->I2SIOCFG;
                pState->sRegState.regAMQCFG       = I2Sn(ui32Module)->AMQCFG;
                pState->sRegState.regI2SCLKCFG    = I2Sn(ui32Module)->CLKCFG;
                pState->sRegState.regIPBIRPT      = I2Sn(ui32Module)->IPBIRPT;
                pState->sRegState.regRXUPPERLIMIT = I2Sn(ui32Module)->RXUPPERLIMIT;
                pState->sRegState.regTXLOWERLIMIT = I2Sn(ui32Module)->TXLOWERLIMIT;
                pState->sRegState.regINTEN        = I2Sn(ui32Module)->INTEN;
                pState->sRegState.regI2SDMACFG    = I2Sn(ui32Module)->DMACFG;
                pState->sRegState.regINTDIV       = I2Sn(ui32Module)->INTDIV;
                pState->sRegState.regFRACDIV      = I2Sn(ui32Module)->FRACDIV;

                pState->sRegState.bValid = true;
            }

            if (pState->prefix.s.bEnable == true)
            {
                //
                // Record enabled state and disable I2S.
                //
                pState->bModuleEnabledBeforePowerdown = true;
                am_hal_i2s_disable(pHandle);
            }

            //
            // Disable APB clock.
            //
            am_hal_crm_control_I2S0_CLOCK_SET(false);

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable(eI2SPowerModule);

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
// Interrupt enable.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_enable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    if ((ui32IntMask & AM_HAL_I2S_INT_RXDMACPL) == AM_HAL_I2S_INT_RXDMACPL)
    {
        I2Sn(ui32Module)->IPBIRPT |= I2S0_IPBIRPT_RXDMAM_Msk;
        I2Sn(ui32Module)->INTEN   |= I2S0_INTSTAT_RXDMACPL_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXDMACPL) == AM_HAL_I2S_INT_TXDMACPL)
    {
        I2Sn(ui32Module)->IPBIRPT |= I2S0_IPBIRPT_TXDMAM_Msk;
        I2Sn(ui32Module)->INTEN   |= I2S0_INTSTAT_TXDMACPL_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_RXREQCNT) == AM_HAL_I2S_INT_RXREQCNT)
    {
        I2Sn(ui32Module)->IPBIRPT |= I2S0_IPBIRPT_RXFFM_Msk;
        I2Sn(ui32Module)->INTEN   |= I2S0_INTSTAT_RXREQCNT_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXREQCNT) == AM_HAL_I2S_INT_TXREQCNT)
    {
        I2Sn(ui32Module)->IPBIRPT |= I2S0_IPBIRPT_TXFFM_Msk;
        I2Sn(ui32Module)->INTEN   |= I2S0_INTSTAT_TXREQCNT_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXFIFO_EMPTY) == AM_HAL_I2S_INT_TXFIFO_EMPTY)
    {
        I2Sn(ui32Module)->IPBIRPT |= I2S0_IPBIRPT_TXEM_Msk;
        I2Sn(ui32Module)->INTEN   |= I2S0_INTSTAT_IPB_Msk;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt disable.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_disable(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    if ((ui32IntMask & AM_HAL_I2S_INT_RXDMACPL) == AM_HAL_I2S_INT_RXDMACPL)
    {
        I2Sn(ui32Module)->IPBIRPT &= ~I2S0_IPBIRPT_RXDMAM_Msk;
        I2Sn(ui32Module)->INTEN   &= ~I2S0_INTSTAT_RXDMACPL_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXDMACPL) == AM_HAL_I2S_INT_TXDMACPL)
    {
        I2Sn(ui32Module)->IPBIRPT &= ~I2S0_IPBIRPT_TXDMAM_Msk;
        I2Sn(ui32Module)->INTEN   &= ~I2S0_INTSTAT_TXDMACPL_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_RXREQCNT) == AM_HAL_I2S_INT_RXREQCNT)
    {
        I2Sn(ui32Module)->IPBIRPT &= ~I2S0_IPBIRPT_RXFFM_Msk;
        I2Sn(ui32Module)->INTEN   &= ~I2S0_INTSTAT_RXREQCNT_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXREQCNT) == AM_HAL_I2S_INT_TXREQCNT)
    {
        I2Sn(ui32Module)->IPBIRPT &= ~I2S0_IPBIRPT_TXFFM_Msk;
        I2Sn(ui32Module)->INTEN   &= ~I2S0_INTSTAT_TXREQCNT_Msk;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXFIFO_EMPTY) == AM_HAL_I2S_INT_TXFIFO_EMPTY)
    {
        I2Sn(ui32Module)->IPBIRPT &= ~I2S0_IPBIRPT_TXEM_Msk;
        I2Sn(ui32Module)->INTEN   &= ~I2S0_INTSTAT_IPB_Msk;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Interrupt clear.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_clear(void *pHandle, uint32_t ui32IntMask)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    I2Sn(ui32Module)->INTCLR = ui32IntMask;
    *(volatile uint32_t*)(&I2Sn(ui32Module)->INTSTAT);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the interrupt status.
//
//*****************************************************************************
uint32_t
am_hal_i2s_interrupt_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // If requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        *pui32Status = I2Sn(ui32Module)->INTSTAT;
        *pui32Status &= I2Sn(ui32Module)->INTEN;
    }
    else
    {
        *pui32Status = I2Sn(ui32Module)->INTSTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Returns the DMA status(TXMDASTAT/RXDMASTAT)
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_status_get(void *pHandle, uint32_t *pui32Status, am_hal_i2s_xfer_dir_e xfer)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // If requested, only return the interrupts that are enabled.
    //
    if (xfer == AM_HAL_I2S_XFER_RX)
    {
        *pui32Status = I2Sn(ui32Module)->RXDMASTAT;
    }
    else if (xfer == AM_HAL_I2S_XFER_TX)
    {
      *pui32Status = I2Sn(ui32Module)->TXDMASTAT;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_i2s_interrupt_service(void *pHandle, uint32_t ui32IntMask, am_hal_i2s_config_t* psConfig)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = ((am_hal_i2s_state_t*)pHandle)->ui32Module;

    //
    // I2S DMA error processing.
    //
    if (I2Sn(ui32Module)->TXDMASTAT & AM_HAL_I2S_STAT_DMA_TX_ERR)
    {
        am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_TX);
    }

    if (I2Sn(ui32Module)->RXDMASTAT & AM_HAL_I2S_STAT_DMA_RX_ERR)
    {
        am_hal_i2s_dma_error(pHandle, AM_HAL_I2S_XFER_RX);
    }

    //
    // Reload the DMA address and the transfer size.
    //
    #ifdef USE_I2S_TWO_STAGE_DMA
    if ((ui32IntMask & AM_HAL_I2S_INT_RXDMACPL) && (pState->ui32RxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->RXDMASTAT_b.RXDMACPL = 0;
        if ((I2Sn(ui32Module)->DMAENNEXTCTRL & I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk) == 0)
        {
            I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong) ? pState->ui32RxBufferPing : pState->ui32RxBufferPong;
            I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL  |= I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk;
        }
        else
        {
            return AM_HAL_STATUS_HW_ERR;
        }

    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXDMACPL) && (pState->ui32TxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->TXDMASTAT_b.TXDMACPL = 0;
        if ((I2Sn(ui32Module)->DMAENNEXTCTRL & I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk) == 0)
        {
            I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong) ? pState->ui32TxBufferPing : pState->ui32TxBufferPong;
            I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL  |= I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
        }
        else
        {
            return AM_HAL_STATUS_HW_ERR;
        }
    }
    #else
    //
    // In the previous I2S DMA, writing DMA registers needs deasserting T/RXDMAEN first.
    //
    if ((ui32IntMask & AM_HAL_I2S_INT_RXDMACPL) && (pState->ui32RxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->DMACFG_b.RXDMAEN     = 0;
        I2Sn(ui32Module)->RXDMASTAT_b.RXDMACPL = 0;
        I2Sn(ui32Module)->RXDMAADDR            = pState->ui32RxBufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong) ? pState->ui32RxBufferPing : pState->ui32RxBufferPong;
        I2Sn(ui32Module)->RXDMATOTCNT          = pState->ui32RxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->DMACFG_b.RXDMAEN     = 1;
    }

    if ((ui32IntMask & AM_HAL_I2S_INT_TXDMACPL) && (pState->ui32TxBufferPong != 0xFFFFFFFF))
    {
        I2Sn(ui32Module)->DMACFG_b.TXDMAEN     = 0;
        I2Sn(ui32Module)->TXDMASTAT_b.TXDMACPL = 0;
        I2Sn(ui32Module)->TXDMAADDR            = pState->ui32TxBufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong) ? pState->ui32TxBufferPing : pState->ui32TxBufferPong;
        I2Sn(ui32Module)->TXDMATOTCNT          = pState->ui32TxBufferSizeBytes >> 2;
        I2Sn(ui32Module)->DMACFG_b.TXDMAEN     = 1;
    }
    #endif

    //
    // I2S FIFO interrupts handling.
    //
    if (ui32IntMask & AM_HAL_I2S_INT_IPB)
    {
        am_hal_i2s_ipb_interrupt_service(pHandle);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S IPB interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_i2s_ipb_interrupt_service(void *pHandle)
{
    uint32_t ui32Module;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    ui32Module = pState->ui32Module;

    uint32_t ui32IntMask = I2Sn(ui32Module)->IPBIRPT;
    //
    // When the number of samples in the Transmit FIFO drops below the value in the Transmit
    // FIFO Lower Limit register (TXLOWERLIMIT)
    //
    // Transmit FIFO become empty
    //
    if (ui32IntMask & I2S0_IPBIRPT_TXEI_Msk)
    {
        //
        // clear TX_Ei
        //
        I2Sn(ui32Module)->IPBIRPT_b.TXEI = 0x0;

        //I2Sn(ui32Module)->I2SCTL_b.TXEN  = 0x0;
        //I2Sn(ui32Module)->I2SCTL_b.TXRST = 0x0;
    }

    if (ui32IntMask & I2S0_IPBIRPT_TXFFI_Msk)
    {
        I2Sn(ui32Module)->IPBIRPT_b.TXFFI = 0x0;
    }
    //
    // Receive FIFO become full
    //
    if (ui32IntMask & I2S0_IPBIRPT_RXFI_Msk)
    {
        //
        // To clear TX_Ei and RX_Fi, '0' must be written in these fields.
        // Otherwise, the interrupt signal will remain active.
        //
        I2Sn(ui32Module)->IPBIRPT_b.RXFI = 0x0;

        //I2Sn(ui32Module)->I2SCTL_b.RXEN  = 0x0;
        //I2Sn(ui32Module)->I2SCTL_b.RXRST = 0x0;
    }
    //
    // Receive fifo high limit interrupt
    //
    if (ui32IntMask & I2S0_IPBIRPT_RXFFI_Msk)
    {
        I2Sn(ui32Module)->IPBIRPT_b.RXFFI = 0x0;
    }
    //
    // Return the status.
    //
    return ui32Status;
}

//*****************************************************************************
//
// DMA transaction configuration.
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_configure(void *pHandle, am_hal_i2s_config_t* psConfig, am_hal_i2s_transfer_t *pTransferCfg)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Save the buffers.
    //
    pState->ui32RxBufferPtr  = pState->ui32RxBufferPing = pTransferCfg->ui32RxTargetAddr;
    pState->ui32RxBufferPong = pTransferCfg->ui32RxTargetAddrReverse;
    pState->ui32TxBufferPtr  = pState->ui32TxBufferPing = pTransferCfg->ui32TxTargetAddr;
    pState->ui32TxBufferPong = pTransferCfg->ui32TxTargetAddrReverse;
    pState->ui32RxBufferSizeBytes = pTransferCfg->ui32RxTotalCount * 4;
    pState->ui32TxBufferSizeBytes = pTransferCfg->ui32TxTotalCount * 4;

    //
    // Save the handle.
    //
    psConfig->eTransfer      = pTransferCfg;
    //
    // Clear the interrupts
    //
    I2Sn(ui32Module)->INTCLR = AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL |
                               AM_HAL_I2S_INT_TXREQCNT | AM_HAL_I2S_INT_RXREQCNT |
                               AM_HAL_I2S_INT_IPB;

    I2Sn(ui32Module)->IPBIRPT = 0x0;

    //
    // If the USE_NEW_DMA macro is defined, set DMACFG_b.NEXTDMAEN to enable
    // this feature.
    //
    #ifdef USE_I2S_TWO_STAGE_DMA
    I2Sn(ui32Module)->DMACFG_b.NEXTDMAEN = 1;
    #else
    I2Sn(ui32Module)->DMACFG_b.NEXTDMAEN = 0;
    #endif

    //
    // Load the DMA address and the transfer size.
    // Enable I2S interrupts.
    //
    switch ( psConfig->eXfer )
    {
        case AM_HAL_I2S_XFER_RX:

            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPing;
            I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPing;
            I2Sn(ui32Module)->RXDMATOTCNT = pState->ui32RxBufferSizeBytes >> 2;
            #endif

            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_RXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_RXDMACPL;
            break;

        case AM_HAL_I2S_XFER_TX:
            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPing;
            I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPing;
            I2Sn(ui32Module)->TXDMATOTCNT = pState->ui32TxBufferSizeBytes >> 2;
            #endif

            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_TXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_TXDMACPL;
            break;

        case AM_HAL_I2S_XFER_RXTX:
            #ifdef USE_I2S_TWO_STAGE_DMA
            I2Sn(ui32Module)->TXDMAADDRNEXT   = pState->ui32TxBufferPing;
            I2Sn(ui32Module)->TXDMATOTCNTNEXT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->RXDMAADDRNEXT   = pState->ui32RxBufferPing;
            I2Sn(ui32Module)->RXDMATOTCNTNEXT = pState->ui32RxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->DMAENNEXTCTRL   = I2S0_DMAENNEXTCTRL_RXDMAENNEXT_Msk | I2S0_DMAENNEXTCTRL_TXDMAENNEXT_Msk;
            #else
            I2Sn(ui32Module)->TXDMAADDR   = pState->ui32TxBufferPing;
            I2Sn(ui32Module)->TXDMATOTCNT = pState->ui32TxBufferSizeBytes >> 2;
            I2Sn(ui32Module)->RXDMAADDR   = pState->ui32RxBufferPing;
            I2Sn(ui32Module)->RXDMATOTCNT = pState->ui32RxBufferSizeBytes >> 2;
            #endif

            I2Sn(ui32Module)->IPBIRPT = AM_HAL_I2S_INT_IPBIRPT_RXDMA | AM_HAL_I2S_INT_IPBIRPT_TXDMA;
            I2Sn(ui32Module)->INTEN   = AM_HAL_I2S_INT_TXDMACPL | AM_HAL_I2S_INT_RXDMACPL;
            break;
    }
    //
    // High Priority (service immediately)
    //
    I2Sn(ui32Module)->DMACFG_b.RXDMAPRI = 0x1;
    I2Sn(ui32Module)->DMACFG_b.TXDMAPRI = 0x1;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// am_hal_i2s_dma_get_buffer
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_get_buffer(void *pHandle, am_hal_i2s_xfer_dir_e xfer)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t ui32BufferPtr;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;

    //
    // Get the buffer on which the DMA isn't working.
    //
    if ( AM_HAL_I2S_XFER_RX == xfer )
    {
        // "Pong = 0xFFFFFFFF" means that only the ping buffer works, just return the ping buffer.
        if (pState->ui32RxBufferPong == 0xFFFFFFFF)
        {
            ui32BufferPtr = pState->ui32RxBufferPing;
        }
        else
        {
            #ifdef USE_I2S_TWO_STAGE_DMA
            ui32BufferPtr = pState->ui32RxBufferPtr;
            #else
            ui32BufferPtr = (pState->ui32RxBufferPtr == pState->ui32RxBufferPong)? pState->ui32RxBufferPing: pState->ui32RxBufferPong;
            #endif
        }
    }
    else
    {
        // "Pong = 0xFFFFFFFF" means that only the ping buffer works, just return the ping buffer.
        if (pState->ui32TxBufferPong == 0xFFFFFFFF)
        {
            ui32BufferPtr = pState->ui32TxBufferPing;
        }
        else
        {
            #ifdef USE_I2S_TWO_STAGE_DMA
            ui32BufferPtr = pState->ui32TxBufferPtr;
            #else
            ui32BufferPtr = (pState->ui32TxBufferPtr == pState->ui32TxBufferPong)? pState->ui32TxBufferPing: pState->ui32TxBufferPong;
            #endif
        }
    }

    return ui32BufferPtr;
}

//*****************************************************************************
//
// I2S enable function
//
//*****************************************************************************
uint32_t
am_hal_i2s_enable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    uint32_t ui32Status;
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    am_hal_clkmgr_user_id_e userID = AM_HAL_CLKMGR_USER_ID_I2S0;

    if (pState->prefix.s.bEnable == true)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    //
    // Request HFRC for DMA operations.
    //
    ui32Status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, userID);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    //
    // Set MCLKOUT.
    //
    ui32Status = i2s_mclkout_set(ui32Module, pState->eMclkout, pState->ui32MclkoutDiv);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        return ui32Status;
    }

    if (pState->bInternalMclkRequired)
    {
        //
        // Set MCLK.
        //
        ui32Status = i2s_clock_set(ui32Module, pState->eClockSel, pState->ui32ClockDivideRatio);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            return ui32Status;
        }

        //
        // MCLKEN can only control clocks from the non-NCO CLKGEN.
        //
        I2Sn(ui32Module)->CLKCFG_b.MCLKEN = 1;

        //
        // REFCLKEN can only control clocks from the NCO CLKGEN.
        //
        I2Sn(ui32Module)->CLKCFG_b.REFCLKEN = 1;
    }
    else
    {
        //
        // If the internal MCLK isn't required, gate it.
        //
        I2Sn(ui32Module)->CLKCFG_b.MCLKEN = 0;
        I2Sn(ui32Module)->CLKCFG_b.REFCLKEN = 0;
    }

    pState->prefix.s.bEnable = true;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I2S disable function
//
//*****************************************************************************
uint32_t
am_hal_i2s_disable(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t*)pHandle;
    uint32_t ui32Module = pState->ui32Module;
    am_hal_clkmgr_user_id_e userID = AM_HAL_CLKMGR_USER_ID_I2S0;

    if (pState->prefix.s.bEnable == false)
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, userID);

    if (pState->bInternalMclkRequired)
    {
        //
        // MCLKEN can only control clocks from the non-NCO CLKGEN.
        //
        I2Sn(ui32Module)->CLKCFG_b.MCLKEN = 0;

        //
        // REFCLKEN can only control clocks from the NCO CLKGEN.
        //
        I2Sn(ui32Module)->CLKCFG_b.REFCLKEN = 0;

        //
        // Set I2S clock source to CLK_OFF and release this clock.
        //
        i2s_clock_set(ui32Module, AM_HAL_I2S_CLKSEL_OFF, 1);
    }

    i2s_mclkout_set(ui32Module, AM_HAL_I2S_MCLKOUT_SEL_OFF, 1);

    pState->prefix.s.bEnable = false;

    return AM_HAL_STATUS_SUCCESS;
} // am_hal_i2s_disable()

//*****************************************************************************
//
// am_hal_i2s_dma_transfer_complete
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_transfer_complete(void *pHandle)
{
    //
    // Check the handle.
    //
    AM_HAL_I2S_CHK_HANDLE(pHandle);

    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // Once completed, software must first write the DMACFG register to 0,
    // prior to making any update
    //
    I2Sn(ui32Module)->DMACFG = 0x0;
    //
    // Clear dma status.
    //
    I2Sn(ui32Module)->RXDMASTAT = 0x0;
    I2Sn(ui32Module)->TXDMASTAT = 0x0;
    I2Sn(ui32Module)->I2SCTL = 0;
    I2Sn(ui32Module)->TXDMATOTCNT = 0;
    I2Sn(ui32Module)->RXDMATOTCNT = 0;
    I2Sn(ui32Module)->TXDMATOTCNTNEXT = 0;
    I2Sn(ui32Module)->RXDMATOTCNTNEXT = 0;
    I2Sn(ui32Module)->TXCHANID = 0;
    I2Sn(ui32Module)->RXCHANID = 0;
    I2Sn(ui32Module)->INTCLR = 0xFFFFFFFF;
    I2Sn(ui32Module)->I2SCTL = 0x22;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// am_hal_i2s_tx_fifo_empty
//
//*****************************************************************************
bool
am_hal_i2s_tx_fifo_empty(void *pHandle)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // AM_HAL_I2S_INT_TXDMACPL is always triggered before the completion of FIFO TX,
    // So check the FIFOCNT to guarantee all datas transfered completely before next DMA transaction.
    //
    if ( I2Sn(ui32Module)->TXFIFOSTATUS_b.TXFIFOCNT == 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//*****************************************************************************
//
// am_hal_i2s_dma_error
//
//*****************************************************************************
uint32_t
am_hal_i2s_dma_error(void *pHandle, am_hal_i2s_xfer_dir_e xfer)
{
    am_hal_i2s_state_t *pState = (am_hal_i2s_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;
    //
    // If an error condition did occur during a DMA operation, the DMA must first be disabled
    //
    I2Sn(ui32Module)->DMACFG = 0x0;
    //
    // DMA status bits cleared.
    //
    if (xfer == AM_HAL_I2S_XFER_RX)
    {
        I2Sn(ui32Module)->RXDMASTAT = 0x0;
    }
    else if (xfer == AM_HAL_I2S_XFER_TX)
    {
        I2Sn(ui32Module)->TXDMASTAT = 0x0;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
