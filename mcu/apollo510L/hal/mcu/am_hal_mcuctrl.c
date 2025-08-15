//*****************************************************************************
//
//! @file am_hal_mcuctrl.c
//!
//! @brief Functions for interfacing with the MCUCTRL.
//!
//! @addtogroup mcuctrl4_ap510L MCUCTRL - MCU Control
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides functions for interfacing with the MCU Control
//!          (MCUCTRL) module on Apollo5 devices. It supports device information
//!          retrieval, external clock control, status monitoring, and chip
//!          configuration for system management and initialization.
//!
//! @section hal_mcuctrl_features Key Features
//!
//! 1. @b Device @b Information: Retrieve chip ID, revision, and vendor information.
//! 2. @b Memory @b Configuration: Access memory size and configuration data.
//! 3. @b External @b Clock: Manage external clock status and configuration.
//! 4. @b System @b Control: Provide system control and status operations.
//! 5. @b Feature @b Detection: Identify device features and capabilities.
//!
//! @section hal_mcuctrl_functionality Functionality
//!
//! - Retrieve device information and chip identification
//! - Access memory configuration and size information
//! - Manage external clock status and configuration
//! - Provide system control and status operations
//! - Support for device feature detection and configuration
//!
//! @section hal_mcuctrl_usage Usage
//!
//! 1. Get device information using am_hal_mcuctrl_info_get()
//! 2. Retrieve memory configuration as needed
//! 3. Check external clock status
//! 4. Access system control functions
//! 5. Query device features and capabilities
//!
//! @section hal_mcuctrl_configuration Configuration
//!
//! - @b Device @b Info: Access chip ID, revision, and vendor data
//! - @b Memory @b Sizes: Retrieve ITCM, DTCM, and SSRAM configurations
//! - @b External @b Clocks: Configure external clock sources
//! - @b System @b Control: Set up system control operations
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
// This is part of revision release_sdk5_2_a_1-29944d3085 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
//
//! Lookup table for memory sizes as derived from the SKU register.
//! Apollo510L has 256KB DTCM, 1.7M SSRAM, no ITCM(0KB).
//!  0: ITCM    size in KB
//!  1: DTCM    size in KB
//!  2: SSRAM   size in KB
//
static const uint16_t
g_am_hal_mcuctrl_sku_ram_size[AM_HAL_MCUCTRL_SKU_SSRAM_SIZE_N][3] =
{
    {0, 256, 1024},    //! 0x0:  0KB ITCM + 256KB DTCM + 1024KB SSRAM
    {0, 256, 1792},     //! 0x1: 0KB ITCM + 256KB DTCM + 1792KB SSRAM
};

//
//! Lookup table for MRAM sizes as derived from the SKU register.
//
static const uint16_t
g_am_hal_mcuctrl_sku_mram_size[AM_HAL_MCUCTRL_SKU_MRAM_SIZE_N] =
{
     1024,
     2048,
};

// ****************************************************************************
//
//  device_info_get()
//  Gets all relevant device information.
//
// ****************************************************************************
#define JEDEC_RD_DELAY

static void
device_info_get(am_hal_mcuctrl_device_t *psDevice)
{
    //
    // Read the Part Number.
    //
    psDevice->ui32ChipPN = MCUCTRL->CHIPPN;

    //
    // Read the Chip ID0.
    //
    psDevice->ui32ChipID0 = MCUCTRL->CHIPID0;

    //
    // Read the Chip ID1.
    //
    psDevice->ui32ChipID1 = MCUCTRL->CHIPID1;

    //
    // Read the Chip Revision.
    //
    psDevice->ui32ChipRev = MCUCTRL->CHIPREV;

    //
    // Read the Chip VENDOR ID.
    //
    psDevice->ui32VendorID = MCUCTRL->VENDORID;

    //
    // Read the SKU.
    //
    psDevice->ui32SKU = MCUCTRL->SKU;

    //
    // Qualified from Part Number.
    //
    psDevice->ui32Qualified = 1;

    //
    // MRAM size as derived from the SKU register.
    //
    psDevice->ui32MRAMSize = g_am_hal_mcuctrl_sku_mram_size[MCUCTRL->SKU_b.SKUMRAMSIZE] * 1024;

    //
    // ITCM size as derived from the SKU register.
    //
    psDevice->ui32ITCMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][0] * 1024;

    //
    // DTCM size as derived from the SKU register.
    //
    psDevice->ui32DTCMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][1] * 1024;

    //
    // Shared SRAM size as derived from the SKU register.
    //
    psDevice->ui32SSRAMSize = g_am_hal_mcuctrl_sku_ram_size[MCUCTRL->SKU_b.SKUSRAMSIZE][2] * 1024;

    //
    // Now, let's look at the JEDEC info.
    // The full partnumber is 12 bits total, but is scattered across 2 registers.
    // Bits [11:8] are 0xE.
    // Bits [7:4] are 0xE for Apollo, 0xD for Apollo2.
    // Bits [3:0] are defined differently for Apollo and Apollo2.
    //   For Apollo, the low nibble is 0x0.
    //   For Apollo2, the low nibble indicates flash and SRAM size.
    //
    psDevice->ui32JedecPN  = JEDEC->PID0_b.PNL8 << 0;
    JEDEC_RD_DELAY
    psDevice->ui32JedecPN |= JEDEC->PID1_b.PNH4 << 8;
    JEDEC_RD_DELAY

    //
    // JEPID is the JEP-106 Manufacturer ID Code, which is assigned to Ambiq as
    //  0x1B, with parity bit is 0x9B.  It is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecJEPID  = JEDEC->PID1_b.JEPIDL << 0;
    JEDEC_RD_DELAY
    psDevice->ui32JedecJEPID |= JEDEC->PID2_b.JEPIDH << 4;
    JEDEC_RD_DELAY

    //
    // CHIPREV is 8 bits located across 2 registers.
    //
    psDevice->ui32JedecCHIPREV  = JEDEC->PID2_b.CHIPREVH4 << 4;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCHIPREV |= JEDEC->PID3_b.CHIPREVL4 << 0;
    JEDEC_RD_DELAY

    //
    // Let's get the Coresight ID (32-bits across 4 registers)
    // For Apollo and Apollo2, it's expected to be 0xB105100D.
    //
    psDevice->ui32JedecCID  = JEDEC->CID3_b.CID << 24;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID2_b.CID << 16;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID1_b.CID <<  8;
    JEDEC_RD_DELAY
    psDevice->ui32JedecCID |= JEDEC->CID0_b.CID <<  0;
    JEDEC_RD_DELAY

} // device_info_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_control()
//  Apply various specific commands/controls on the MCUCTRL module.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_control(am_hal_mcuctrl_control_e eControl, void *pArgs)
{
    volatile uint32_t ui32Reg;

    switch ( eControl )
    {
        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_ENABLE:
            //
            // Configure the bits in XTALCTRL that enable external 32KHz clock.
            // This API defaults to enabling EXTCLK32K in XTAL mode.
            // To initialize as EXT clock mode, pArgs shall be pointer to a
            // boolean variable containing value "true".
            //
            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALPDNB_Msk                      |
                         MCUCTRL_XTALCTRL_XTALCOMPPDNB_Msk                  |
                         MCUCTRL_XTALCTRL_XTALCOMPBYPASS_Msk                |
                         MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);

            //
            if ( (pArgs != NULL) && (*((bool *)pArgs) == true) )
            {
                ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRDNCORE)       |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRDNCOMP)   |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_BYPCOMP)   |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            }
            else
            {
                ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALPDNB,       MCUCTRL_XTALCTRL_XTALPDNB_PWRUPCORE)       |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPPDNB,   MCUCTRL_XTALCTRL_XTALCOMPPDNB_PWRUPCOMP)   |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOMPBYPASS, MCUCTRL_XTALCTRL_XTALCOMPBYPASS_USECOMP)   |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                           _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_EN);
            }
            MCUCTRL->XTALCTRL = ui32Reg;
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32K_DISABLE:
            //
            // Configure the bits in XTALCTRL that disable external 32KHz
            // clock, thus re-configuring for the crystal.
            //
            ui32Reg  = MCUCTRL->XTALCTRL;
            ui32Reg &= ~(MCUCTRL_XTALCTRL_XTALCOREDISFB_Msk                 |
                         MCUCTRL_XTALCTRL_XTALSWE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALCTRL_XTALCOREDISFB,  MCUCTRL_XTALCTRL_XTALCOREDISFB_EN)         |
                       _VAL2FLD(MCUCTRL_XTALCTRL_XTALSWE,        MCUCTRL_XTALCTRL_XTALSWE_OVERRIDE_DIS);
            MCUCTRL->XTALCTRL = ui32Reg;
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_control()


uint32_t
am_hal_mcuctrl_extclk32k_status_get(am_hal_mcuctrl_ext32k_status_e *peStatus)
{
    if (MCUCTRL->XTALCTRL_b.XTALSWE)
    {
        if ( MCUCTRL->XTALCTRL_b.XTALCOMPBYPASS )
        {
            *peStatus = AM_HAL_MCUCTRL_EXT32K_STATUS_EXT_CLK;
        }
        else
        {
            *peStatus = AM_HAL_MCUCTRL_EXT32K_STATUS_XTAL;
        }
    }
    else
    {
        *peStatus = AM_HAL_MCUCTRL_EXT32K_STATUS_OFF;
    }


    return AM_HAL_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  am_hal_mcuctrl_status_get()
// This function returns  current status of the MCUCTRL as obtained from
// various registers of the MCUCTRL block.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_status_get(am_hal_mcuctrl_status_t *psStatus)
{
    uint32_t ui32Status;

    if ( psStatus == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    psStatus->bDebuggerLockout =
        _FLD2VAL(MCUCTRL_DEBUGGER_LOCKOUT, MCUCTRL->DEBUGGER);

    psStatus->bADCcalibrated =
        _FLD2VAL(MCUCTRL_ADCCAL_ADCCALIBRATED, MCUCTRL->ADCCAL);

    psStatus->bBattLoadEnabled =
        _FLD2VAL(MCUCTRL_ADCBATTLOAD_BATTLOAD, MCUCTRL->ADCBATTLOAD);

    ui32Status = MCUCTRL->BOOTLOADER;

    psStatus->bSecBootOnColdRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOT, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOT_ERROR);
    psStatus->bSecBootOnWarmRst =
        (_FLD2VAL(MCUCTRL_BOOTLOADER_SECBOOTONRST, ui32Status) != MCUCTRL_BOOTLOADER_SECBOOTONRST_ERROR);

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_status_get()

// ****************************************************************************
//
//  trim_version_get()
//  Get TRIM version which is stored in INFO1.
//
// ****************************************************************************
static uint32_t
trim_version_get(am_hal_mcuctrl_feature_t *psFeature)
{
    uint32_t ui32Ret, ui32TrimVer;

    //
    // Get trim information from the Apollo510L device.
    // For Apollo510L, this data is stored in INFO1 and is typically encoded
    // specifically for different silicon versions.
    //
    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
                                AM_REG_OTP_INFO1_TRIM_REV_O / 4,
                                1,
                                &ui32TrimVer);

    psFeature->ui32trimver = 0x0;   // Initialize the trim data
    if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
    {
        if (ui32TrimVer <= 254)
        {
            psFeature->trimver_b.ui8TrimVerMaj = 2;
            psFeature->trimver_b.ui8TrimVerMin = (uint8_t)ui32TrimVer + 1;
            psFeature->trimver_b.bTrimVerPCM   = 1;
            psFeature->trimver_b.bTrimVerValid = 1;
        }
        else
        {
            //
            // Default to non-PCM numbered device.
            //
            psFeature->trimver_b.ui8TrimVerMaj = 0;
            psFeature->trimver_b.ui8TrimVerMin = (uint8_t)ui32TrimVer;
            psFeature->trimver_b.bTrimVerPCM   = 0;
            psFeature->trimver_b.bTrimVerValid = 1;
        }
    }
    else
    {
        psFeature->trimver_b.ui8TrimVerMaj = 0;
        psFeature->trimver_b.ui8TrimVerMin = (uint8_t)ui32TrimVer;
        psFeature->trimver_b.bTrimVerPCM   = 0;
        psFeature->trimver_b.bTrimVerValid = 0;
    }

    return ui32Ret;
} // trim_version_get()

// ****************************************************************************
//
//  am_hal_mcuctrl_info_get()
//  Get information of the given MCUCTRL item.
//
// ****************************************************************************
uint32_t
am_hal_mcuctrl_info_get(am_hal_mcuctrl_infoget_e eInfoGet, void *pInfo)
{
    if ( pInfo == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    am_hal_mcuctrl_feature_t *psFeature = (am_hal_mcuctrl_feature_t *)pInfo;

    switch ( eInfoGet )
    {
        case AM_HAL_MCUCTRL_INFO_FEATURES_AVAIL:
            //
            // Decode ITCM, DTCM, SSRAM sizes.
            //
            switch(MCUCTRL->SKU_b.SKUSRAMSIZE)
            {
                case 0:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_1M;
                    break;
                case 1:
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_1P75M;
                    break;
            }
            //
            // Decode MRAM sizes.
            //
            psFeature->eMRAMSize        = (am_hal_mcuctrl_mram_e)MCUCTRL->SKU_b.SKUMRAMSIZE;
            psFeature->bSupportHPMode   = (MCUCTRL->SKU_b.SKUTURBOSPOT > 0);
            psFeature->bMIPIDSI         = (MCUCTRL->SKU_b.SKUMIPIDSI > 0);
            psFeature->bGPU             = (MCUCTRL->SKU_b.SKUGFX > 0);
            psFeature->bUSB             = (MCUCTRL->SKU_b.SKUUSB > 0);
            psFeature->bSecBootFeature  = (MCUCTRL->SKU_b.SKUSECURESPOT > 0);
            psFeature->bFPU             = (MCUCTRL->SKU_b.SKUFPU > 0);
            psFeature->eMVECfg          = (am_hal_mcuctrl_mve_e)MCUCTRL->SKU_b.SKUMVE;
            psFeature->bDISP            = (MCUCTRL->SKU_b.SKUDISP > 0);
            psFeature->bRadioDisable    = (MCUCTRL->SKU_b.SKURADIODISABLE > 0);
            psFeature->bRadioBTBLE      = (MCUCTRL->SKU_b.SKURADIOBTBLE > 0);
            psFeature->bRadioBLE        = (MCUCTRL->SKU_b.SKURADIOMM > 0);
            psFeature->bCM4DEBUG        = (MCUCTRL->SKU_b.SKUCM4DEBUG > 0);

            trim_version_get(psFeature);
            break;

        case AM_HAL_MCUCTRL_INFO_DEVICEID:
            device_info_get((am_hal_mcuctrl_device_t *)pInfo);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return success status.
    //
    return AM_HAL_STATUS_SUCCESS;

} // am_hal_mcuctrl_info_get()

//*****************************************************************************
//
// am_hal_mcuctrl_finecnt_get()
// Read Radio Fine Counter with 0.5us precision
//
//*****************************************************************************
uint32_t
am_hal_mcuctrl_finecnt_get(void)
{
    uint32_t ui32CurrVal;

    ui32CurrVal = MCUCTRL->RADIOFINECNT;
    return ui32CurrVal;
} // am_hal_mcuctrl_finecnt_get()

//*****************************************************************************
//
// am_hal_mcuctrl_clkncnt_get()
// Read Radio CLKN Counter with a 312.5us precision
//
//*****************************************************************************
uint32_t
am_hal_mcuctrl_clkncnt_get(void)
{
    uint32_t ui32CurrVal;

    ui32CurrVal = MCUCTRL->RADIOCLKNCNT;
    return ui32CurrVal;
} // am_hal_mcuctrl_clkncnt_get()

//*****************************************************************************
//
// am_hal_mcuctrl_usb_phy_ldo0p9_enable()
// USB PHY LDO (0.9V) Switch Control
//
//*****************************************************************************
void
am_hal_mcuctrl_usb_phy_ldo0p9_enable(bool bEnable)
{
    if ( bEnable )
    {
        // Disable the pulldown to disable USB LDO active discharge.
        MCUCTRL->USBLDOCTRL |= MCUCTRL_USBLDOCTRL_USBLDOPULLDNDIS_Msk;
        // Enable USB LDO.
        MCUCTRL->USBLDOCTRL |= MCUCTRL_USBLDOCTRL_USBLDOPDNB_Msk ;
    }
    else
    {
        //Disable USB LDO.
        MCUCTRL->USBLDOCTRL &= ~MCUCTRL_USBLDOCTRL_USBLDOPDNB_Msk;
        //Enable the pulldown to enable USB LDO active discharge.
        MCUCTRL->USBLDOCTRL &= ~MCUCTRL_USBLDOCTRL_USBLDOPULLDNDIS_Msk;
    }
}

//*****************************************************************************
//
// am_hal_mcuctrl_i3c_phy_pulldown_enable()
// I3CPHY Pulldown Swiitch Control
//
//*****************************************************************************
void
am_hal_mcuctrl_i3c_phy_pulldown_enable(bool bEnable)
{
    if ( bEnable )
    {
        MCUCTRL->I3CPHYCTRL = MCUCTRL_I3CPHYCTRL_I3CPHYPULLDN_PULLDNEN;
    }
    else
    {
        MCUCTRL->I3CPHYCTRL = MCUCTRL_I3CPHYCTRL_I3CPHYPULLDN_PULLDNDIS;
    }
} // am_hal_mcuctrl_i3c_phy_pulldown_enable

// ****************************************************************************
//
//  am_hal_mcuctrl_subsys_codebase_set()
//  Radio subsystem code base address from 0x400000 to 0x5FFFFF(2MB),16Bytes align
// ****************************************************************************
uint32_t
am_hal_mcuctrl_rss_codebase_set(uint32_t ui32Codebase_addr)
{
    //Check 16Bytes align
    if ( ui32Codebase_addr & 0xF )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    else if ( (ui32Codebase_addr < 0x400000) || (ui32Codebase_addr > 0x5FFFFF) )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    MCUCTRL->CM4CODEBASE = ui32Codebase_addr;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
