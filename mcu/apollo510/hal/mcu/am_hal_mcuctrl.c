//*****************************************************************************
//
//! @file am_hal_mcuctrl.c
//!
//! @brief Functions for interfacing with the MCUCTRL.
//!
//! @addtogroup mcuctrl4 MCUCTRL - MCU Control
//! @ingroup apollo510_hal
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
// This is part of revision release_sdk5p0p0-5f68a8286b of the AmbiqSuite Development Package.
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
//!  0: ITCM    size in KB
//!  1: DTCM    size in KB
//!  2: SSRAM   size in KB
//
static const uint16_t
g_am_hal_mcuctrl_sku_ram_size[AM_HAL_MCUCTRL_SKU_SSRAM_SIZE_N][3] =
{
    {128, 256, 1024},    //! 0x0:  128KB ITCM + 256KB DTCM + 1024KB SSRAM
    {128, 256, 2048},    //! 0x1:  128KB ITCM + 256KB DTCM + 2048KB SSRAM
    {256, 512, 2048},    //! 0x2:  256KB ITCM + 512KB DTCM + 2048KB SSRAM
    {256, 512, 3072},    //! 0x3:  256KB ITCM + 512KB DTCM + 3072KB SSRAM
};

//
//! Lookup table for MRAM sizes as derived from the SKU register.
//
static const uint16_t
g_am_hal_mcuctrl_sku_mram_size[AM_HAL_MCUCTRL_SKU_MRAM_SIZE_N] =
{
     1024,
     2048,
     3072,
     4096
};

// ****************************************************************************
// MCUCTRL XTALHSCAP Globals for Cooper Device
// Refer to App Note Apollo4 Blue 32MHz Crystal Calibration
// ****************************************************************************
uint32_t g_ui32xtalhscap2trim = XTALHSCAP2TRIM_DEFAULT;
uint32_t g_ui32xtalhscaptrim = XTALHSCAPTRIM_DEFAULT;

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
    uint32_t ui32Status;

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

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START:
            //
            // Clear the bit fields to be configured.
            //
            ui32Reg = MCUCTRL->XTALHSTRIMS;
            ui32Reg &= ~(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Msk       |
                         MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Msk        |
                         MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM_Msk |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM_Msk  |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM_Msk         |
                         MCUCTRL_XTALHSTRIMS_XTALHSSPARE_Msk);
            //
            // Set the default trim code for CAP1/CAP2, it impacts frequency accuracy and should be retrimmed
            //
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
            //
            // Set the transconductance of crystal to maximum, it accelerate the startup sequence
            //
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3)                      |
            //
            // Tune the bias generator
            //
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15)                 |
            //
            // Set the bias of crystal to maximum
            //
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127)                    |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
            //
            // Enable xtalhs_comp and pn_improve
            //
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk              |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);

            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)       |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            //
            // Turn on crystal oscillator
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSPDNB = 1;
            //
            // Inject HFRC clock to accelerate the startup sequence
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSINJECTIONENABLE = 1;
            //
            // Turn on xtalhs_ibst_enable
            // Maximize the bias current to accelerate the startup sequence
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSIBSTENABLE = 1;
            //
            // Wait 5us to make the setting above effective
            //
            am_hal_delay_us(5);
            //
            // Turn off the clock injection
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSINJECTIONENABLE = 0;
            //
            // Apply external source
            //
            if ( (pArgs != NULL) && (*((bool *)pArgs) == true) )
            {
                ui32Reg = MCUCTRL->XTALHSCTRL;
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                 |
                              MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk           |
                              MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)          |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0)    |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
                MCUCTRL->XTALHSCTRL = ui32Reg;
            }

            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_NORMAL:
            //
            // Clear the bit fields to be configured.
            //
            ui32Reg = MCUCTRL->XTALHSTRIMS;
            ui32Reg &= ~(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Msk       |
                         MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Msk        |
                         MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM_Msk |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM_Msk  |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM_Msk         |
                         MCUCTRL_XTALHSTRIMS_XTALHSSPARE_Msk);
            //
            // Set the default trim code for CAP1/CAP2, it impacts frequency accuracy and should be retrimmed
            //
            ui32Reg = _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
            //
            // Set the transconductance of crystal to maximum, it accelerate the startup sequence
            //
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 3)                      |
            //
            // Tune the bias generator
            //
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 15)                 |
            //
            // Set the bias of crystal to maximum
            //
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 127)                    |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                      _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
            //
            // Enable xtalhs_comp and pn_improve
            //
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk                  |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)           |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 1);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            //
            // Turn on xtalhs_pdnb
            //
            MCUCTRL->XTALHSCTRL_b.XTALHSPDNB = 1;
            //
            // Apply external source
            //
            ui32Reg = MCUCTRL->XTALHSCTRL;
            if ( (pArgs != NULL) && (*((bool *)pArgs) == true) )
            {
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                 |
                              MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)          |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 1);
            }
            else
            {
                ui32Reg &=  ~(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk         |
                              MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk);
                ui32Reg |=  _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0)  |
                            _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 1);
            }

            MCUCTRL->XTALHSCTRL = ui32Reg;
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_DISABLE:
            //
            // Clear the bit fields to be configured.
            //
            ui32Reg = MCUCTRL->XTALHSTRIMS;
            ui32Reg &= ~(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM_Msk       |
                         MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM_Msk        |
                         MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM_Msk |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM_Msk  |
                         MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM_Msk      |
                         MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM_Msk         |
                         MCUCTRL_XTALHSTRIMS_XTALHSSPARE_Msk);
            //
            // Disable EXTCLK32M
            //
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAP2TRIM, g_ui32xtalhscap2trim)    |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSCAPTRIM, g_ui32xtalhscaptrim)      |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVETRIM, 0)                      |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMP2TRIM, 3)                 |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASCOMPTRIM, 8)                  |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSIBIASTRIM, 24)                     |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSRSTRIM, 0)                         |
                       _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSSPARE, 0);
            MCUCTRL->XTALHSTRIMS = ui32Reg;
            //
            // Power down XTALHS and XTALHS_COMP, disable external clock and BIST, PN_IMPROVE.
            //
            ui32Reg  = MCUCTRL->XTALHSCTRL;
            ui32Reg &= ~(MCUCTRL_XTALHSCTRL_XTALHSPDNB_Msk                  |
                         MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK_Msk         |
                         MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB_Msk              |
                         MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE_Msk            |
                         MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE_Msk);
            ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNB, 0)           |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK, 0)  |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSCOMPPDNB, 1)       |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSIBSTENABLE, 0)     |
                       _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPDNPNIMPROVE, 0);
            MCUCTRL->XTALHSCTRL = ui32Reg;
            break;

        //
        // Application should set GP46 to the function AM_HAL_PIN_FN_CLKOUT_32M
        // before caliing AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_CLKOUT_ENABLE.
        //
        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_CLKOUT_ENABLE:
            //
            // Request clock
            //
            ui32Status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_XTAL_HS,
                                                     AM_HAL_CLKMGR_USER_ID_CLKOUT_XTAL);
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }
            //
            // Set drive strength
            //
            if (pArgs != NULL)  // check if pArgs is valid
            {
                uint32_t ui32Argval = *((uint32_t *)pArgs);
                if ((ui32Argval == 0) || ((ui32Argval >= 3) && (ui32Argval <= 7)))
                {
                    ui32Reg = MCUCTRL->XTALHSTRIMS;
                    ui32Reg &= ~MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH_Msk;
                    ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, *((uint32_t *)pArgs));
                    MCUCTRL->XTALHSTRIMS = ui32Reg;
                }
                else
                {
                    return AM_HAL_STATUS_INVALID_ARG;
                }
            }
            else // Fix drive strength
            {
                ui32Reg = MCUCTRL->XTALHSTRIMS;
                ui32Reg &= ~MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH_Msk;
                ui32Reg |= _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 0x4);
                MCUCTRL->XTALHSTRIMS = ui32Reg;
            }
            //
            // Set XTALHSPADOUTEN
            //
            MCUCTRL->XTALHSCTRL |= _VAL2FLD(MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN, 1);
            break;

        case AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_CLKOUT_DISABLE:
            //
            // Set drive strength to 7 - all buffers off
            //
            MCUCTRL->XTALHSTRIMS |= _VAL2FLD(MCUCTRL_XTALHSTRIMS_XTALHSDRIVERSTRENGTH, 7);
            //
            // Clear XTALHSPADOUTEN
            //
            MCUCTRL->XTALHSCTRL &= ~MCUCTRL_XTALHSCTRL_XTALHSPADOUTEN_Msk;
            //
            // Release clock
            //
            ui32Status = am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_XTAL_HS,
                                                     AM_HAL_CLKMGR_USER_ID_CLKOUT_XTAL);
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }
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

uint32_t
am_hal_mcuctrl_extclk32m_status_get(am_hal_mcuctrl_ext32m_status_e *peStatus)
{

    // MCUCTRL_XTALHSCTRL_XTALHSEXTERNALCLOCK
    if (MCUCTRL->XTALHSCTRL_b.XTALHSEXTERNALCLOCK)
    {
        *peStatus = AM_HAL_MCUCTRL_EXT32M_STATUS_EXT_CLK;
    }
    else if (MCUCTRL->XTALHSCTRL_b.XTALHSPDNB)
    {
        *peStatus = AM_HAL_MCUCTRL_EXT32M_STATUS_XTAL;
    }
    else
    {
        *peStatus = AM_HAL_MCUCTRL_EXT32M_STATUS_OFF;
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
    // Get trim information from the Apollo510 device.
    // For Apollo510, this data is stored in INFO1 and is typically encoded
    // specifically for different silicon versions.
    //
    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1,
                                AM_REG_OTP_INFO1_TRIM_REV_O / 4,
                                1,
                                &ui32TrimVer);

    psFeature->ui32trimver = 0x0;   // Initialize the trim data
    if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
    {
        if ( (MCUCTRL->CHIPREV_b.REVMAJ == MCUCTRL_CHIPREV_REVMAJ_B)    &&
             (MCUCTRL->CHIPREV_b.REVMIN >= MCUCTRL_CHIPREV_REVMIN_REV1) &&
             (ui32TrimVer <= 254) )
        {
            //
            // This is a PCM 2.x device
            // Apollo510 B2 PCM trim is numbered as 0-based.
            // All other Apollo510 chip revisions are 1-based.
            //
            if ( MCUCTRL->CHIPREV_b.REVMIN != MCUCTRL_CHIPREV_REVMIN_REV2 )
            {
                //
                // Trimcode is 1-based, 0-base it (only B2 is 0-based).
                //
                --ui32TrimVer;
            }
            psFeature->trimver_b.ui8TrimVerMaj = 2;
            psFeature->trimver_b.ui8TrimVerMin = (uint8_t)ui32TrimVer;
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
                    psFeature->eITCMSize        = AM_HAL_MCUCTRL_ITCM_128K;
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_1M;
                    break;
                case 1:
                    psFeature->eITCMSize        = AM_HAL_MCUCTRL_ITCM_128K;
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_256K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_2M;
                    break;
                case 2:
                    psFeature->eITCMSize        = AM_HAL_MCUCTRL_ITCM_256K;
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_512K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_2M;
                    break;
                case 3:
                    psFeature->eITCMSize        = AM_HAL_MCUCTRL_ITCM_256K;
                    psFeature->eDTCMSize        = AM_HAL_MCUCTRL_DTCM_512K;
                    psFeature->eSharedSRAMSize  = AM_HAL_MCUCTRL_SSRAM_3M;
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
// End Doxygen group.
//! @}
//
//*****************************************************************************
