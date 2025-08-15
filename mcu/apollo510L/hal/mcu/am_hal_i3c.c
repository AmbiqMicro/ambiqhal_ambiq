//*****************************************************************************
//
//! @file am_hal_i3c.c
//!
//! @brief Hardware abstraction for the Improved Inter-Integrated Circuit (I3C).
//!
//! @addtogroup i3c_ap510L I3C - Improved Inter-Integrated Circuit
//! @ingroup apollo510L_hal
//! @{
//!
//! Purpose: This module provides a high-level interface for the I3C (Improved
//! Inter-Integrated Circuit) peripheral in Apollo5 devices. It supports advanced
//! communication features beyond traditional I2C while maintaining backward
//! compatibility.
//!
//! @section hal_i3c_features Key Features
//!
//! 1. @b High @b Speed: Support for higher data rates than I2C.
//! 2. @b Hot-Join: Dynamic attachment of new devices.
//! 3. @b In-Band @b Interrupts: Device-initiated communication.
//! 4. @b Multi-Protocol: I3C and legacy I2C support.
//! 5. @b Advanced @b Control: Enhanced command and data handling.
//!
//! @section hal_i3c_functionality Functionality
//!
//! - Initialize and configure I3C interface
//! - Support multiple transfer modes
//! - Handle dynamic addressing
//! - Process in-band interrupts
//! - Manage device discovery
//!
//! @section hal_i3c_usage Usage
//!
//! 1. Configure I3C controller parameters
//! 2. Set up device addressing and discovery
//! 3. Perform data transfers
//! 4. Handle device events and interrupts
//!
//! @section hal_i3c_configuration Configuration
//!
//! - @b Speed @b Modes: Configure communication speeds
//! - @b Address @b Settings: Set up device addressing
//! - @b Transfer @b Options: Configure data transfer modes
//! - @b Interrupt @b Handling: Set up event processing
//!
//
//*****************************************************************************

//*****************************************************************************
//
// ${}
//copyright
// This is part of revision release_sdk5_2_a_1-29944d3085 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "am_mcu_apollo.h"

#include "am_util_stdio.h"
#include "am_util_debug.h"

//*****************************************************************************
//
// Private Types.
//
//*****************************************************************************

#define AM_HAL_MAGIC_I3C 0x3C3C3C
#define AM_HAL_I3C_CHK_HANDLE(h)                                                                                      \
    ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_I3C))

#define AM_HAL_I3C_DEBUG(fmt, ...) am_util_debug_printf("[I3C HAL] line %04d - "fmt, __LINE__, ##__VA_ARGS__)

//
//! Power Save-Restore register state
//
typedef struct
{
    bool     bValid;
    uint32_t regHCCONTROL;
    uint32_t regRHSCONTROL;
    uint32_t regCHUNKCONTROL;
    uint32_t regRHCONTROL;
    uint32_t regRESETCTRL;
    uint32_t regIBINOTIFYCTRL;
    uint32_t regQUEUETHLDCTRL;
    uint32_t regQUEUESIZECTRL;
    uint32_t regINTRSTATUSENABLE;
    uint32_t regINTRSIGNALENABLE;
    uint32_t regPIOINTRSTATUSENABLE;
    uint32_t regPIOINTRSIGNALENABLE;
    uint32_t regRHINTRSTATUSENABLE;
    uint32_t regRHINTRSIGNALENABLE;
    uint32_t regDEVICEADDR;
    uint32_t regDAT0;
    uint32_t regDAT1;
    uint32_t regCRSETUP;
    uint32_t regIBISETUP;
    uint32_t regRHCMDRINGBASELO;
    uint32_t regRHCMDRINGBASEHI;
    uint32_t regRHRESPRINGBASELO;
    uint32_t regRHRESPRINGBASEHI;
    uint32_t regRHIBISTATUSRINGBASELO;
    uint32_t regRHIBISTATUSRINGBASEHI;
    uint32_t regRHIBIDATARINGBASELO;
    uint32_t regRHIBIDATARINGBASEHI;
    uint32_t regI2CURD1SCLLOWCNT;
    uint32_t regI2CURD1SCLHIGHCNT;
    uint32_t regI2CURD2SCLLOWCNT;
    uint32_t regI2CURD2SCLHIGHCNT;
    uint32_t regI2CURD3SCLLOWCNT;
    uint32_t regI2CURD3SCLHIGHCNT;
} am_hal_i3c_register_state_t;

//
//! I3C State structure.
//
typedef struct
{
    //
    //! Handle validation prefix.
    //
    am_hal_handle_prefix_t prefix;

    //
    //! Physical module number.
    //
    uint32_t ui32Module;

    //
    //! Link to the I3C host
    //
    am_hal_i3c_host_t *pHost;

    //
    //! Link to the I3C device
    //
    am_hal_i3c_device_t *pDevice;

    //
    //! Store the command information
    //
    uint32_t ui32CmdDesc0;
    uint32_t ui32CmdDesc1;
    uint32_t ui32Resp;

    //
    //! Store the data transfer information
    //
    uint32_t *pui32Buf;
    uint32_t ui32DataLen;
    uint32_t ui32DataLeft;
    uint32_t ui32DelayUs;
    am_hal_i3c_dir_e eDir;

    //
    //! Store IBI information
    //
    uint32_t *pui32IbiStatBuf;
    uint32_t *pui32IbiDataBuf;

    //
    //! Power Save-Restore register state
    //
    am_hal_i3c_register_state_t registerState;
} am_hal_i3c_state_t;

//*****************************************************************************
//
// Global Variables.
//
//*****************************************************************************
static am_hal_i3c_state_t g_I3C_State[AM_REG_I3C_NUM_MODULES];
static am_hal_i3c_host_t g_I3C_Host[AM_REG_I3C_NUM_MODULES];
static am_hal_i3c_ring_t g_I3C_Ring[AM_REG_I3C_NUM_MODULES];
static uint8_t gui8I3Ctid[AM_REG_I3C_NUM_MODULES] = {0};

//*****************************************************************************
//
// Internal Functions.
//
//*****************************************************************************


//
// Prepare for I3C Dynamic Address Assignment
//
static uint32_t am_hal_i3c_prepare_daa(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    am_hal_i3c_cmd_desc0_t cmd_desc0;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));

    cmd_desc0.DaaCmdDesc0.attr   = 2;
    cmd_desc0.DaaCmdDesc0.tid    = 0;
    cmd_desc0.DaaCmdDesc0.cmd    = pCmdData->ui8Id;
    cmd_desc0.DaaCmdDesc0.rsvd0  = 0;
    cmd_desc0.DaaCmdDesc0.idx    = 0;
    cmd_desc0.DaaCmdDesc0.rsvd1  = 0;
    cmd_desc0.DaaCmdDesc0.devcnt = 1;
    cmd_desc0.DaaCmdDesc0.roc    = 1;
    cmd_desc0.DaaCmdDesc0.toc    = 1;

    pCmdData->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData->ui32CmdDesc1 = 0;

    pCmdData->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
    pCmdData->ui32CmdDesc0 |= AM_HAL_I3C_CMD0_DESPC_TOC | AM_HAL_I3C_CMD0_DESPC_ROC | ((uint32_t)pCmdData->ui8Tid << 3);
    gui8I3Ctid[pI3CState->ui32Module]++;

    pI3CState->ui32CmdDesc0 = pCmdData->ui32CmdDesc0;
    pI3CState->ui32CmdDesc1 = pCmdData->ui32CmdDesc1;

    AM_HAL_I3C_DEBUG("DAA CmdDesc0 = 0x%x\n", pCmdData->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("DAA CmdDesc1 = 0x%x\n", pCmdData->ui32CmdDesc1);

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare for sending Common Command Code (CCC)
//
static uint32_t am_hal_i3c_prepare_ccc(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    am_hal_i3c_cmd_desc0_t cmd_desc0;
    am_hal_i3c_cmd_desc1_t cmd_desc1;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));
    memset(&cmd_desc1, 0, sizeof(cmd_desc1));

    if ( pCmdData->eDir == AM_HAL_I3C_DIR_WRITE &&  pCmdData->ui32DataLen <= 4 )
    {
        cmd_desc0.ImmCmdDesc0.attr = 1;
        cmd_desc0.ImmCmdDesc0.tid  = 0;
        cmd_desc0.ImmCmdDesc0.cmd  = pCmdData->ui8Id;
        cmd_desc0.ImmCmdDesc0.cp   = 1;
        cmd_desc0.ImmCmdDesc0.idx  = 0;
        cmd_desc0.ImmCmdDesc0.rsvd = 0;
        cmd_desc0.ImmCmdDesc0.dtt  = pCmdData->ui32DataLen;
        cmd_desc0.ImmCmdDesc0.mode = pCmdData->eSpeedMode;
        cmd_desc0.ImmCmdDesc0.rnw  = 0;
        cmd_desc0.ImmCmdDesc0.roc  = 0;
        cmd_desc0.ImmCmdDesc0.toc  = 0;

        if (pCmdData->ui32DataLen >= 4)
        {
            cmd_desc1.ImmCmdDesc1.byte4 = *(uint8_t *)(pCmdData->pui8Buf + 3) ;
        }
        if (pCmdData->ui32DataLen >= 3)
        {
            cmd_desc1.ImmCmdDesc1.byte3 = *(uint8_t *)(pCmdData->pui8Buf + 2) ;
        }
        if (pCmdData->ui32DataLen >= 2)
        {
            cmd_desc1.ImmCmdDesc1.byte2 = *(uint8_t *)(pCmdData->pui8Buf + 1) ;
        }
        if (pCmdData->ui32DataLen >= 1)
        {
            cmd_desc1.ImmCmdDesc1.byte1 = *(uint8_t *)(pCmdData->pui8Buf) ;
        }

    }
    else
    {
        cmd_desc0.RegCmdDesc0.attr = 0;
        cmd_desc0.RegCmdDesc0.tid  = 0;
        cmd_desc0.RegCmdDesc0.cmd  = pCmdData->ui8Id;
        cmd_desc0.RegCmdDesc0.cp   = 1;
        cmd_desc0.RegCmdDesc0.idx  = 0;
        cmd_desc0.RegCmdDesc0.rsvd = 0;
        cmd_desc0.RegCmdDesc0.sre  = 0;
        cmd_desc0.RegCmdDesc0.dbp  = 0;
        cmd_desc0.RegCmdDesc0.mode = pCmdData->eSpeedMode;
        cmd_desc0.RegCmdDesc0.rnw  = pCmdData->eDir;
        cmd_desc0.RegCmdDesc0.roc  = 0;
        cmd_desc0.RegCmdDesc0.toc  = 0;

        cmd_desc1.RegCmdDesc1.data_len = pCmdData->ui32DataLen;
    }



    pCmdData->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData->ui32CmdDesc1 = cmd_desc1.ui32CmdDesc1;

    pCmdData->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
    pCmdData->ui32CmdDesc0 |= AM_HAL_I3C_CMD0_DESPC_TOC | AM_HAL_I3C_CMD0_DESPC_ROC | ((uint32_t)pCmdData->ui8Tid << 3);
    gui8I3Ctid[pI3CState->ui32Module]++;

    pI3CState->ui32CmdDesc0 = pCmdData->ui32CmdDesc0;
    pI3CState->ui32CmdDesc1 = pCmdData->ui32CmdDesc1;
    pI3CState->pui32Buf = (uint32_t *)(pCmdData->pui8Buf);
    pI3CState->ui32DataLen = pCmdData->ui32DataLen;
    pI3CState->ui32DataLeft = pCmdData->ui32DataLeft;
    pI3CState->eDir = pCmdData->eDir;
    pI3CState->pHost->eSpeedMode = pCmdData->eSpeedMode;
    pI3CState->pHost->pCmdData = pCmdData;

    AM_HAL_I3C_DEBUG("CCC CmdDesc0 = 0x%x\n", pCmdData->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("CCC CmdDesc1 = 0x%x\n", pCmdData->ui32CmdDesc1);

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare for PIO Transfer Command
//
static uint32_t am_hal_i3c_prepare_pio_cmd(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    am_hal_i3c_cmd_desc0_t cmd_desc0;
    am_hal_i3c_cmd_desc1_t cmd_desc1;
    bool bCmdPresent = false;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));
    memset(&cmd_desc1, 0, sizeof(cmd_desc1));

    if ( pCmdData->eSpeedMode == AM_HAL_I3C_HDR_DDR || pCmdData->eSpeedMode == AM_HAL_I3C_HDR_TS )
    {
        bCmdPresent = true;
    }
    else
    {
        bCmdPresent = false;
    }

    if (pCmdData->bComboCmd)
    {
        cmd_desc0.ComboCmdDesc0.attr      = 3;
        cmd_desc0.ComboCmdDesc0.tid       = 0;
        cmd_desc0.ComboCmdDesc0.cmd       = pCmdData->ui8Id;
        cmd_desc0.ComboCmdDesc0.cp        = bCmdPresent;
        cmd_desc0.ComboCmdDesc0.idx       = 0;
        cmd_desc0.ComboCmdDesc0.rsvd      = 0;
        cmd_desc0.ComboCmdDesc0.lenthpos  = 0;
        cmd_desc0.ComboCmdDesc0.firstmode = 1;
        cmd_desc0.ComboCmdDesc0.ofs16bit  = pCmdData->bCombo16bitOfs;
        cmd_desc0.ComboCmdDesc0.mode      = pCmdData->eSpeedMode;
        cmd_desc0.ComboCmdDesc0.rnw       = pCmdData->eDir;
        cmd_desc0.ComboCmdDesc0.roc       = 0;
        cmd_desc0.ComboCmdDesc0.toc       = 0;

        cmd_desc1.ComboCmdDesc1.data_len   = pCmdData->ui32DataLen;
        cmd_desc1.ComboCmdDesc1.sub_offset = pCmdData->ui16ComboOfs;
    }
    else
    {
        cmd_desc0.RegCmdDesc0.attr = 0;
        cmd_desc0.RegCmdDesc0.tid  = 0;
        cmd_desc0.RegCmdDesc0.cmd  = pCmdData->ui8Id;
        cmd_desc0.RegCmdDesc0.cp   = bCmdPresent;
        cmd_desc0.RegCmdDesc0.idx  = 0;
        cmd_desc0.RegCmdDesc0.rsvd = 0;
        cmd_desc0.RegCmdDesc0.sre  = 0;
        cmd_desc0.RegCmdDesc0.dbp  = 0;
        cmd_desc0.RegCmdDesc0.mode = pCmdData->eSpeedMode;
        cmd_desc0.RegCmdDesc0.rnw  = pCmdData->eDir;
        cmd_desc0.RegCmdDesc0.roc  = 0;
        cmd_desc0.RegCmdDesc0.toc  = 0;

        cmd_desc1.RegCmdDesc1.data_len = pCmdData->ui32DataLen;
    }

    pCmdData->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData->ui32CmdDesc1 = cmd_desc1.ui32CmdDesc1;

    pCmdData->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
    pCmdData->ui32CmdDesc0 |= AM_HAL_I3C_CMD0_DESPC_TOC | AM_HAL_I3C_CMD0_DESPC_ROC | ((uint32_t)pCmdData->ui8Tid << 3);
    gui8I3Ctid[pI3CState->ui32Module]++;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare for PIO Transfer Command
//
static uint32_t am_hal_i3c_prepare_combo_i2c_cmd(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData1, am_hal_i3c_cmd_data_t *pCmdData2)
{
    am_hal_i3c_cmd_desc0_t cmd_desc0;
    am_hal_i3c_cmd_desc1_t cmd_desc1;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( !pCmdData1 || !pCmdData2 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));
    memset(&cmd_desc1, 0, sizeof(cmd_desc1));

    cmd_desc0.ImmCmdDesc0.attr = 1;
    cmd_desc0.ImmCmdDesc0.tid  = 0;
    cmd_desc0.ImmCmdDesc0.cmd  = 0;
    cmd_desc0.ImmCmdDesc0.cp   = 0;
    cmd_desc0.ImmCmdDesc0.idx  = 0;
    cmd_desc0.ImmCmdDesc0.rsvd = 0;
    cmd_desc0.ImmCmdDesc0.dtt  = 1;
    cmd_desc0.ImmCmdDesc0.mode = pCmdData1->eSpeedMode;
    cmd_desc0.ImmCmdDesc0.rnw  = 0;
    cmd_desc0.ImmCmdDesc0.roc  = 1;
    cmd_desc0.ImmCmdDesc0.toc  = 0;

    if ( pCmdData1->bCombo16bitOfs )
    {
        cmd_desc1.ImmCmdDesc1.byte1 = (uint8_t)(pCmdData1->ui16ComboOfs >> 8);
        cmd_desc1.ImmCmdDesc1.byte2 = (uint8_t)pCmdData1->ui16ComboOfs;
        cmd_desc0.ImmCmdDesc0.dtt  = 2;
    }
    else
    {
        cmd_desc1.ImmCmdDesc1.byte1 = (uint8_t)pCmdData1->ui16ComboOfs;
    }

    pCmdData1->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData1->ui32CmdDesc1 = cmd_desc1.ui32CmdDesc1;

    pCmdData1->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
    pCmdData1->ui32CmdDesc0 |= (uint32_t)pCmdData1->ui8Tid << 3;
    gui8I3Ctid[pI3CState->ui32Module]++;

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));
    memset(&cmd_desc1, 0, sizeof(cmd_desc1));

    cmd_desc0.RegCmdDesc0.attr = 0;
    cmd_desc0.RegCmdDesc0.tid  = 0;
    cmd_desc0.RegCmdDesc0.cmd  = 0;
    cmd_desc0.RegCmdDesc0.cp   = 0;
    cmd_desc0.RegCmdDesc0.idx  = 0;
    cmd_desc0.RegCmdDesc0.rsvd = 0;
    cmd_desc0.RegCmdDesc0.sre  = 0;
    cmd_desc0.RegCmdDesc0.dbp  = 0;
    cmd_desc0.RegCmdDesc0.mode = pCmdData2->eSpeedMode;
    cmd_desc0.RegCmdDesc0.rnw  = pCmdData2->eDir;
    cmd_desc0.RegCmdDesc0.roc  = 1;
    cmd_desc0.RegCmdDesc0.toc  = 1;

    cmd_desc1.RegCmdDesc1.data_len = pCmdData2->ui32DataLen;

    pCmdData2->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData2->ui32CmdDesc1 = cmd_desc1.ui32CmdDesc1;

    pCmdData2->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
    pCmdData2->ui32CmdDesc0 |= (uint32_t)pCmdData1->ui8Tid << 3;
    gui8I3Ctid[pI3CState->ui32Module]++;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare for DMA Transfer Command
//
static uint32_t am_hal_i3c_prepare_dma_cmd(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    am_hal_i3c_cmd_desc0_t cmd_desc0;
    am_hal_i3c_cmd_desc1_t cmd_desc1;
    bool bCmdPresent = false;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    memset(&cmd_desc0, 0, sizeof(cmd_desc0));
    memset(&cmd_desc1, 0, sizeof(cmd_desc1));

    if ( pCmdData->eSpeedMode == AM_HAL_I3C_HDR_DDR || pCmdData->eSpeedMode == AM_HAL_I3C_HDR_TS )
    {
        bCmdPresent = true;
    }
    else
    {
        bCmdPresent = false;
    }

    if (pCmdData->bComboCmd)
    {
        cmd_desc0.ComboCmdDesc0.attr      = 3;
        cmd_desc0.ComboCmdDesc0.tid       = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
        cmd_desc0.ComboCmdDesc0.cmd       = pCmdData->ui8Id;
        cmd_desc0.ComboCmdDesc0.cp        = bCmdPresent;
        cmd_desc0.ComboCmdDesc0.idx       = 0;
        cmd_desc0.ComboCmdDesc0.rsvd      = 0;
        cmd_desc0.ComboCmdDesc0.lenthpos  = 0;
        cmd_desc0.ComboCmdDesc0.firstmode = 1;
        cmd_desc0.ComboCmdDesc0.ofs16bit  = pCmdData->bCombo16bitOfs;
        cmd_desc0.ComboCmdDesc0.mode      = pCmdData->eSpeedMode;
        cmd_desc0.ComboCmdDesc0.rnw       = pCmdData->eDir;
        cmd_desc0.ComboCmdDesc0.roc       = 1;
        cmd_desc0.ComboCmdDesc0.toc       = 1;

        cmd_desc1.ComboCmdDesc1.data_len   = pCmdData->ui32DataLen;
        cmd_desc1.ComboCmdDesc1.sub_offset = pCmdData->ui16ComboOfs;
    }
    else
    {
        cmd_desc0.RegCmdDesc0.attr = 0;
        cmd_desc0.RegCmdDesc0.tid  = gui8I3Ctid[pI3CState->ui32Module] & 0xF;
        cmd_desc0.RegCmdDesc0.cmd  = pCmdData->ui8Id;
        cmd_desc0.RegCmdDesc0.cp   = bCmdPresent;
        cmd_desc0.RegCmdDesc0.idx  = 0;
        cmd_desc0.RegCmdDesc0.rsvd = 0;
        cmd_desc0.RegCmdDesc0.sre  = 0;
        cmd_desc0.RegCmdDesc0.dbp  = 0;
        cmd_desc0.RegCmdDesc0.mode = pCmdData->eSpeedMode;
        cmd_desc0.RegCmdDesc0.rnw  = pCmdData->eDir;
        cmd_desc0.RegCmdDesc0.roc  = 1;
        cmd_desc0.RegCmdDesc0.toc  = 1;

        cmd_desc1.RegCmdDesc1.data_len = pCmdData->ui32DataLen;
    }

    pCmdData->ui32CmdDesc0 = cmd_desc0.ui32CmdDesc0;
    pCmdData->ui32CmdDesc1 = cmd_desc1.ui32CmdDesc1;
    pCmdData->ui8Tid = gui8I3Ctid[pI3CState->ui32Module] & 0xF;

    gui8I3Ctid[pI3CState->ui32Module]++;

    AM_HAL_I3C_DEBUG("DMA CmdDesc0 = 0x%x\n", pCmdData->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("DMA CmdDesc1 = 0x%x\n", pCmdData->ui32CmdDesc1);

    return AM_HAL_STATUS_SUCCESS;
}

//
// Prepare for I3C Xfer
//
static uint32_t am_hal_i3c_prepare_xfer(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    uint8_t ui8Parity = 0;
    uint8_t ui8DynamicAddr = pI3CState->pDevice->ui8DynamicAddr & 0x7F;
    uint8_t ui8CmdHdrCode = 0;

    while ( ui8DynamicAddr )
    {
        ui8Parity ^= (ui8DynamicAddr & 1);
        ui8DynamicAddr >>= 1;
    }

    ui8DynamicAddr = (pI3CState->pDevice->ui8DynamicAddr & 0x7F) | ((!ui8Parity) << 7);

    if ( pCmdData->eSpeedMode == AM_HAL_I3C_HDR_DDR || pCmdData->eSpeedMode == AM_HAL_I3C_HDR_TS )
    {
        if ( pCmdData->eDir == AM_HAL_I3C_DIR_READ )
        {
            if ( pCmdData->ui8Id > 0x80 )
            {
                ui8CmdHdrCode = pCmdData->ui8Id;
            }
            else
            {
                ui8CmdHdrCode = 0x80;
            }
        }
    }

    pI3C->DAT0 = _VAL2FLD(I3C_DAT0_DEVICE, pI3CState->pDevice->eDeviceType)              |
                 _VAL2FLD(I3C_DAT0_STATICADDRESS, pI3CState->pDevice->ui8StaticAddr)     |
                 _VAL2FLD(I3C_DAT0_IBIPAYLOAD, (pI3CState->pDevice->ui8BCR >> 2) & 0x1)  |
                 _VAL2FLD(I3C_DAT0_DAT0DYNAMICADDRESS, ui8DynamicAddr);
    if ( pI3CState->pDevice->eDeviceType == AM_HAL_I3C_DEVICE_I3C )
    {
        pI3C->DAT1 = _VAL2FLD(I3C_DAT1_AUTOCMDMODE, pCmdData->eSpeedMode) | _VAL2FLD(I3C_DAT1_AUTOCMDHDRCODE, ui8CmdHdrCode);
    }
    else
    {
        pI3C->DAT1 = 0;
    }

    uint32_t ui32Config =  _VAL2FLD(I3C_DATABUFFERTHLDCTRL_RXBUFTHLD, 0) | _VAL2FLD(I3C_DATABUFFERTHLDCTRL_TXBUFTHLD, 0);
    pI3C->DATABUFFERTHLDCTRL = ui32Config;

    ui32Config = _VAL2FLD(I3C_QUEUETHLDCTRL_IBISTATUSTHLD, 1) |
                 _VAL2FLD(I3C_QUEUETHLDCTRL_IBIDATATHLD, 63)  |
                 _VAL2FLD(I3C_QUEUETHLDCTRL_RESPBUFTHLD, 1)   |
                 _VAL2FLD(I3C_QUEUETHLDCTRL_CMDEMPTYBUFTHLD, 1);
    pI3C->QUEUETHLDCTRL = ui32Config;

    pI3CState->ui32CmdDesc0 = pCmdData->ui32CmdDesc0;
    pI3CState->ui32CmdDesc1 = pCmdData->ui32CmdDesc1;
    pI3CState->pui32Buf = (uint32_t *)(pCmdData->pui8Buf);
    pI3CState->ui32DataLen = pCmdData->ui32DataLen;
    pI3CState->ui32DataLeft = pCmdData->ui32DataLeft;
    pI3CState->eDir = pCmdData->eDir;
    pI3CState->pHost->eSpeedMode = pCmdData->eSpeedMode;
    pI3CState->pHost->pCmdData = pCmdData;

    if ( pI3CState->pDevice->eDeviceType == AM_HAL_I3C_DEVICE_I2C )
    {
        pI3CState->ui32DelayUs = 200;
    }
    else
    {
        pI3CState->ui32DelayUs = 20;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// Get Device Characteristic Table(DCT)
//
static uint32_t am_hal_i3c_get_dct(am_hal_i3c_state_t *pI3CState)
{
        uint32_t ui32DCT0Pid, ui32DCT1Pid;
        uint8_t ui8DCR, ui8BCR;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    ui32DCT0Pid = pI3C->DCT0;
    ui32DCT1Pid = pI3C->DCT1 & 0xFFFF;
    ui8DCR = pI3C->DCT2 & 0xFF;
    ui8BCR = (pI3C->DCT2 >> 8) & 0xFF;

    pI3CState->pDevice->ui8DCR = ui8DCR;
    pI3CState->pDevice->ui8BCR = ui8BCR;
    pI3CState->pDevice->ui64PID = ((uint64_t)ui32DCT1Pid << 32) | (uint64_t)ui32DCT0Pid;

#ifdef AM_DEBUG_PRINTF
    uint32_t ui8DevDynAddr = pI3C->DCT3 & 0xFF;
    AM_HAL_I3C_DEBUG("Device PID = 0x%x\n", pI3CState->pDevice->ui64PID);
    AM_HAL_I3C_DEBUG("Device BCR = 0x%x\n", pI3CState->pDevice->ui8BCR);
    AM_HAL_I3C_DEBUG("Device DCR = 0x%x\n", pI3CState->pDevice->ui8DCR);
    AM_HAL_I3C_DEBUG("Device Dynamic Address = 0x%x\n", ui8DevDynAddr);
#endif

    return AM_HAL_STATUS_SUCCESS;
}

//
// Get the command response after sending out the command
//
static uint32_t am_hal_i3c_get_response(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    uint32_t ui32RegResp = 0;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pCmdData )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    uint32_t ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
    while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_RESPREADYSTAT_Msk) && (ui32Timeout > 0) )
    {
        if ( --ui32Timeout > 0 )
        {
            am_hal_delay_us(AM_HAL_I3C_DELAY_US);
        }
        else
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }

    ui32RegResp = pI3C->RESPONSEQUEUEPORT;
    AM_HAL_I3C_DEBUG("Command Response is 0x%x\n", ui32RegResp);

    pCmdData->ui32Resp = ui32RegResp;
    pI3CState->ui32Resp =  ui32RegResp;

    return AM_HAL_STATUS_SUCCESS;
}

//
// Send the command in PIO mode
//
static uint32_t am_hal_i3c_pio_send_cmd(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    uint32_t ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
    while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_CMDQUEUEREADYSTAT_Msk) && (ui32Timeout > 0) )
    {
        if ( --ui32Timeout > 0 )
        {
            am_hal_delay_us(AM_HAL_I3C_DELAY_US);
        }
        else
        {
            return AM_HAL_STATUS_TIMEOUT;
        }
    }

    //
    // Write the command queue part
    //
    pI3C->COMMANDQUEUEPORT = pCmdData->ui32CmdDesc0;
    pI3C->COMMANDQUEUEPORT = pCmdData->ui32CmdDesc1;
    AM_HAL_I3C_DEBUG("Send CMD in PIO mode\n", 0);
    AM_HAL_I3C_DEBUG("CmdDesc0 = 0x%x\n", pCmdData->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("CmdDesc1 = 0x%x\n", pCmdData->ui32CmdDesc1);

    return AM_HAL_STATUS_SUCCESS;
}

//
// Send the command in PIO mode for combo i2c xfer
//
static uint32_t am_hal_i3c_pio_combo_i2c_xfer(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData1, am_hal_i3c_cmd_data_t *pCmdData2)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);
    uint32_t ui32RegResp, ui32XferLen, ui32Timeout, ui32PioStatMask;


    //
    // Write the command queue part for first phase
    //
    pI3C->COMMANDQUEUEPORT = pCmdData1->ui32CmdDesc0;
    pI3C->COMMANDQUEUEPORT = pCmdData1->ui32CmdDesc1;
    AM_HAL_I3C_DEBUG("Program Cmd Queue Port for first phase\n", 0);
    AM_HAL_I3C_DEBUG("Cmd1Desc0 = 0x%x\n", pCmdData1->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("Cmd1Desc1 = 0x%x\n", pCmdData1->ui32CmdDesc1);
    ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
    while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_RESPREADYSTAT_Msk) && (ui32Timeout > 0) )
    {
        if ( --ui32Timeout > 0 )
        {
            am_hal_delay_us(AM_HAL_I3C_DELAY_US);
        }
        else
        {
            AM_HAL_I3C_DEBUG("First CMD Timeout!\n", 0);
            return AM_HAL_STATUS_TIMEOUT;
        }
    }

    ui32RegResp = pI3C->RESPONSEQUEUEPORT;

    //
    // Write the command queue part for second phase
    //
    pI3C->COMMANDQUEUEPORT = pCmdData2->ui32CmdDesc0;
    pI3C->COMMANDQUEUEPORT = pCmdData2->ui32CmdDesc1;
    AM_HAL_I3C_DEBUG("Program Cmd Queue Port for second phase\n", 0);
    AM_HAL_I3C_DEBUG("Cmd2Desc0 = 0x%x\n", pCmdData2->ui32CmdDesc0);
    AM_HAL_I3C_DEBUG("Cmd2Desc1 = 0x%x\n", pCmdData2->ui32CmdDesc1);

    if ( pCmdData2->eDir == AM_HAL_I3C_DIR_READ )
    {
        ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
        if ( pCmdData2->ui32DataLen >= 8 )
        {
            ui32PioStatMask = I3C_PIOINTRSTATUS_RXTHLDSTAT_Msk;
        }
        else
        {
            ui32PioStatMask = I3C_PIOINTRSTATUS_RESPREADYSTAT_Msk;
        }
        while ( !(pI3C->PIOINTRSTATUS & ui32PioStatMask) && (ui32Timeout > 0) )
        {
            if ( --ui32Timeout > 0 )
            {
                    am_hal_delay_us(AM_HAL_I3C_DELAY_US);
            }
            else
            {
                AM_HAL_I3C_DEBUG("Second CMD Timeout!\n", 0);
                return AM_HAL_STATUS_TIMEOUT;
            }
        }

        ui32RegResp = pI3C->RESPONSEQUEUEPORT;
        AM_HAL_I3C_DEBUG("Command Response is 0x%x\n", ui32RegResp);

        pCmdData2->ui32Resp = ui32RegResp;
        pI3CState->ui32Resp =  ui32RegResp;
    }


    while ( pI3CState->ui32DataLeft > 0)
    {
        if ( pI3CState->ui32DataLeft >= 8 )
        {
            ui32XferLen = 8;

        }
        else
        {
            ui32XferLen = pI3CState->ui32DataLeft;
        }

        if ( pI3CState->eDir == AM_HAL_I3C_DIR_READ )
        {
            for (uint32_t i = 0; i < ui32XferLen; i += 4)
            {
                *pI3CState->pui32Buf++ = pI3C->XFERDATAPORT;
            }

            pI3CState->ui32DataLeft = pI3CState->ui32DataLeft - ui32XferLen;

            if ( pI3CState->ui32DataLeft > 0 )
            {
                ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
                if ( pI3CState->ui32DataLeft >= 8 )
                {
                    while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_RXTHLDSTAT_Msk) && (ui32Timeout > 0) )
                    {
                        if ( --ui32Timeout > 0 )
                        {
                                am_hal_delay_us(AM_HAL_I3C_DELAY_US);
                        }
                        else
                        {
                            return AM_HAL_STATUS_TIMEOUT;
                        }
                    }
                }
                else
                {
                    am_hal_delay_us(pI3CState->ui32DelayUs * pI3CState->ui32DataLeft);
                }

            }
        }
        else
        {
            ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
            while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_TXTHLDSTAT_Msk) && (ui32Timeout > 0) )
            {
                if ( --ui32Timeout > 0 )
                {
                        am_hal_delay_us(AM_HAL_I3C_DELAY_US);
                }
                else
                {
                    return AM_HAL_STATUS_TIMEOUT;
                }
            }

            for (uint32_t i = 0; i < ui32XferLen; i += 4)
            {
                pI3C->XFERDATAPORT = *pI3CState->pui32Buf++;
            }

            pI3CState->ui32DataLeft = pI3CState->ui32DataLeft - ui32XferLen;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// Do the PIO block transfer
//
static uint32_t am_hal_i3c_pio_xfer_data(am_hal_i3c_state_t *pI3CState)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);
    uint32_t ui32XferLen, ui32RegResp, ui32PioStatMask;
    uint32_t ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;

    while ( pI3CState->ui32DataLeft > 0 )
    {
        if ( pI3CState->ui32DataLeft >= 8 )
        {
            ui32XferLen = 8;
        }
        else
        {
            ui32XferLen = pI3CState->ui32DataLeft;
        }

        if ( pI3CState->eDir == AM_HAL_I3C_DIR_READ )
        {
            if ( pI3CState->ui32DataLen < 8 )
            {
                ui32PioStatMask = I3C_PIOINTRSTATUS_RESPREADYSTAT_Msk;
                ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
                while ( !(pI3C->PIOINTRSTATUS & ui32PioStatMask) && (ui32Timeout > 0) )
                {
                    if ( --ui32Timeout > 0 )
                    {
                            am_hal_delay_us(pI3CState->ui32DelayUs);
                    }
                    else
                    {
                        AM_HAL_I3C_DEBUG("PIO Read Timeout!\n", 0);
                        return AM_HAL_STATUS_TIMEOUT;
                    }
                }

                ui32RegResp = pI3C->RESPONSEQUEUEPORT;
                pI3CState->ui32Resp =  ui32RegResp;
                AM_HAL_I3C_DEBUG("Command Response is 0x%x\n", ui32RegResp);
                am_hal_delay_us(pI3CState->ui32DelayUs * pI3CState->ui32DataLeft);
            }
            else if ( pI3CState->ui32DataLeft >= 8)
            {
                ui32PioStatMask = I3C_PIOINTRSTATUS_RXTHLDSTAT_Msk;
                ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
                while ( !(pI3C->PIOINTRSTATUS & ui32PioStatMask) && (ui32Timeout > 0) )
                {
                    if ( --ui32Timeout > 0 )
                    {
                        am_hal_delay_us(pI3CState->ui32DelayUs);
                    }
                    else
                    {
                        AM_HAL_I3C_DEBUG("PIO Read Timeout!\n", 0);
                        return AM_HAL_STATUS_TIMEOUT;
                    }
                }
            }
            else
            {
                ui32RegResp = pI3C->RESPONSEQUEUEPORT;
                pI3CState->ui32Resp =  ui32RegResp;
                AM_HAL_I3C_DEBUG("Command Response is 0x%x\n", ui32RegResp);
                am_hal_delay_us(pI3CState->ui32DelayUs * pI3CState->ui32DataLeft);
            }

            for (uint32_t i = 0; i < ui32XferLen; i += 4)
            {
                *pI3CState->pui32Buf++ = pI3C->XFERDATAPORT;
            }

            pI3CState->ui32DataLeft = pI3CState->ui32DataLeft - ui32XferLen;
        }
        else
        {
            ui32Timeout = AM_HAL_I3C_TIMEOUT_CNT;
            while ( !(pI3C->PIOINTRSTATUS & I3C_PIOINTRSTATUS_TXTHLDSTAT_Msk) && (ui32Timeout > 0) )
            {
                if ( --ui32Timeout > 0 )
                {
                    am_hal_delay_us(AM_HAL_I3C_DELAY_US);
                }
                else
                {
                    return AM_HAL_STATUS_TIMEOUT;
                }
            }

            for (uint32_t i = 0; i < ui32XferLen; i += 4)
            {
                pI3C->XFERDATAPORT = *pI3CState->pui32Buf++;
            }

            pI3CState->ui32DataLeft = pI3CState->ui32DataLeft - ui32XferLen;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//
// DMA Mode initialization
//
static uint32_t am_hal_i3c_dma_init(am_hal_i3c_state_t *pI3CState)
{
    uint8_t ui8RingNum = 1;
    am_hal_i3c_ring_t *pRing = pI3CState->pHost->pRing;
    am_hal_i3c_rh_t   *pRHeader;

    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    pI3C->RHSCONTROL_b.MAXHEADERCOUNT = ui8RingNum;

    for ( uint32_t i = 0; i < ui8RingNum; i++ )
    {
        pRHeader = &pRing->RHeader[i];
        pRHeader->ui32XferStructSize = pI3C->CRSETUP_b.XFERSTRUCTSIZE;
        pRHeader->ui32RespStructSize = pI3C->CRSETUP_b.RESPSTRUCTSIZE;
        pRHeader->ui32IbiStatStructSize = pI3C->IBISETUP_b.IBISTATUSSTRUCTSIZE;
        *(uint32_t *)(&pI3C->RHCMDRINGBASELO + i * 128) = pRHeader->ui32XferAddr;
        *(uint32_t *)(&pI3C->RHCMDRINGBASEHI + i * 128) = 0;
        *(uint32_t *)(&pI3C->RHRESPRINGBASELO + i * 128) = pRHeader->ui32RespAddr;
        *(uint32_t *)(&pI3C->RHRESPRINGBASEHI + i * 128) = 0;
        *(uint32_t *)(&pI3C->RHIBISTATUSRINGBASELO + i * 128) = pRHeader->ui32IbiStatAddr;
        *(uint32_t *)(&pI3C->RHIBISTATUSRINGBASEHI + i * 128) = 0;
        *(uint32_t *)(&pI3C->RHIBIDATARINGBASELO + i * 128) = pRHeader->ui32IbiDataAddr;
        *(uint32_t *)(&pI3C->RHIBIDATARINGBASEHI + i * 128) = 0;
        *(uint32_t *)(&pI3C->CRSETUP + i * 128) = AM_HAL_I3C_MAX_RING_SIZE;
        *(uint32_t *)(&pI3C->CHUNKCONTROL + i * 128) = 0;
        *(uint32_t *)(&pI3C->IBISETUP + i * 128) = _VAL2FLD(I3C_IBISETUP_CHUNKCOUNT, 4)
                                                 | _VAL2FLD(I3C_IBISETUP_CHUNKSIZE, 0)
                                                 | _VAL2FLD(I3C_IBISETUP_IBISTATUSRINGSIZE, 255);
    }


    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;

}

//
// Legacy I2C Combo DMA Xfer
//
static uint32_t am_hal_i3c_dma_combo_i2c_xfer_data(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData1, am_hal_i3c_cmd_data_t *pCmdData2)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);
    am_hal_i3c_ring_t *pRing = pI3CState->pHost->pRing;
    am_hal_i3c_rh_t   *pRHeader = &pRing->RHeader[0];
    uint32_t ui32RHop1, ui32CrDeQptr, ui32EnQptr;
    uint32_t *pRingData;

    ui32RHop1  = pI3C->RHOPERATION1;
    ui32EnQptr = pI3C->RHOPERATION1_b.CRENQPTR;

    pRHeader->pXfer = pRing->pui32CmdRingBuf;

    pRingData = pRHeader->pXfer;
    *pRingData++ = pCmdData1->ui32CmdDesc0;
    *pRingData++ = pCmdData1->ui32CmdDesc1;

    pRHeader->DataDesc.bBLP = false;
    pRHeader->DataDesc.bIOC = false;
    pRHeader->DataDesc.ui16BlkSize = 1;
    pRHeader->DataDesc.ui32DatPtrLO = 0;
    pRHeader->DataDesc.ui32DatPtrHI = 0;

    *pRingData++ = ((uint32_t)pRHeader->DataDesc.bBLP << 31) | ((uint32_t)pRHeader->DataDesc.bIOC << 30) | pRHeader->DataDesc.ui16BlkSize;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrLO;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrHI;

    *pRingData++ = pCmdData2->ui32CmdDesc0;
    *pRingData++ = pCmdData2->ui32CmdDesc1;

    if ( pCmdData2->pui8Buf == NULL )
    {
        pCmdData2->ui32DataLen = 0;
    }

    pRHeader->DataDesc.bBLP = false;
    pRHeader->DataDesc.bIOC = true;
    pRHeader->DataDesc.ui16BlkSize = pCmdData2->ui32DataLen;
    if ( pCmdData2->ui32DataLen )
    {
        pRHeader->DataDesc.ui32DatPtrLO = (uint32_t)pCmdData2->pui8Buf;
        pRHeader->DataDesc.ui32DatPtrHI = 0;
    }
    else
    {
        pRHeader->DataDesc.ui32DatPtrLO = 0;
        pRHeader->DataDesc.ui32DatPtrHI = 0;
    }

    *pRingData++ = ((uint32_t)pRHeader->DataDesc.bBLP << 31) | ((uint32_t)pRHeader->DataDesc.bIOC << 30) | pRHeader->DataDesc.ui16BlkSize;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrLO;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrHI;

    ui32EnQptr = ui32EnQptr + 1; // Depth of CRQUEUE is 2
    if ( ui32EnQptr > 2 )
    {
        ui32EnQptr = 1;
    }
    ui32CrDeQptr = pI3C->RHOPERATION2_b.CRDEQPTR;
    if (ui32EnQptr == ui32CrDeQptr)
    {
        AM_HAL_I3C_DEBUG("Error! DeQptr == CrDeQptr = 0x%x !\n", ui32CrDeQptr);
        return AM_HAL_STATUS_IN_USE;
    }

    ui32RHop1 = (pI3C->RHOPERATION1 & (~I3C_RHOPERATION1_CRENQPTR_Msk)) | _VAL2FLD(I3C_RHOPERATION1_CRENQPTR, 2);
    pI3C->RHOPERATION1 = ui32RHop1;

    pI3C->RHCONTROL = _VAL2FLD(I3C_RHCONTROL_ENABLE, 1) | _VAL2FLD(I3C_RHCONTROL_RS, 1);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// Do the DMA transfer
//
static uint32_t am_hal_i3c_dma_xfer_data(am_hal_i3c_state_t *pI3CState, am_hal_i3c_cmd_data_t *pCmdData)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);
    am_hal_i3c_ring_t *pRing = pI3CState->pHost->pRing;
    am_hal_i3c_rh_t   *pRHeader = &pRing->RHeader[0];
    uint32_t ui32RHop1, ui32CrDeQptr, ui32EnQptr;
    uint32_t *pRingData;

    ui32RHop1  = pI3C->RHOPERATION1;
    ui32EnQptr = pI3C->RHOPERATION1_b.CRENQPTR;

    pRHeader->pXfer = pRing->pui32CmdRingBuf;

    pRingData = pRHeader->pXfer;
    *pRingData++ = pCmdData->ui32CmdDesc0;
    *pRingData++ = pCmdData->ui32CmdDesc1;

    if ( pCmdData->pui8Buf == NULL )
    {
        pCmdData->ui32DataLen = 0;
    }

    pRHeader->DataDesc.bBLP = false;
    pRHeader->DataDesc.bIOC = true;
    pRHeader->DataDesc.ui16BlkSize = pCmdData->ui32DataLen;
    if ( pCmdData->ui32DataLen )
    {
        pRHeader->DataDesc.ui32DatPtrLO = (uint32_t)pCmdData->pui8Buf;
        pRHeader->DataDesc.ui32DatPtrHI = 0;
    }
    else
    {
        pRHeader->DataDesc.ui32DatPtrLO = 0;
        pRHeader->DataDesc.ui32DatPtrHI = 0;
    }

    *pRingData++ = ((uint32_t)pRHeader->DataDesc.bBLP << 31) | ((uint32_t)pRHeader->DataDesc.bIOC << 30) | pRHeader->DataDesc.ui16BlkSize;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrLO;
    *pRingData++ = pRHeader->DataDesc.ui32DatPtrHI;

    ui32EnQptr = ui32EnQptr + 1; // Depth of CRQUEUE is 2
    if ( ui32EnQptr > 2 )
    {
        ui32EnQptr = 1;
    }
    ui32CrDeQptr = pI3C->RHOPERATION2_b.CRDEQPTR;
    if (ui32EnQptr == ui32CrDeQptr)
    {
        AM_HAL_I3C_DEBUG("Error! DeQptr == CrDeQptr = 0x%x !\n", ui32CrDeQptr);
        return AM_HAL_STATUS_IN_USE;
    }

    ui32RHop1 = (pI3C->RHOPERATION1 & (~I3C_RHOPERATION1_CRENQPTR_Msk)) | _VAL2FLD(I3C_RHOPERATION1_CRENQPTR, 1);
    pI3C->RHOPERATION1 = ui32RHop1;

    pI3C->RHCONTROL = _VAL2FLD(I3C_RHCONTROL_ENABLE, 1) | _VAL2FLD(I3C_RHCONTROL_RS, 1);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//
// DMA transfer done
//
static void am_hal_i3c_dma_xfer_done(am_hal_i3c_state_t *pI3CState, am_hal_i3c_rh_t *pRHeader)
{
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    uint32_t ui32RHop1, ui32CrDeQptr, ui32DeQptr, ui32Resp, ui32Timeout;
    uint32_t *pRingResp;

    pRHeader->pResp = pI3CState->pHost->pRing->pui32RespRingBuf;

    ui32DeQptr = pRHeader->ui32DeQptr;
    ui32Timeout = 0xFF;
    do
    {
        ui32CrDeQptr = pI3C->RHOPERATION2_b.CRDEQPTR;
        AM_HAL_I3C_DEBUG("DeQptr = 0x%x CrDeQptr = 0x%x\n", ui32DeQptr, ui32CrDeQptr);
        pRingResp = (uint32_t *)pRHeader->pResp + pRHeader->ui32RespStructSize * ui32DeQptr;
        ui32Resp = *pRingResp;
        pI3CState->ui32Resp = ui32Resp;
        AM_HAL_I3C_DEBUG("DMA Xfer Resp = 0x%x\n", ui32Resp);
        ui32DeQptr = ui32DeQptr + 1;
        pRHeader->ui32DeQptr = ui32DeQptr;
        if ( ui32DeQptr == ui32CrDeQptr )
        {
            AM_HAL_I3C_DEBUG("DeQptr = CrDeQptr = 0x%x\n", ui32DeQptr);
            break;
        }
        ui32Timeout--;
    } while (ui32Timeout > 0);

    ui32RHop1 = (pI3C->RHOPERATION1 & (~I3C_RHOPERATION1_CRSWDEQPTR_Msk)) | _VAL2FLD(I3C_RHOPERATION1_CRSWDEQPTR, ui32DeQptr);
    pI3C->RHOPERATION1 = ui32RHop1;

    pI3C->RHCONTROL = 0;
}

//
// I3C IBI handling in PIO mode
//
static void am_hal_i3c_pio_ibi_handling(am_hal_i3c_state_t *pI3CState)
{
    am_hal_i3c_ibi_status_t IbI_Status;
    uint32_t ui32IbiStatus;
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

    ui32IbiStatus = pI3C->IBIPORT;
    *pI3CState->pui32IbiStatBuf = ui32IbiStatus;
    IbI_Status.ui32IbiStatus = ui32IbiStatus;
    if ( IbI_Status.IbiStatus.error )
    {
        AM_HAL_I3C_DEBUG("IBI Status Error! IBI Status = 0x%x\n", ui32IbiStatus);
    }

    if ( IbI_Status.IbiStatus.ibi_sts )
    {
        AM_HAL_I3C_DEBUG("IBI was handled with NACK\n", 0);
    }
    else
    {
        AM_HAL_I3C_DEBUG("IBI was hanlded with ACK\n", 0);
    }

    if ( IbI_Status.IbiStatus.dat_len )
    {
        for (uint32_t i = 0; i < IbI_Status.IbiStatus.dat_len; i += 4)
        {
            *pI3CState->pui32IbiDataBuf++ = pI3C->IBIPORT;
        }
    }

    AM_HAL_I3C_DEBUG("IBI Status = 0x%x\n", ui32IbiStatus);
    AM_HAL_I3C_DEBUG("IBI Data Length = 0x%x\n", IbI_Status.IbiStatus.dat_len);
    AM_HAL_I3C_DEBUG("IBI ID = 0x%x\n", IbI_Status.IbiStatus.ibi_id >> 1);
    AM_HAL_I3C_DEBUG("IBI RnW = 0x%x\n", IbI_Status.IbiStatus.ibi_id & 0x1);
}

//
// I3C IBI handling in DMA mode
//
static void am_hal_i3c_dma_ibi_handling(am_hal_i3c_state_t *pI3CState)
{
    am_hal_i3c_ibi_status_t IbI_Status;
    uint32_t ui32IbiStatus;
    uint32_t ui32RHop1;
    I3C_Type *pI3C = I3Cn(pI3CState->ui32Module);

#ifdef AM_DEBUG_PRINTF
    uint32_t ui32IbiSwDeQptr = pI3C->RHOPERATION1_b.IBISWDEQPTR;
    uint32_t ui32IbiEnQptr = pI3C->RHOPERATION2_b.IBIENQPTR;
    AM_HAL_I3C_DEBUG("ui32IbiSwDeQptr = 0x%x\n", ui32IbiSwDeQptr);
    AM_HAL_I3C_DEBUG("ui32IbiEnQptr = 0x%x\n", ui32IbiEnQptr);
#endif

    ui32IbiStatus = *pI3CState->pui32IbiStatBuf;
    IbI_Status.ui32IbiStatus = ui32IbiStatus;

    ui32RHop1 = (pI3C->RHOPERATION1 & (~I3C_RHOPERATION1_IBISWDEQPTR_Msk)) | _VAL2FLD(I3C_RHOPERATION1_IBISWDEQPTR, 1);
    pI3C->RHOPERATION1 = ui32RHop1;

    if ( IbI_Status.IbiStatus.error )
    {
        AM_HAL_I3C_DEBUG("DMA IBI Status Error! IBI Status = 0x%x\n", ui32IbiStatus);
    }

    if ( IbI_Status.IbiStatus.ibi_sts )
    {
        AM_HAL_I3C_DEBUG("DMA IBI was handled with NACK\n", 0);
    }
    else
    {
        AM_HAL_I3C_DEBUG("DMA IBI was hanlded with ACK\n", 0);
    }

    AM_HAL_I3C_DEBUG("DMA IBI Status = 0x%x\n", ui32IbiStatus);
    AM_HAL_I3C_DEBUG("DMA IBI Data Length = 0x%x\n", IbI_Status.IbiStatus.dat_len);
    AM_HAL_I3C_DEBUG("DMA IBI ID = 0x%x\n", IbI_Status.IbiStatus.ibi_id >> 1);
    AM_HAL_I3C_DEBUG("DMA IBI RnW = 0x%x\n", IbI_Status.IbiStatus.ibi_id & 0x1);
}

//
// I3C blocking combo transfer with legacy I2C device
// This function performs a combo transaction in PIO mode with i2c device.
//
static uint32_t am_hal_i3c_combo_i2c_blocking_transfer(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_cmd_data_t CmdData1, CmdData2;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pDevice = &psTransaction->Device;

    memset(&CmdData1, 0, sizeof(CmdData1));
    memset(&CmdData2, 0, sizeof(CmdData2));

    CmdData1.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData1.eSpeedMode = psTransaction->eSpeedMode;
    CmdData1.ui16ComboOfs = psTransaction->ui16ComboOfs;
    CmdData1.bCombo16bitOfs = psTransaction->bCombo16bitOfs;


    CmdData2.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData2.eDir = psTransaction->eDirection;
    CmdData2.eSpeedMode = psTransaction->eSpeedMode;
    CmdData2.ui32DataLen = psTransaction->ui32NumBytes;
    CmdData2.ui32DataLeft = psTransaction->ui32NumBytes;

    if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
    {
        CmdData2.pui8Buf = (uint8_t *)psTransaction->pui32RxBuffer;
    }
    else
    {
        CmdData2.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;
    }

    if ( am_hal_i3c_prepare_combo_i2c_cmd(pI3CState, &CmdData1, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_pio_combo_i2c_xfer(pI3CState, &CmdData1, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( psTransaction->eDirection == AM_HAL_I3C_DIR_WRITE )
    {
        if ( am_hal_i3c_get_response(pI3CState, &CmdData2) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}


//
// I3C non-blocking combo transfer with legacy I2C device
// This function performs a combo transaction in DMA mode with i2c device.
//
static uint32_t am_hal_i3c_combo_i2c_nonblocking_transfer(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_ring_t *pRing = pI3CState->pHost->pRing;
    am_hal_i3c_cmd_data_t CmdData1, CmdData2;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pDevice = &psTransaction->Device;

    memset(&CmdData1, 0, sizeof(CmdData1));
    memset(&CmdData2, 0, sizeof(CmdData2));

    CmdData1.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData1.eSpeedMode = psTransaction->eSpeedMode;
    CmdData1.ui16ComboOfs = psTransaction->ui16ComboOfs;
    CmdData1.bCombo16bitOfs = psTransaction->bCombo16bitOfs;


    CmdData2.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData2.eDir = psTransaction->eDirection;
    CmdData2.eSpeedMode = psTransaction->eSpeedMode;
    CmdData2.ui32DataLen = psTransaction->ui32NumBytes;
    CmdData2.ui32DataLeft = psTransaction->ui32NumBytes;

    pRing->pui32CmdRingBuf = psTransaction->pui32CmdRingBuf;
    pRing->ui32CmdRingBufLen = psTransaction->ui32CmdRingBufLen;
    pRing->pui32RespRingBuf = psTransaction->pui32RespRingBuf;
    pRing->ui32RespRingBufLen = psTransaction->ui32RespRingBufLen;
    pRing->RHeader[0].ui32XferAddr = (uint32_t)psTransaction->pui32CmdRingBuf;
    pRing->RHeader[0].ui32RespAddr = (uint32_t)psTransaction->pui32RespRingBuf;
    pRing->RHeader[0].ui32IbiStatAddr = (uint32_t)psTransaction->pui32IbiStatBuf;
    pRing->RHeader[0].ui32IbiDataAddr = (uint32_t)psTransaction->pui32IbiDataBuf;
    pRing->RHeader[0].ui32DeQptr = 0;

    if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
    {
        CmdData2.pui8Buf = (uint8_t *)psTransaction->pui32RxBuffer;
    }
    else
    {
        CmdData2.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;
    }

    if ( am_hal_i3c_prepare_combo_i2c_cmd(pI3CState, &CmdData1, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_dma_init(pI3CState) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_dma_combo_i2c_xfer_data(pI3CState, &CmdData1, &CmdData2) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// External Functions.
//
//*****************************************************************************

//*****************************************************************************
//
// I3C initialization function
//
//*****************************************************************************
uint32_t am_hal_i3c_initialize(uint32_t ui32Module, void **ppHandle)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check that the request module is in range.
    //
    if ( ui32Module >= AM_REG_I3C_NUM_MODULES )
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Check for valid arguements.
    //
    if ( !ppHandle )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Check if the handle is unallocated.
    //
    if ( g_I3C_State[ui32Module].prefix.s.bInit )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Initialize the handle.
    //
    g_I3C_State[ui32Module].prefix.s.bInit = true;
    g_I3C_State[ui32Module].prefix.s.magic = AM_HAL_MAGIC_I3C;
    g_I3C_State[ui32Module].ui32Module = ui32Module;
    g_I3C_State[ui32Module].pHost = &g_I3C_Host[ui32Module];
    g_I3C_State[ui32Module].pHost->pRing = &g_I3C_Ring[ui32Module];

    //
    // Return the handle.
    //
    *ppHandle = (void *)&g_I3C_State[ui32Module];

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C deinitialize function
//
//*****************************************************************************
uint32_t am_hal_i3c_deinitialize(void *pHandle)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Reset the handle.
    //
    pI3CState->prefix.s.bInit = false;
    pI3CState->ui32Module = 0;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C controller power control function
//
// This function updates the peripheral to a given power state.
//
//*****************************************************************************
uint32_t am_hal_i3c_controller_power_control(void *pHandle, am_hal_sysctrl_power_state_e ePowerState, bool bRetainState)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Decode the requested power state and update I3C operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:

            if ( bRetainState && !pI3CState->registerState.bValid )
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Clock request.
            //
            ui32Status = am_hal_clkmgr_clock_request(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_I3C + pI3CState->ui32Module));
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I3C + pI3CState->ui32Module));

            //
            // Enable I3C clock.
            //
            CLKGEN->CLKCTRL_b.I3CCLKEN = CLKGEN_CLKCTRL_I3CCLKEN_ENABLE;

            if ( bRetainState )
            {
                //
                // Restore I3C registers
                //
                I3Cn(pI3CState->ui32Module)->HCCONTROL = pI3CState->registerState.regHCCONTROL;
                I3Cn(pI3CState->ui32Module)->RHSCONTROL = pI3CState->registerState.regRHSCONTROL;
                I3Cn(pI3CState->ui32Module)->CHUNKCONTROL = pI3CState->registerState.regCHUNKCONTROL;
                I3Cn(pI3CState->ui32Module)->RHCONTROL = pI3CState->registerState.regRHCONTROL;
                I3Cn(pI3CState->ui32Module)->RESETCTRL = pI3CState->registerState.regRESETCTRL;
                I3Cn(pI3CState->ui32Module)->IBINOTIFYCTRL = pI3CState->registerState.regIBINOTIFYCTRL;
                I3Cn(pI3CState->ui32Module)->QUEUETHLDCTRL = pI3CState->registerState.regQUEUETHLDCTRL;
                I3Cn(pI3CState->ui32Module)->QUEUESIZECTRL = pI3CState->registerState.regQUEUESIZECTRL;
                I3Cn(pI3CState->ui32Module)->INTRSTATUSENABLE = pI3CState->registerState.regINTRSTATUSENABLE;
                I3Cn(pI3CState->ui32Module)->INTRSIGNALENABLE = pI3CState->registerState.regINTRSIGNALENABLE;
                I3Cn(pI3CState->ui32Module)->PIOINTRSTATUSENABLE = pI3CState->registerState.regPIOINTRSTATUSENABLE;
                I3Cn(pI3CState->ui32Module)->PIOINTRSIGNALENABLE = pI3CState->registerState.regPIOINTRSIGNALENABLE;
                I3Cn(pI3CState->ui32Module)->RHINTRSTATUSENABLE = pI3CState->registerState.regRHINTRSTATUSENABLE;
                I3Cn(pI3CState->ui32Module)->RHINTRSIGNALENABLE = pI3CState->registerState.regRHINTRSIGNALENABLE;
                I3Cn(pI3CState->ui32Module)->DEVICEADDR = pI3CState->registerState.regDEVICEADDR;
                I3Cn(pI3CState->ui32Module)->DAT0 = pI3CState->registerState.regDAT0;
                I3Cn(pI3CState->ui32Module)->DAT1 = pI3CState->registerState.regDAT1;
                I3Cn(pI3CState->ui32Module)->CRSETUP = pI3CState->registerState.regCRSETUP;
                I3Cn(pI3CState->ui32Module)->IBISETUP = pI3CState->registerState.regIBISETUP;
                I3Cn(pI3CState->ui32Module)->RHCMDRINGBASELO = pI3CState->registerState.regRHCMDRINGBASELO;
                I3Cn(pI3CState->ui32Module)->RHCMDRINGBASEHI = pI3CState->registerState.regRHCMDRINGBASEHI;
                I3Cn(pI3CState->ui32Module)->RHRESPRINGBASELO = pI3CState->registerState.regRHRESPRINGBASELO;
                I3Cn(pI3CState->ui32Module)->RHRESPRINGBASEHI = pI3CState->registerState.regRHRESPRINGBASEHI;
                I3Cn(pI3CState->ui32Module)->RHIBISTATUSRINGBASELO = pI3CState->registerState.regRHIBISTATUSRINGBASELO;
                I3Cn(pI3CState->ui32Module)->RHIBISTATUSRINGBASEHI = pI3CState->registerState.regRHIBISTATUSRINGBASEHI;
                I3Cn(pI3CState->ui32Module)->RHIBIDATARINGBASELO = pI3CState->registerState.regRHIBIDATARINGBASELO;
                I3Cn(pI3CState->ui32Module)->RHIBIDATARINGBASEHI = pI3CState->registerState.regRHIBIDATARINGBASEHI;
                I3Cn(pI3CState->ui32Module)->I2CURD1SCLLOWCNT = pI3CState->registerState.regI2CURD1SCLLOWCNT;
                I3Cn(pI3CState->ui32Module)->I2CURD1SCLHIGHCNT = pI3CState->registerState.regI2CURD1SCLHIGHCNT;
                I3Cn(pI3CState->ui32Module)->I2CURD2SCLLOWCNT = pI3CState->registerState.regI2CURD2SCLLOWCNT;
                I3Cn(pI3CState->ui32Module)->I2CURD2SCLHIGHCNT = pI3CState->registerState.regI2CURD2SCLHIGHCNT;
                I3Cn(pI3CState->ui32Module)->I2CURD3SCLLOWCNT = pI3CState->registerState.regI2CURD3SCLLOWCNT;
                I3Cn(pI3CState->ui32Module)->I2CURD3SCLHIGHCNT = pI3CState->registerState.regI2CURD3SCLHIGHCNT;
                pI3CState->registerState.bValid = false;
            }
            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if ( bRetainState )
            {
                //
                // Save I3C Registers
                //
                pI3CState->registerState.regHCCONTROL = I3Cn(pI3CState->ui32Module)->HCCONTROL;
                pI3CState->registerState.regRHSCONTROL = I3Cn(pI3CState->ui32Module)->RHSCONTROL;
                pI3CState->registerState.regCHUNKCONTROL = I3Cn(pI3CState->ui32Module)->CHUNKCONTROL;
                pI3CState->registerState.regRHCONTROL = I3Cn(pI3CState->ui32Module)->RHCONTROL;
                pI3CState->registerState.regRESETCTRL = I3Cn(pI3CState->ui32Module)->RESETCTRL;
                pI3CState->registerState.regIBINOTIFYCTRL = I3Cn(pI3CState->ui32Module)->IBINOTIFYCTRL;
                pI3CState->registerState.regQUEUETHLDCTRL = I3Cn(pI3CState->ui32Module)->QUEUETHLDCTRL;
                pI3CState->registerState.regQUEUESIZECTRL = I3Cn(pI3CState->ui32Module)->QUEUESIZECTRL;
                pI3CState->registerState.regINTRSTATUSENABLE = I3Cn(pI3CState->ui32Module)->INTRSTATUSENABLE;
                pI3CState->registerState.regINTRSIGNALENABLE = I3Cn(pI3CState->ui32Module)->INTRSIGNALENABLE;
                pI3CState->registerState.regPIOINTRSTATUSENABLE = I3Cn(pI3CState->ui32Module)->PIOINTRSTATUSENABLE;
                pI3CState->registerState.regPIOINTRSIGNALENABLE = I3Cn(pI3CState->ui32Module)->PIOINTRSIGNALENABLE;
                pI3CState->registerState.regRHINTRSTATUSENABLE = I3Cn(pI3CState->ui32Module)->RHINTRSTATUSENABLE;
                pI3CState->registerState.regRHINTRSIGNALENABLE = I3Cn(pI3CState->ui32Module)->RHINTRSIGNALENABLE;
                pI3CState->registerState.regDEVICEADDR = I3Cn(pI3CState->ui32Module)->DEVICEADDR;
                pI3CState->registerState.regDAT0 = I3Cn(pI3CState->ui32Module)->DAT0;
                pI3CState->registerState.regDAT1 = I3Cn(pI3CState->ui32Module)->DAT1;
                pI3CState->registerState.regCRSETUP = I3Cn(pI3CState->ui32Module)->CRSETUP;
                pI3CState->registerState.regIBISETUP = I3Cn(pI3CState->ui32Module)->IBISETUP;
                pI3CState->registerState.regRHCMDRINGBASELO = I3Cn(pI3CState->ui32Module)->RHCMDRINGBASELO;
                pI3CState->registerState.regRHCMDRINGBASEHI = I3Cn(pI3CState->ui32Module)->RHCMDRINGBASEHI;
                pI3CState->registerState.regRHRESPRINGBASELO = I3Cn(pI3CState->ui32Module)->RHRESPRINGBASELO;
                pI3CState->registerState.regRHRESPRINGBASEHI = I3Cn(pI3CState->ui32Module)->RHRESPRINGBASEHI;
                pI3CState->registerState.regRHIBISTATUSRINGBASELO = I3Cn(pI3CState->ui32Module)->RHIBISTATUSRINGBASELO;
                pI3CState->registerState.regRHIBISTATUSRINGBASEHI = I3Cn(pI3CState->ui32Module)->RHIBISTATUSRINGBASEHI;
                pI3CState->registerState.regRHIBIDATARINGBASELO = I3Cn(pI3CState->ui32Module)->RHIBIDATARINGBASELO;
                pI3CState->registerState.regRHIBIDATARINGBASEHI = I3Cn(pI3CState->ui32Module)->RHIBIDATARINGBASEHI;
                pI3CState->registerState.regI2CURD1SCLLOWCNT = I3Cn(pI3CState->ui32Module)->I2CURD1SCLLOWCNT;
                pI3CState->registerState.regI2CURD1SCLHIGHCNT = I3Cn(pI3CState->ui32Module)->I2CURD1SCLHIGHCNT;
                pI3CState->registerState.regI2CURD2SCLLOWCNT = I3Cn(pI3CState->ui32Module)->I2CURD2SCLLOWCNT;
                pI3CState->registerState.regI2CURD2SCLHIGHCNT = I3Cn(pI3CState->ui32Module)->I2CURD2SCLHIGHCNT;
                pI3CState->registerState.regI2CURD3SCLLOWCNT = I3Cn(pI3CState->ui32Module)->I2CURD3SCLLOWCNT;
                pI3CState->registerState.regI2CURD3SCLHIGHCNT = I3Cn(pI3CState->ui32Module)->I2CURD3SCLHIGHCNT;
                pI3CState->registerState.bValid = true;
            }

            //
            // Disable all the interrupts.
            //
            am_hal_i3c_intr_status_disable(pHandle, 0xFFFFFFFF);

            //
            // Disable I3C clock.
            //
            CLKGEN->CLKCTRL_b.I3CCLKEN = CLKGEN_CLKCTRL_I3CCLKEN_DISABLE;

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I3C + pI3CState->ui32Module));

            //
            // Clock release.
            //
            ui32Status = am_hal_clkmgr_clock_release(AM_HAL_CLKMGR_CLK_ID_HFRC, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_I3C + pI3CState->ui32Module));
            if ( ui32Status != AM_HAL_STATUS_SUCCESS )
            {
                return ui32Status;
            }
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
// I3C PHY power control function
//
// This function updates the I3C PHY to a given power state.
//
//*****************************************************************************
uint32_t am_hal_i3c_phy_power_control(void *pHandle, am_hal_sysctrl_power_state_e ePowerState)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Decode the requested power state and update I3C PHY operation accordingly.
    //
    switch (ePowerState)
    {
        case AM_HAL_SYSCTRL_WAKE:
            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I3CPHY + pI3CState->ui32Module));
            break;

        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable((am_hal_pwrctrl_periph_e)(AM_HAL_PWRCTRL_PERIPH_I3CPHY + pI3CState->ui32Module));
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
// I3C setup host function
//
// This function checks the capabilites of I3C host controller and configures
// host controller's Xfer mode and speed.
//
//*****************************************************************************
uint32_t am_hal_i3c_setup_host(void *pHandle, am_hal_i3c_xfer_mode_e eXferMode, am_hal_i3c_speed_mode_e eSpeedMode)
{
    I3C_Type *pI3C;
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3C = I3Cn(pI3CState->ui32Module);

    pI3CState->pHost->pHandle = pHandle;
    pI3CState->pHost->ui32Module = pI3CState->ui32Module;
    AM_HAL_I3C_DEBUG("Setup I3C Host Controller%d\n\n", pI3CState->pHost->ui32Module);

    pI3CState->pHost->ui32HciVersion = pI3C->HCIVERSION;
    AM_HAL_I3C_DEBUG("I3C HCI Version is 0x%x\n", pI3CState->pHost->ui32HciVersion);

    pI3CState->pHost->ui32Capabilities = pI3C->HCCAPABILITIES;
    AM_HAL_I3C_DEBUG("I3C HCCAPABILITIES is 0x%x\n", pI3CState->pHost->ui32Capabilities);

    pI3CState->pHost->ui32DatTableSize = pI3C->DATSECTIONOFFSET_b.DATTABLESIZE;
    AM_HAL_I3C_DEBUG("DAT TABLE SIZE is 0x%x\n", pI3CState->pHost->ui32DatTableSize);

    pI3CState->pHost->ui32DatTableOffset = pI3C->DATSECTIONOFFSET_b.DATTABLEOFFSET;
    AM_HAL_I3C_DEBUG("DAT TABLE OFFSET is 0x%x\n", pI3CState->pHost->ui32DatTableOffset);

    pI3CState->pHost->ui32DctSectionOffset = pI3C->DCTSECTIONOFFSET;
    AM_HAL_I3C_DEBUG("DCT SECTION OFFSET is 0x%x\n", pI3CState->pHost->ui32DctSectionOffset);

    pI3CState->pHost->ui32RHSectionOffset = pI3C->RINGHEADERSSECTIONOFFSET;
    AM_HAL_I3C_DEBUG("RING HEADERS SECTION OFFSET is 0x%x\n", pI3CState->pHost->ui32RHSectionOffset);

    pI3CState->pHost->ui32PioSectionOffset = pI3C->PIOSECTIONOFFSET;
    AM_HAL_I3C_DEBUG("PIO SECTION OFFSET is 0x%x\n", pI3CState->pHost->ui32PioSectionOffset);

    pI3CState->pHost->ui32EXTCAPSectionOffset = pI3C->EXTCAPSSECTIONOFFSET;
    AM_HAL_I3C_DEBUG("EXTCAPS SECTION OFFSET is 0x%x\n", pI3CState->pHost->ui32EXTCAPSectionOffset);

    pI3CState->pHost->ui32CAPID = pI3C->EXTCAPHEADER_b.CAPID;
    AM_HAL_I3C_DEBUG("CAPID is 0x%x\n", pI3CState->pHost->ui32CAPID);

    pI3CState->pHost->ui32CAPLENGTH = pI3C->EXTCAPHEADER_b.CAPLENGTH;
    AM_HAL_I3C_DEBUG("CAPLENGTH is 0x%x\n", pI3CState->pHost->ui32CAPLENGTH);

    pI3CState->pHost->ui32QueueSize = pI3C->QUEUESIZECTRL_b.CRQUEUESIZE;
    AM_HAL_I3C_DEBUG("CR QUEUE SIZE is 0x%x\n", pI3CState->pHost->ui32QueueSize);

    pI3CState->pHost->ui32RxBufSize = pI3C->QUEUESIZECTRL_b.RXDATABUFFERSIZE;
    AM_HAL_I3C_DEBUG("RX DATA BUFFERSIZE is 0x%x\n", pI3CState->pHost->ui32RxBufSize);

    pI3CState->pHost->ui32TxBufSize  = pI3C->QUEUESIZECTRL_b.TXDATABUFFERSIZE;
    AM_HAL_I3C_DEBUG("TX DATA BUFFERSIZE is 0x%x\n", pI3CState->pHost->ui32TxBufSize);

    pI3CState->pHost->ui32IbiQueueSize = pI3C->QUEUESIZECTRL_b.IBI;
    AM_HAL_I3C_DEBUG("IBI Queue Size is 0x%x\n", pI3CState->pHost->ui32IbiQueueSize);

    //
    // Set I3C Host Speed Mode
    //
    pI3CState->pHost->eSpeedMode = eSpeedMode;

    //
    // Set I3C Host Xfer Mode (PIO or DMA).
    //
    if ( eXferMode == AM_HAL_I3C_XFER_DMA )
    {
        pI3CState->pHost->eXferMode = AM_HAL_I3C_XFER_DMA;

        //
        // Enable Host Controller Bus and set DMA mode.
        //
        pI3C->HCCONTROL = _VAL2FLD(I3C_HCCONTROL_BUSENABLE, 1) | _VAL2FLD(I3C_HCCONTROL_MODESELECTOR, 0);
        AM_HAL_I3C_DEBUG("Set I3C HOST into DMA Mode\n", 0);
        AM_HAL_I3C_DEBUG("Enable Host Controller Bus\n", 0);
    }
    else
    {
        //
        // Default transfer mode is PIO
        //
        pI3CState->pHost->eXferMode = AM_HAL_I3C_XFER_PIO;

        //
        // Enable Host Controller Bus and set PIO mode.
        //
        pI3C->HCCONTROL = _VAL2FLD(I3C_HCCONTROL_BUSENABLE, 1) | _VAL2FLD(I3C_HCCONTROL_MODESELECTOR, 1);
        AM_HAL_I3C_DEBUG("Set I3C HOST into PIO Mode\n", 0);
        AM_HAL_I3C_DEBUG("Enable Host Controller Bus\n", 0);
    }

    pI3CState->pHost->bInited = true;
    AM_HAL_I3C_DEBUG("HCCONTROL is 0x%x\n", pI3C->HCCONTROL);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C blocking transfer function
//
// This function performs a transaction on the I3C in PIO mode.
//
//*****************************************************************************
uint32_t am_hal_i3c_blocking_transfer(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_cmd_data_t CmdData;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( pI3CState->pHost->eXferMode != AM_HAL_I3C_XFER_PIO )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( psTransaction->Device.eDeviceType == AM_HAL_I3C_DEVICE_I2C && psTransaction->bComboXfer)
    {
        return am_hal_i3c_combo_i2c_blocking_transfer(pHandle, psTransaction);
    }
    else
    {
        pI3CState->pDevice = &psTransaction->Device;
        pI3CState->pui32IbiStatBuf = psTransaction->pui32IbiStatBuf;
        pI3CState->pui32IbiDataBuf = psTransaction->pui32IbiDataBuf;

        memset(&CmdData, 0, sizeof(CmdData));

        CmdData.ui8Addr = psTransaction->Device.ui8DynamicAddr;
        CmdData.ui8Id = psTransaction->ui8CCCID;
        CmdData.eDir = psTransaction->eDirection;
        CmdData.eSpeedMode = psTransaction->eSpeedMode;
        CmdData.ui32DataLen = psTransaction->ui32NumBytes;
        CmdData.ui32DataLeft = psTransaction->ui32NumBytes;
        CmdData.bComboCmd = psTransaction->bComboXfer;
        CmdData.bCombo16bitOfs = psTransaction->bCombo16bitOfs;
        CmdData.ui16ComboOfs = psTransaction->ui16ComboOfs;

        if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
        {
            CmdData.pui8Buf = (uint8_t *)psTransaction->pui32RxBuffer;
        }
        else
        {
            CmdData.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;
        }

        if ( am_hal_i3c_prepare_pio_cmd(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_pio_send_cmd(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_pio_xfer_data(pI3CState) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( psTransaction->eDirection == AM_HAL_I3C_DIR_WRITE )
        {
            if ( am_hal_i3c_get_response(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
            {
                return AM_HAL_STATUS_FAIL;
            }
        }

        return AM_HAL_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// I3C send CCC command function
//
// This function sends Common Command Code (CCC) to slave devices.
//
//*****************************************************************************
uint32_t am_hal_i3c_send_ccc_cmd(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_cmd_data_t CmdData;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pDevice = &psTransaction->Device;
    pI3CState->pui32IbiStatBuf = psTransaction->pui32IbiStatBuf;
    pI3CState->pui32IbiDataBuf = psTransaction->pui32IbiDataBuf;

    memset(&CmdData, 0, sizeof(CmdData));
    CmdData.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData.ui8Id = psTransaction->ui8CCCID;
    CmdData.eDir = psTransaction->eDirection;
    CmdData.eSpeedMode = psTransaction->eSpeedMode;
    CmdData.ui32DataLen = psTransaction->ui32NumBytes;
    CmdData.ui32DataLeft = psTransaction->ui32NumBytes;
    CmdData.bComboCmd = psTransaction->bComboXfer;
    CmdData.bCombo16bitOfs = psTransaction->bCombo16bitOfs;
    CmdData.ui16ComboOfs = psTransaction->ui16ComboOfs;
    if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
    {
        CmdData.pui8Buf = (uint8_t *)psTransaction->pui32RxBuffer;
    }
    else
    {
        CmdData.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;
    }
    AM_HAL_I3C_DEBUG("I3C Send CCC CMD (ID = 0x%x)\n", psTransaction->ui8CCCID );


    if ( am_hal_i3c_prepare_ccc(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( pI3CState->pHost->eXferMode == AM_HAL_I3C_XFER_PIO)
    {
        if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_pio_send_cmd(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
        {

            if ( am_hal_i3c_pio_xfer_data(pI3CState) != AM_HAL_STATUS_SUCCESS )
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
        else
        {
            if (psTransaction->ui32NumBytes > 4)
            {
                if ( am_hal_i3c_pio_xfer_data(pI3CState) != AM_HAL_STATUS_SUCCESS )
                {
                    return AM_HAL_STATUS_FAIL;
                }
            }

            if ( am_hal_i3c_get_response(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
    }
    else
    {
        pI3CState->pHost->pRing->pui32CmdRingBuf = psTransaction->pui32CmdRingBuf;
        pI3CState->pHost->pRing->ui32CmdRingBufLen = psTransaction->ui32CmdRingBufLen;
        pI3CState->pHost->pRing->pui32RespRingBuf = psTransaction->pui32RespRingBuf;
        pI3CState->pHost->pRing->ui32RespRingBufLen = psTransaction->ui32RespRingBufLen;
        pI3CState->pHost->pRing->RHeader[0].ui32XferAddr = (uint32_t)psTransaction->pui32CmdRingBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32RespAddr = (uint32_t)psTransaction->pui32RespRingBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32IbiStatAddr = (uint32_t)psTransaction->pui32IbiStatBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32IbiDataAddr = (uint32_t)psTransaction->pui32IbiDataBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32DeQptr = 0;
        if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_dma_init(pI3CState) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_dma_xfer_data(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }
    }

    return AM_HAL_STATUS_SUCCESS;

}


//*****************************************************************************
//
// I3C Dynamic Address Assignment function
//
// This function performs I3C Dynamic Address Assignment.
//
//*****************************************************************************
uint32_t am_hal_i3c_do_daa(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_cmd_data_t CmdData;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pDevice = &psTransaction->Device;
    pI3CState->pui32IbiStatBuf = psTransaction->pui32IbiStatBuf;
    pI3CState->pui32IbiDataBuf = psTransaction->pui32IbiDataBuf;

    memset(&CmdData, 0, sizeof(CmdData));

    CmdData.ui8Addr = psTransaction->Device.ui8DynamicAddr;
    CmdData.ui8Id = psTransaction->ui8CCCID;
    CmdData.eDir = psTransaction->eDirection;
    CmdData.eSpeedMode = psTransaction->eSpeedMode;
    CmdData.ui32DataLen = psTransaction->ui32NumBytes;
    CmdData.ui32DataLeft = psTransaction->ui32NumBytes;
    CmdData.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;

    if ( am_hal_i3c_prepare_daa(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    if ( pI3CState->pHost->eXferMode == AM_HAL_I3C_XFER_PIO )
    {

        if ( am_hal_i3c_pio_send_cmd(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_get_response(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( CmdData.ui8Id == AM_HAL_I3C_CCC_ENTDAA )
        {
            if ( am_hal_i3c_get_dct(pI3CState) != AM_HAL_STATUS_SUCCESS )
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
    }
    else
    {
        pI3CState->pHost->pRing->pui32CmdRingBuf = psTransaction->pui32CmdRingBuf;
        pI3CState->pHost->pRing->ui32CmdRingBufLen = psTransaction->ui32CmdRingBufLen;
        pI3CState->pHost->pRing->pui32RespRingBuf = psTransaction->pui32RespRingBuf;
        pI3CState->pHost->pRing->ui32RespRingBufLen = psTransaction->ui32RespRingBufLen;
        pI3CState->pHost->pRing->RHeader[0].ui32XferAddr = (uint32_t)psTransaction->pui32CmdRingBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32RespAddr = (uint32_t)psTransaction->pui32RespRingBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32IbiStatAddr = (uint32_t)psTransaction->pui32IbiStatBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32IbiDataAddr = (uint32_t)psTransaction->pui32IbiDataBuf;
        pI3CState->pHost->pRing->RHeader[0].ui32DeQptr = 0;

        if ( am_hal_i3c_dma_init(pI3CState) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_dma_xfer_data(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( CmdData.ui8Id == AM_HAL_I3C_CCC_ENTDAA )
        {
            if ( am_hal_i3c_get_dct(pI3CState) != AM_HAL_STATUS_SUCCESS )
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
    }

    return AM_HAL_STATUS_SUCCESS;

}

//*****************************************************************************
//
// I3C Non-Blocking transfer function
//
// This function performs a transaction on the I3C in DMA mode.
//
//*****************************************************************************
uint32_t am_hal_i3c_nonblocking_transfer(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_ring_t *pRing = pI3CState->pHost->pRing;
    am_hal_i3c_cmd_data_t CmdData;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( pI3CState->pHost->eXferMode != AM_HAL_I3C_XFER_DMA )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( psTransaction->Device.eDeviceType == AM_HAL_I3C_DEVICE_I2C && psTransaction->bComboXfer)
    {
        return am_hal_i3c_combo_i2c_nonblocking_transfer(pHandle, psTransaction);
    }
    else
    {
        pI3CState->pDevice = &psTransaction->Device;

        memset(&CmdData, 0, sizeof(CmdData));

        CmdData.ui8Addr = psTransaction->Device.ui8DynamicAddr;
        CmdData.ui8Id = psTransaction->ui8CCCID;
        CmdData.eDir = psTransaction->eDirection;
        CmdData.eSpeedMode = psTransaction->eSpeedMode;
        CmdData.ui32DataLen = psTransaction->ui32NumBytes;
        CmdData.ui32DataLeft = psTransaction->ui32NumBytes;
        CmdData.bComboCmd = psTransaction->bComboXfer;
        CmdData.bCombo16bitOfs = psTransaction->bCombo16bitOfs;
        CmdData.ui16ComboOfs = psTransaction->ui16ComboOfs;

        pRing->pui32CmdRingBuf = psTransaction->pui32CmdRingBuf;
        pRing->ui32CmdRingBufLen = psTransaction->ui32CmdRingBufLen;
        pRing->pui32RespRingBuf = psTransaction->pui32RespRingBuf;
        pRing->ui32RespRingBufLen = psTransaction->ui32RespRingBufLen;
        pRing->RHeader[0].ui32XferAddr = (uint32_t)psTransaction->pui32CmdRingBuf;
        pRing->RHeader[0].ui32RespAddr = (uint32_t)psTransaction->pui32RespRingBuf;
        pRing->RHeader[0].ui32IbiStatAddr = (uint32_t)psTransaction->pui32IbiStatBuf;
        pRing->RHeader[0].ui32IbiDataAddr = (uint32_t)psTransaction->pui32IbiDataBuf;
        pRing->RHeader[0].ui32DeQptr = 0;

        if ( psTransaction->eDirection == AM_HAL_I3C_DIR_READ )
        {
            CmdData.pui8Buf = (uint8_t *)psTransaction->pui32RxBuffer;
        }
        else
        {
            CmdData.pui8Buf = (uint8_t *)psTransaction->pui32TxBuffer;
        }

        if ( am_hal_i3c_prepare_dma_cmd(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_prepare_xfer(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_dma_init(pI3CState) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        if ( am_hal_i3c_dma_xfer_data(pI3CState, &CmdData) != AM_HAL_STATUS_SUCCESS )
        {
            return AM_HAL_STATUS_FAIL;
        }

        return AM_HAL_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
// Register I3C Callback Function
//
//*****************************************************************************
uint32_t am_hal_i3c_register_evt_callback(void *pHandle, am_hal_i3c_event_cb_t pfunCallback)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    if ( !pI3CState->pHost )
    {
        AM_HAL_I3C_DEBUG("I3C Host needs to be initialized firstly\n", 0);
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pHost->pfunCallback = pfunCallback;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set user defined I2C mode speed
//
//*****************************************************************************
uint32_t am_hal_i3c_set_i2c_speed(void *pHandle, am_hal_i3c_speed_mode_e eI2CSpeedMode, uint8_t ui8LowCnt, uint8_t ui8HighCnt)
{
    I3C_Type *pI3C;
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3C = I3Cn(pI3CState->ui32Module);

    switch (eI2CSpeedMode)
    {
        case AM_HAL_I3C_I2C_MODE2:
            pI3C->I2CURD1SCLLOWCNT  = ui8LowCnt;
            pI3C->I2CURD1SCLHIGHCNT = ui8HighCnt;
            break;
        case AM_HAL_I3C_I2C_MODE3:
            pI3C->I2CURD2SCLLOWCNT  = ui8LowCnt;
            pI3C->I2CURD2SCLHIGHCNT = ui8HighCnt;
            break;
        case AM_HAL_I3C_I2C_MODE4:
            pI3C->I2CURD3SCLLOWCNT  = ui8LowCnt;
            pI3C->I2CURD3SCLHIGHCNT = ui8HighCnt;
            break;
        default:
            return AM_HAL_STATUS_INVALID_OPERATION;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C IBI Enable function
//
//*****************************************************************************
uint32_t am_hal_i3c_ibi_enable(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->pui32IbiStatBuf = psTransaction->pui32IbiStatBuf;
    pI3CState->pui32IbiDataBuf = psTransaction->pui32IbiDataBuf;

    if ( pI3CState->pHost->eXferMode == AM_HAL_I3C_XFER_PIO )
    {
        ui32Status = am_hal_i3c_intr_signal_enable(pI3CState, AM_HAL_I3C_INT_PIO_IBISTATUSTHLD);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
            return ui32Status;
        }
    }
    else
    {
        ui32Status = am_hal_i3c_intr_signal_enable(pI3CState, AM_HAL_I3C_INT_IBIREADY | AM_HAL_I3C_INT_IBIRINGFULL);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
            return ui32Status;
        }
    }

    psTransaction->ui8CCCID = AM_HAL_I3C_CCC_ENEC(psTransaction->Device.ui8DynamicAddr == AM_HAL_I3C_BROADCAST_ADDR);

    if ( am_hal_i3c_send_ccc_cmd(pHandle, psTransaction) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C IBI Disable function
//
//*****************************************************************************
uint32_t am_hal_i3c_ibi_disable(void *pHandle, am_hal_i3c_transfer_t *psTransaction)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !psTransaction )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION


    psTransaction->ui8CCCID = AM_HAL_I3C_CCC_DISEC(psTransaction->Device.ui8DynamicAddr == AM_HAL_I3C_BROADCAST_ADDR);

    if ( am_hal_i3c_send_ccc_cmd(pHandle, psTransaction) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C Enable function
//
//*****************************************************************************
uint32_t am_hal_i3c_enable(void *pHandle)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3CState->prefix.s.bEnable = true;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C Disable function
//
//*****************************************************************************
uint32_t am_hal_i3c_disable(void *pHandle)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    I3C_Type *pI3C;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    pI3C = I3Cn(pI3CState->ui32Module);
    //
    // Disable Host Controller Bus
    //
    pI3C->HCCONTROL_b.BUSENABLE = 0;

    pI3CState->prefix.s.bEnable = false;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C interrupt status enable function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_status_enable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;

    //
    // Set the interrupt enables according to the mask.
    //
    I3Cn(ui32Module)->INTRSTATUSENABLE    |= (ui32IntMask & 0xFC00);
    I3Cn(ui32Module)->PIOINTRSTATUSENABLE |= (ui32IntMask & 0x3FF);
    I3Cn(ui32Module)->RHINTRSTATUSENABLE  |= (ui32IntMask & 0xFFFF0000) >> 16;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C interrupt status disable function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_status_disable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;
    //
    // Clear the interrupt enables according to the mask.
    //
    I3Cn(ui32Module)->INTRSTATUSENABLE    &= ~(ui32IntMask & 0xFC00);
    I3Cn(ui32Module)->PIOINTRSTATUSENABLE &= ~(ui32IntMask & 0x3FF);
    I3Cn(ui32Module)->RHINTRSTATUSENABLE  &= ~((ui32IntMask & 0xFFFF0000) >> 16);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C interrupt status get function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_status_get(void *pHandle, uint32_t *pui32Status, bool bEnabledOnly)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;

    //
    // if requested, only return the interrupts that are enabled.
    //
    if ( bEnabledOnly )
    {
        uint32_t ui32RetVal = (I3Cn(ui32Module)->PIOINTRSTATUS & 0x3FF) | (I3Cn(ui32Module)->INTRSTATUS & 0xFC00) | (I3Cn(ui32Module)->RHINTRSTATUS << 16);
        *pui32Status = ui32RetVal & ((I3Cn(ui32Module)->PIOINTRSTATUSENABLE & 0x3FF) | (I3Cn(ui32Module)->INTRSTATUSENABLE & 0xFC00) | (I3Cn(ui32Module)->RHINTRSTATUSENABLE << 16));
    }
    else
    {
        *pui32Status = (I3Cn(ui32Module)->PIOINTRSTATUS & 0x3FF) | (I3Cn(ui32Module)->INTRSTATUS & 0xFC00) | (I3Cn(ui32Module)->RHINTRSTATUS << 16);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C interrupt status clear function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_status_clear(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;

    //
    // Write 1 to clear interrupt status
    //
    I3Cn(ui32Module)->INTRSTATUS    |= (ui32IntMask & 0xFC00);
    I3Cn(ui32Module)->PIOINTRSTATUS |= (ui32IntMask & 0x3FF);
    I3Cn(ui32Module)->RHINTRSTATUS  |= (ui32IntMask & 0xFFFF0000) >> 16;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C normal interrupt signal enable function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_signal_enable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;

    //
    // Set the interrupt enables according to the mask.
    //
    I3Cn(ui32Module)->INTRSIGNALENABLE    |= ui32IntMask & 0xFC00;
    I3Cn(ui32Module)->PIOINTRSIGNALENABLE |= ui32IntMask & 0x3FF;
    I3Cn(ui32Module)->RHINTRSIGNALENABLE  |= (ui32IntMask & 0xFFFF0000) >> 16;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C normal status signal disable function
//
//*****************************************************************************
uint32_t am_hal_i3c_intr_signal_disable(void *pHandle, uint32_t ui32IntMask)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    uint32_t ui32Module;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Module = pI3CState->ui32Module;

    //
    // Clear the interrupt enables according to the mask.
    //
    I3Cn(ui32Module)->INTRSIGNALENABLE    &= ~(ui32IntMask & 0xFC00);
    I3Cn(ui32Module)->PIOINTRSIGNALENABLE &= ~(ui32IntMask & 0x3FF);
    I3Cn(ui32Module)->RHINTRSIGNALENABLE  &= ~((ui32IntMask & 0xFFFF0000) >> 16);

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C host controller initialization
//
//*****************************************************************************
uint32_t am_hal_i3c_controller_init(uint32_t ui32Module,
                                    void **ppHandle,
                                    am_hal_i3c_xfer_mode_e eXferMode,
                                    am_hal_i3c_speed_mode_e eSpeedMode)
{

    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    // Initialize the I3C instance.
    //
    ui32Status = am_hal_i3c_initialize(ui32Module, ppHandle);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Disable I3C PHY Pull Down.
    //
    am_hal_mcuctrl_i3c_phy_pulldown_enable(false);

    //
    // Power on I3C controller.
    //
    ui32Status = am_hal_i3c_controller_power_control(*ppHandle, AM_HAL_SYSCTRL_WAKE, false);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Power on I3C PHY.
    //
    ui32Status = am_hal_i3c_phy_power_control(*ppHandle, AM_HAL_SYSCTRL_WAKE);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    //  Enable I3C.
    //
    ui32Status = am_hal_i3c_enable(*ppHandle);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Set up I3C Host.
    //
    ui32Status = am_hal_i3c_setup_host(*ppHandle, eXferMode, eSpeedMode);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C host controller deinitialization
//
//*****************************************************************************
uint32_t am_hal_i3c_controller_deinit(void *pHandle, bool bPullDown)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    //
    //  Disable I3C.
    //
    ui32Status = am_hal_i3c_disable(pHandle);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Power off I3C PHY.
    //
    ui32Status = am_hal_i3c_phy_power_control(pHandle, AM_HAL_SYSCTRL_NORMALSLEEP);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Power off I3C controller.
    //
    ui32Status = am_hal_i3c_controller_power_control(pHandle, AM_HAL_SYSCTRL_NORMALSLEEP, false);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // I3C PHY Pull Down Switch Control.
    //
    am_hal_mcuctrl_i3c_phy_pulldown_enable(bPullDown);

    //
    // Deinitialize the I3C instance.
    //
    ui32Status = am_hal_i3c_deinitialize(pHandle);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// I3C interrupt service routine
//
//*****************************************************************************
uint32_t am_hal_i3c_interrupt_service(void *pHandle, uint32_t ui32IntStatus)
{
    am_hal_i3c_state_t *pI3CState = (am_hal_i3c_state_t *)pHandle;
    am_hal_i3c_rh_t *pRHeader;
    I3C_Type *pI3C = NULL;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_I3C_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( !pI3CState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pI3C = I3Cn(pI3CState->ui32Module);

    if ( ui32IntStatus & I3C_PIOINTRSTATUS_IBISTATUSTHLDSTAT_Msk )
    {
        pI3C->PIOINTRSIGNALENABLE &= ~I3C_PIOINTRSIGNALENABLE_IBITHLDSIGNALEN_Msk;
        am_hal_i3c_pio_ibi_handling(pI3CState);
        pI3CState->pHost->Event.eType = AM_HAL_I3C_EVT_IBI;
        pI3CState->pHost->Event.pCtxt = pI3CState->pHost;
        if ( pI3CState->pHost->pfunCallback )
        {
            pI3CState->pHost->pfunCallback(&pI3CState->pHost->Event);
        }
    }

    if ( ui32IntStatus & I3C_PIOINTRSTATUS_RXTHLDSTAT_Msk )
    {
        pI3C->PIOINTRSIGNALENABLE &= ~I3C_PIOINTRSIGNALENABLE_RXTHLDSIGNALEN_Msk;
    }

    if ( ui32IntStatus & I3C_PIOINTRSTATUS_TXTHLDSTAT_Msk )
    {
        pI3C->PIOINTRSIGNALENABLE &= ~I3C_PIOINTRSIGNALENABLE_TXTHLDSIGNALEN_Msk;
    }

    if ( ui32IntStatus & I3C_PIOINTRSTATUS_RESPREADYSTAT_Msk )
    {
        pI3C->PIOINTRSIGNALENABLE &= ~I3C_PIOINTRSIGNALENABLE_RESPREADYSIGNALEN_Msk;
    }

    if ( ui32IntStatus & I3C_PIOINTRSTATUS_CMDQUEUEREADYSTAT_Msk )
    {
        pI3C->PIOINTRSIGNALENABLE &= ~I3C_PIOINTRSIGNALENABLE_CMDQUEUEREADYSIGNALEN_Msk;
    }

    if ( (ui32IntStatus >> 16) & (I3C_RHINTRSTATUS_IBIREADYSTAT_Msk | I3C_RHINTRSTATUS_IBIRINGFULLSTAT_Msk) )
    {
        if ( (ui32IntStatus >> 16) & I3C_RHINTRSTATUS_IBIREADYSTAT_Msk )
        {
            pI3C->RHINTRSIGNALENABLE &= ~I3C_RHINTRSIGNALENABLE_IBIREADYSIGNALEN_Msk;
        }
        else
        {
            pI3C->RHINTRSIGNALENABLE &= ~I3C_RHINTRSIGNALENABLE_IBIRINGFULLSIGNALEN_Msk;
        }
        am_hal_i3c_dma_ibi_handling(pI3CState);
        pI3CState->pHost->Event.eType = AM_HAL_I3C_EVT_IBI;
        pI3CState->pHost->Event.pCtxt = pI3CState->pHost;
        if ( pI3CState->pHost->pfunCallback )
        {
            pI3CState->pHost->pfunCallback(&pI3CState->pHost->Event);
        }
    }

    if ( (ui32IntStatus >> 16) & (I3C_RHINTRSTATUS_TRANSFERCOMPLETIONSTAT_Msk | I3C_RHINTRSTATUS_RHTRANSFERERRSTAT_Msk) )
    {
        pRHeader = &pI3CState->pHost->pRing->RHeader[0];
        am_hal_i3c_dma_xfer_done(pI3CState, pRHeader);

        if ( (ui32IntStatus >> 16) & I3C_RHINTRSTATUS_RHTRANSFERERRSTAT_Msk )
        {
            pI3CState->pHost->Event.eType = AM_HAL_I3C_EVT_ERROR;
            pI3CState->pHost->Event.pCtxt = pI3CState->pHost;
        }
        else
        {
            if (pI3CState->eDir == AM_HAL_I3C_DIR_WRITE)
            {
                pI3CState->pHost->Event.eType = AM_HAL_I3C_EVT_TX_COMPLETE;
                pI3CState->pHost->Event.pCtxt = pI3CState->pHost;
            }
            else
            {
                pI3CState->pHost->Event.eType = AM_HAL_I3C_EVT_RX_COMPLETE;
                pI3CState->pHost->Event.pCtxt = pI3CState->pHost;
            }
        }

        if ( pI3CState->pHost->pfunCallback )
        {
            pI3CState->pHost->pfunCallback(&pI3CState->pHost->Event);
        }
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
