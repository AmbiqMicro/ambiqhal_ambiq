//*****************************************************************************
//
//! @file am_hal_security.c
//!
//! @brief Functions for on-chip security features
//!
//! @addtogroup security Security Functionality
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
//  Local defines.
//*****************************************************************************
//
//! Maximum iterations for hardware CRC to finish
//
#define MAX_CRC_WAIT        100000

#define AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_PROG   0x00000001
#define AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_READ   0x00000002

//*****************************************************************************
//
// Globals
//
//*****************************************************************************

//*****************************************************************************
//
// Get Device Security Info
//
// This will retrieve the security information for the device
//
//*****************************************************************************
uint32_t
am_hal_security_get_info(am_hal_security_info_t *pSecInfo)
{
    uint32_t ui32Ret;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( !pSecInfo )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    ui32Ret = am_hal_info0_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO0,
                                AM_REG_OTP_INFO0_SECURITY_VERSION_O / 4,
                                1, &pSecInfo->info0Version);

    if ( ui32Ret != AM_HAL_STATUS_SUCCESS )
    {
        pSecInfo->info0Version = 0;
        return ui32Ret;
    }

    pSecInfo->bInfo0Valid = am_hal_info0_valid();

    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == PWRCTRL_DEVPWRSTATUS_PWRSTCRYPTO_OFF)   ||
        (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 0) )
    {
        //
        // Crypto is not accessible
        //
        pSecInfo->lcs = AM_HAL_SECURITY_LCS_UNDEFINED;
    }
    else
    {
        pSecInfo->lcs = (am_hal_security_device_lcs_e)CRYPTO->LCSREG_b.LCSREG;
    }

    pSecInfo->sbrVersion = g_am_hal_bootrom_helper.bootrom_version_info() ;

    //
    // Fetch the SBL Version and Date Code
    //
    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_MRAM_INFO1, \
                                AM_REG_INFO1_SBL_VERSION_0_O / 4, \
                                1, \
                                &pSecInfo->sblVersion );
    if ( ui32Ret != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Ret;
    }

    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_MRAM_INFO1, \
                                AM_REG_INFO1_SBL_VERSION_1_O / 4, \
                                1, \
                                &pSecInfo->sblVersionAddInfo );
    return ui32Ret;

} // am_hal_security_get_info()

//*****************************************************************************
//
// Get Device Security SOCID
//
// This will retrieve the SOCID information for the device
//
//*****************************************************************************
uint32_t
am_hal_security_get_socid(am_hal_security_socid_t *pSocId)
{
    uint32_t ux, ui32Ret;
    uint32_t *pui32SrcSocId, *pui32DstSocId;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( pSocId == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    if ( g_sINFO1regs.ui32INFO1GlobalValid == INFO1GLOBALVALID )
    {
        pui32SrcSocId = &g_sINFO1regs.ui32SOCID0;
        pui32DstSocId = &pSocId->socid[0];

        for ( uint32_t ux = 0; ux < AM_HAL_SECURITY_SOCID_NUMWORDS; ux++ )
        {
            *pui32DstSocId++ = *pui32SrcSocId++;
        }
        ui32Ret = AM_HAL_STATUS_SUCCESS;
    }
    else
    {
        ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_OTP_INFO1,
                                    AM_REG_OTP_INFO1_SOCID0_O / 4,
                                    AM_HAL_SECURITY_SOCID_NUMWORDS,
                                    &pSocId->socid[0]);
    }

    if ( ui32Ret != AM_HAL_STATUS_SUCCESS )
    {
        pui32DstSocId = &pSocId->socid[0];
        for (ux = 0; ux < AM_HAL_SECURITY_SOCID_NUMWORDS; ux++ )
        {
            *pui32DstSocId++ = 0x00000000;
        }
    }

    return ui32Ret;

} // am_hal_security_get_socid()

//*****************************************************************************
//
// Set the key for specified lock
//
// This will program the lock registers for the specified lock and key
//
//*****************************************************************************
uint32_t
am_hal_security_set_key(am_hal_security_locktype_t lockType, am_hal_security_128bkey_t *pKey)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if ( pKey == NULL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (lockType)
    {
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_PROG:
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_READ:
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    SECURITY->LOCKCTRL = lockType;
    SECURITY->KEY0 = pKey->keys.key0;
    SECURITY->KEY1 = pKey->keys.key1;
    SECURITY->KEY2 = pKey->keys.key2;
    SECURITY->KEY3 = pKey->keys.key3;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_security_set_key()

//*****************************************************************************
//
// Get the current status of the specified lock
//
// This will get the lock status for specified lock - true implies unlocked
// Note that except for customer lock, other locks are self-locking on status read
//
//*****************************************************************************
uint32_t
am_hal_security_get_lock_status(am_hal_security_locktype_t lockType, bool *pbUnlockStatus)
{
    uint32_t unlockMask;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (pbUnlockStatus == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    switch ( lockType )
    {
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_PROG:
            unlockMask = AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_PROG;
            break;
        case AM_HAL_SECURITY_LOCKTYPE_CUSTOTP_READ:
            unlockMask = AM_HAL_SECURITY_LOCKSTAT_CUSTOTP_READ;
            break;
        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }
    *pbUnlockStatus = SECURITY->LOCKSTAT & unlockMask;

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_security_get_lock_status()

//*****************************************************************************
//
// Compute CRC32 for a specified payload
//
// This function uses the hardware engine to compute CRC32 on an arbitrary data
// payload.  The payload can reside in any contiguous memory including external
// memory.
//
//*****************************************************************************
uint32_t
am_hal_crc32(uint32_t ui32StartAddr, uint32_t ui32SizeBytes, uint32_t *pui32Crc)
{
    uint32_t status, ui32CRC32;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (pui32Crc == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Make sure size is multiple of 4 bytes
    //
    if (ui32SizeBytes & 0x3)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

#endif // AM_HAL_DISABLE_API_VALIDATION

    //
    // Program the CRC engine to compute the crc
    //
    ui32CRC32                 = 0xFFFFFFFF;
    SECURITY->RESULT          = ui32CRC32;
    SECURITY->SRCADDR         = ui32StartAddr;
    SECURITY->LEN             = ui32SizeBytes;
    SECURITY->CTRL_b.FUNCTION = SECURITY_CTRL_FUNCTION_CRC32;

    //
    // Start the CRC
    //
    SECURITY->CTRL_b.ENABLE = 1;

    //
    // Wait for CRC to finish
    //
    status = am_hal_delay_us_status_change(MAX_CRC_WAIT,
        (uint32_t)&SECURITY->CTRL, SECURITY_CTRL_ENABLE_Msk, 0);

    if (status == AM_HAL_STATUS_SUCCESS)
    {
        if (SECURITY->CTRL_b.CRCERROR)
        {
            status = AM_HAL_STATUS_HW_ERR;
        }
        *pui32Crc = SECURITY->RESULT;
    }

    return status;

} // am_hal_crc32()

//*****************************************************************************
//
//! @brief  Hardcoded function - to Run supplied main program
//!
//! @param  r0 = vtor - address of the vector table
//!
//! @return Returns None
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION >= 6000000)
__attribute__((naked))
static void
bl_run_main(uint32_t *vtor)
{
    __asm
    (
        "   movw    r3, #0xED08\n\t"    // Store the vector table pointer of the new image into VTOR.
        "   movt    r3, #0xE000\n\t"
        "   str     r0, [r3, #0]\n\t"
        "   ldr     r3, [r0, #0]\n\t"   // Load the new stack pointer into R1 and the new reset vector into R2.
        "   ldr     r2, [r0, #4]\n\t"
        "   mov     sp, r3\n\t"         // Set the stack pointer for the new image.
        "   bx      r2\n\t"            // Jump to the new reset vector.
    );
}
#elif defined(__GNUC_STDC_INLINE__)
__attribute__((naked))
static void
bl_run_main(uint32_t *vtor)
{
    __asm
    (
        "   movw    r3, #0xED08\n\t"    // Store the vector table pointer of the new image into VTOR.
        "   movt    r3, #0xE000\n\t"
        "   str     r0, [r3, #0]\n\t"
        "   ldr     r3, [r0, #0]\n\t"   // Load the new stack pointer into R1 and the new reset vector into R2.
        "   ldr     r2, [r0, #4]\n\t"
        "   mov     sp, r3\n\t"         // Set the stack pointer for the new image.
        "   bx      r2\n\t"            // Jump to the new reset vector.
    );
}
#elif defined(__IAR_SYSTEMS_ICC__)
__stackless static inline void
bl_run_main(uint32_t *vtor)
{
    __asm volatile (
          "    movw    r3, #0xED08\n"    // Store the vector table pointer of the new image into VTOR.
          "    movt    r3, #0xE000\n"
          "    str     r0, [r3, #0]\n"
          "    ldr     r3, [r0, #0]\n"   // Load the new stack pointer into R1 and the new reset vector into R2.
          "    ldr     r2, [r0, #4]\n"
          "    mov     sp, r3\n"         // Set the stack pointer for the new image.
          "    bx      r2\n"            // Jump to the new reset vector.
          );
}
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//*****************************************************************************
//
// @brief  Helper function to Perform exit operations for a secondary bootloader
//
// @param  pImage - The address of the image to give control to
//
// This function does the necessary security operations while exiting from a
// a secondary bootloader program. If still open, it locks the infoc key region,
// as well as further updates to the flash protection register.
// It also checks if it needs to halt to honor a debugger request.
// If an image address is specified, control is transferred to the same on exit.
//
// @return Returns AM_HAL_STATUS_SUCCESS on success, if no image address specified
// If an image address is provided, a successful execution results in transfer to
// the image - and this function does not return.
//
//*****************************************************************************
uint32_t
am_hal_bootloader_exit(uint32_t *pImage, bool bEnableDebuggerOnExit)
{
    am_hal_reset_status_t sResetStatus;

    //
    // Lock the protection register to prevent further region locking
    // CAUTION!!! - Cannot do RMW on BOOTLOADER register as all writable
    //              bits in this register are Write 1 to clear
    //
    MCUCTRL->BOOTLOADER = _VAL2FLD(MCUCTRL_BOOTLOADER_PROTLOCK, MCUCTRL_BOOTLOADER_PROTLOCK_LOCK);

    //
    // Check if we need to halt (debugger request)
    //
    if ( bEnableDebuggerOnExit )
    {
        //
        // Enable debugger
        //
        am_hal_dcu_update(true, AM_HAL_DCU_DEBUGGER);
        if ( MCUCTRL->DEBUGGER & AM_HAL_DCU_DEBUGGER)
        {
            //
            // This could be the case if INFOC->DCU_DISABLEOVERRIDE is set
            // Remove the overrides
            //
            MCUCTRL->DEBUGGER &= ~AM_HAL_DCU_DEBUGGER;
        }

        //
        // Get reset status
        //
        am_hal_reset_status_get(&sResetStatus);

        //
        // Act on debugger reset-halt if deferred by SBL
        //
        if ( (sResetStatus.eStatus & RSTGEN_STAT_AIRCRSTAT_Msk) &&
             (MCUCTRL->SCRATCH0 & 0x01) )
        {
            //
            // Clear the flag in scratch0
            //
            MCUCTRL->SCRATCH0 &= ~0x1;
            uint32_t dhcsr = DCB->DSCSR;

            //
            // Halt the core
            //
            DCB->DSCSR = (uint32_t)(0xA05FUL << 16) | (dhcsr & 0xFFFF) | 0x3;

            //
            // Resume from halt
            //
        }
    }

    //
    // Give control to supplied image
    //
    if ( pImage )
    {
        bl_run_main(pImage);

        //
        // Does not return
        //
    }

    return AM_HAL_STATUS_SUCCESS;

} // am_hal_bootloader_exit()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
