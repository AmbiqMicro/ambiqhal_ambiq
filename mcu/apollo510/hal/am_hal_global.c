//*****************************************************************************
//
//! @file am_hal_global.c
//!
//! @brief Locate global variables here.
//!
//! @addtogroup globals_ap510 Globals - HAL globals
//! @ingroup apollo510_hal
//! @{
//!
//! Purpose: This module contains global variables that are used throughout the
//! HAL, but not necessarily those designated as const (which typically end up
//! in flash). Consolidating globals here will make it easier to manage them.
//!
//! One use in particular is that it uses a global HAL flags variable that
//! contains flags used in various parts of the HAL.
//!
//! @section hal_global_features Key Features
//!
//! 1. @b Version @b Information: HAL version tracking and compiler identification.
//! 2. @b FPGA @b Support: Runtime FPGA frequency configuration and management.
//! 3. @b System @b Utilities: Fundamental system functions and utilities.
//! 4. @b Global @b State: Centralized global variable management.
//! 5. @b Compatibility: Support for various compiler and platform configurations.
//!
//! @section hal_global_functionality Functionality
//!
//! - Provide HAL version and compiler information
//! - Manage FPGA frequency settings and configurations
//! - Supply system-wide utility functions
//! - Handle global state and configuration
//! - Support for development and debugging tools
//!
//! @section hal_global_usage Usage
//!
//! 1. Access version information via global variables
//! 2. Configure FPGA settings as needed
//! 3. Use utility functions for system operations
//! 4. Access global state throughout the HAL
//!
//! @section hal_global_configuration Configuration
//!
//! - @b APOLLO5_FPGA: Enable/disable FPGA-specific features
//! - @b Compiler @b Support: Automatic detection of compiler features
//! - @b Version @b Macros: HAL version and revision tracking
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
// This is part of revision release_sdk5p1p0-366b80e084 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//*****************************************************************************
//
// Version information
//
//*****************************************************************************
const uint8_t  g_ui8HALcompiler[] = COMPILER_VERSION;
const am_hal_version_t g_ui32HALversion =
{
    .s.bAMREGS  = false,
    .s.Major    = AM_HAL_VERSION_MAJ,
    .s.Minor    = AM_HAL_VERSION_MIN,
    .s.Revision = AM_HAL_VERSION_REV
};

//*****************************************************************************
//
// Static function for reading the timer value.
//
//*****************************************************************************
#if defined (__GNUC__)          // ARM6 and GCC compiler
#ifndef __ARMCC_VERSION         // naked attribute with input parameter only
                                // allowed on gcc compiler
__attribute__((naked))
#endif
void
am_hal_triple_read(uint32_t ui32TimerAddr, uint32_t ui32Data[])
{
  __asm (
    " push  {R1, R4}\n"
    " mrs   R4, PRIMASK\n"
    " cpsid i\n"
    " nop\n"
    " ldr   R1, [R0, #0]\n"
    " ldr   R2, [R0, #0]\n"
    " ldr   R3, [R0, #0]\n"
    " msr   PRIMASK, r4\n"
    " pop   {R0, R4}\n"
    " str   R1, [R0, #0]\n"
    " str   R2, [R0, #4]\n"
    " str   R3, [R0, #8]\n"
#ifndef __ARMCC_VERSION
    " bx    lr          \n"     // Return to caller, required for naked attribute function
#endif
#ifdef  __ARMCC_VERSION         // Clobber register list and input operand assignment
                                // valid for ARMCC compiler only. GCC compiler directly
                                // utilizes the register because of its naked attribute
    :
    : [ui32TimerAddr] "r" (ui32TimerAddr),
      [ui32Data] "r" (&ui32Data[0])
    : "r0", "r1", "r2", "r3", "r4"
#endif
  );
}
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma diag_suppress = Pe940   // Suppress IAR compiler warning about missing
                                // return statement on a non-void function
__stackless void
am_hal_triple_read( uint32_t ui32TimerAddr, uint32_t ui32Data[])
{
    __asm(" push    {r1, r4}    ");         // Save r1=ui32Data, r4
    __asm(" mrs     r4, PRIMASK ");         // Save current interrupt state
    __asm(" cpsid   i           ");         // Disable INTs while reading the reg
    __asm(" ldr     r1, [r0, #0]");         // Read the designated register 3 times
    __asm(" ldr     r2, [r0, #0]");         //  "
    __asm(" ldr     r3, [r0, #0]");         //  "
    __asm(" msr     PRIMASK, r4 ");         // Restore interrupt state
    __asm(" pop     {r0, r4}    ");         // Get r0=ui32Data, restore r4
    __asm(" str     r1, [r0, #0]");         // Store 1st read value to array
    __asm(" str     r2, [r0, #4]");         // Store 2nd read value to array
    __asm(" str     r3, [r0, #8]");         // Store 3rd read value to array
    __asm(" bx      lr          ");         // Return to caller
}
#pragma diag_default = Pe940    // Restore IAR compiler warning
#else
#error Compiler is unknown, please contact Ambiq support team
#endif

//*****************************************************************************
//
//! Timer ISR for HAL internal use
//
//*****************************************************************************
void
hal_internal_timer_isr(void)
{
#if AM_HAL_SPOTMGR_INTERNAL_TIMER_NEEDED
    //
    // Interrupt service for SPOT manager timer
    //
    am_hal_spotmgr_boost_timer_interrupt_service();
#endif
}

#if defined(__GNUC__)
//
// Stub functions for some standard functions.
// These are used to avoid GCC warnings at link time.
//
#define ENOSYS 88   /* Function not implemented */ /* Defined in errno.h */

//
// Define all stubs as weak.
//
extern int _exit(void)              __attribute ((weak));
extern int _kill(void)              __attribute ((weak));
extern int _write(void)             __attribute ((weak));
extern int _getpid(void)            __attribute ((weak));

extern int _close(void)             __attribute ((weak));
extern int _lseek(void)             __attribute ((weak));
extern int _read(void)              __attribute ((weak));
extern int _sbrk(void)              __attribute ((weak));
extern int _fstat(void)             __attribute ((weak));
extern int _isatty(void)            __attribute ((weak));

//
// Stub functions.
//
int _exit(void)     { return ENOSYS; }
int _kill(void)     { return ENOSYS; }
int _write(void)    { return ENOSYS; }
int _getpid(void)   { return ENOSYS; }

int _close(void)    { return ENOSYS; }
int _lseek(void)    { return ENOSYS; }
int _read(void)     { return ENOSYS; }
int _sbrk(void)     { return ENOSYS; }
int _fstat(void)    { return ENOSYS; }
int _isatty(void)   { return ENOSYS; }
#endif // __GNUC__


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
