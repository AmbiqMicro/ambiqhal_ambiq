//*****************************************************************************
//
//! @file am_hal_sysctrl.h
//!
//! @brief Functions for interfacing with the M55 system control registers
//!
//! @addtogroup sysctrl4_ap510L SYSCTRL - System Control
//! @ingroup apollo330P_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2026, Ambiq Micro, Inc.
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
// This is part of revision v5.2.0-zephyr-685438d73f of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_HAL_SYSCTRL_H
#define AM_HAL_SYSCTRL_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Definitions for sleep mode parameter
//! @{
//
//*****************************************************************************
typedef enum
{
  AM_HAL_SYSCTRL_SLEEP_NORMAL = 0,
  AM_HAL_SYSCTRL_SLEEP_DEEP,
  AM_HAL_SYSCTRL_SLEEP_DEEPER
} am_hal_sysctrl_sleep_type_e;

#define AM_HAL_SYSCTRL_SLEEP_DEEPMAX    AM_HAL_SYSCTRL_SLEEP_DEEPER
//! @}

//*****************************************************************************
//
//! Definition of Global Power State enumeration
//
//*****************************************************************************
typedef enum
{
  AM_HAL_SYSCTRL_WAKE,
  AM_HAL_SYSCTRL_NORMALSLEEP,
  AM_HAL_SYSCTRL_DEEPSLEEP
} am_hal_sysctrl_power_state_e;

#define AM_HAL_SYSCTRL_CM4_NO_RADIO_INDICATOR    0xFFFFFFFFUL

#define SYNC_READ       0x47FF0000

//*****************************************************************************
//
//! Write flush - This function will hold the bus until all queued write
//! operations on System Bus have completed, thereby guaranteeing that all
//! writes to APB have been flushed.
//
//*****************************************************************************
#define am_hal_sysctrl_sysbus_write_flush()     AM_REGVAL(SYNC_READ)

//*****************************************************************************
//
//! Write flush - This function will return once all queued write
//! operations have completed, thereby guaranteeing that all
//! writes have been flushed.
//! This works across all the buses - AXI and APB
//
//*****************************************************************************
#define am_hal_sysctrl_bus_write_flush()        am_hal_cachectrl_dcache_invalidate(NULL, true)

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
extern bool g_bFrcBuckAct;
extern bool g_bIpcPending;
// Number of CM55 and peripheral users of HFXTAL_48M before CM55 enters deep sleep.
extern uint32_t g_ui32HfxtalUserCount;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Place the core into sleep, deepsleep or deepersleep.
//!
//! @param eSleepType - Normal or Deep, Deeper sleep.
//!
//! This function puts the MCU to sleep, deepsleep or deepersleep depending on eSleepType.
//!
//! Valid values for eSleepType are:
//!
//!     AM_HAL_SYSCTRL_SLEEP_NORMAL
//!     AM_HAL_SYSCTRL_SLEEP_DEEP
//!     AM_HAL_SYSCTRL_SLEEP_DEEPER
//
//*****************************************************************************
extern void am_hal_sysctrl_sleep(am_hal_sysctrl_sleep_type_e eSleepType);

//*****************************************************************************
//
//! @brief Control the buck state in deepsleep
//!
//! @param bFrcBuckAct - True for forcing buck active in deepsleep
//!                    - False for not forcing buck active in deepsleep
//!
//! If you want to manually force the buck stay active in deepsleep mode,
//! am_hal_sysctrl_force_buck_active_in_deepsleep must
//! be called for setting g_bAppFrcBuckAct to true before
//! calling am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP).
//! If anyone of spotmgr and
//! am_hal_sysctrl_force_buck_active_in_deepsleep forced buck stay active, buck
//! will stay active in deepsleep.
//
//*****************************************************************************
extern void am_hal_sysctrl_force_buck_active_in_deepsleep(bool bFrcBuckAct);

//*****************************************************************************
//
//! @brief Inform CM55 SPOT Manager that CM4 is going to sleep
//!        for ui32SleepDurationInMs
//!
//! @param ui32SleepDurationInMs Sleep duration in milliseconds
//! @param ui32BuckActInAdvInMs Switch buck to active by N ms before CM4 waking up.
//!
//! @return Status code
//!
//! This function is typically called in the CM55 IPC message handler.
//! It is used to notify CM55 via IPC about CM4 system power state changes:
//! 1. CM4 is entering deep sleep.
//! 2. HFXTAL 48M status:
//!    - If not used as the clock source by CM55 or its peripherals:
//!        it is either being turned off or already turned off.
//!    - Otherwise:
//!        it remains enabled.
//! 3. Baseband/RF subsystem is being powered down or already powered off.
//! 4. CM4 is required to enter deep sleep earlier than CM55 and exit sleep
//!    later than CM55.
//
//*****************************************************************************
extern uint32_t
am_hal_sysctrl_cm4_sleep_notify(uint32_t ui32SleepDurationInMs, uint32_t ui32BuckActInAdvInMs);

//*****************************************************************************
//
//! @brief Report whether all IPC from CM55 to CM4 got replied.
//! Must report pending before CM55 sends an IPC msg,
//! and report idle after CM55 received all replies from CM4.
//!
//! @param bPending True for pending, False for back to idle
//!
//! @return Status code
//
//*****************************************************************************
extern uint32_t
am_hal_sysctrl_ipc_pending_notify(bool bPending);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SYSCTRL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

