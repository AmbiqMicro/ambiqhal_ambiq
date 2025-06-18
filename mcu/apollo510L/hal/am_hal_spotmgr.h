//*****************************************************************************
//
//! @file am_hal_spotmgr.h
//!
//! @brief SPOT manager functions that manage power states.
//!
//! @addtogroup spotmgr510l SPOTMGR - SPOT Manager
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
#ifndef AM_HAL_SPOTMGR_H
#define AM_HAL_SPOTMGR_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Configurations
//
//*****************************************************************************
//! Disable handling for trimver1
#define AM_HAL_SPOTMGR_TRIMVER_1_DISABLE   0
//! Disable handling for trimver2
#define AM_HAL_SPOTMGR_TRIMVER_2_DISABLE   0

//! Macros to force all PCM version to be disabled when AM_HAL_DISABLE_SPOTMGR
//! is defined
#ifdef AM_HAL_DISABLE_SPOTMGR
#undef AM_HAL_SPOTMGR_TRIMVER_1_DISABLE
#undef AM_HAL_SPOTMGR_TRIMVER_2_DISABLE
#define AM_HAL_SPOTMGR_TRIMVER_1_DISABLE    1
#define AM_HAL_SPOTMGR_TRIMVER_2_DISABLE    1
#endif

//! Macros to indicate whether Internal Timer is required by SPOT manager
#define AM_HAL_SPOTMGR_INTERNAL_TIMER_NEEDED    (0)

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************

//! Mask of all available peripherals for DEVPWRST
#define DEVPWRST_ALL_PERIPH_MASK   0xFFFFE6FE
//! Mask of all available peripherals for AUDSSPWRST
#define AUDSSPWRST_ALL_PERIPH_MASK 0x44
//! Mask of DEV peripherals monitored by SPOT manager
#define DEVPWRST_MONITOR_PERIPH_MASK   DEVPWRST_ALL_PERIPH_MASK
//! Mask of AUDSS peripherals monitored by SPOT manager
#define AUDSSPWRST_MONITOR_PERIPH_MASK AUDSSPWRST_ALL_PERIPH_MASK

//*****************************************************************************
//
//! SPOTmanager stimulus and profiles
//
//*****************************************************************************

//
//! CPU/CM55 status enum
//
typedef enum
{
    AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_LP,
    AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP1,
    AM_HAL_SPOTMGR_CPUSTATE_ACTIVE_HP2,
    AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEP,
    AM_HAL_SPOTMGR_CPUSTATE_SLEEP_DEEPER,
    AM_HAL_SPOTMGR_CPUSTATE_SLEEP_ARM, // Place holder - may be used in the future
    AM_HAL_SPOTMGR_CPUSTATE_SLEEP_AMB, // Place holder - may be used in the future
} am_hal_spotmgr_cpu_state_e;

//
//! GPU status enum
//! We never transition from LP to HP or vice versa - we always go to off first.
//
typedef enum
{
    AM_HAL_SPOTMGR_GPUSTATE_OFF,
    AM_HAL_SPOTMGR_GPUSTATE_ACTIVE,
} am_hal_spotmgr_gpu_state_e;

//
//! Temperature range enum
//
typedef enum
{
    //! -40C ~ 50C
    AM_HAL_SPOTMGR_TEMPCO_RANGE_LOW,
    //! > 50C
    AM_HAL_SPOTMGR_TEMPCO_RANGE_HIGH,
    AM_HAL_SPOTMGR_TEMPCO_OUT_OF_RANGE
} am_hal_spotmgr_tempco_range_e;

//
//! CPU, GPU, memories and peripherals power status, temperature range.
//
typedef struct
{
    uint32_t ui32DevPwrSt; // Place holder - may be used in the future
    uint32_t ui32AudSSPwrSt; // Place holder - may be used in the future
    uint32_t ui32MemPwrSt; // Place holder - may be used in the future
    uint32_t ui32SsramPwrSt; // Place holder - may be used in the future
    am_hal_spotmgr_tempco_range_e eTempRange;
    am_hal_spotmgr_cpu_state_e eCpuState;
    am_hal_spotmgr_gpu_state_e eGpuState;
} am_hal_spotmgr_power_status_t;

//
//! Temperature Stimulus param struct
//
typedef struct
{
    //! Current temperature
    float fTemperature;
    //! returned value for lower temperature range
    float fRangeLower;
    //! returned value for higher temperature range
    float fRangeHigher;
} am_hal_spotmgr_tempco_param_t;

//
//! Stimulus for power state transition
//
typedef enum
{
    //! CPU power and performance state
    AM_HAL_SPOTMGR_STIM_CPU_STATE,
    //! GPU power and performance state
    AM_HAL_SPOTMGR_STIM_GPU_STATE,
    //! Temperature range
    AM_HAL_SPOTMGR_STIM_TEMP,
    //! Peripherals power state included in DEVPWRSTATUS regs, except GPU.
    AM_HAL_SPOTMGR_STIM_DEVPWR,
    //! Peripherals power state included in AUDSSPWRSTATUS regs.
    AM_HAL_SPOTMGR_STIM_AUDSSPWR,
    //! Memory power state included in MEMPWRSTATUS regs.
    AM_HAL_SPOTMGR_STIM_MEMPWR, // Place holder - may be used in the future
    //! SSRAM power state included in SSRAMPWRST regs.
    AM_HAL_SPOTMGR_STIM_SSRAMPWR, // Place holder - may be used in the future
} am_hal_spotmgr_stimulus_e;


//
// Include addtional header file for each PCM
//
#include "am_hal_spotmgr_trimver_1.h"

//
// Define macro AM_HAL_SPOTMGR_PROFILING to enable SPOTMGR profiling functionality.
// Define macro AM_HAL_SPOTMGR_PROFILING_VERBOSE to enable verbose entries.
//
// #define AM_HAL_SPOTMGR_PROFILING
// #define AM_HAL_SPOTMGR_PROFILING_VERBOSE

//
// SPOT manager Profiling struct and markers
//
#ifdef AM_HAL_SPOTMGR_PROFILING
typedef struct
{
  union
  {
    struct
    {
      uint32_t pwrState  : 6;
      uint32_t tonState  : 6;
      uint32_t eStimulus : 4;
      uint32_t bOn       : 2;
      uint32_t resv      : 14;
    } s;
    uint32_t u32;
  } u;
  uint32_t args;
} am_hal_spotmgr_changelog_t;

#define AM_HAL_SPOTMGR_PROFILING_PWRST_INVALID 0x3F
#define AM_HAL_SPOTMGR_PROFILING_TONST_INVALID 0x3F
#define AM_HAL_SPOTMGR_PROFILING_TONST_VERBOSE 0x3E
#define AM_HAL_SPOTMGR_PROFILING_ESTIM_INVALID 0xF
#define AM_HAL_SPOTMGR_PROFILING_BON_INVALID   0x3
#endif

//*****************************************************************************
//
//! Global variables
//
//*****************************************************************************
//! Trim Version booleans for optimization of trim version eval
extern bool g_bIsTrimver1OrNewer;

//*****************************************************************************
//
//! @brief Power states update
//!        This API should be called before turning things on and after turning
//!        things off, or before switching to high performance mode and after
//!        switching to low power mode, or when temperature range is changed.
//!
//! @param eStimulus - Stimilus for power states transition. For GPU state/power
//!                    changes, please use AM_HAL_SPOTMGR_STIM_GPU_STATE but not
//!                    AM_HAL_SPOTMGR_STIM_DEVPWR.
//! @param bOn       - Only needs to be set to true/false when turning on/off
//!                    peripherals or memories included in DEVPWRSTATUS,
//!                    AUDSSPWRSTATUS, MEMPWRSTATUS and SSRAMPWRST. It is
//!                    ignored when updating temperature, CPU state and GPU state.
//! @param pArgs     - Pointer to arguments for power states update, assign it
//!                    to NULL if not needed.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_CPU_STATE,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_cpu_state_e enum.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_GPU_STATE,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_gpu_state_e enum.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_TEMP,
//! bOn is ignored, and pArgs must point to a am_hal_spotmgr_tempco_param_t struct.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_DEVPWR,
//! bOn must be set to true when turning on a peripheral,
//! bOn must be set to false when turning off a peripheral,
//! and pArgs must point to the DEVPWRSTATUS_MASK of the peripheral to be opened
//! when turning on a peripheral, pArgs is ignored when turning off a peripheral.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_AUDSSPWR,
//! bOn must be set to true when turning on a peripheral,
//! bOn must be set to false when turning off a peripheral,
//! and pArgs must point to the AUDSSPWRSTATUS_MASK of the peripheral to be opened
//! when turning on a peripheral, pArgs is ignored when turning off a peripheral.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_MEMPWR,
//! bOn is ignored, and pArgs must point to the entire MEMPWRSTATUS.
//!
//! When eStimulus is AM_HAL_SPOTMGR_STIM_SSRAMPWR,
//! bOn must be set to true when turning on partial or entire SSRAM,
//! bOn must be set to false when turning off partial or entire SSRAM,
//! and pArgs must point to the expected SSRAMPWRST when turning on,
//! pArgs is ignored when turning off.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_power_state_update(am_hal_spotmgr_stimulus_e eStimulus, bool bOn, void *pArgs);

//*****************************************************************************
//
//! @brief Restore power state to POR default
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_default_reset(void);

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just before enabling
//!        SIMOBUCK
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_simobuck_init_bfr_enable(void);

//*****************************************************************************
//
//! @brief SIMOBUCK initialziation handling at stage just after enabling
//!        SIMOBUCK
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_simobuck_init_aft_enable(void);

#if NO_TEMPSENSE_IN_DEEPSLEEP
//*****************************************************************************
//
//! @brief Prepare SPOT manager for suspended tempco during deep sleep
//!
//! @return SUCCESS or other Failures.
//!
//! This API is to be called before entering deepsleep if there is no handling
//! to execute temperature sensing periodically after the deepsleep is entered.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_tempco_suspend(void);
#endif

#if AM_HAL_PWRCTRL_SIMOLP_AUTOSWITCH
//*****************************************************************************
//
//! @brief Initialize registers for SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_simobuck_lp_autosw_init(void);

//*****************************************************************************
//
//! @brief Enable SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_simobuck_lp_autosw_enable(void);

//*****************************************************************************
//
//! @brief Disable SIMOBUCK LP auto switch
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_simobuck_lp_autosw_disable(void);
#endif

//*****************************************************************************
//
//! @brief SPOT manager init
//!        This API should be called from am_hal_pwrctrl_low_power_init, to
//!        initialise SPOT manager.
//!
//! @return SUCCESS or other Failures.
//
//*****************************************************************************
extern uint32_t am_hal_spotmgr_init(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SPOTMGR_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************


