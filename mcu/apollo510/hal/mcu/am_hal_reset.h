//*****************************************************************************
//
//! @file am_hal_reset.h
//!
//! @brief Hardware abstraction layer for the Reset Generator module.
//!
//! @addtogroup rstgen2 Reset - Reset Generator (RSTGEN)
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
#ifndef AM_HAL_RSTGEN_H
#define AM_HAL_RSTGEN_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// RESET specific definitions.
//
//*****************************************************************************
//**************************************
//
//! Reset Generator configuration values
//
//**************************************
typedef enum
{
    AM_HAL_RESET_BROWNOUT_HIGH_ENABLE,
    AM_HAL_RESET_WDT_RESET_ENABLE,
    AM_HAL_RESET_BROWNOUT_HIGH_DISABLE,
    AM_HAL_RESET_WDT_RESET_DISABLE
} am_hal_reset_configure_e;

//**************************************
//
//! Reset Generator control operations
//
//**************************************
typedef enum
{
    AM_HAL_RESET_CONTROL_SWPOR,
    AM_HAL_RESET_CONTROL_SWPOI,
} am_hal_reset_control_e;

//**************************************
//
//! Reset Status
//
//**************************************
typedef enum
{
    AM_HAL_RESET_STATUS_EXTERNAL    = RSTGEN_STAT_EXRSTAT_Msk,
    AM_HAL_RESET_STATUS_POA         = RSTGEN_STAT_POASTAT_Msk,
    AM_HAL_RESET_STATUS_BOD         = RSTGEN_STAT_BORSTAT_Msk,
    AM_HAL_RESET_STATUS_SWPOR       = RSTGEN_STAT_SWRSTAT_Msk,
    AM_HAL_RESET_STATUS_SWPOI       = RSTGEN_STAT_POIRSTAT_Msk,
    AM_HAL_RESET_STATUS_DEBUGGER    = RSTGEN_STAT_DBGRSTAT_Msk,
    AM_HAL_RESET_STATUS_WDT         = RSTGEN_STAT_WDRSTAT_Msk,
    AM_HAL_RESET_STATUS_BOUNREG     = RSTGEN_STAT_BOUSTAT_Msk,
    AM_HAL_RESET_STATUS_BOCORE      = RSTGEN_STAT_BOCSTAT_Msk,
    AM_HAL_RESET_STATUS_BOMEM       = RSTGEN_STAT_BOFSTAT_Msk,
    AM_HAL_RESET_STATUS_BOHPMEM     = RSTGEN_STAT_BOSSTAT_Msk,
    AM_HAL_RESET_STATUS_RSVD        = RSTGEN_STAT_BOSSTAT_Msk + 1,
    AM_HAL_RESET_STATUS_AIRCR       = RSTGEN_STAT_AIRCRSTAT_Msk
} am_hal_reset_status_e;

#define AM_HAL_RESET_STATUS_MASK    0x1FFF

//**************************************
//
//! RESET status structure
//
//**************************************
typedef struct
{
    am_hal_reset_status_e
            eStatus;        // Return all status bits from RSTGEN.STAT
    bool    bEXTStat;       // External reset
    bool    bPOAStat;       // Power-On reset
    bool    bBODStat;       // Brown-Out reset
    bool    bSWPORStat;     // SW Power-On reset
    bool    bSWPOIStat;     // SW Power On Initialization reset
    bool    bDBGRStat;      // Debugger reset
    bool    bWDTStat;       // Watch Dog Timer reset
    bool    bBOUnregStat;   // Unregulated Supply Brownout event
    bool    bBOCOREStat;    // Core Regulator Brownout event
    bool    bBOMEMStat;     // Memory Regulator Brownout event
    bool    bBOHPMEMStat;   // High power Memory Regulator Brownout event
    bool    bRsvd;
    bool    bAIRCRStat;     // AIRCR SYSRESETREQ
} am_hal_reset_status_t;

//
//! Define interrupt bit(s)
//
#define AM_HAL_RESET_INTERRUPT_BODH     RSTGEN_INTEN_BODH_Msk

// Global variable used to capture the reset status
extern uint32_t gAmHalResetStatus;

//*****************************************************************************
//
//! @brief Enable and configure the Reset controller.
//!
//! This function will configure the specified reset conditions.
//!
//! @param eConfigure - One of configuration enumerations.
//!     AM_HAL_RESET_BROWNOUT_HIGH_ENABLE
//!     AM_HAL_RESET_WDT_RESET_ENABLE
//!     AM_HAL_RESET_BROWNOUT_HIGH_DISABLE
//!     AM_HAL_RESET_WDT_RESET_DISABLE
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_reset_configure(am_hal_reset_configure_e eConfigure);

//*****************************************************************************
//
//! @brief Reset generator control function.
//!
//! This function will perform various reset functions including assertion
//! of software resets.
//!
//! @param eControl - One of the control enumerations.
//!     AM_HAL_RESET_CONTROL_SWPOR - power on reset, which results in a reset of
//!         all blocks except for registers in clock gen, RTC, stimer, PMU.
//!         Equivalent to the reset state obtained by a hardware reset, use of
//!         the ARM AIRCR (Application Interrupt and Reset Control Register)
//!         core register, debugger reset, watchdog timer expiration, or
//!         brown-out event.
//!     AM_HAL_RESET_CONTROL_SWPOI - power on initialization, which results in a
//!         reset of all blocks except for registers in clock gen, RTC, stimer.
//!         The POI reset level is required in order to enable configuration
//!         changes such as memory protection.
//! @param pArgs - Pointer to arguments for Control Switch Case
//!
//! @return status - generic or interface specific status.
//! When resetting the chip (SWPOR or SWPOI), the function will obviously
//! not return to the caller.
//
//*****************************************************************************
extern uint32_t am_hal_reset_control(am_hal_reset_control_e eControl,
                                     void *pArgs);

//*****************************************************************************
//
//! @brief Return status of the reset generator.
//!
//! This function will get the status bits from the reset generator.
//! The status value shows the type of reset(s) that have occurred since power
//! on
//! Application MUST call this API at least once before going to deepsleep
//! Otherwise this API will not provide correct reset status
//!
//! @param psStatus - Pointer to a data structure to receive the status
//! information. Most members of the structure are booleans that receive
//! the status of a particular bit.
//!
//! The eStatus member, however, returns a bitmask of one or more of the
//! following values:
//!     AM_HAL_RESET_STATUS_EXTERNAL
//!     AM_HAL_RESET_STATUS_POA
//!     AM_HAL_RESET_STATUS_BOD
//!     AM_HAL_RESET_STATUS_SWPOR
//!     AM_HAL_RESET_STATUS_SWPOI
//!     AM_HAL_RESET_STATUS_DEBUGGER
//!     AM_HAL_RESET_STATUS_WDT
//!     AM_HAL_RESET_STATUS_BOUNREG
//!     AM_HAL_RESET_STATUS_BOCORE
//!     AM_HAL_RESET_STATUS_BOMEM
//!     AM_HAL_RESET_STATUS_BOHPMEM
//!     AM_HAL_RESET_STATUS_BOLPCORE
//!
//! @return status. If the API was never called before a valid reset status
//! could be captured, AM_HAL_STATUS_FAIL is returned.
//! Otherwise AM_HAL_STATUS_SUCCESS implies valid reset status returned
//
//*****************************************************************************
extern uint32_t am_hal_reset_status_get(am_hal_reset_status_t *psStatus);

//*****************************************************************************
//
//! @brief Enable selected RSTGEN Interrupts.
//!
//! Use this function to enable the interrupts.
//!
//! @param ui32IntMask - One or more of the following bits, any of which can
//! be ORed together.
//!   AM_HAL_RESET_INTERRUPT_BODH
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_reset_interrupt_enable(uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Disable selected RSTGEN Interrupts.
//!
//! Use this function to disable the RSTGEN interrupts.
//!
//! @param ui32IntMask - One or more of the following bits, any of which can
//! be ORed together.
//!   AM_HAL_RESET_INTERRUPT_BODH
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_reset_interrupt_disable(uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Reset generator interrupt clear
//!
//! This function clears the reset generator interrupts.
//!
//! @param ui32IntMask - One or more of the following bits, any of which can
//! be ORed together.
//!   AM_HAL_RESET_INTERRUPT_BODH
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_reset_interrupt_clear(uint32_t ui32IntMask);

//*****************************************************************************
//
//! @brief Get interrupt status of reset generator.
//!
//! This function returns the interrupt status for the reset generator.
//!
//! @param bEnabledOnly determines whether disabled interrupts are included in
//! the status.
//! @param pui32IntStatus - ptr to uint32_t to return the interrupt status.
//!
//! The following are valid status bits.
//!   AM_HAL_RESET_INTERRUPT_BODH
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_reset_interrupt_status_get(bool bEnabledOnly,
                                                  uint32_t *pui32IntStatus);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_RSTGEN_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

