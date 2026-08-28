//*****************************************************************************
//
//! @file am_hal_cc312_bypass.h
//!
//! @brief Hardware abstraction for the CryptoCell-312 bypass DMA
//!
//! @addtogroup cc312_bypass_ap510L CC312 Bypass - DMA Copy
//! @ingroup apollo510L_hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_HAL_CC312_BYPASS_H
#define AM_HAL_CC312_BYPASS_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Copy a buffer through the CryptoCell DMA with no transformation.
//!
//! The bypass path routes DIN straight to DOUT, so the engine acts as a
//! copy engine. It is useful when a buffer has to move while the CryptoCell
//! already owns it, avoiding a round trip through the CPU.
//!
//! Both buffers must be reachable by the CryptoCell DMA and must not
//! overlap.
//!
//! @param input  - Source buffer.
//! @param output - Destination buffer.
//! @param length - Number of bytes to copy.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_bypass_copy(const uint8_t *input,
                                         uint8_t *output,
                                         uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_BYPASS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
