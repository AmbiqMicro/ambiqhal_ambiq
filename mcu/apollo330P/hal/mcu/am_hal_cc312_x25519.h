//*****************************************************************************
//
//! @file am_hal_cc312_x25519.h
//!
//! @brief Hardware abstraction for CryptoCell-312 X25519
//!
//! @addtogroup cc312_x25519_ap330P CC312 X25519 - Montgomery Curve
//! @ingroup apollo330P_hal
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

#ifndef AM_HAL_CC312_X25519_H
#define AM_HAL_CC312_X25519_H

#include "am_hal_cc312.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name X25519 sizes. Both the scalar and the u-coordinate are 32 bytes,
//! little endian, as RFC 7748 defines them.
//! @{
//
//*****************************************************************************
#define AM_HAL_X25519_KEY_SIZE          32  //!< Scalar size (bytes).
#define AM_HAL_X25519_POINT_SIZE        32  //!< u-coordinate size (bytes).
//! @}

//*****************************************************************************
//
//! @brief X25519 scalar multiplication.
//!
//! Computes the u-coordinate of scalar * point on Curve25519 with the
//! Montgomery ladder, which the PKA runs. The scalar is clamped internally
//! exactly as RFC 7748 requires, so a caller may pass raw random bytes.
//!
//! Both inputs and the output are 32 byte little endian values.
//!
//! @param scalar - 32 byte scalar.
//! @param point  - 32 byte input u-coordinate.
//! @param result - 32 byte output u-coordinate.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_x25519_scalar_mult(const uint8_t *scalar,
                                                const uint8_t *point,
                                                uint8_t *result);

//*****************************************************************************
//
//! @brief X25519 public key from a private scalar.
//!
//! Multiplies the scalar by the curve base point, u = 9.
//!
//! @param scalar - 32 byte private scalar.
//! @param result - 32 byte public u-coordinate.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_cc312_x25519_base_mult(const uint8_t *scalar,
                                              uint8_t *result);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CC312_X25519_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
