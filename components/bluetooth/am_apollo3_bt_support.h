//*****************************************************************************
//
//! @file am_apollo3_bt_support.h
//!
//! @brief Bluetooth support for the Apollo3 Blue Series SOC.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2024, Ambiq Micro, Inc.
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
//*****************************************************************************

#ifndef AM_APOLLO3_BT_SUPPORT_H
#define AM_APOLLO3_BT_SUPPORT_H

#ifdef __cplusplus
extern "C"
{
#endif

// Tx power level in dBm.
typedef enum
{
  TX_POWER_LEVEL_MINUS_10P0_dBm = 0x4,
  TX_POWER_LEVEL_MINUS_5P0_dBm = 0x5,
  TX_POWER_LEVEL_0P0_dBm = 0x8,
  TX_POWER_LEVEL_PLUS_3P0_dBm = 0xF,
  TX_POWER_LEVEL_INVALID = 0x10,
} txPowerLevel_t;

//*****************************************************************************
//
//! @brief Initialize the Apollo3x BLE controller driver.
//!
//! @return status of BLE controller initialization.
//
//*****************************************************************************
uint32_t am_apollo3_bt_controller_init(void);

//*****************************************************************************
//
//! @brief Deinitialize the Apollo3x BLE controller driver.
//!
//! @return status of BLE controller deinitialization.
//
//*****************************************************************************
uint32_t am_apollo3_bt_controller_deinit(void);

//*****************************************************************************
//
//! @brief BLE ISR preprocessing.
//
//*****************************************************************************
void am_apollo3_bt_isr_pre(void);

#ifdef __cplusplus
}
#endif

#endif // AM_APOLLO3_BT_SUPPORT_H
