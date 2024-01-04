//*****************************************************************************
//
//! @file am_apollo3_bt_support.h
//!
//! @brief Support functions for the Bluetooth controller in Apollo3.
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

uint32_t am_apollo3_bt_controller_init(void);
void am_apollo3_bt_isr_pre(void);

#ifdef __cplusplus
}
#endif

#endif // AM_APOLLO3_BT_SUPPORT_H
