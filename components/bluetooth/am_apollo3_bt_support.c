//*****************************************************************************
//
//! @file am_apollo3_bt_support.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "am_mcu_apollo.h"
#include "am_apollo3_bt_support.h"

#define XTAL_STABILITY_MAX_RETRIES         10

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

// BLE module handle
static void *BLE;

extern am_hal_ble_state_t g_sBLEState[];

//*****************************************************************************
//
//  Initialize the Apollo3x BLE controller driver.
//
//*****************************************************************************
uint32_t am_apollo3_bt_controller_init(void)
{
    uint32_t ui32NumXtalRetries = 0;

    if (g_sBLEState[0].prefix.s.bInit)
    {
        BLE = &g_sBLEState[0];
    }
    else
    {
        am_hal_ble_initialize(0, &BLE);
        am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_ACTIVE);
    }

    //
    // Configure and enable the BLE interface.
    //
    uint32_t ui32Status = AM_HAL_STATUS_FAIL;
    while (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_hal_ble_config(BLE, &am_hal_ble_default_config);
        //
        // Delay 1s for 32768Hz clock stability. This isn't required unless this is
        // our first run immediately after a power-up.
        //
        k_sleep(K_SECONDS(1));

        //
        // Attempt to boot the radio.
        //
        ui32Status = am_hal_ble_boot(BLE);

        //
        // Check our status.
        //
        if (ui32Status == AM_HAL_STATUS_SUCCESS)
        {
            //
            // If the radio is running, we can exit this loop.
            //
            break;
        }
        else if (ui32Status == AM_HAL_BLE_32K_CLOCK_UNSTABLE)
        {
            //
            // If the radio is running, but the clock looks bad, we can try to
            // restart.
            //
            am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);
            am_hal_ble_deinitialize(BLE);

            //
            // We won't restart forever. After we hit the maximum number of
            // retries, we'll just return with failure.
            //
            if (ui32NumXtalRetries++ < XTAL_STABILITY_MAX_RETRIES)
            {
                k_sleep(K_SECONDS(1));
                am_hal_ble_initialize(0, &BLE);
                am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_ACTIVE);
            }
            else
            {
                return AM_HAL_STATUS_FAIL;
            }
        }
        else
        {
            am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);
            am_hal_ble_deinitialize(BLE);
            //
            // If the radio failed for some reason other than 32K Clock
            // instability, we should just report the failure and return.
            //
            return AM_HAL_STATUS_FAIL;
        }
    }

    //
    // Set the BLE TX Output power to 0dBm.
    //
    am_hal_ble_tx_power_set(BLE, TX_POWER_LEVEL_0P0_dBm);

    am_hal_ble_int_clear(BLE, (AM_HAL_BLE_INT_CMDCMP |
                               AM_HAL_BLE_INT_DCMP |
                               AM_HAL_BLE_INT_BLECIRQ));

    am_hal_ble_int_enable(BLE, (AM_HAL_BLE_INT_CMDCMP |
                                AM_HAL_BLE_INT_DCMP |
                                AM_HAL_BLE_INT_BLECIRQ));

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Deinitialize the Apollo3x BLE controller driver.
//
//*****************************************************************************
uint32_t am_apollo3_bt_controller_deinit(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_ble_power_control(BLE, AM_HAL_BLE_POWER_OFF);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    //
    // Give some time to power off the BLE controller
    //
    k_sleep(K_SECONDS(1));

    ui32Status = am_hal_ble_deinitialize(BLE);

    return ui32Status;
}

//*****************************************************************************
//
// BLE ISR preprocessing.
//
//*****************************************************************************
void am_apollo3_bt_isr_pre(void)
{
    uint32_t ui32Status = am_hal_ble_int_status(BLE, true);
    am_hal_ble_int_clear(BLE, ui32Status);
}
