//*****************************************************************************
//
//! @file am_apollo3_bt_support.c
//!
//! @brief Bluetooth support for the Apollo3 Blue Series SOC.
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
// Boot the radio.
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
        return AM_HAL_STATUS_FAIL;
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
//
//*****************************************************************************
void am_apollo3_bt_isr_pre(void)
{
    uint32_t ui32Status = am_hal_ble_int_status(BLE, true);
    am_hal_ble_int_clear(BLE, ui32Status);
}
