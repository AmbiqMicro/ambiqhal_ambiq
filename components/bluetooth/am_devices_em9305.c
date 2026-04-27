//*****************************************************************************
//
//! @file am_devices_em9305.c
//!
//! @brief An implementation of the Apollo interface to EM9305 using the IOM.
//!
//! @addtogroup em9305 EM9305 BLE Device Driver
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
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
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "am_mcu_apollo.h"
#include "am_devices_em9305.h"

LOG_MODULE_REGISTER(am_devices_em9305, CONFIG_LOG_DEFAULT_LEVEL);

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//! Active state entered event signature
static uint8_t active_state_entered_evt[] = {0x04, 0xFF, 0x01, 0x01};

//! SPI lock when a transmission is in progress
static bool spiTxInProgress;

//! EM9305 status ok flag
static volatile bool Em9305status_ok;

//! EM9305 callback structure
static am_devices_em9305_callback_t g_Em9305cb;

//! GPIO operations structure (to be initialized by the driver layer)
static struct
{
    void (*set_reset)(bool state);
    bool (*get_reset)(void);
    bool (*get_irq)(void);
    void (*cs_set)(void);
    void (*cs_release)(void);
} g_gpio_ops;

//*****************************************************************************
//
//! @brief Set the reset state of EM9305.
//!
//! @param data - reset state (true = set, false = clear)
//
//*****************************************************************************
void am_devices_em9305_set_reset_state(bool data)
{
    if (g_gpio_ops.set_reset)
    {
        g_gpio_ops.set_reset(data);
    }
}

//*****************************************************************************
//
//! @brief Get the reset state of EM9305.
//!
//! @return Current reset state
//
//*****************************************************************************
bool am_devices_em9305_get_reset_state(void)
{
    if (g_gpio_ops.get_reset)
    {
        return g_gpio_ops.get_reset();
    }
    return false;
}

//*****************************************************************************
//
//! @brief Get the IRQ pin state.
//!
//! @return IRQ pin state
//
//*****************************************************************************
static bool irq_pin_state(void)
{
    if (g_gpio_ops.get_irq)
    {
        return g_gpio_ops.get_irq();
    }
    return false;
}

//*****************************************************************************
//
//! @brief Set the CS pin (active high for EM9305).
//
//*****************************************************************************
static void bt_em9305_cs_set(void)
{
    if (g_gpio_ops.cs_set)
    {
        g_gpio_ops.cs_set();
    }
}

//*****************************************************************************
//
//! @brief Release the CS pin.
//
//*****************************************************************************
static void bt_em9305_cs_release(void)
{
    if (g_gpio_ops.cs_release)
    {
        g_gpio_ops.cs_release();
    }
}

//*****************************************************************************
//
//! @brief Wait for EM9305 to be ready.
//
//*****************************************************************************
static void bt_em9305_wait_ready(void)
{
    uint16_t i;

    for (i = 0; i < WAIT_EM9305_RDY_TIMEOUT; i++)
    {
        if (irq_pin_state())
        {
            break;
        }
        k_busy_wait(100);
    }

    if (i >= WAIT_EM9305_RDY_TIMEOUT)
    {
        LOG_WRN("EM9305 ready timeout after %d ms", WAIT_EM9305_RDY_TIMEOUT * 100 / 1000);
    }
}

//*****************************************************************************
//
//! @brief Start EM9305 TX transaction and get available buffer size.
//!
//! @param transceive - SPI transceive function pointer
//!
//! @return Available buffer size in EM9305
//
//*****************************************************************************
static uint8_t am_devices_em9305_tx_starts(bt_spi_transceive_fun transceive)
{
    uint8_t sCommand[2] = {EM9305_SPI_HEADER_TX, 0x00};
    uint8_t sStas[2] = {0, 0};
    uint8_t ret = 0;

    // Indicates that a SPI transfer is in progress
    spiTxInProgress = true;

    // Select the EM9305
    bt_em9305_cs_set();

    // Wait for EM9305 ready
    bt_em9305_wait_ready();

    // Check ready again
    if (!irq_pin_state())
    {
        bt_em9305_cs_release();
        spiTxInProgress = false;
        LOG_ERR("wait em9305 ready timeout");
        return ret;
    }

    for (uint32_t i = 0; i < EM9305_STS_CHK_CNT_MAX; i++)
    {
        // Select the EM9305
        bt_em9305_cs_set();
        ret = transceive(sCommand, 2, sStas, 2);
        if (ret)
        {
            LOG_ERR("%s: spi write error %d", __func__, ret);
            return ret;
        }

        if (ret != AM_HAL_STATUS_SUCCESS)
        {
            LOG_ERR("%s: ret =%d ", __func__, ret);
            return 0;
        }

        // Check if the EM9305 is ready and the rx buffer size is not zero
        if ((sStas[0] == EM9305_STS1_READY_VALUE) && (sStas[1] != 0x00))
        {
            break;
        }
        bt_em9305_cs_release();
    }

    return sStas[1];
}

//*****************************************************************************
//
//! @brief End EM9305 TX transaction.
//
//*****************************************************************************
static void am_devices_em9305_tx_ends(void)
{
    // Deselect the EM9305
    bt_em9305_cs_release();

    // Indicates that the SPI transfer is finished
    spiTxInProgress = false;
}

//*****************************************************************************
//
//! @brief Send data to EM9305 via SPI.
//!
//! @param pui8Values - pointer to data buffer
//! @param ui32NumBytes - number of bytes to send
//! @param transceive - SPI transceive function pointer
//!
//! @return Status of transmission
//
//*****************************************************************************
int am_devices_em9305_blocking_write(uint8_t *pui8Values, uint16_t ui32NumBytes,
                                      bt_spi_transceive_fun transceive)
{
    uint32_t ui32ErrorStatus = AM_DEVICES_EM9305_STATUS_SUCCESS;
    int ret = -ENOTSUP;
    uint8_t data[EM9305_BUFFER_SIZE];
    uint8_t em9305BufSize = 0;

    if (ui32NumBytes <= EM9305_BUFFER_SIZE)
    {
        for (uint32_t i = 0; i < ui32NumBytes;)
        {
            em9305BufSize = am_devices_em9305_tx_starts(transceive);

            if (em9305BufSize == 0x00)
            {
                ui32ErrorStatus = AM_DEVICES_EM9305_RX_FULL;
                LOG_ERR("EM9305_RX_FULL");
                am_devices_em9305_tx_ends();
                break;
            }

            uint32_t len = (em9305BufSize < (ui32NumBytes - i)) ? em9305BufSize
                                                                : (ui32NumBytes - i);

            // Check again if there is room to send more data
            if ((len > 0) && (em9305BufSize))
            {
                memcpy(data, pui8Values + i, len);
                i += len;

                // Write to the IOM
                // Transmit the message
                ret = transceive(data, len, NULL, 0);

                if (ret != AM_HAL_STATUS_SUCCESS)
                {
                    ui32ErrorStatus = AM_DEVICES_EM9305_DATA_TRANSFER_ERROR;
                    LOG_ERR("%s: ret= %d", __func__, ret);
                }
            }
            am_devices_em9305_tx_ends();
        }
    }
    else
    {
        ui32ErrorStatus = AM_DEVICES_EM9305_DATA_LENGTH_ERROR;
        LOG_ERR("%s: error (STATUS ERROR) Packet Too Large", __func__);
    }

    return ui32ErrorStatus;
}

//*****************************************************************************
//
//! @brief Initialize the BLE controller driver.
//!
//! @param cb pointer of BLE Controller callback
//!
//! @return Status of initialization
//
//*****************************************************************************
uint32_t am_devices_em9305_init(am_devices_em9305_callback_t *cb)
{
    if ((!cb) || (!cb->write) || (!cb->reset))
    {
        return AM_DEVICES_EM9305_STATUS_ERROR;
    }

    // Register the callback functions
    g_Em9305cb.write = cb->write;
    g_Em9305cb.reset = cb->reset;
    g_Em9305cb.reset();

    // Wait for EM9305 activated status ok
    while (!Em9305status_ok)
    {
        ;
    }

    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Check if EM9305 received the active state entered event.
//!
//! @param data - pointer to received data
//! @param len - length of received data
//!
//! @return true if active state entered event was received
//
//*****************************************************************************
bool am_devices_em9305_check_active_state_event(uint8_t *data, uint16_t len)
{
    bool ret = false;

    if (memcmp(data, active_state_entered_evt, sizeof(active_state_entered_evt)) == 0)
    {
        LOG_INF("EM9305 enter active state");
        Em9305status_ok = true;
        ret = true;
    }

    return ret;
}

//*****************************************************************************
//
//! @brief Register GPIO operations.
//!
//! This function should be called by the driver layer to register GPIO
//! operations before calling am_devices_em9305_init().
//!
//! @param set_reset - function to set reset pin
//! @param get_reset - function to get reset pin state
//! @param get_irq - function to get IRQ pin state
//! @param cs_set - function to set CS pin
//! @param cs_release - function to release CS pin
//
//*****************************************************************************
void am_devices_em9305_register_gpio_ops(void (*set_reset)(bool),
                                          bool (*get_reset)(void),
                                          bool (*get_irq)(void),
                                          void (*cs_set)(void),
                                          void (*cs_release)(void))
{
    g_gpio_ops.set_reset = set_reset;
    g_gpio_ops.get_reset = get_reset;
    g_gpio_ops.get_irq = get_irq;
    g_gpio_ops.cs_set = cs_set;
    g_gpio_ops.cs_release = cs_release;
}

//*****************************************************************************
//
//! @brief Get SPI TX in progress status.
//!
//! @return true if SPI TX is in progress
//
//*****************************************************************************
bool am_devices_em9305_get_spi_tx_status(void)
{
    return spiTxInProgress;
}

//*****************************************************************************
//
//! @brief Deinitialize the BLE controller driver.
//!
//! This function puts the EM9305 in reset state and clears internal state.
//!
//! @return Status of deinitialization
//
//*****************************************************************************
uint32_t am_devices_em9305_deinit(void)
{
    // Hold the controller in reset state
    am_devices_em9305_set_reset_state(true);

    // Give some time for the reset to take effect
    k_sleep(K_MSEC(10));

    // Clear internal state variables
    spiTxInProgress = false;
    Em9305status_ok = false;

    // Clear callback structure
    g_Em9305cb.write = NULL;
    g_Em9305cb.reset = NULL;

    // Note: GPIO operations and pin configurations should be handled
    // by the driver layer (apollox_blue.c) as it owns the GPIO resources

    LOG_INF("EM9305 deinitialized");

    return AM_DEVICES_EM9305_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reset the EM9305 controller.
//!
//! This function performs a hardware reset sequence on the EM9305 controller.
//
//*****************************************************************************
void am_devices_em9305_controller_reset(void)
{
    // Reset the controller
    am_devices_em9305_set_reset_state(false);

    // Take controller out of reset
    k_sleep(K_MSEC(2));
    am_devices_em9305_set_reset_state(true);
    k_sleep(K_MSEC(2));
    am_devices_em9305_set_reset_state(false);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
