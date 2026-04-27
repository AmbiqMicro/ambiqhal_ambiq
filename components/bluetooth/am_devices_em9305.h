//*****************************************************************************
//
//! @file am_devices_em9305.h
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

#ifndef AM_DEVICES_EM9305_H
#define AM_DEVICES_EM9305_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! Type definitions and constants.
//
//*****************************************************************************

#define EM9305_STS_CHK_CNT_MAX         10       //!< Check EM9305 status count
#define WAIT_EM9305_RDY_TIMEOUT        12000    //!< EM9305 timeout value (1.2 sec)
#define EM9305_BUFFER_SIZE             259      //!< Length of RX buffer
#define EM9305_SPI_HEADER_TX           0x42     //!< SPI TX header byte
#define EM9305_SPI_HEADER_RX           0x81     //!< SPI RX header byte
#define EM9305_STS1_READY_VALUE        0xC0     //!< SPI Ready byte

//
//! Status codes for EM9305 operations
//
typedef enum
{
    AM_DEVICES_EM9305_STATUS_SUCCESS,
    AM_DEVICES_EM9305_STATUS_ERROR,
    AM_DEVICES_EM9305_RX_FULL,
    AM_DEVICES_EM9305_TX_BUSY,
    AM_DEVICES_EM9305_NO_DATA_TX,
    AM_DEVICES_EM9305_NOT_READY,
    AM_DEVICES_EM9305_DATA_LENGTH_ERROR,
    AM_DEVICES_EM9305_DATA_TRANSFER_ERROR,
    AM_DEVICES_EM9305_CMD_TRANSFER_ERROR,
} am_devices_em9305_status_t;

//
//! BLE controller function callback for SPI transceive
//! Note: Uses void* to match apollox_blue.h typedef
//
typedef int (*bt_spi_transceive_fun)(void *tx, uint32_t tx_len, void *rx, uint32_t rx_len);

//
//! BLE controller function callback for SPI transmit
//
typedef int (*spi_transmit_fun)(uint8_t *data, uint16_t len);

//
//! BLE controller callback structure
//
typedef struct
{
    /**
     *************************************************************************************
     * @brief Starts a data transmission via IOM
     *
     * @param[in]  data        Pointer to the TX buffer
     * @param[in]  len         Size of the transmission
     * @return                 Status of data transmission, 0 is success.
     *************************************************************************************
     */
    spi_transmit_fun write;

    /**
     *************************************************************************************
     * @brief Reset the BLE controller via RESET GPIO.
     *
     *************************************************************************************
     */
    void (*reset)(void);
} am_devices_em9305_callback_t;

//*****************************************************************************
//
//! @brief Initialize the BLE controller driver.
//!
//! @param cb pointer of BLE Controller callback
//!
//! @return Status of initialization
//
//*****************************************************************************
uint32_t am_devices_em9305_init(am_devices_em9305_callback_t *cb);

//*****************************************************************************
//
//! @brief Deinitialize the BLE controller driver.
//!
//! This function puts the EM9305 in reset state and clears internal state.
//!
//! @return Status of deinitialization
//
//*****************************************************************************
uint32_t am_devices_em9305_deinit(void);

//*****************************************************************************
//
//! @brief Set the reset state of EM9305.
//!
//! @param data - reset state (true = set, false = clear)
//
//*****************************************************************************
void am_devices_em9305_set_reset_state(bool data);

//*****************************************************************************
//
//! @brief Get the reset state of EM9305.
//!
//! @return Current reset state
//
//*****************************************************************************
bool am_devices_em9305_get_reset_state(void);

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
                                      bt_spi_transceive_fun transceive);

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
bool am_devices_em9305_check_active_state_event(uint8_t *data, uint16_t len);

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
                                          void (*cs_release)(void));

//*****************************************************************************
//
//! @brief Get SPI TX in progress status.
//!
//! @return true if SPI TX is in progress
//
//*****************************************************************************
bool am_devices_em9305_get_spi_tx_status(void);

//*****************************************************************************
//
//! @brief Reset the EM9305 controller.
//!
//! This function performs a hardware reset sequence on the EM9305 controller.
//
//*****************************************************************************
void am_devices_em9305_controller_reset(void);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_EM9305_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
