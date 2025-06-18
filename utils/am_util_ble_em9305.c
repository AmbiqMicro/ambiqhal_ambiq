//*****************************************************************************
//
//! @file am_util_ble_em9305.c
//!
//! @brief EM9305 BLE functions not covered by the HAL.
//!
//! This file contains functions for interacting with the Apollo4 BLE hardware
//! that are not already covered by the HAL. Most of these commands either
//! adjust RF settings or facilitate RF testing operations.
//!
//! @addtogroup em9305_util EM9305 - BLE Functions
//! @ingroup utils
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_util_delay.h"
#include "am_mcu_apollo.h"
#include "am_devices_em9305.h"
#include "am_util_stdio.h"


//*****************************************************************************
//
// Statics
//
//*****************************************************************************

//*****************************************************************************
//
// Globals
//
//*****************************************************************************


//*****************************************************************************
//
// Reset ble controller
//
//*****************************************************************************
uint32_t
am_util_ble_hci_reset(void *pHandle)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(0)] = HCI_RAW_CMD(0x0C03, 0);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    return (am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// Set BLE sleep enable/disable for the BLE core.
// enable = 'true' set sleep enable, enable = 'false' set sleep disable
//
//*****************************************************************************
uint32_t
am_util_ble_sleep_set(void* pHandle, bool enable)
{
    uint32_t ui32BytesNum = 0;

    am_util_stdio_printf("set sleep %s\r\n", enable?"TRUE":"FALSE");

    am_devices_em9305_buffer(16) sResponse = {0};
    uint32_t ui32ErrorStatus = AM_DEVICES_EM9305_STATUS_SUCCESS;

    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_SET_SLEEP_OPTION_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_SET_SLEEP_OPTION, enable);
    ui32ErrorStatus = am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);

    if ( ui32ErrorStatus != AM_DEVICES_EM9305_STATUS_SUCCESS )
    {
        return ui32ErrorStatus;
    }

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// set the tx power of BLE
// values.
// ui32TxPower: enum txPowerLevel_t defined in hci_drv_em9305.h
//
//*****************************************************************************
uint32_t
am_util_ble_tx_power_set(void* pHandle, uint8_t ui32TxPower)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_SET_TX_POWER_LEVEL_CMD_LENGTH)] = HCI_VSC_CMD(HCI_VSC_SET_TX_POWER_LEVEL, ui32TxPower);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};
    return (am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
//! @brief To do directly transmit test where the DUT generates test reference
//! packets at a fixed interval.
//!
//! @param channel - Transmit channel, ranges 0x00 - 0x27
//! @param test_data_len - Length of payload in each packet, range 0x00-0xFF
//! @param pkt_payload -  Packet payload, ranges 0x00 - 0x07
//!
//! @return Status Code
//
//*****************************************************************************
uint32_t
am_util_ble_transmit_test(void *pHandle, uint8_t channel, uint8_t test_data_len, uint8_t pkt_payload)
{
    // Fill the buffer with the specific command we want to write
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(3)] = HCI_RAW_CMD(0x201E, 3, channel, test_data_len, pkt_payload);

    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    // issue the HCI command initate transmit test
    return am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);
}

//*****************************************************************************
//
//! @brief End test.
//!
//! @param pHandle - pointer to BLE Handle
//! @param recvpackets - pointer for RX Packets
//!
//! @return Status Code
//
//*****************************************************************************
uint32_t
am_util_ble_end_test(void *pHandle, uint32_t *recvpackets)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint32_t ui32ErrorStatus = AM_DEVICES_EM9305_STATUS_SUCCESS;
    uint8_t end_cmd[HCI_VSC_CMD_LENGTH(0)] = HCI_RAW_CMD(0x201F, 0);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    ui32ErrorStatus = am_devices_em9305_command_write(pHandle, (uint32_t*)end_cmd, sizeof(end_cmd), sResponse.words, &ui32BytesNum);
    *recvpackets = (sResponse.bytes[8] << 8) + sResponse.bytes[7];
    // Delay additional time to end test completely.
    am_util_delay_ms(10);

    return ui32ErrorStatus;
}

//*****************************************************************************
//
// to do directly output modulation signal. change channel ranges from 0 to 0x27, pattern from 0 to 7.
//
//*****************************************************************************
uint32_t
am_util_ble_trasmitter_test_ex(void *pHandle, uint8_t tx_test_mode, uint8_t channel, uint8_t test_data_len, uint8_t pkt_payload, uint8_t pyh, uint8_t tx_power)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_START_TRANSMIT_TEST_CMD_LENGTH)] = HCI_RAW_CMD(HCI_VSC_START_TRANSMIT_TEST_CMD_OPCODE, HCI_VSC_START_TRANSMIT_TEST_CMD_LENGTH, tx_test_mode, channel, test_data_len, pkt_payload, pyh, tx_power);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    return (am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// to do directly receiver test. change channel ranges from 0 to 0x27
//
//*****************************************************************************
uint32_t
am_util_ble_receiver_test_ex(void *pHandle, uint8_t channel)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t start_cmd[HCI_VSC_CMD_LENGTH(1)] = HCI_RAW_CMD(0x201D, 1, channel);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    //
    // issue the HCI command with to init for the channel 1
    //
    return am_devices_em9305_command_write(pHandle, (uint32_t*)start_cmd, sizeof(start_cmd), sResponse.words, &ui32BytesNum);
}


//*****************************************************************************
//
// Set public address
//
//*****************************************************************************
uint32_t
am_util_ble_set_bd_addr(void *pHandle, uint8_t* bd_addr)
{
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_SET_DEV_PUB_ADDR_CMD_OPCODE)] = HCI_RAW_CMD(HCI_VSC_SET_DEV_PUB_ADDR_CMD_OPCODE, HCI_VSC_SET_DEV_PUB_ADDR_CMD_LENGTH, bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    return (am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum));
}

//*****************************************************************************
//
// Get w2pass information
//
//*****************************************************************************
uint32_t
am_util_ble_get_w2pass_info(void *pHandle, char *info)
{
    uint32_t status = AM_DEVICES_EM9305_STATUS_SUCCESS;
    uint32_t w2pass_addr = 0x407F80;
    //
    // Fill the buffer with the specific command we want to write, and send it.
    //
    uint8_t write_cmd[HCI_VSC_CMD_LENGTH(HCI_VSC_READ_AT_ADDRESS_CMD_LENGTH)] = HCI_RAW_CMD(HCI_VSC_READ_AT_ADDRESS_CMD_OPCODE, HCI_VSC_READ_AT_ADDRESS_CMD_LENGTH, UINT32_TO_BYTES(w2pass_addr), 8);

    //
    // Make a buffer big enough to hold the register write command, and a
    // second one big enough to hold the response.
    //
    uint32_t ui32BytesNum = 0;
    am_devices_em9305_buffer(16) sResponse = {0};

    status = am_devices_em9305_command_write(pHandle, (uint32_t*)write_cmd, sizeof(write_cmd), sResponse.words, &ui32BytesNum);

    memcpy(info, &sResponse.bytes[7], 8);
    am_util_stdio_printf("EM9305 w2pass info: %s \r\n", info);
    return status;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

