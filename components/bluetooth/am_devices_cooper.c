//*****************************************************************************
//
//! @file am_devices_cooper.c
//!
//! @brief An implementation of the Apollo inteface to Cooper using the IOM.
//!
//! @addtogroup cooper Cooper BLE Device Driver
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
// This is part of revision release_sdk_4_4_0-3c5977e664 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_devices_cooper.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_controller, CONFIG_LOG_DEFAULT_LEVEL);

//*****************************************************************************
//
// Timing and configuration.
//
//*****************************************************************************
#define AM_DEVICES_COOPER_SBL_SEND_WAIT_TIMEOUT        K_MSEC(1000)

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
uint8_t am_devices_cooper_nvds[HCI_VSC_UPDATE_NVDS_CFG_CMD_LENGTH] =
{
    NVDS_PARAMETER_MAGIC_NUMBER,
    NVDS_PARAMETER_SLEEP_ALGO_DUR,
    NVDS_PARAMETER_LPCLK_DRIFT,
    NVDS_PARAMETER_EXT_WAKEUP_TIME,
    NVDS_PARAMETER_OSC_WAKEUP_TIME
};

//
//!
//
static am_devices_cooper_sbl_update_state_t gsSblUpdateState;

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sFwImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW,
    0
};

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sInfo0PatchImage =
{
    0,
    sizeof(am_sbl_info0_patch_blob_t),
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0,
    0
};

//
//!
//
static am_devices_cooper_sbl_update_data_t     g_sInfo1PatchImage =
{
    0,
    0,
    AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1,
    0
};

//
//! Cooper callback
//
static am_devices_cooper_callback_t g_CooperCb;

//
//! Semaphore to control the SBL packet sending
//
static K_SEM_DEFINE(sem_sbl_send, 0, 1);

//
//! Cooper initialization state
//
static uint32_t g_CooperInitState = 0;

//*****************************************************************************
//
//  Initialize the BLE controller driver.
//
//*****************************************************************************
uint32_t am_devices_cooper_init(am_devices_cooper_callback_t* cb)
{
    uint32_t ui32Status;
    am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_STARTUP);
	am_devices_cooper_image_update_init();

    if ((!cb) || (!cb->write) || (!cb->reset))
    {
        return AM_DEVICES_COOPER_STATUS_ERROR;
    }

    // Register the callback functions
    g_CooperCb.write = cb->write;
    g_CooperCb.reset = cb->reset;

    // Start the BLE controller firmware update machine
    ui32Status = am_devices_cooper_update_image();
    while ((ui32Status != AM_DEVICES_COOPER_SBL_STATUS_OK) &&
        (ui32Status != AM_DEVICES_COOPER_SBL_STATUS_FAIL))
    {
        if (k_sem_take(&sem_sbl_send, AM_DEVICES_COOPER_SBL_SEND_WAIT_TIMEOUT) == 0)
        {
            ui32Status = am_devices_cooper_update_image();
        }
        else
        {
            return AM_DEVICES_COOPER_STATUS_TIMEOUT;
        }

    }

    return AM_DEVICES_COOPER_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  Reset BLE Controller.
//
//*****************************************************************************
void
am_devices_cooper_reset(void)
{
    g_CooperCb.reset();
}

//*****************************************************************************
//
// Receive a packet from the Cooper SBL.
//
//*****************************************************************************
void am_devices_cooper_handshake_recv(uint8_t* pBuf, uint16_t len)
{
    am_sbl_host_msg_hdr_t* msg;
    uint32_t crc32;

    // Verify the received data CRC
    msg = (am_sbl_host_msg_hdr_t*)pBuf;
    am_hal_crc32((uint32_t)&msg->msgType, msg->msgLength - sizeof(uint32_t), &crc32);

    gsSblUpdateState.pWorkBuf = (uint32_t*)msg;
    gsSblUpdateState.bRxCrcCheckPass = (crc32 == msg->msgCrc);
    gsSblUpdateState.ui32RxPacketLength = len;

    k_sem_give(&sem_sbl_send);
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
static void send_hello(void)
{
    am_sbl_host_msg_hdr_t msg;
    msg.msgType = AM_SBL_HOST_MSG_HELLO;
    msg.msgLength = sizeof(am_sbl_host_msg_hdr_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgType, msg.msgLength - sizeof(uint32_t), &msg.msgCrc);
    g_CooperCb.write((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "UPDATE" packet.
//
//*****************************************************************************
static void send_update(uint32_t imgBlobSize)
{
    am_sbl_host_msg_update_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_UPDATE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_update_t);
    msg.imageSize = imgBlobSize;
    // Check if we are downloading a newer FW versiion
    if ((gsSblUpdateState.ui32CooperFWImageVersion < g_sFwImage.version)
         || (gsSblUpdateState.ui32CooperVerRollBackConfig & 0x00000001))
    {
        msg.versionNumber = g_sFwImage.version;
    }
    else
    {
        msg.versionNumber = gsSblUpdateState.ui32CooperFWImageVersion;
    }
    msg.NumPackets = gsSblUpdateState.ui32TotalPackets + 1; // One addition packet as header will be a seperate packet

    // imageSize will be zero if Apollo4 has no available image/patch for Cooper to load
    // set maxPacketSize to invalid parameter to let Cooper to reply NACK and clear signature
    if ( msg.imageSize == 0 )
    {
        msg.maxPacketSize = AM_DEVICES_COOPER_SBL_UPADTE_INVALID_PSI_PKT_SIZE;
    }
    else
    {
        msg.maxPacketSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
    }
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    g_CooperCb.write((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "Data" packet.
//
//*****************************************************************************
static void send_data(uint32_t address, uint32_t size, uint32_t pktNumber)
{
    // reuse same buffer for receiving
    am_sbl_host_msg_data_t* msg = (am_sbl_host_msg_data_t*)gsSblUpdateState.pWorkBuf;
    msg->msgHdr.msgType = AM_SBL_HOST_MSG_DATA;
    msg->msgHdr.msgLength = sizeof(am_sbl_host_msg_data_t) + size;
    msg->packetNumber = pktNumber;
    memcpy((uint8_t*)msg->data, (uint8_t*)address, size);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t) & (msg->msgHdr.msgType), msg->msgHdr.msgLength - sizeof(uint32_t), &msg->msgHdr.msgCrc);
    g_CooperCb.write((uint8_t*)msg, (sizeof(am_sbl_host_msg_data_t) + size));
}

//*****************************************************************************
//
// Send a "Reset" packet.
//
//*****************************************************************************
void send_reset(void)
{
    am_sbl_host_msg_reset_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_RESET;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_reset_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    g_CooperCb.write((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "FW Continue  packet.
//
//*****************************************************************************
static void send_fwContinue(void)
{
    am_sbl_host_msg_fw_continue_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_FW_CONTINUE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_fw_continue_t);
    //
    // Compute CRC
    //
    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    g_CooperCb.write((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Update the state machine based on the image to download
//
//*****************************************************************************
static bool am_devices_cooper_sbl_update_state_data(uint32_t ui32updateType)
{
    // Pointer to the data to be updated
    am_devices_cooper_sbl_update_data_t* p_sUpdateImageData = NULL;
    if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
    {
        p_sUpdateImageData = &g_sFwImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0 )
    {
        p_sUpdateImageData = &g_sInfo0PatchImage;
    }
    else if ( ui32updateType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1 )
    {
        p_sUpdateImageData = &g_sInfo1PatchImage;
    }
    else
    {
        return false;
    }
    // Check if the data is valid
    if (    (p_sUpdateImageData != NULL)                &&
            (p_sUpdateImageData->pImageAddress != 0 )    &&
            (p_sUpdateImageData->imageSize != 0 )       &&
            (p_sUpdateImageData->imageType == ui32updateType) )
    {
        // Load the INFO 0 Patch address
        gsSblUpdateState.pImageBuf          = p_sUpdateImageData->pImageAddress;
        // Image size
        gsSblUpdateState.ui32ImageSize      = p_sUpdateImageData->imageSize;
        // image type
        gsSblUpdateState.ui32ImageType      = p_sUpdateImageData->imageType;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = gsSblUpdateState.ui32ImageSize - AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = gsSblUpdateState.pImageBuf + AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = gsSblUpdateState.ui32DataSize / AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
        if (  (gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE) != 0 )
        {
            gsSblUpdateState.ui32TotalPackets++;
        }
        gsSblUpdateState.ui32PacketNumber = 0;
        return true;
    }

    return false;
}
//*****************************************************************************
//
//  Initialize the Image Update state machine
//
//*****************************************************************************
void am_devices_cooper_image_update_init(void)
{
    // Initialize state machine
    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT;
    // Load the image address
    gsSblUpdateState.pImageBuf          = NULL;
    // Image size
    gsSblUpdateState.ui32ImageSize      = 0;
    // image type
    gsSblUpdateState.ui32ImageType      = AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE;
    // Get the size of the data without headers
    gsSblUpdateState.ui32DataSize       = 0;
    // Get the start address of the data without headers
    gsSblUpdateState.pDataBuf           = NULL;
    // Calculate number of packets
    gsSblUpdateState.ui32TotalPackets   = 0;
    // Initialize Packet number in progress
    gsSblUpdateState.ui32PacketNumber   = 0;
    // Initialize the processing buffer
    gsSblUpdateState.pWorkBuf           = NULL;
    // Initialize the RX CRC check result
    gsSblUpdateState.bRxCrcCheckPass    = false;
    // Initialzie the length of receive packet
    gsSblUpdateState.ui32RxPacketLength = 0;
}

//*****************************************************************************
//
// @breif Update Image
// @return uint32_t
//
//*****************************************************************************
uint32_t am_devices_cooper_update_image(void)
{
    uint32_t     ui32dataPktSize = 0;
    uint32_t     ui32Size        = gsSblUpdateState.ui32RxPacketLength;
    uint32_t     ui32Ret         = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    am_sbl_host_msg_status_t*    psStatusMsg;
    am_sbl_host_msg_ack_nack_t*  psAckMsg;
    switch (gsSblUpdateState.ui32SblUpdateState)
    {
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_INIT:
            //
            // Send the "HELLO" message to connect to the interface.
            //
            send_hello();
            gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO;
            // Tell application that we are not done with SBL
            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO:
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( gsSblUpdateState.bRxCrcCheckPass == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_hello();
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Check the status
                psStatusMsg = (am_sbl_host_msg_status_t*) (gsSblUpdateState.pWorkBuf);
                gsSblUpdateState.ui32CooperSblStatus = psStatusMsg->bootStatus;
                // Get the Cooper FW version
                if ( psStatusMsg->versionNumber == AM_DEVICES_COOPER_SBL_DEFAULT_FW_VERSION )
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = 0;
                }
                else
                {
                    gsSblUpdateState.ui32CooperFWImageVersion = psStatusMsg->versionNumber;
                }
                LOG_INF("BLE Controller Info:");
                /*
                 * Before Cooper firmware version 1.19 (0x00000113), only the lower 16-bit of 32-bit Cooper firmware version
                 * word was used to identify Cooper firmware. It was limited to distinguish the difference of testing binaries.
                 * To restructure the Cooper firmware version to a.b.c.d from a.b may solve this problem.
                 * The higher 16-bit is used to identify the major and minor version of based release firmware.
                 * The lower 16-bit is used to identify the version for testing before next release.
                 * Originally the code only prints the lower 16-bit of FW version, need to print all the bytes
                 * based on new structure of firmware version now.
                 */
                if ((psStatusMsg->versionNumber & 0xFFFF0000) == 0)
                {
                    LOG_INF("\tFW Ver:      %d.%d", (psStatusMsg->versionNumber & 0xF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                else
                {
                    LOG_INF("\tFW Ver:      %d.%d.%d.%d", (psStatusMsg->versionNumber & 0xFF000000) >> 24, (psStatusMsg->versionNumber & 0xFF0000) >> 16,
                                                                        (psStatusMsg->versionNumber & 0xFF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                if (ui32Size == sizeof(am_sbl_host_msg_status_t))
                {
                    // Get the version rollback configuration
                    gsSblUpdateState.ui32CooperVerRollBackConfig = psStatusMsg->verRollBackStatus;
                    if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_EN )
                    {
                        LOG_DBG("Version RollBack Enabled ");
                    }
                    else if ( psStatusMsg->verRollBackStatus == AM_DEVICES_COOPER_SBL_STAT_VER_ROLL_BACK_DBL )
                    {
                        LOG_DBG("Version RollBack Disabled ");
                    }
                    else
                    {
                        LOG_DBG("Version RollBack Config invalid !!! ");
                    }
                    LOG_INF("\tChip ID0:    0x%x", psStatusMsg->copperChipIdWord0);
                    LOG_INF("\tChip ID1:    0x%x", psStatusMsg->copperChipIdWord1);

                    gsSblUpdateState.ui32copperChipIdWord0 = psStatusMsg->copperChipIdWord0;
                    gsSblUpdateState.ui32copperChipIdWord1 = psStatusMsg->copperChipIdWord1;

                }
                else
                {
                    gsSblUpdateState.ui32CooperVerRollBackConfig = 0x0;
                }
                LOG_DBG("bootStatus 0x%x", psStatusMsg->bootStatus);

                // check if the Boot Status is success
                if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_SUCCESS )
                {
                    // Check if we have some FW available
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        // Check if we have a newer FW version
                        if ( psStatusMsg->versionNumber < g_sFwImage.version )
                        {
                            // We have newer FW available, Letus upgrade
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                            if ((g_sFwImage.version & 0xFFFF0000) == 0)
                            {
                                LOG_INF("Received new BLE Controller FW version = %d.%d Going for upgrade", (g_sFwImage.version & 0xF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                            else
                            {
                                LOG_INF("Received new BLE Controller FW version = %d.%d.%d.%d Going for upgrade", (g_sFwImage.version & 0xFF000000) >> 24, (g_sFwImage.version & 0xFF0000) >> 16,
                                                                                                                            (g_sFwImage.version & 0xFF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                        }
                    }
                    // If we don't have any FW or any newer FW then continue with the current FW in Cooper
                    if ( ui32Ret != AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW )
                    {
                        // We don't have any other FW, so continue with one already there is Cooper device
                        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                        // Not done yet
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                        LOG_INF("No new image to upgrade");
                        // Send the command to continue to FW
                        send_fwContinue();
                        LOG_INF("BLE Controller FW Auth Passed, Continue with FW");
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_FW_UPDATE_REQ )
                {
                    LOG_INF("BLE Controller Requires FW");
                    if (  am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_FW;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO0_UPDATE_REQ )
                {
                    LOG_INF("BLE Controller Requires INFO 0");
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_0) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_0;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_COOPER_SBL_STAT_RESP_INFO1_UPDATE_REQ )
                {
                    LOG_INF("BLE Controller Requires INFO 1");
                    if ( am_devices_cooper_sbl_update_state_data(AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_INFO_1) == true )
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_INFO_1;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else
                {
                    LOG_ERR("BLE Controller Wrong Response");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            if (  (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_OK) || (ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_FAIL) ||
                  (gsSblUpdateState.ui32SblUpdateState == AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK) )
            {
                // Do nothing
            }
            else
            {
                // for the case ui32Ret == AM_DEVICES_COOPER_SBL_STATUS_UPDATE_IMAGE_FAIL,
                // it indicates Cooper has available FW/Info0/Info1 signature and requests update,
                // but Apollo4 does not have such image/patch at this moment, gsSblUpdateState.ui32ImageSize should be zero.
                // Need to send_update with invalid parameter to let Cooper reply NACK and clear signature

                // Update the state machine
                gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE;
                // Send the update message
                send_update(gsSblUpdateState.ui32ImageSize);
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_UPDATE:
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( gsSblUpdateState.bRxCrcCheckPass == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_update(gsSblUpdateState.ui32ImageSize);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Process the response
                if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_ACK) && (NULL != gsSblUpdateState.pImageBuf))
                {
                    // Save the status
                    gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                    // Change the state
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA;
                    // Send the Encrypted image header - first 64 bytes
                    send_data((uint32_t)gsSblUpdateState.pImageBuf,
                            AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    LOG_INF("BLE controller upgrade in progress, wait...");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
                else if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_NACK) && (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_INVALID_PARAM) )
                {
                    LOG_INF("Clear Cooper Signature, reset Cooper and talk with SBL again");
                    // Add some delay for Cooper SBL to clear signature
                    k_sleep(K_MSEC(1200));
                    am_devices_cooper_reset();
                    gsSblUpdateState.pImageBuf        = NULL;
                    gsSblUpdateState.ui32ImageSize    = 0;
                    gsSblUpdateState.ui32ImageType    = AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_NONE;
                    gsSblUpdateState.ui32DataSize     = 0;
                    gsSblUpdateState.pDataBuf         = NULL;
                    gsSblUpdateState.ui32TotalPackets = 0;
                    gsSblUpdateState.ui32PacketNumber = 0;

                    // Send the "HELLO" message to connect to the interface.
                    send_hello();
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_HELLO;
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
                else
                {
                    LOG_ERR("Update Failed !!!");
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_DATA:
            // Read the "ACK/NACK" response from the IOS.
            if ( gsSblUpdateState.bRxCrcCheckPass == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    if ( gsSblUpdateState.ui32PacketNumber == 0 )
                    {
                        // Send the Encrypted image header - first 64 bytes
                        send_data((uint32_t)gsSblUpdateState.pImageBuf,
                                  AM_DEVICES_COOPER_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    }
                    else
                    {
                        //Check if this is the last packet - Increase by one as we have already decremented after TX
                        if (  (gsSblUpdateState.ui32TotalPackets + 1) == 1 )
                        {
                            // Get the size of the leftover data
                            ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            if (ui32dataPktSize == 0)
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                        }
                        else
                        {
                            ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                        }
                        // Resend the same packet - Need to decrement the packet numbers as those are already incremented
                        send_data((uint32_t) gsSblUpdateState.pDataBuf + ( (gsSblUpdateState.ui32PacketNumber - 1) * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                  ui32dataPktSize, gsSblUpdateState.ui32PacketNumber);
                    }
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Save the status
                gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
                if (  (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_DATA ) || (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_UPDATE_STATUS) )
                {
                    if (  (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS) || (psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SEQ) )
                    {
                        if ( gsSblUpdateState.ui32TotalPackets > 0 )
                        {
                            //Check if this is the last packet
                            if ( gsSblUpdateState.ui32TotalPackets == 1 )
                            {
                                // Get the size of the left over data
                                ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                if (ui32dataPktSize == 0)
                                {
                                    ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                }
                            }
                            else
                            {
                                ui32dataPktSize = AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                            send_data((uint32_t) gsSblUpdateState.pDataBuf + (gsSblUpdateState.ui32PacketNumber * AM_DEVICES_COOPER_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                      ui32dataPktSize, gsSblUpdateState.ui32PacketNumber + 1);
                            gsSblUpdateState.ui32TotalPackets--;
                            // increment the packet number as we have already sent the header
                            gsSblUpdateState.ui32PacketNumber++;
                            ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                        }
                        else
                        {
                            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
                            {
                                // If FW is updated successfuly, then jump to BLE image
                                if ( gsSblUpdateState.ui32ImageType == AM_DEVICES_COOPER_SBL_UPDATE_IMAGE_TYPE_FW )
                                {
                                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK;
                                    gsSblUpdateState.ui32CooperFWImageVersion = g_sFwImage.version;
                                    // Not done yet
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                                    // Send the command to continue to FW
                                    send_fwContinue();
                                    // If INFO 0 or INFO 1 is updated successfully, the apply send reset
                                }
                                else
                                {
                                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
                                }
                            }
                            else
                            {
                                LOG_ERR("Update fails status = 0x%x", psAckMsg->status);
                                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                            }
                        }
                    }
                    else
                    {
                        LOG_ERR("Update fails status = 0x%x", psAckMsg->status);
                        // We have received NACK
                        ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                    }
                }
                else
                {
                    // Wrong Response type
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_COOPER_SBL_UPDATE_STATE_IMAGE_OK:
        {
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( gsSblUpdateState.bRxCrcCheckPass == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_COOPER_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_fwContinue();
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
            }
            // Get the response status
            psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
            // Save the status
            gsSblUpdateState.ui32CooperSblStatus = psAckMsg->status;
            if ( psAckMsg->status == AM_DEVICES_COOPER_SBL_ACK_RESP_SUCCESS )
            {
                // FW has gone to BLE, end the SBL driver state machine
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_OK;
            }
            else
            {
                ui32Ret = AM_DEVICES_COOPER_SBL_STATUS_FAIL;
            }
        }
        break;
        default:
            // Bad state, update the state machine
            break;
    }
    return ui32Ret;
}

//*****************************************************************************
//
//  Get cooper firmware image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_FwImage(am_devices_cooper_sbl_update_data_t *pFwImage )
{
    if (pFwImage != NULL)
    {
        memcpy(&g_sFwImage, pFwImage, sizeof(am_devices_cooper_sbl_update_data_t));
        // Get version from the firmware image
        g_sFwImage.version = (pFwImage->pImageAddress[27] << 24) | (pFwImage->pImageAddress[26] << 16) | (pFwImage->pImageAddress[25] << 8) | (pFwImage->pImageAddress[24]);
    }

    return (pFwImage != NULL);
}

//*****************************************************************************
//
//  Get cooper info1 image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_info1_patch(am_devices_cooper_sbl_update_data_t *pInfo1Image)
{
    if (pInfo1Image != NULL)
    {
        memcpy(&g_sInfo1PatchImage, pInfo1Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo1Image != NULL);
}

//*****************************************************************************
//
//  Get cooper info0 image from local binary
//
//*****************************************************************************
bool am_devices_cooper_get_info0_patch(am_devices_cooper_sbl_update_data_t *pInfo0Image)
{
    if (pInfo0Image != NULL)
    {
        memcpy(&g_sInfo0PatchImage, pInfo0Image, sizeof(am_devices_cooper_sbl_update_data_t));
    }

    return (pInfo0Image != NULL);
}


//*****************************************************************************
//
//  Reset the BLE controller and check if there's request to update
//
//*****************************************************************************
uint32_t am_devices_cooper_reset_with_sbl_check(void)
{
    uint32_t u32SblStatus = 0;
    am_devices_cooper_reset();
    am_devices_cooper_set_initialize_state(AM_DEVICES_COOPER_STATE_STARTUP);
    am_devices_cooper_image_update_init();
    u32SblStatus = AM_DEVICES_COOPER_SBL_STATUS_INIT;
    u32SblStatus = am_devices_cooper_update_image();
    while ((u32SblStatus != AM_DEVICES_COOPER_SBL_STATUS_OK) &&
        (u32SblStatus != AM_DEVICES_COOPER_SBL_STATUS_FAIL))
    {
        if (k_sem_take(&sem_sbl_send, AM_DEVICES_COOPER_SBL_SEND_WAIT_TIMEOUT) == 0)
        {
            u32SblStatus = am_devices_cooper_update_image();
        }
        else
        {
            return AM_DEVICES_COOPER_STATUS_TIMEOUT;
        }

    }

    if (u32SblStatus == AM_DEVICES_COOPER_SBL_STATUS_OK)
    {
        // need to wait a bit to jump from SBL to Cooper application firmware
        k_sleep(K_MSEC(10));
        LOG_INF("Update Done");
        return AM_DEVICES_COOPER_STATUS_SUCCESS;
    }
    else
    {
        LOG_ERR("BLE Controller SBL Error 0x%x", u32SblStatus);
        return u32SblStatus;
    }
}

//*****************************************************************************
//
//  Get the Cooper initialization state
//
//*****************************************************************************
uint32_t am_devices_cooper_get_initialize_state(void)
{
    return g_CooperInitState;
}

//*****************************************************************************
//
//  Set the Cooper initialization state
//
//*****************************************************************************
void am_devices_cooper_set_initialize_state(uint32_t state)
{
    g_CooperInitState = state;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
