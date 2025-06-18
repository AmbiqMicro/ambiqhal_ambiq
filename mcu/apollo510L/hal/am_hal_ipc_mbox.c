//*****************************************************************************
//
//! @file am_hal_ipc_mbox.c
//!
//! @brief Functions for Interprocessor Communication Messaging Mailboxes
//!
//! @addtogroup ipc_mbox IPC_MBOX - Interprocessor Communication Messaging Mailboxes
//! @ingroup apollo510L_hal
//! @{

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
// This is part of revision release_sdk5_2_a_0-438c93f352 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// MACRO DEFINITIONS
//
//*****************************************************************************
//
// CM4 will send IPCINIT interrupt to CM55 when the IPC mailbox is ready. CM55
// can only access IPC mailbox registers after receiving the IPCINIT interrupt,
// otherwise it will trigger a hardfault. So CM55 should always check if the IPC
// mailbox is initialized before any IPC mailbox register operation.
//
#if (CONFIG_TEST)
// The IPC mailbox may be used in testsuites for signalling and the IPCINIT interrupt
// is ignored. Do not check the state in such cases.
#define CHECK_MBOX_INIT_STATE()
#else
#define CHECK_MBOX_INIT_STATE()                                                       \
    do                                                                                \
    {                                                                                 \
        if (am_hal_ipc_mbox_init_state_get() == AM_HAL_IPC_MBOX_INIT_STATE_NOT_READY) \
        {                                                                             \
            return AM_HAL_STATUS_INVALID_OPERATION;                                   \
        }                                                                             \
    } while (0)
#endif
//*****************************************************************************
//
// GLOBAL FUNCTION DEFINITIONS
//
//*****************************************************************************

//*****************************************************************************
//
// VARIABLE DEFINITIONS
//
//*****************************************************************************
static am_hal_ipc_mbox_init_state_e g_IpcMboxInitState = AM_HAL_IPC_MBOX_INIT_STATE_NOT_READY;
static am_hal_ipc_mbox_handler_t ipcMbox_pfnHandlers[AM_HAL_IPC_MBOX_SIGNAL_MSG_END
                                                   - AM_HAL_IPC_MBOX_SIGNAL_MSG_START];
static void *ipcMbox_pvIrqArgs[AM_HAL_IPC_MBOX_SIGNAL_MSG_END - AM_HAL_IPC_MBOX_SIGNAL_MSG_START];

//*****************************************************************************
//
// Set IPC mailbox DSP to MCU interrupt threshold.
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_threshold_set(uint32_t threshold)
{
    CHECK_MBOX_INIT_STATE();

    if ((threshold > IPC_MBOX_MAX_THRESHOLD) || (threshold < IPC_MBOX_MIN_THRESHOLD))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    CM55IPC->D2MIT_b.D2MTHRESHOLD = (threshold & CM55IPC_D2MIT_D2MTHRESHOLD_Msk);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get IPC mailbox M2D or D2M interrupt threshold.
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_threshold_get(am_hal_ipc_mbox_channel_direction_e dir,
                                       uint32_t *pui32Threshold)
{
    CHECK_MBOX_INIT_STATE();

    if (pui32Threshold == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (dir)
    {
        case AM_HAL_IPC_MBOX_CHANNEL_DIR_IN:
            *pui32Threshold = CM55IPC->D2MIT_b.D2MTHRESHOLD;
        break;

        case AM_HAL_IPC_MBOX_CHANNEL_DIR_OUT:
            *pui32Threshold = CM55IPC->M2DIT_b.M2DTHRESHOLD;
        break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Configure IPC mailbox DSP to MCU interrupt.
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_interrupt_configure(am_hal_ipc_mbox_int_ctrl_e eControl,
                                             am_hal_ipc_mbox_int_channel_e eChannel)
{
    CHECK_MBOX_INIT_STATE();

    if ((eControl >= AM_HAL_IPC_MBOX_INT_CTRL_NUM) || (eChannel >= AM_HAL_IPC_MBOX_INT_CHANNEL_NUM))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (eControl)
    {
        case AM_HAL_IPC_MBOX_INT_CTRL_ENABLE:
            switch (eChannel)
            {
                case AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD:
                    CM55IPC->D2MIE_b.D2MTHRESHOLDIRQEN = 1;
                    break;

                case AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR:
                    CM55IPC->D2MIE_b.D2MERRORIRQEN = 1;
                    break;

                default:
                    break;
            }
            break;

        case AM_HAL_IPC_MBOX_INT_CTRL_DISABLE:
            switch (eChannel)
            {
                case AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD:
                    CM55IPC->D2MIE_b.D2MTHRESHOLDIRQEN = 0;
                    break;

                case AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR:
                    CM55IPC->D2MIE_b.D2MERRORIRQEN = 0;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get IPC mailbox interrupt status
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_interrupt_status_get(am_hal_ipc_mbox_channel_direction_e dir,
                                              uint32_t *pui32IntStatus)
{
    CHECK_MBOX_INIT_STATE();

    if (pui32IntStatus == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (dir)
    {
        case AM_HAL_IPC_MBOX_CHANNEL_DIR_IN:
            *pui32IntStatus = CM55IPC->D2MIS;
        break;

        case AM_HAL_IPC_MBOX_CHANNEL_DIR_OUT:
            *pui32IntStatus = CM55IPC->M2DIS;
        break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get IPC mailbox interrupt enable state
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_interrupt_enable_get(am_hal_ipc_mbox_channel_direction_e dir,
                                              uint32_t *pui32IntEnable)
{
    CHECK_MBOX_INIT_STATE();

    if (pui32IntEnable == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (dir)
    {
        case AM_HAL_IPC_MBOX_CHANNEL_DIR_IN:
            *pui32IntEnable = CM55IPC->D2MIE;
        break;

        case AM_HAL_IPC_MBOX_CHANNEL_DIR_OUT:
            *pui32IntEnable = CM55IPC->M2DIE;
        break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Read certain length mailbox data from the FIFO
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_data_read(uint32_t *data, uint32_t len)
{
    CHECK_MBOX_INIT_STATE();

    if ((data == NULL) || (len == 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((CM55IPC->STATUS_b.D2MEMPTY) || (len > CM55IPC->STATUS_b.D2MPEND))
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        data[i] = CM55IPC->D2MDATA;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Flush mailbox data in the read FIFO
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_data_flush(uint32_t *data)
{
    uint32_t buf;

    CHECK_MBOX_INIT_STATE();

    for (uint32_t i = 0; i < IPC_MBOX_FIFO_MAX_DEPTH; i++)
    {
        if (CM55IPC->STATUS_b.D2MEMPTY)
        {
            break;
        }

        buf = CM55IPC->D2MDATA;

        // Save data if needed
        if (data != NULL)
        {
            data[i] = buf;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Write certain length mailbox data to the FIFO
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_data_write(uint32_t *data, uint32_t len)
{
    CHECK_MBOX_INIT_STATE();

    if ((data == NULL) || (len == 0))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ((CM55IPC->STATUS_b.M2DFULL) || (len > (IPC_MBOX_FIFO_MAX_DEPTH - CM55IPC->STATUS_b.M2DPEND)))
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        CM55IPC->M2DDATA = data[i];
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get IPC mailbox's status
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_status_get(uint32_t *status)
{
    CHECK_MBOX_INIT_STATE();

    if (status == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *status = CM55IPC->STATUS;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get IPC mailbox error
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_error_status_get(uint32_t *pui32IntMsk)
{
    CHECK_MBOX_INIT_STATE();

    if (pui32IntMsk == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    *pui32IntMsk = CM55IPC->D2MERROR;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear IPC mailbox error
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_error_clear(uint32_t ui32IntMsk)
{
    CHECK_MBOX_INIT_STATE();

    // write "1" to clear the error bit
    CM55IPC->D2MERROR = ui32IntMsk & (CM55IPC_D2MERROR_D2MEMPTYERROR_Msk |
                                      CM55IPC_D2MERROR_D2MFULLERROR_Msk |
                                      CM55IPC_D2MERROR_M2DFULLERROR_Msk |
                                      CM55IPC_D2MERROR_IPCINIT_Msk);
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Clear IPC mailbox interrupt status
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_interrupt_clear(am_hal_ipc_mbox_int_channel_e eChannel)
{
    CHECK_MBOX_INIT_STATE();

    if (eChannel >= AM_HAL_IPC_MBOX_INT_CHANNEL_NUM)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    switch (eChannel)
    {
        case AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD:
            if (CM55IPC->D2MIS_b.D2MTHRESHOLDIRQ)
            {
                // Threshold interrupt, write "1" to clear the bit
                CM55IPC->D2MIS_b.D2MTHRESHOLDIRQ = 1;
            }
            break;

        case AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR:
            if (CM55IPC->D2MIS_b.D2MERRORIRQ)
            {
                // Error interrupt, write "1" to clear the bit
                CM55IPC->D2MIS_b.D2MERRORIRQ = 1;
            }
            break;

        default:
            break;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set the IPC mailbox initialization state
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_init_state_set(am_hal_ipc_mbox_init_state_e eState)
{
    if (eState >= AM_HAL_IPC_MBOX_INIT_STATE_NUM)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    g_IpcMboxInitState = eState;
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Get the IPC mailbox initialization state
//
//*****************************************************************************
am_hal_ipc_mbox_init_state_e am_hal_ipc_mbox_init_state_get(void)
{
    return g_IpcMboxInitState;
}

//*****************************************************************************
//
// IPC mailbox message received handler.
// Note: This is a weak function to allow customization of event handling
//
//*****************************************************************************
__attribute__((weak))
void am_hal_ipc_mbox_msg_handler(void)
{
    uint32_t ui32Data = 0;

    am_hal_ipc_mbox_data_read(&ui32Data, 1);
    am_hal_ipc_mbox_interrupt_clear(AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD);
    am_hal_ipc_mbox_service(ui32Data);
}

//*****************************************************************************
//
// IPC mailbox data ISR
//
//*****************************************************************************
void am_ipc_pend_msg_isr(void)
{
    am_hal_ipc_mbox_msg_handler();
}

//*****************************************************************************
//
// IPC mailbox error ISR
//
//*****************************************************************************
void am_ipc_err_isr(void)
{
    uint32_t ui32Status;

    //
    // The mailbox from remote (CM4) will raise an IPC initialization interrupt
    // when it boots up and mailbox is ready.
    //
    if (AM_HAL_IPC_MBOX_ERROR_IPCINIT)
    {
        am_hal_ipc_mbox_init_state_set(AM_HAL_IPC_MBOX_INIT_STATE_IPCINIT_RECEIVED);
        am_hal_ipc_mbox_init();
        return;
    }

    am_hal_ipc_mbox_error_status_get(&ui32Status);
    am_hal_ipc_mbox_error_clear(ui32Status);
    am_hal_ipc_mbox_interrupt_clear(AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR);
}

//*****************************************************************************
//
// Initialize the IPC mailbox
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_init(void)
{
    uint32_t ui32Status;

    // Clear the error
    if (am_hal_ipc_mbox_error_clear(0xFFFFFFFF) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    // Clear the interrupt status
    if ((am_hal_ipc_mbox_interrupt_clear(AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD) != AM_HAL_STATUS_SUCCESS)
        || (am_hal_ipc_mbox_interrupt_clear(AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR) != AM_HAL_STATUS_SUCCESS))
    {
        return AM_HAL_STATUS_FAIL;
    }

    // Init FIFO threshold
    ui32Status = am_hal_ipc_mbox_threshold_set(IPC_MBOX_DEFAULT_THRESHOLD);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    // Enable the interrupt
    if ((am_hal_ipc_mbox_interrupt_configure(AM_HAL_IPC_MBOX_INT_CTRL_ENABLE, AM_HAL_IPC_MBOX_INT_CHANNEL_THRESHOLD) != AM_HAL_STATUS_SUCCESS)
        || (am_hal_ipc_mbox_interrupt_configure(AM_HAL_IPC_MBOX_INT_CTRL_ENABLE, AM_HAL_IPC_MBOX_INT_CHANNEL_ERROR) != AM_HAL_STATUS_SUCCESS))
    {
        return AM_HAL_STATUS_FAIL;
    }

    // The IPC mailbox has been initialized successfully
    am_hal_ipc_mbox_init_state_set(AM_HAL_IPC_MBOX_INIT_STATE_INITIALIZED);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Register the IPC mailbox interrupt handler
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_handler_register(uint32_t ui32Signal,
                                          am_hal_ipc_mbox_handler_t pfnHandler,
                                          void *pArg)
{
    if ((ui32Signal < AM_HAL_IPC_MBOX_SIGNAL_MSG_START)
        || (ui32Signal >= AM_HAL_IPC_MBOX_SIGNAL_MSG_END))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ipcMbox_pfnHandlers[ui32Signal - AM_HAL_IPC_MBOX_SIGNAL_MSG_START] = pfnHandler;
    ipcMbox_pvIrqArgs[ui32Signal - AM_HAL_IPC_MBOX_SIGNAL_MSG_START] = pArg;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Execute the register IPC mailbox interrupt handler
//
//*****************************************************************************
uint32_t am_hal_ipc_mbox_service(uint32_t ui32Signal)
{
    am_hal_ipc_mbox_handler_t pfnHandler;
    void *pArg;

    if ((ui32Signal < AM_HAL_IPC_MBOX_SIGNAL_MSG_START)
        || (ui32Signal >= AM_HAL_IPC_MBOX_SIGNAL_MSG_END))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    pfnHandler = ipcMbox_pfnHandlers[ui32Signal - AM_HAL_IPC_MBOX_SIGNAL_MSG_START];
    pArg = ipcMbox_pvIrqArgs[ui32Signal - AM_HAL_IPC_MBOX_SIGNAL_MSG_START];
    if (pfnHandler)
    {
        //
        // Found the handler, execute it.
        //
        pfnHandler(pArg);
    }
    else
    {
        //
        // No handler was registered, return error.
        //
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
