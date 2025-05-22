//*****************************************************************************
//
//! @file am_hal_uart_stream.c
//!
//! @brief UART streaming mode Hardware abstraction
//!
//! @addtogroup uart_stream UART Stream Functionality
//! @ingroup apollo510_hal
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

#include <string.h>
#include "am_mcu_apollo.h"



#define AM_HAL_MAGIC_UART_STREAM               0xEA9E07

#define AM_HAL_UART_CHK_HANDLE(h)                                             \
((h) &&                                                                   \
((am_hal_handle_prefix_t *)(h))->s.bInit &&                              \
(((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_UART_STREAM))

//*****************************************************************************
//
//! UART FIFO Size
//
//*****************************************************************************
#define AM_HAL_UART_FIFO_MAX 32


//
//! uart register definitions
//! used when saving uart
//
typedef struct
{
    uint32_t regILPR;
    uint32_t regIBRD;
    uint32_t regFBRD;
    uint32_t regLCRH;
    uint32_t regCR;
    uint32_t regIFLS;
    uint32_t regIER;
    uint32_t regDMACFG;
    bool bValid;
}
am_hal_uart_register_state_t;

//*****************************************************************************
//
//! Structure for holding UART HAL state information.
//! Used internally, allocated in application
//! No user-modifiable variables here
//
//*****************************************************************************
typedef struct
{
    //
    //! For internal verification purposes
    //
    am_hal_handle_prefix_t prefix;

    //
    //! Saved register state for power-up/power-down
    //
    am_hal_uart_register_state_t sRegState;

    //
    //! UART module number.
    //
    uint32_t ui32Module;

    //
    //! Pointer to the dma buffer descriptor queue
    //! This is allocated by the calling function in application space
    //! and is attached to this struct via a call to
    //! am_hal_uart_stream_dmaQueueInit
    //! @note Since it is allocated and passed in, this usually shouldn't be on the stack
    //
    am_hal_uart_dma_config_t sDmaQueue;

    //
    //! Contains streaming state and data for tx
    //
    am_hal_uart_stream_tx_params_t            sTx_params;
    //
    //! Contains streaming state and data for rx
    //
    am_hal_uart_stream_rx_params_t            sRx_params;

    //
    //! In ISR service this keeps track of requests for a tx callback
    //! At the end of the ISR service, if this is non-zero,
    //! and a callback function is defined. The callback function is called
    //! this is not a setting
    //
    am_hal_uart_stream_tx_complete_options_t eTxCompleteNotificationEnabled;

    //
    //! Contains computed config data (baud rate, clock)
    //
    am_hal_uart_config_out_t sComputedConfig;

    am_hal_uart_streaming_dma_mode_e eStreamingDmaMode;

    //
    //! Busy wait, use as a flag in the ISR service, this is not a setting
    //
    bool                bBusyWaitEnabled;


    bool                psActiveState;
}
am_hal_uart_stream_state_t;

static am_hal_uart_stream_state_t g_am_hal_uart_stream_state[AM_REG_UART_NUM_MODULES];

static inline am_hal_uart_stream_status_t rxSingleBuffDMAHandler( volatile UART0_Type *pUart,
                                                                  am_hal_uart_stream_state_t *psState);
static inline am_hal_uart_stream_status_t rxDoubleBuffDMAHandler( volatile UART0_Type *pUart,
                                                                  am_hal_uart_stream_state_t *psState);
static inline am_hal_uart_stream_status_t txCircularBuffDMAHandler( volatile UART0_Type *pUart,
                                                                    am_hal_uart_stream_state_t *psState);
static inline am_hal_uart_stream_status_t txDoubleBuffDMAHandler( volatile UART0_Type *pUart,
                                                                  am_hal_uart_stream_state_t *psState);
static inline void uart_stream_unload_rxFifo(am_hal_stream_queue_t *psRxQueue,
                                             volatile UART0_Type *pUart);

static void am_hal_uart_stream_queue_init( am_hal_stream_queue_t *psQueue,
                                           uint8_t *pui8Data,
                                           uint32_t ui32BufferSize);



//*****************************************************************************
//
//! @brief  set uart baud rate
//!
//! @param ui32Module
//! @param ui32DesiredBaudrate
//! @param pui32ActualBaud
//!
//! @return
//
//*****************************************************************************
static uint32_t config_baudrate(uint32_t ui32Module,
                                uint32_t ui32DesiredBaudrate,
                                uint32_t *pui32ActualBaud);

//*****************************************************************************
//
//! @brief Configures a UART module with specified settings.
//!
//! This initializes and configures a UART module with the provided
//! settings such as clock source, baud rate, flow control, parity, data
//! format, and FIFO levels. It also handles necessary clock requests, power
//! settings.
//!
//! @param ui32UartModule The UART module number to configure. Must be a valid
//!                       module index.
//! @param psCfgOut Pointer to an output configuration structure
//!                 (am_hal_uart_config_out_t) that will store the computed
//!                 config details, including clock source and actual baud rate.
//! @param psConfig Pointer to an preconfigured input configuration structure
//!
//! @return AM_HAL_STATUS_SUCCESS if the configuration is successful.
//!         AM_HAL_STATUS_INVALID_ARG if an invalid argument is provided.
//!         AM_HAL_STATUS_OUT_OF_RANGE if invalid baud rate or configuration is specified.
//!         Other status codes as appropriate for internal errors or failures.
//
//*****************************************************************************

static uint32_t am_hal_uart_cmn_configure(uint32_t ui32UartModule,
                                          am_hal_uart_config_out_t *psCfgOut,
                                          const am_hal_uart_config_t *psConfig);

//*****************************************************************************
//
// UART ISR used for streaming API
//
//*****************************************************************************
uint32_t
am_hal_uart_stream_interrupt_clr_set( void *pUartHandle,
                                      uint32_t ui32Clear,
                                      uint32_t ui32Set)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;
    uint32_t ui32Module = psState->ui32Module;

    UARTn(ui32Module)->IER = (UARTn(ui32Module)->IER & ~ui32Clear) | ui32Set;

    return AM_HAL_STATUS_SUCCESS;

}

//*****************************************************************************
//
//! @brief This handles queue init
//!
//! @param psQueue          pointer to queue
//! @param pui8Data         pointer to dataBuffer
//! @param ui32BufferSize   size of dataBuffer
//!
//
//*****************************************************************************
static void
am_hal_uart_stream_queue_init( am_hal_stream_queue_t *psQueue,
                               uint8_t *pui8Data,
                               uint32_t ui32BufferSize)
{
    psQueue->ui32WriteIndex = 0;
    psQueue->ui32ReadIndex = 0;
    psQueue->ui32Length = 0;
    psQueue->ui32Capacity = ui32BufferSize;
    psQueue->pui8Data = pui8Data;
}

//*****************************************************************************
//
//! @brief This handles DMA complete interrupt for single buffered RX DMA
//!
//! @param pUart  pointer to uart register struct
//! @param psState pointer uart state struct
//!
//! @return RX dma status
//
//*****************************************************************************
static inline am_hal_uart_stream_status_t
rxSingleBuffDMAHandler( volatile UART0_Type *pUart,
                        am_hal_uart_stream_state_t *psState )
{
    psState->sDmaQueue.bDMAActive      = false;
    pUart->DCR              = 0;  // stop dma
    pUart->RSR              &= ~(UART0_RSR_DMAERR_Msk | UART0_RSR_DMACPL_Msk);
    pUart->IER              &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Pos);  // disable dma complete and error interrupts
    pUart->IEC              = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk; // clear interrupts

    am_hal_uart_stream_rx_params_t *pRxParams = &psState->sRx_params;

    am_hal_uart_stream_dma_rx_descriptor_entry_t *psActive = &pRxParams->tDescriptor[0];
    psActive->bLastComplete = true;
    psActive->ui32BuffHasData = 1;

    am_hal_uart_stream_status_t eRetVal = AM_HAL_UART_RX_DMA_COMPLETE;

    if ( pRxParams->pfRxCompleteCallback)
    {
        am_hal_uart_stream_callback_data_t tCBdata;
        tCBdata.pui8Buffer          = (uint8_t *) psActive->ui32BuffBaseAddr;
        tCBdata.ui32BufferSize      = psActive->ui32NumBytes;
        tCBdata.eStatus             = eRetVal ;
        tCBdata.hasData           = &psActive->ui32BuffHasData;
        pRxParams->pfRxCompleteCallback( &tCBdata );
    }
    return eRetVal;
}

//*****************************************************************************
//
//! @brief This handles DMA complete interrupt for double buffered RX DMA
//! @details  The goal is to get DMA started in the alternate buffer ASAP to keep from losing data
//! The application has the goal of using this data before the alternate buffer fills
//!
//! @param pUart  pointer to uart register struct
//! @param psState pointer to uart state struct
//!
//! @return rx dma status
//
//*****************************************************************************
static inline am_hal_uart_stream_status_t
rxDoubleBuffDMAHandler( volatile UART0_Type *pUart,
                        am_hal_uart_stream_state_t *psState )
{
    pUart->DCR              = 0;  // stop dma
    pUart->RSR              &= ~(UART0_RSR_DMAERR_Msk | UART0_RSR_DMACPL_Msk);
    pUart->IEC              = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk ; // clear DMA interrupts

    am_hal_uart_stream_rx_params_t *pRxParams = &psState->sRx_params;

    am_hal_uart_stream_dma_rx_descriptor_entry_t *psActive = pRxParams->activeDmaRxDesc;
    psActive->bLastComplete = true;
    am_hal_uart_stream_dma_rx_descriptor_entry_t *psNext   = psActive->nextDesc;
    psNext->bLastComplete   = false;

    pRxParams->activeDmaRxDesc = psNext;

    pUart->TARGADDR         = psNext->ui32BuffBaseAddr;  // set new address
    uint32_t  ui32TxCount   = psNext->ui32NumBytes;      //
    if (ui32TxCount > AM_HAL_MAX_UART_DMA_SIZE)
    {
        ui32TxCount         = AM_HAL_MAX_UART_DMA_SIZE;
    }
    pUart->COUNT            = ui32TxCount;

    am_hal_uart_stream_status_t eRetVal;

    if (psActive->ui32BuffHasData)
    {
        pUart->IER          &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Pos);  // disable dma complete and error interrupts
        psState->sDmaQueue.bDMAActive  = false;
        eRetVal             = AM_HAL_UART_RX_DMA_OVERFLOW;
    }
    else
    {
        pUart->DCR          = UART0_DCR_DMAPRI_Msk | UART0_DCR_RXDMAE_Msk;  // restart rx DMA
        psState->sDmaQueue.bDMAActive  = true;
        psActive->ui32BuffHasData = 1 ;
        eRetVal             = (am_hal_uart_stream_status_t) (AM_HAL_UART_RX_DMA_BUSY | AM_HAL_UART_RX_DMA_COMPLETE);
    }

    if (pRxParams->pfRxCompleteCallback)
    {
        am_hal_uart_stream_callback_data_t tCBdata;
        tCBdata.pui8Buffer          = (uint8_t *) psActive->ui32BuffBaseAddr;
        tCBdata.ui32BufferSize      = psActive->ui32NumBytes;
        tCBdata.eStatus             = eRetVal;
        tCBdata.hasData           = &psActive->ui32BuffHasData;

        pRxParams->pfRxCompleteCallback( &tCBdata );
    }

    return (am_hal_uart_stream_status_t) eRetVal;
}

//*****************************************************************************
//
//! @brief This handles DMA complete interrupt for Circular buffered TX DMA
//! @details  This is called with the DMA complete interrupt in Circular buffered
//! tx mode. This will first check the current DMA descriptor if all of it's data
//! has been transmitted, and then will check the next descriptor for any data to
//! start transmitting. If there is any more data to TX, it will start a new DMA
//! with that data. If no data is left, it will enable the tx complete interrupt
//! so the user will be notified when the tx is complete.
//!
//! @param pUart  pointer to uart register struct(hardware registers)
//! @param psState pointer to uart state struct
//!
//! @return tx dma status
//
//*****************************************************************************
static inline am_hal_uart_stream_status_t
txCircularBuffDMAHandler( volatile UART0_Type *pUart,
                          am_hal_uart_stream_state_t *psState )
{
    am_hal_uart_stream_tx_params_t *pTxParams = &psState->sTx_params;
    am_hal_uart_stream_dma_tx_descriptor_entry_t *activeDesc = pTxParams->activeDmaTxDesc;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (activeDesc == NULL)
    {
        // This is an unexpected internal error, disable this DMA transfer.
        // @note, if a dma complete interrupt occurs, there should be an active descriptor.
        pUart->DCR      = 0;  // stop dma
        pUart->IER      &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk | UART0_IER_TXCMPMIM_Msk);
        pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

        return AM_HAL_UART_STREAM_STATUS_INTERNAL_DMA_ERROR;
    }

#endif
    if (activeDesc->ui32NumBytes == 0)
    {
        //
        // All the data in this descriptor has been transmitted, switch to the next descriptor.
        // The next descriptor may also contain no data: That will be taken care of below.
        //
        activeDesc->ui32NumDmaQueued = 0;
        activeDesc->ui32StartAddress = activeDesc->ui32BuffBaseAddr; // reset start address
        activeDesc               = activeDesc->nextDesc;
        //
        // }else(activeDesc->ui32NumBytes != 0){
        // if activeDesc->ui32NumBytes is non-zero:
        // this descriptor still contains data to send (probably due to dma size limit on the previous DMA)
        //
    }
    //
    // This is a circular buffer scheme
    // check if there is another descriptor/buffer to start transmitting
    //
    am_hal_uart_stream_dma_tx_descriptor_entry_t *currDesc = activeDesc;

    if (currDesc->ui32NumBytes)
    {
        //
        // The next buffer contains data, so start sending that now.
        // Make it the new current buffer, and set that buffer active.
        //
        pTxParams->activeDmaTxDesc = currDesc;

        pUart->DCR              = 0;  // stop dma
        pUart->TARGADDR         = currDesc->ui32StartAddress;  // set new address
        uint32_t  ui32TxCount   = currDesc->ui32NumBytes;      //
        if (ui32TxCount > AM_HAL_MAX_UART_DMA_SIZE)
        {
            ui32TxCount         = AM_HAL_MAX_UART_DMA_SIZE;
        }

        pUart->COUNT                = ui32TxCount;
        currDesc->ui32NumDmaQueued  = ui32TxCount;
        currDesc->ui32NumBytes      -= ui32TxCount;
        currDesc->ui32StartAddress  += ui32TxCount;

        __DMB();
        //
        // disable tx complete interrupts, enable dma interrupts
        //
        pUart->IER      = (pUart->IER & ~(UART0_IER_TXIM_Msk | UART0_IER_TXCMPMIM_Msk)) |
                          (UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk);
        //
        // clear and setup interrupts
        //
        pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                          UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

        pUart->DCR      = UART0_DCR_DMAPRI_Msk | UART0_DCR_TXDMAE_Msk;  // enable dma

        if (pTxParams->nextDmaWrtDesc == currDesc)
        {
            //
            // The current (active DMA) descr is now the same as the append descriptor,
            // That is invalid (but expected).
            // Now need to advance and init the append descriptor.
            //
            am_hal_uart_stream_dma_tx_descriptor_entry_t *pNextWrite = currDesc->nextDesc;  // next append descriptor
            //
            // Init the new append descriptor
            // Take the write address (start + number) for the existing current descriptor
            // and use that in the next append descriptor.
            //
            uint32_t nextStartAddr      = currDesc->ui32StartAddress;
            if (nextStartAddr >= pTxParams->queueEndAddr )
            {
                nextStartAddr           = pTxParams->queueStartAddr;
            }
            pNextWrite->ui32StartAddress     = nextStartAddr;
            pNextWrite->ui32NumBytes         = 0; // should already be zero
            pTxParams->nextDmaWrtDesc      = pNextWrite;

        }
        return AM_HAL_UART_STREAM_STATUS_TX_DMA_BUSY;
    }

    //
    // no data left
    //
    if (pTxParams->activeDmaTxDesc)
    {
        pTxParams->activeDmaTxDesc->ui32NumDmaQueued = 0;
        pTxParams->activeDmaTxDesc = NULL;
    }

    pUart->DCR              = 0;  // stop dma

    //
    // clear and disable tx and DMA interrupts
    // non DMA code, don't clear tx complete, it may have completed before this code has been run
    //
    pUart->IER = (pUart->IER & ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk)) |
                 (UART0_IER_TXCMPMIM_Msk);
    pUart->IEC = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_TXIC_Msk;

    return AM_HAL_UART_STREAM_STATUS_TX_DMA_COMPLETE;
}

//*****************************************************************************
//
//! @brief This handles DMA complete interrupt for double buffered TX DMA
//! @details  This is called with the DMA complete interrupt in Circular buffered
//! tx mode. This will first check the current DMA descriptor if all of its data
//! has been transmitted, and then will check the alternate buffer descriptor
//! for any data to start transmitting.
//! If there is any more data to TX, it will start a new DMA
//! with that data. If no data is left, it will enable the tx complete interrupt
//! so the user will be notified when the tx is complete.
//!
//! @param pUart  pointer to uart register struct
//! @param psState pointer to uart state struct
//!
//! @return tx dma status
//
//*****************************************************************************
static inline am_hal_uart_stream_status_t
txDoubleBuffDMAHandler( volatile UART0_Type *pUart,
                        am_hal_uart_stream_state_t *psState )
{
    am_hal_uart_stream_tx_params_t *pTxParams = &psState->sTx_params;
    am_hal_uart_stream_dma_tx_descriptor_entry_t *activeDesc = pTxParams->activeDmaTxDesc;
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (activeDesc == NULL)
    {
        // This is an unexpected internal error, disable this DMA transfer.
        // @note, if a dma complete interrupt occurs, there should be an active descriptor.
        pUart->DCR      = 0;  // stop dma
        pUart->IER      &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk | UART0_IER_TXCMPMIM_Msk);
        pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

        return AM_HAL_UART_STREAM_STATUS_INTERNAL_DMA_ERROR;
    }
#endif
    if (activeDesc->ui32NumBytes == 0)
    {
        //
        // All the data in this descriptor has been transmitted, switch to the next descriptor.
        // The next descriptor may also contain no data: That will be taken care of below.
        //
        activeDesc->ui32NumDmaQueued = 0;
        activeDesc->ui32StartAddress = activeDesc->ui32BuffBaseAddr; // reset start address
        activeDesc               = activeDesc->nextDesc;
        //
        // }else(activeDesc->ui32NumBytes != 0){
        // if activeDesc->ui32NumBytes is non-zero:
        // this descriptor still contains data to send (probably due to dma size limit on the previous DMA)
        //
    }
    //
    // This is a double buffer scheme
    //
    if (activeDesc->ui32NumBytes)
    {
        //
        // The buffer contains data, so start sending that now.
        //
        pTxParams->activeDmaTxDesc = activeDesc;
        pUart->DCR              = 0;  // stop dma
        pUart->TARGADDR         = activeDesc->ui32StartAddress;  // set new address

        uint32_t count = activeDesc->ui32NumBytes;      // set new dma size
        if ( count > AM_HAL_MAX_UART_DMA_SIZE)
        {
            count = AM_HAL_MAX_UART_DMA_SIZE;
            //
            // do not advance to next dma descriptor, there is data left
            //
        }

        pUart->COUNT                 = count;      // set new dma size
        activeDesc->ui32NumDmaQueued = count;
        activeDesc->ui32NumBytes     -= count;
        activeDesc->ui32StartAddress += count;
        __DMB();                //

        //
        // disable tx complete interrupts, enable dma interrupts
        //
        pUart->IER              = (pUart->IER & ~(UART0_IER_TXIM_Msk | UART0_IER_TXCMPMIM_Msk)) |
                                  (UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk);
        //
        // clear and setup interrupts
        //
        pUart->IEC              = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                                  UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

        pUart->DCR              = UART0_DCR_DMAPRI_Msk | UART0_DCR_TXDMAE_Msk;  // enable dma
        return AM_HAL_UART_STREAM_STATUS_TX_DMA_BUSY;
    }
    //
    // no more data, DMA complete, stop dma
    //
    if (pTxParams->activeDmaTxDesc)
    {
        pTxParams->activeDmaTxDesc->ui32NumDmaQueued = 0;
        pTxParams->activeDmaTxDesc = NULL;
        psState->sDmaQueue.bDMAActive = false;
    }

    pUart->DCR              = 0;  // stop dma

    //
    // clear and disable tx and DMA interrupts
    // Tx fifo is probably still emptying, don't stop tx, optionally enable tx complete interrupt,
    // check tx comp handling code, may need to be rewritten, which could cause compatibility headache with older
    // non DMA code, don't clear tx complete, it may have completed before this code has been run
    //
    pUart->IER              =
        (pUart->IER & ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk)) |
        (UART0_IER_TXCMPMIM_Msk);
    pUart->IEC              = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_TXIC_Msk;


    return  AM_HAL_UART_STREAM_STATUS_TX_DMA_COMPLETE;
}
//*****************************************************************************
//
// streaming isr service
// manages isr behavior for Stream API calls
//
//*****************************************************************************
am_hal_uart_stream_status_t
am_hal_uart_interrupt_stream_service(void *pUartHandle)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return AM_HAL_UART_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    volatile UART0_Type *pUart = UARTn(psState->ui32Module);

    am_hal_uart_stream_status_t ui32RetStat = AM_HAL_UART_STREAM_STATUS_SUCCESS;

    //
    // manage rx fifo data
    //
    if (pUart->MIS & (UART0_MIS_RTMIS_Msk | UART0_MIS_RXMIS_Msk))
    {
        //
        // read the fifo data, save into the rx queue
        //
        am_hal_uart_stream_rx_params_t *psRxParams = &psState->sRx_params;
        am_hal_stream_queue_t *pRxQ = &psRxParams->sRxQueue;
        uint32_t ui32QueSize = pRxQ->ui32Capacity;
        uint8_t *pui8QueBuff = pRxQ->pui8Data;
        //
        // @note: if the output buffer is read from a higher priority ISR
        //  this next block should be in a critical section
        //
        {
            uint32_t ui32WrtIdx   = pRxQ->ui32WriteIndex;
            uint32_t ui32NumInQue = pRxQ->ui32Length;

            //
            // unload fifo
            // loop while there is data in the queue
            // and there is storage to save the incoming data
            //
            while (!(pUart->FR & UART0_FR_RXFE_Msk))
            {
                uint32_t ui32RdDr = pUart->DR;

                //
                // capture any read error flags
                //
                ui32RetStat |= (ui32RdDr & AM_HAL_UART_STREAM_STATUS_INTRNL_MSK);
                pui8QueBuff[ui32WrtIdx] = (uint8_t) ui32RdDr;
                ui32NumInQue++;
                if (++ui32WrtIdx >= ui32QueSize)
                {
                    ui32WrtIdx = 0;
                }

                if (ui32NumInQue > ui32QueSize)
                {
                    //
                    // queue is at the limit, can't write this data
                    //
                    ui32NumInQue = ui32QueSize;
                    ui32RetStat |= AM_HAL_UART_STREAM_STATUS_RX_QUEUE_FULL;
                    break;
                }
            }
            pRxQ->ui32WriteIndex = ui32WrtIdx;
            if (pRxQ->ui32Length != ui32NumInQue)
            {
                //
                // new data has been added to the rx buffer
                //
                ui32RetStat |= AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL;
                pRxQ->ui32Length = ui32NumInQue;
            }

            //
            // Clear these Interrupts (rx fifo and rx timeout)
            //
            pUart->IEC = (UART0_IEC_RTIC_Msk | UART0_IEC_RXIC_Msk);


            if (psRxParams->pfRxCompleteCallback &&
                (psRxParams->ui32RxCallbackPending == 0) &&
                psRxParams->ui32RxCallbackThreshold &&
                (ui32NumInQue >= psRxParams->ui32RxCallbackThreshold))
            {
                //
                // perform callback when:
                //  - a callback function and
                //  - callback threshold has been defined and
                //  - the number of rx bytes in buffer >= threshold.
                //  -- called function/process must clear pRxQueue->ui32RxCallbackPending to re-enable
                //      this callback
                //
                am_hal_uart_stream_callback_data_t tCBdata;
                tCBdata.pui8Buffer        = pui8QueBuff;
                tCBdata.ui32BufferSize    = ui32NumInQue;
                tCBdata.eStatus             = ui32RetStat;
                psRxParams->ui32RxCallbackPending = 1;
                tCBdata.hasData           = &psRxParams->ui32RxCallbackPending;
                psRxParams->pfRxCompleteCallback(&tCBdata);
            }
        }
    } // ui32IES_int & (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT

    //
    // =============== tx management ================================
    //

    //
    // rx and tx dma support
    //
    if (pUart->MIS & UART0_MIS_DMAEMIS_Msk)
    {
        //
        // DMA error
        // clear interrupts,
        // Shut all DMA off
        //
        if (pUart->DCR & UART0_DCR_TXDMAE_Msk )
        {
            pUart->IER      &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk | UART0_IER_TXCMPMIM_Msk);
            pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;
        }
        else
        {
            pUart->IER      &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_RTIM_Msk | UART0_IER_RXIM_Msk);
            pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk | UART0_IEC_RXIC_Msk | UART0_IEC_RTIC_Pos;
        }
        pUart->DCR      = 0;  // stop dma

        ui32RetStat     |= AM_HAL_UART_STREAM_STATUS_DMA_ERROR;
        psState->sDmaQueue.bDMAActive      = false;
        psState->bBusyWaitEnabled = false;

    } // pUart->MIS & UART0_MIS_DMAEMIS_Msk, dma error


    if (pUart->MIS & UART0_MIS_DMACPMIS_Msk)
    {
        //
        // Handle DMA complete interrupt
        //
        switch (psState->eStreamingDmaMode)
        {
            case AM_HAL_UART_DMA_RX_DOUBLE:
                ui32RetStat |= rxDoubleBuffDMAHandler(pUart, psState);
                break;
            case AM_HAL_UART_DMA_RX_SINGLE:
                ui32RetStat |= rxSingleBuffDMAHandler(pUart, psState);
                break;
            case AM_HAL_UART_DMA_TX_DOUBLE_BUFFER:
                ui32RetStat |= txDoubleBuffDMAHandler(pUart, psState);
                break;
            case AM_HAL_UART_DMA_TX_SINGLE_BUFFER:
                ui32RetStat |= txCircularBuffDMAHandler(pUart, psState);
                break;
            default:
                ui32RetStat |= AM_HAL_UART_STREAM_DMA_CFG_ERROR;
                break;
        }
    } // pUart->MIS & UART0_MIS_DMACPMIS_Msk :: big dma complete interrupt block

    //
    // manage tx fifo data
    //
    if (pUart->MIS & UART0_MIS_TXMIS_Msk)
    {
        //
        // When here, the tx interrupt is enabled
        // and the interrupt is active
        //
        am_hal_stream_queue_t *pTxQ = &psState->sTx_params.sTxQueue;
        //
        // @note: This critical section is not needed if:
        // uart send isn't called in higher priority ISRs
        //
        AM_CRITICAL_BEGIN
            uint32_t ui32NumInQue = pTxQ->ui32Length;
            if (ui32NumInQue)
            {
                //
                // There is data to transmit
                // move data from the tx queue to the tx fifo
                //
                uint8_t *pui8QueBuff = pTxQ->pui8Data;
                uint32_t ui32QueSize = pTxQ->ui32Capacity;
                uint32_t ui32RdIdx = pTxQ->ui32ReadIndex;
                //
                // Clear these Tx Interrupts
                //
                pUart->IEC = UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

                while (ui32NumInQue && !(pUart->FR & UART0_FR_TXFF_Msk))
                {
                    pUart->DR = pui8QueBuff[ui32RdIdx];
                    if (++ui32RdIdx >= ui32QueSize)
                    {
                        ui32RdIdx = 0;
                    }
                    ui32NumInQue--;
                }
                pTxQ->ui32ReadIndex = ui32RdIdx;
                pTxQ->ui32Length = ui32NumInQue;
                if (ui32NumInQue == 0)
                {
                    //
                    // Nothing left in queue, disable this interrupt
                    // enable the tx complete interrupt
                    //

                    am_hal_uart_stream_tx_complete_options_t eTxOpts = psState->sTx_params.eTxCompleteMode;
                    if ( eTxOpts & eAM_HAL_TX_COMPL_TX_COMPLETE)
                    {
                        pUart->IER = (pUart->IER & ~UART0_IER_TXIM_Msk) | UART0_IER_TXCMPMIM_Msk;
                    }
                    else
                    {
                        pUart->IER &= ~UART0_IER_TXIM_Msk;
                        if (eTxOpts & eAM_HAL_TX_COMPL_BUSY_WAIT_COMPLETE)
                        {
                            psState->bBusyWaitEnabled = true;
                        }
                    }

                }
                else
                {
                    //
                    // There is still data in the queue,
                    // so at least one more TXIM interrupt is needed
                    // tx complete interrupt is not needed until the queue is empty
                    //
                    pUart->IER = (pUart->IER & ~UART0_IER_TXCMPMIM_Msk) | UART0_IER_TXIM_Msk;
                }
                ui32RetStat |= AM_HAL_UART_STREAM_STATUS_TX_BUSY;
            }
            else
            {
                //
                // there is nothing in the queue
                // clear and disable this interrupt, this code should not be executed
                // there could still be some data in the tx fifo
                //
                pUart->IER &= ~UART0_IER_TXIM_Msk;

            }
        AM_CRITICAL_END

    } // ui32IES_int & UART0_IER_TXIM_Msk

    if (pUart->MIS & UART0_MIS_TXCMPMMIS_Msk)
    {

        //
        // tx complete, clear and disable this interrupt
        //
        pUart->IEC = UART0_IEC_TXCMPMIC_Msk;
        //
        // @note: This critical section is not needed if:
        // uart send isn't called in higher priority ISRs
        //
        AM_CRITICAL_BEGIN
            pUart->IER &= ~UART0_IER_TXCMPMIM_Msk;
        AM_CRITICAL_END
        ui32RetStat &= ~AM_HAL_UART_STREAM_STATUS_TX_BUSY;
        ui32RetStat |= AM_HAL_UART_STREAM_STATUS_TX_COMPLETE;
        if (psState->sTx_params.eTxCompleteMode & eAM_HAL_TX_COMPL_BUSY_WAIT_COMPLETE)
        {
            psState->bBusyWaitEnabled = true;
        }
        if (psState->sTx_params.eTxCallbackNotificationOptions & eAM_HAL_TX_COMPL_TX_COMPLETE)
        {
            psState->eTxCompleteNotificationEnabled |= eAM_HAL_TX_COMPL_TX_COMPLETE;
        }

    } // pUart->MIS & UART0_MIS_TXCMPMMIS_Msk

    if (psState->bBusyWaitEnabled)
    {
        psState->bBusyWaitEnabled = false;
        while (pUart->FR & UART0_FR_BUSY_Msk)
        {
            // @todo do we need a timeout here
        }
        ui32RetStat |= AM_HAL_UART_STREAM_STATUS_BUSY_WAIT_CMPL;

        if (psState->sTx_params.eTxCallbackNotificationOptions & eAM_HAL_TX_COMPL_BUSY_WAIT_COMPLETE)
        {
            psState->eTxCompleteNotificationEnabled |= eAM_HAL_TX_COMPL_BUSY_WAIT_COMPLETE;
        }

    }

    if (psState->eTxCompleteNotificationEnabled)
    {
        psState->eTxCompleteNotificationEnabled = eAM_HAL_TX_COMPL_NO_NOTIFICATION;
        if (psState->sTx_params.pfTxCompleteCallback)
        {
            am_hal_uart_stream_callback_data_t sCallbackData;
            sCallbackData.eStatus = ui32RetStat;
            psState->sTx_params.pfTxCompleteCallback( &sCallbackData );
        }
    }


    return ui32RetStat;

} // am_hal_uart_interrupt_queue_service

//*****************************************************************************
//
// Choose the correct function based on DMA mode for tx append.
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_stream_uart_append_tx(void *pUartHandle,
                             uint8_t *pui8Buff,
                             uint32_t ui32NumBytes)
{
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    am_hal_uart_errors_t eRetVal = (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_ARG;

    if ( psState )
    {
        am_hal_uart_streaming_dma_mode_e eDmaMode = psState->eStreamingDmaMode;
        switch(eDmaMode)
        {
            case AM_HAL_UART_DMA_TX_DOUBLE_BUFFER:
                eRetVal = am_hal_uart_append_tx_double(pUartHandle, pui8Buff, ui32NumBytes);
                break;

            case AM_HAL_UART_DMA_TX_SINGLE_BUFFER:
                eRetVal = am_hal_uart_append_tx_single(pUartHandle, pui8Buff, ui32NumBytes);
                break;

            case AM_HAL_UART_DMA_NONE:
            case AM_HAL_UART_DMA_RX_SINGLE:
            case AM_HAL_UART_DMA_RX_DOUBLE:
            default:
                    eRetVal = am_hal_uart_append_tx_fifo(pUartHandle, pui8Buff, ui32NumBytes);
            break;
        }
    }

    return eRetVal;
}

//*****************************************************************************
//
// Append data into the uart tx output queue, not using DMA
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_append_tx_fifo(void *pUartHandle,
                           uint8_t *pui8Buff,
                           uint32_t ui32NumBytes)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t)AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    am_hal_stream_queue_t *pTxQ = &psState->sTx_params.sTxQueue;
    if (pTxQ->pui8Data == NULL)
    {
        //
        // the user needs to define the queue
        //
        return AM_HAL_UART_ERR_DMA_NO_INIT;
    }

    volatile UART0_Type *pUart = UARTn(psState->ui32Module);

    am_hal_uart_errors_t eReturnStat = AM_HAL_UART_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN
    do
    {
        //
        // clear pending tx interrupts
        //
        pUart->IEC = UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

        uint32_t bytesInQueue = pTxQ->ui32Length;

        //
        // Only write to fifo once, before or after filling the queue:
        // if queue is empty write buffer fifo first, then reducing
        // (or eliminating) the need to save that data in the queue.
        //
        bool fifoFilled = false;
        if (bytesInQueue == 0)
        {
            fifoFilled = true;
            //
            // nothing in the queue, so start by dumping incoming data into fifo
            //
            while (ui32NumBytes && !(pUart->FR & UART0_FR_TXFF_Msk))
            {
                pUart->DR = *pui8Buff++;
                ui32NumBytes--;
            }
        }

        uint8_t  *txQueueBuff = pTxQ->pui8Data;
        uint32_t ui32Wi       = pTxQ->ui32WriteIndex;
        uint32_t ui32Maxi    = pTxQ->ui32Capacity;

        if (ui32NumBytes)
        {
            //
            // put the remainder in the queue
            //
            bytesInQueue += ui32NumBytes;
            if (bytesInQueue > ui32Maxi)
            {
                eReturnStat = AM_HAL_UART_ERR_BUFFER_OVERFILL;
                break;
            }

            do
            {
                //
                // fill circular buffer
                //
                txQueueBuff[ui32Wi] = *pui8Buff++;
                if (++ui32Wi >= ui32Maxi)
                {
                    ui32Wi = 0;
                }
            }
            while (--ui32NumBytes);
            pTxQ->ui32WriteIndex = ui32Wi;
        }

        if (!fifoFilled)
        {
            //
            // fifo has not been filled this pass, so
            // fill fifo with data from queue
            //
            uint32_t ui32RdIdx = pTxQ->ui32ReadIndex;

            while (bytesInQueue && !(pUart->FR & UART0_FR_TXFF_Msk))
            {
                //
                // move data from circular buffer into queue
                //
                pUart->DR = txQueueBuff[ui32RdIdx];
                if (++ui32RdIdx >= ui32Maxi)
                {
                    ui32RdIdx = 0;
                }
                bytesInQueue--;
            }
            pTxQ->ui32ReadIndex = ui32RdIdx;
        }
        pTxQ->ui32Length     = bytesInQueue;

        if (bytesInQueue)
        {
            //
            // enable the tx fifo low interrupt, that will manage fifo filling from queue
            //
            pUart->IER = (pUart->IER & ~UART0_IER_TXCMPMIM_Msk) | UART0_IER_TXIM_Msk;
        }
        else
        {
            //
            // queue is empty, but fifo has data, enable transmit complete interrupt
            //
            pUart->IER = (pUart->IER & ~UART0_IER_TXIM_Msk) | UART0_IER_TXCMPMIM_Msk;
        }
    }
    while(false);

    AM_CRITICAL_END

    return eReturnStat;
} // am_hal_uart_append_tx

//****************************** DMA TX Streaming Code ********************************

//*****************************************************************************
//
// Append tx data with DMA
// this is using the "one buffer / triple descriptor" method (circular buffer)
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_append_tx_single(void *pUartHandle,
                             uint8_t *pui8Buff,
                             uint32_t ui32NumBytes)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    if (ui32NumBytes == 0)
    {
        //
        // no data, nothing to do here
        //
        return AM_HAL_UART_STATUS_SUCCESS;
    }

    am_hal_uart_stream_tx_params_t *psTxParams = &psState->sTx_params;
    if (psTxParams == NULL )
    {
        return AM_HAL_UART_DMA_SETUP_ERROR;
    }

    if ( !psTxParams->bDmaQueueInited || (psState->sDmaQueue.eActiveQueueType != eAM_HAL_UART_TX_ACTIVE_DMA) )
    {
        return AM_HAL_UART_ERR_DMA_NO_INIT;
    }

    am_hal_uart_stream_dma_tx_descriptor_entry_t *q = psTxParams->tDescriptor;
    am_hal_uart_errors_t eReturnVal = AM_HAL_UART_STATUS_SUCCESS;

    //
    // Since the uart interrupt is also working on these DMA data structures,
    // start a critical section (disable interrupts) when queueing DMA data
    //
    AM_CRITICAL_BEGIN

    do
    {
        //
        //! compute number of bytes currently in use.
        //
        uint32_t ui32BytesQueued = q[0].ui32NumBytes + q[1].ui32NumBytes + q[2].ui32NumBytes;
        if (psTxParams->activeDmaTxDesc)
        {
            ui32BytesQueued += psTxParams->activeDmaTxDesc->ui32NumDmaQueued;
        }

        //
        // compute total allocated buffer size
        //
        uint32_t ui32BuffSize = psTxParams->queueEndAddr - psTxParams->queueStartAddr;

        //
        // compute how many bytes can be queued
        //
        uint32_t ui32RoomLeft = ui32BuffSize - ui32BytesQueued;

        if (ui32RoomLeft < ui32NumBytes)
        {
            //
            // return not enough room for data
            //
            eReturnVal = AM_HAL_UART_ERR_BUFFER_OVERFILL;
            break;
        }

        //
        // if here, the data will fit in the buffer allocated
        // find next copy location
        //
        am_hal_uart_stream_dma_tx_descriptor_entry_t *psNextQ;
        bool bStartDMA = false;
        if (psTxParams->activeDmaTxDesc == 0)
        {
            //
            // no active dma
            //
            if ( ui32BytesQueued )
            {
                //
                // Error condition.
                // This is inconsistent/invalid behavior, NO active DMA,
                // but data is queued.
                //
                eReturnVal = AM_HAL_UART_ERR_MEMORY_ERROR_01;
                break;
            }

            if (ui32NumBytes <= AM_HAL_UART_FIFO_MAX)
            {
                volatile UART0_Type *pUart = UARTn(psState->ui32Module);
                //
                // contents can fit if fifo is empty
                // is fifo empty then no need for DMA
                //
                if (pUart->FR & UART0_FR_TXFE_Msk)
                {

                    pUart->DCR      = 0;  // stop dma

                    //
                    // fifo is empty, dump data into fifo
                    //
                    do
                    {
                        pUart->DR = *pui8Buff++;
                    }
                    while (--ui32NumBytes);

                    //
                    // setup interrupts
                    // set TX complete interrupt, this could be optional
                    // no need for DMA interrupt
                    //
                    pUart->IER      = (pUart->IER & ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk)) |
                                      (UART0_IER_TXCMPMIM_Msk);

                    pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                                      UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;
                    break; // exit critical section and this function
                }
            } // if ui32NumBytes <= AM_HAL_UART_FIFO_MAX

            psNextQ                     = &psTxParams->tDescriptor[0];  // use the first descriptor
            psNextQ->ui32StartAddress   = psTxParams->queueStartAddr;   // start at the beginning of circ buffer
            //psDma->nextDmaWrtDesc       = psNextQ->nextDesc;       // set active write descriptor
            bStartDMA                   = true;                    // since dma is not running, dma needs to be started
            psTxParams->activeDmaTxDesc      = psNextQ;
            psTxParams->nextDmaWrtDesc       = psNextQ;                 // once dma is started will need to advance this

        }
        else
        {
            //
            // there is dma running, so grab the current append/write descriptor
            // there will be no need to start dma in this function
            //
            psNextQ                     = psTxParams->nextDmaWrtDesc;
        }

        //
        // add new data at the end of any existing data
        //
        uint32_t ui32CopyStartAddr = psNextQ->ui32StartAddress + psNextQ->ui32NumBytes;
        uint32_t ui32BuffHardEnd   = psTxParams->queueEndAddr;  // end of the circular buffer
        if (ui32CopyStartAddr >= ui32BuffHardEnd)
        {
            //
            // this is a serious error, existing buffer is over-limit (off the end of the array)
            // for some reason
            //
            eReturnVal = AM_HAL_UART_ERR_MEMORY_ERROR_02;
            break;
        }

        //
        // find the end of the new data in the accumulating descriptor
        //
        uint32_t ui32NewEnd   = ui32CopyStartAddr + ui32NumBytes;  // updated end of data in from start
        uint32_t ui32CopySize = ui32NumBytes;

        if (ui32NewEnd >= ui32BuffHardEnd)
        {
            //
            // this copy will run off the end of the circular buffer
            // need to use two buffers to make this fit
            //  reduce the size of the first copy
            //
            ui32CopySize = ui32BuffHardEnd - ui32CopyStartAddr;

            psNextQ->ui32NumBytes += ui32CopySize;

            //
            // will still need to do another copy, below, for the remainder of the data
            //
            memcpy((void *) ui32CopyStartAddr, pui8Buff, ui32CopySize);
            ui32NumBytes -= ui32CopySize;
            if (ui32NumBytes & 0x80000000)
            {
                //
                // This value has gone negative, a serious error. This shouldn't happen
                // This test can be removed if code proves reliable
                //
                eReturnVal = AM_HAL_UART_ERR_MEMORY_ERROR_03;
                break;
            }

            //
            // advance the write/append queue, since this wrapped
            //
            psNextQ                     = psNextQ->nextDesc;
            psNextQ->ui32StartAddress   = psTxParams->queueStartAddr; // start at the beginning of the buffer
            psNextQ->ui32NumBytes       = ui32NumBytes;
            psTxParams->nextDmaWrtDesc    = psNextQ;

            if (ui32NumBytes)
            {
                // buffer wrap
                // so that ui32CopyStartAddr address is always the beginning of the buffer
                // this buffer should be empty at this point

                pui8Buff                += ui32CopySize;  // advance buffer pointer
                memcpy((void *) psNextQ->ui32StartAddress, pui8Buff, ui32NumBytes);
            } // ui32Numbytes != 0: second buffer copy
        }
        else // ui32NewEnd >= ui32BuffHardEnd
        {
            //
            // all data will fit in buffer with one copy
            //
            memcpy((void *) ui32CopyStartAddr, pui8Buff, ui32CopySize);
            psNextQ->ui32NumBytes += ui32CopySize;
        } // ui32NewEnd >= ui32BuffHardEnd

        if (bStartDMA)
        {
            am_hal_uart_stream_dma_tx_descriptor_entry_t *psActiveDesc = psTxParams->activeDmaTxDesc;

            volatile UART0_Type *pUart = UARTn(psState->ui32Module);

            pUart->DCR      = 0;  // stop dma

            pUart->TARGADDR             = psActiveDesc->ui32StartAddress;
            uint32_t  ui32TxCount       = psActiveDesc->ui32NumBytes;
            if (ui32TxCount > AM_HAL_MAX_UART_DMA_SIZE)
            {
                ui32TxCount                 = AM_HAL_MAX_UART_DMA_SIZE;
            }
            pUart->COUNT                    = ui32TxCount;
            psActiveDesc->ui32NumDmaQueued  = ui32TxCount;
            psActiveDesc->ui32NumBytes      -= ui32TxCount;
            psActiveDesc->ui32StartAddress  += ui32TxCount;

            __DMB();        // Ensure data is copied before starting DMA (it is necessary)

            //
            // clear and setup interrupts
            //
            pUart->IER      = (pUart->IER & ~(UART0_IER_TXCMPMIM_Msk | UART0_IER_TXIM_Msk)) |
                              (UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk);
            pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                              UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

            pUart->DCR      = UART0_DCR_DMAPRI_Msk | UART0_DCR_TXDMAE_Msk;  // start tx dma

            if (psActiveDesc == psTxParams->nextDmaWrtDesc)
            {
                //
                // the active buffer cannot also be the append buffer, need to advance the append buffer
                //
                am_hal_uart_stream_dma_tx_descriptor_entry_t *psAppendDesc = psActiveDesc->nextDesc;
                uint32_t ui32StartAddr = psActiveDesc->ui32StartAddress + psActiveDesc->ui32NumBytes;
                if ( ui32StartAddr >= ui32BuffHardEnd )
                {
                    ui32StartAddr = psTxParams->queueStartAddr;  // static code analysis says this can't happen
                }
                psAppendDesc->ui32StartAddress  = ui32StartAddr;
                psAppendDesc->ui32NumBytes      = 0;
                psTxParams->nextDmaWrtDesc           = psAppendDesc;
            } // psActiveDesc == psDma->nextDmaWrtDesc :  end of advance append desc

        } // end of start DMA
    }
    while( false );


    AM_CRITICAL_END

    return eReturnVal;

} // am_hal_uart_append_tx

//*****************************************************************************
//
// This is for appending tx data while doing double buffered DMA
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_append_tx_double(void *pUartHandle,
                             uint8_t *pui8Buff,
                             uint32_t ui32NumBytes)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    if (ui32NumBytes == 0)
    {
        // no data, nothing to do here
        return AM_HAL_UART_STATUS_SUCCESS;
    }

    am_hal_uart_stream_tx_params_t *psTxParams = &psState->sTx_params;
    if (!psTxParams->bDmaQueueInited || (psState->sDmaQueue.eActiveQueueType != eAM_HAL_UART_TX_ACTIVE_DMA))
    {
        return AM_HAL_UART_ERR_DMA_NO_INIT;
    }

    am_hal_uart_errors_t eRetVal = AM_HAL_UART_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN
        do
        {
            //
            // find the current descriptor
            //
            am_hal_uart_stream_dma_tx_descriptor_entry_t *currD = psTxParams->activeDmaTxDesc;

            //
            // find the descriptor that will be used to copy data into
            //
            am_hal_uart_stream_dma_tx_descriptor_entry_t *psPendD;
            if (currD == NULL)
            {
                //
                // nothing transmitting
                //
                if (ui32NumBytes <= AM_HAL_UART_FIFO_MAX)
                {
                    //
                    // contents can fit if the fifo is empty.
                    // is fifo empty?
                    // no need for DMA
                    //
                    volatile UART0_Type *pUart = UARTn(psState->ui32Module);

                    pUart->DCR = 0;  // stop dma

                    if (pUart->FR & UART0_FR_TXFE_Msk)
                    {
                        //
                        // fifo is empty, dump data into fifo
                        //
                        do
                        {
                            pUart->DR = *pui8Buff++;
                        }
                        while (--ui32NumBytes);

                        //
                        // setup interrupts
                        // set TX complete interrupt:
                        // tx complete interrupt could be used to disable uar transmitter
                        // for power savings.
                        //  UART0_IER_RTIM_Msk - delete this line
                        //
                        pUart->IER =
                            (pUart->IER & ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk | UART0_IER_TXIM_Msk)) |
                            (UART0_IER_TXCMPMIM_Msk)  ;

                        pUart->IEC = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                                     UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

                        break; // exit critical section and function

                    }
                } // if ui32NumBytes <= AM_HAL_UART_FIFO_MAX

                psPendD                     = &psTxParams->tDescriptor[0];
                uint32_t ui32BaseAddr       = psPendD->ui32BuffBaseAddr;
                psPendD->ui32StartAddress   = ui32BaseAddr;
                psPendD->ui32NumBytes       = 0;
            }
            else //currD == NULL
            {
                //
                // DMA is active, so choose the other buffer
                //
                psPendD = currD->nextDesc;
            } //currD == NULL

            //
            // compute unused buffer size
            //
            uint32_t ui32NumInQueue = psPendD->ui32NumBytes;
            uint32_t ui32RoomLeft   = psPendD->ui32BufferSize - ui32NumInQueue;
            if (ui32RoomLeft < ui32NumBytes)
            {
                //
                // this data won't fit
                // exit with error
                //
                eRetVal = AM_HAL_UART_ERR_BUFFER_OVERFILL;
                break;  // exit critical section and function
            }

            //
            // there is room left to save data in the output buffer
            //
            uint32_t ui32CopyStartAddr = psPendD->ui32StartAddress + ui32NumInQueue;  // start address in buffer
            memcpy((void *) ui32CopyStartAddr, pui8Buff, ui32NumBytes);          // copy data into buffer
            psPendD->ui32NumBytes = ui32NumInQueue + ui32NumBytes;             // update buffer size

            if (currD == NULL)
            {
                //
                // dma is not running, start dma
                //
                psTxParams->activeDmaTxDesc    = psPendD;

                volatile UART0_Type *pUart = UARTn(psState->ui32Module);

                pUart->DCR                = 0;  // stop dma
                pUart->TARGADDR           = psPendD->ui32StartAddress;  // set start address
                uint32_t ui32TxCount      = psPendD->ui32NumBytes;      // number of bytes for DMA
                if (ui32TxCount > AM_HAL_MAX_UART_DMA_SIZE)
                {
                    ui32TxCount           = AM_HAL_MAX_UART_DMA_SIZE;
                }
                psPendD->ui32StartAddress += ui32TxCount;       // advance to next starting address
                psPendD->ui32NumBytes     -= ui32TxCount;       // subtract number of bytes queued via DMA
                pUart->COUNT              = ui32TxCount;        // queue this many bytes

                __DMB();        // This is needed here to ensure data copy is complete before DMA start

                //
                // setup interrupts
                //
                pUart->IER      = (pUart->IER & ~(UART0_IER_TXCMPMIM_Msk | UART0_IER_TXIM_Msk)) |
                                  (UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk);
                pUart->IEC      = UART0_IEC_DMAEIC_Msk | UART0_IEC_DMACPIC_Msk |
                                  UART0_IEC_TXCMPMIC_Msk | UART0_IEC_TXIC_Msk;

                pUart->DCR      = UART0_DCR_DMAPRI_Msk | UART0_DCR_TXDMAE_Msk;  // start DMA

            } // currD == NULL
        }
        while (false); // critical section: do/while

    AM_CRITICAL_END

    return eRetVal;
}
//*****************************************************************************
//
//! @brief This will unload whatever is in the rx fifo into the rx buffer
//!
//! @param psRxQueue  pointer to the rx queue
//! @param pUart      pointer to uart hardware registers
//
//*****************************************************************************
static inline void
uart_stream_unload_rxFifo(am_hal_stream_queue_t *psRxQueue, volatile UART0_Type *pUart)
{
    AM_CRITICAL_BEGIN
    uint32_t ui32Wi       = psRxQueue->ui32WriteIndex;  // this is changed in the isr
    uint32_t ui32Size     = psRxQueue->ui32Capacity;    // not changed in ISR
    uint8_t  *pui8Data    = psRxQueue->pui8Data;        // not changed in ISR
    uint32_t ui32NumInQ   = psRxQueue->ui32Length;
    //
    // unload fifo
    // loop while there is data in the queue
    // and there is storage to save the incoming data
    //
    while (!(pUart->FR & UART0_FR_RXFE_Msk))
    {
        if (ui32NumInQ >= ui32Size)
        {
            break ;
        }
        uint32_t ui32RdDr = pUart->DR;

        //
        // Ignoring read error flags
        //
        pui8Data[ui32Wi] = (uint8_t) ui32RdDr;
        if (++ui32Wi >= ui32Size)
        {
            ui32Wi = 0;
        }
        ui32NumInQ++;

    } // while

    psRxQueue->ui32WriteIndex = ui32Wi;
    if (psRxQueue->ui32Length != ui32NumInQ)
    {
        psRxQueue->ui32Length = ui32NumInQ;
    }
    AM_CRITICAL_END
}

//*****************************************************************************
//
// Get Rx Data in buffer
//
// This function will see how much data in in the rx isr buffer
// this should only be used when rx dma is not used
//
//*****************************************************************************
uint32_t
am_hal_uart_stream_get_num_rx_bytes_in_buffer(void *pUartHandle, bool bUnloadRxFifo)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    am_hal_stream_queue_t *psRxQueue = &psState->sRx_params.sRxQueue;
    if ( bUnloadRxFifo )
    {
        volatile UART0_Type *pUart = UARTn(psState->ui32Module);
        uart_stream_unload_rxFifo(psRxQueue, pUart);
    }

   return  psRxQueue->ui32Length;
}

//*****************************************************************************
//
// Get Rx Data
//
// This function will unload data from the queue and load the data into
// a user supplied buffer.
//
//*****************************************************************************
uint32_t
am_hal_uart_stream_get_rx_data(void *pUartHandle,
                               uint8_t *pui8DestBuff,
                               uint32_t ui32MaxBytesToRead,
                               bool bUnloadRxFifo)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    uint32_t ui32NumRead = 0;
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    if ( ui32MaxBytesToRead )
    {
        am_hal_stream_queue_t *psRxQueue = &psState->sRx_params.sRxQueue;
        if ( bUnloadRxFifo )
        {
            volatile UART0_Type *pUart = UARTn(psState->ui32Module);
            uart_stream_unload_rxFifo( psRxQueue, pUart );
        } // unload fifo

        //
        // read from queue (loaded via uart rx interrupt)
        //
        uint32_t ui32NumInQue = psRxQueue->ui32Length;
        if (ui32NumInQue)
        {
            uint32_t ui32Wi    = psRxQueue->ui32WriteIndex;  // this is changed in the isr
            uint32_t ui32Ri    = psRxQueue->ui32ReadIndex;   // not changed in ISR
            uint32_t ui32Size  = psRxQueue->ui32Capacity;    // not changed in ISR
            uint8_t  *pui8Data = psRxQueue->pui8Data;        // not changed in ISR

            if (ui32Ri >= ui32Size )
            {
                ui32Ri = 0;
            }

            __DMB();

            while(true)
            {
                if (ui32Ri == ui32Wi)
                {
                    //
                    // queue is empty
                    //
                    break;
                }
                //
                // copy data byte from isr buffer to user buffer
                //
                *pui8DestBuff++ = pui8Data[ui32Ri];
                if (++ui32Ri >= ui32Size)
                {
                    ui32Ri = 0;
                }
                if (++ui32NumRead >= ui32MaxBytesToRead)
                {
                    //
                    // read limit reached
                    //
                    break;
                }
            }

            if (ui32NumRead)
            {
                //
                // Update the number of bytes left in queue.
                //
                psRxQueue->ui32ReadIndex = ui32Ri;
                AM_CRITICAL_BEGIN
                    if (psRxQueue->ui32WriteIndex == ui32Ri)
                    {
                        psRxQueue->ui32Length = 0;
                    }
                    else
                    {
                        uint32_t ui32NumInQueue = psRxQueue->ui32Length;
                        if (ui32NumInQueue > ui32NumRead)
                        {
                            ui32NumInQueue -= ui32NumRead;
                        }
                        else
                        {
                            ui32NumInQueue = 0;
                        }
                        psRxQueue->ui32Length = ui32NumInQueue;
                    }
                AM_CRITICAL_END
            }
        } // num in queue
    }

    return ui32NumRead;
}

//*****************************************************************************
//
// Configure Streaming TX DMA Driver
// this is using the "one buffer / triple descriptor" method (circular buffer)
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_stream_configure_tx(void *pUartHandle,
                                const am_hal_uart_stream_tx_config_t *ptxConfig)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;
    am_hal_uart_streaming_dma_mode_e eStreamingDmaMode = psState->eStreamingDmaMode;

    psState->sTx_params.eTxCompleteMode                 = ptxConfig->eTxCompleteMode;
    psState->sTx_params.eTxCallbackNotificationOptions  = ptxConfig->eTxCompleteNotificationAction;
    psState->sTx_params.pfTxCompleteCallback            = ptxConfig->pfTxCallback;

    const am_hal_uart_stream_buff_t *spTxBuffer = &ptxConfig->sTxBuffer;
    psState->sTx_params.bEnableTxQueue = false;
    if (spTxBuffer->pui8Buff && spTxBuffer->ui32BufferSize)
    {
        am_hal_uart_stream_queue_init(&psState->sTx_params.sTxQueue, spTxBuffer->pui8Buff, spTxBuffer->ui32BufferSize);
        psState->sTx_params.bEnableTxQueue = true;
    }


    if ( eStreamingDmaMode == AM_HAL_UART_DMA_TX_DOUBLE_BUFFER ||
         eStreamingDmaMode == AM_HAL_UART_DMA_TX_SINGLE_BUFFER)
    {
        am_hal_uart_dma_config_t *psDmaCfg  = &psState->sDmaQueue;
        psDmaCfg->eActiveQueueType          = eAM_HAL_UART_NO_ACTIVE_DMA;
        am_hal_uart_stream_tx_params_t  *txParams     = &(psState->sTx_params);


        am_hal_uart_stream_dma_tx_descriptor_entry_t *qe    = txParams->tDescriptor;
        memset(qe, 0, sizeof(txParams->tDescriptor));

        qe[0].descIdx           = 0;
        qe[1].descIdx           = 1;
        qe[2].descIdx           = 2;

        txParams->activeDmaTxDesc = 0;
        txParams->nextDmaWrtDesc  = &qe[0];

        uint32_t txQueueAddr    = (uint32_t) psState->sTx_params.sTxQueue.pui8Data;

        if (txQueueAddr == 0)
        {
            return AM_HAL_UART_DMA_CFG_ERROR;
        }

        txParams->bDmaQueueInited = true;
        psDmaCfg->eActiveQueueType = eAM_HAL_UART_TX_ACTIVE_DMA;
        if (eStreamingDmaMode == AM_HAL_UART_DMA_TX_SINGLE_BUFFER)
        {
            //
            // Using three queue descriptors for circular buffer.
            //
            qe[0].nextDesc          = &qe[1];           // close the linked list
            qe[1].nextDesc          = &qe[2];
            qe[2].nextDesc          = &qe[0];

            txParams->queueStartAddr  = txQueueAddr;
            //
            // This is a circular buffer,
            // pre-compute the end of buffer address
            //
            txParams->queueEndAddr    = txParams->queueStartAddr + psState->sTx_params.sTxQueue.ui32Capacity;
        }
        else
        {
            //
            // Using two queue descriptors, one for each buffer
            //
            qe[0].nextDesc          = &qe[1];        // close the linked list
            qe[1].nextDesc          = &qe[0];

            qe[0].ui32StartAddress = txQueueAddr;
            qe[0].ui32BuffBaseAddr = txQueueAddr;

            //
            // split the tx buffer in half for double buffering
            //
            uint32_t ui32SubBufferSize = (psState->sTx_params.sTxQueue.ui32Capacity / 2) & ~0x03;
            uint32_t ui32SecondBufferStartAddr = txQueueAddr + ui32SubBufferSize;
            qe[1].ui32StartAddress = ui32SecondBufferStartAddr;
            qe[1].ui32BuffBaseAddr = ui32SecondBufferStartAddr;

            qe[0].ui32BufferSize = ui32SubBufferSize;
            qe[1].ui32BufferSize = ui32SubBufferSize;
        }
        txParams->bDmaQueueInited  = true;
    }

    return AM_HAL_UART_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Starts configures rx parameters
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_stream_configure_rx(void *pUartHandle,
                                const am_hal_uart_stream_rx_config_t *pRxConfig )
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;
    uint32_t ui32Module = psState->ui32Module;
    if (ui32Module >= AM_REG_UART_NUM_MODULES)
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }

    am_hal_uart_dma_config_t *psDma = &psState->sDmaQueue;
    if (psDma == NULL )
    {
        return AM_HAL_UART_DMA_CFG_ERROR;
    }

    am_hal_uart_stream_rx_params_t *psRxParams = &psState->sRx_params;
    if (psRxParams == NULL )
    {
        return AM_HAL_UART_DMA_SETUP_ERROR;
    }

    psRxParams->bEnableRxQueue = false;
    const am_hal_uart_stream_buff_t *spRxBuffer = &pRxConfig->sRxBuffer;
    if (spRxBuffer->pui8Buff && spRxBuffer->ui32BufferSize)
    {
        am_hal_uart_stream_queue_init(&psRxParams->sRxQueue,
            spRxBuffer->pui8Buff,
            spRxBuffer->ui32BufferSize);
        psRxParams->bEnableRxQueue = true;
    }

    psRxParams->pfRxCompleteCallback    = pRxConfig->pfRxCallback;
    psRxParams->ui32RxCallbackThreshold = pRxConfig->ui32RxCallbackThreshold;
    psRxParams->ui32RxCallbackPending   = 0;

    am_hal_uart_stream_dma_rx_descriptor_entry_t *psDesc = &psRxParams->tDescriptor[0];

    memset(psDesc, 0, sizeof(psRxParams->tDescriptor));

    psRxParams->activeDmaRxDesc = 0;  // This is set when rxdma is started
    uint32_t numBytesToRead    = pRxConfig->ui32NumBytesToRead;
    psDesc[0].ui32NumBytes     = numBytesToRead;
    psDesc[0].ui32BytesToRead  = numBytesToRead;
    psDesc[1].ui32NumBytes     = numBytesToRead;
    psDesc[1].ui32BytesToRead  = numBytesToRead;

    am_hal_uart_streaming_dma_mode_e eStreamingDmaMode = psState->eStreamingDmaMode;
    if ( (eStreamingDmaMode == AM_HAL_UART_DMA_RX_SINGLE) ||
          (eStreamingDmaMode == AM_HAL_UART_DMA_RX_DOUBLE))
    {
        //
        // config DMA settings
        //

        uint32_t rxQueueAddr    = (uint32_t) psRxParams->sRxQueue.pui8Data;
        if (rxQueueAddr == 0)
        {
            return AM_HAL_UART_DMA_CFG_ERROR;
        }

        psState->sDmaQueue.eActiveQueueType = eAM_HAL_UART_RX_ACTIVE_DMA;

        psDesc[1].descIdx           = 1;
        psDesc[0].nextDesc          = &psDesc[1];        // close the linked list
        psDesc[1].nextDesc          = &psDesc[0];

        //psRxParams->activeDmaRxDesc  = 0;
        psRxParams->bDmaQueueInited  = true;

        psDesc[0].ui32BuffBaseAddr  = rxQueueAddr;
        uint32_t queSize            = psRxParams->sRxQueue.ui32Capacity;
        psDesc[0].ui32BufferSize    = queSize;

        if (eStreamingDmaMode == AM_HAL_UART_DMA_RX_DOUBLE)
        {
            //
            // set up for continuous, (double buffered) rx input
            // Use half the buffer size as the a/b size
            // make sure it is at least 4 byte aligned.
            // (assuming the base buffer is aligned).
            //
            uint32_t ui32SubBufferSize = (queSize / 2) & ~0x03;
            uint32_t ui32SecondBufferStartAddr = rxQueueAddr + ui32SubBufferSize;
            psDesc[1].ui32BuffBaseAddr = ui32SecondBufferStartAddr;

            psDesc[0].ui32BufferSize = ui32SubBufferSize;
            psDesc[1].ui32BufferSize = ui32SubBufferSize;
        }
    }

    return AM_HAL_UART_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Starts RX DMA
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_stream_enable_rxDma(void *pUartHandle,
                                bool bEnableRxDMA,
                                bool bClearFifo)
{

#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_errors_t eReturnVal = AM_HAL_UART_STATUS_SUCCESS;
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    uint32_t ui32Module = psState->ui32Module;
    if (bEnableRxDMA)
    {
        volatile UART0_Type *pUart  = UARTn(ui32Module);

        if (bClearFifo)
        {
            int32_t i32LcMax = AM_HAL_UART_FIFO_MAX + 8;  // loop limit counter
            while ( (--i32LcMax > 0) && !(pUart->FR &  UART0_FR_RXFE_Msk) )
            {
                (void) pUart->DR;
            }
        }
        am_hal_uart_stream_rx_params_t *psRxParams = &psState->sRx_params;
        am_hal_uart_stream_dma_rx_descriptor_entry_t *psDesc = &psRxParams->tDescriptor[0];
        psRxParams->activeDmaRxDesc = psDesc;

        uint32_t numBytesToRead = psDesc->ui32BytesToRead;

        if ( numBytesToRead == 0 )
        {
            return AM_HAL_UART_DMA_CFG_ERROR;
        }

        if ( numBytesToRead > psDesc->ui32BufferSize)
        {
            return AM_HAL_UART_RX_BUFFER_TOO_SMALL;
        }

        AM_CRITICAL_BEGIN;
        do
        {
            if ( pUart->DCR & (UART0_DCR_TXDMAE_Msk | UART0_DCR_RXDMAE_Msk) )
            {
                //
                // DMA is active, return error
                //
                eReturnVal =  AM_HAL_UART_DMA_BUSY_ERROR;
                break;
            }

            psState->sDmaQueue.bDMAActive = true ;
            //
            // disable rx interrupts and rx timeout
            //
            pUart->IER = (pUart->IER & ~(UART0_IER_RXIM_Msk | UART0_IER_RTIM_Msk)) | UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk;

            //
            // clear rx interrupts
            //
            pUart->IEC = UART0_IEC_RXIC_Msk;

            psDesc->ui32NumBytes = numBytesToRead;
            if ( psState->eStreamingDmaMode == AM_HAL_UART_DMA_RX_DOUBLE)
            {
                psDesc->nextDesc->ui32NumBytes = numBytesToRead;
            }

            //
            // set up dma buffer
            //
            pUart->TARGADDR     = psDesc->ui32BuffBaseAddr;
            pUart->COUNT        = numBytesToRead;

            //
            // start dma
            //
            pUart->DCR      = UART0_DCR_DMAPRI_Msk | UART0_DCR_RXDMAE_Msk;  // start DMA
        }
        while (false);
        AM_CRITICAL_END
    }
    else
    {
        volatile UART0_Type *pUart  = UARTn(ui32Module);
        // disable DMA interrupts
        AM_CRITICAL_BEGIN;
        pUart->IER &= ~(UART0_IER_DMAEIM_Msk | UART0_IER_DMACPIM_Msk);

        pUart->DCR = 0 ;
        AM_CRITICAL_END

    }

    return eReturnVal;
}

//*****************************************************************************
//
// This will configure rx and tx operation
//
//*****************************************************************************
am_hal_uart_errors_t
am_hal_uart_stream_data_configure(void *pUartHandle,
                                  const am_hal_uart_stream_data_config_t *psDataCfg)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    psState->eStreamingDmaMode = psDataCfg->eStreamingDmaMode;

    am_hal_uart_errors_t eRxCfgErr = am_hal_uart_stream_configure_rx(psState, &psDataCfg->sRxStreamConfig);
    eRxCfgErr |= am_hal_uart_stream_configure_tx(psState, &psDataCfg->sTxStreamConfig);

    return eRxCfgErr;
}

//*****************************************************************************
//
//! Configures the low level (basic) uart settings
//
//*****************************************************************************
uint32_t
am_hal_uart_stream_configure(void *pUartHandle,
                             const am_hal_uart_config_t *psConfig)
{
    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;
    return am_hal_uart_cmn_configure(psState->ui32Module, &psState->sComputedConfig, psConfig);
}

//*****************************************************************************
//
//! Configures the power settings
//
//*****************************************************************************
uint32_t
am_hal_uart_stream_power_control(void *pUartHandle,
                                 am_hal_sysctrl_power_state_e ePowerState,
                                 bool bRetainState)
{
#ifndef AM_HAL_DISABLE_API_VALIDATION
    if (!AM_HAL_UART_CHK_HANDLE(pUartHandle))
    {
        return (am_hal_uart_errors_t) AM_HAL_STATUS_INVALID_HANDLE;
    }
#endif

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    uint32_t ui32Module = psState->ui32Module;

    am_hal_pwrctrl_periph_e eUARTPowerModule = ((am_hal_pwrctrl_periph_e)
                                                (AM_HAL_PWRCTRL_PERIPH_UART0 +
                                                 ui32Module));

    //
    // Decode the requested power state and update UART operation accordingly.
    //
    am_hal_uart_register_state_t *psRegState = &psState->sRegState;
    switch (ePowerState)
    {
        //
        // Turn on the UART.
        //
        case AM_HAL_SYSCTRL_WAKE:
            //
            // Make sure we don't try to restore an invalid state.
            //
            if (bRetainState && psRegState->bValid)
            {
                return AM_HAL_STATUS_INVALID_OPERATION;
            }

            //
            // Enable power control.
            //
            am_hal_pwrctrl_periph_enable(eUARTPowerModule);

            if (bRetainState)
            {

                if ( ( psState->sComputedConfig.ui32BaudRate > 1500000 ) && ( APOLLO5_GE_B1 ) )
                {
                    // Resume D2ASPARE, force uart_gate.clken to be 1, making uart.pclk to be always-on
                    MCUCTRL->D2ASPARE |= (MCUCTRL_D2ASPARE_UART0PLL_Msk << ui32Module);
                }
                am_hal_clkmgr_clock_request(psState->sComputedConfig.eClkSrc, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_UART0 + ui32Module));

                //
                // Restore UART registers
                //
                // AM_CRITICAL_BEGIN
                UARTn(ui32Module)->ILPR = psRegState->regILPR;
                UARTn(ui32Module)->IBRD = psRegState->regIBRD;
                UARTn(ui32Module)->FBRD = psRegState->regFBRD;
                UARTn(ui32Module)->LCRH = psRegState->regLCRH;
                UARTn(ui32Module)->CR   = psRegState->regCR;
                UARTn(ui32Module)->IFLS = psRegState->regIFLS;
                UARTn(ui32Module)->IER  = psRegState->regIER;
                UARTn(ui32Module)->DCR  = psRegState->regDMACFG;
                psRegState->bValid = false;

                // AM_CRITICAL_END
            }
            break;

        //
        // Turn off the UART.
        //
        case AM_HAL_SYSCTRL_NORMALSLEEP:
        case AM_HAL_SYSCTRL_DEEPSLEEP:
            if (bRetainState)
            {
                // AM_CRITICAL_BEGIN

                psRegState->regILPR = UARTn(ui32Module)->ILPR;
                psRegState->regIBRD = UARTn(ui32Module)->IBRD;
                psRegState->regFBRD = UARTn(ui32Module)->FBRD;
                psRegState->regLCRH = UARTn(ui32Module)->LCRH;
                psRegState->regCR   = UARTn(ui32Module)->CR;
                psRegState->regIFLS = UARTn(ui32Module)->IFLS;
                psRegState->regIER  = UARTn(ui32Module)->IER;
                psRegState->regDMACFG = UARTn(ui32Module)->DCR;
                psRegState->bValid = true;

                // AM_CRITICAL_END
            }

            if ( ( psState->sComputedConfig.ui32BaudRate > 1500000 ) && ( APOLLO5_GE_B1 ) )
            {
                // Clear D2ASPARE to save power
                MCUCTRL->D2ASPARE &= ~(MCUCTRL_D2ASPARE_UART0PLL_Msk << ui32Module);
            }
            am_hal_clkmgr_clock_release(psState->sComputedConfig.eClkSrc, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_UART0 + ui32Module));

            //
            // Clear all interrupts before sleeping as having a pending UART
            // interrupt burns power.
            //
            UARTn(ui32Module)->IEC = 0xFFFFFFFF;
            *(volatile uint32_t*)(&UARTn(ui32Module)->MIS);


            //
            // If the user is going to sleep, certain bits of the CR register
            // need to be 0 to be low power and have the UART shut off.
            // Since the user either wishes to retain state which takes place
            // above or the user does not wish to retain state, it is acceptable
            // to set the entire CR register to 0.
            //
            UARTn(ui32Module)->CR = 0;

            //
            // Disable power control.
            //
            am_hal_pwrctrl_periph_disable(eUARTPowerModule);
            break;

        default:
            return AM_HAL_STATUS_INVALID_ARG;
    }

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set Baud Rate based on the UART clock frequency.
//
//*****************************************************************************
#define BAUDCLK     (16) // Number of UART clocks needed per bit.
static uint32_t
config_baudrate(uint32_t ui32Module,
                uint32_t ui32DesiredBaudrate,
                uint32_t *pui32ActualBaud)
{
    uint32_t ui32UartClkFreq;

    //
    // Check that the baudrate is in range.
    //
    switch ( UARTn(ui32Module)->CR_b.CLKSEL )
    {
        case UART0_CR_CLKSEL_PLL_CLK:
            ui32UartClkFreq = 49152000;
            break;

        case UART0_CR_CLKSEL_HFRC_48MHZ:
            ui32UartClkFreq = 48000000;
            break;

        case UART0_CR_CLKSEL_HFRC_24MHZ:
            ui32UartClkFreq = 24000000;
            break;

        case UART0_CR_CLKSEL_HFRC_12MHZ:
            ui32UartClkFreq = 12000000;
            break;

        case UART0_CR_CLKSEL_HFRC_6MHZ:
            ui32UartClkFreq = 6000000;
            break;

        case UART0_CR_CLKSEL_HFRC_3MHZ:
            ui32UartClkFreq = 3000000;
            break;

        default:
            *pui32ActualBaud = 0;
            return AM_HAL_UART_STATUS_CLOCK_NOT_CONFIGURED;
    }

    //
    // Calculate register values.
    //
    {
        uint32_t ui32BaudClk             = BAUDCLK * ui32DesiredBaudrate;
        uint32_t ui32IntegerDivisor      = (ui32UartClkFreq / ui32BaudClk);
        uint64_t ui64IntermediateLong    = ((uint64_t) ui32UartClkFreq * 64) / ui32BaudClk; // Q58.6
        uint64_t ui64FractionDivisorLong = ui64IntermediateLong - ((uint64_t) ui32IntegerDivisor * 64); // Q58.6
        uint32_t ui32FractionDivisor     = (uint32_t) ui64FractionDivisorLong;

        //
        // Check the result.
        //
        if (ui32IntegerDivisor == 0)
        {
            *pui32ActualBaud = 0;
            return AM_HAL_UART_STATUS_BAUDRATE_NOT_POSSIBLE;
        }

        //
        // Write the UART regs.
        //
        UARTn(ui32Module)->IBRD = ui32IntegerDivisor;
        UARTn(ui32Module)->FBRD = ui32FractionDivisor;

        //
        // Return the actual baud rate.
        //
        *pui32ActualBaud = (ui32UartClkFreq / ((BAUDCLK * ui32IntegerDivisor) + ui32FractionDivisor / 4));
    }
    return AM_HAL_STATUS_SUCCESS;
} // config_baudrate()

//*****************************************************************************
//
// Used to configure basic UART settings.
//
//*****************************************************************************
static uint32_t
am_hal_uart_cmn_configure(uint32_t ui32UartModule,
                          am_hal_uart_config_out_t *psCfgOut,
                          const am_hal_uart_config_t *psConfig)
{
    uint32_t ui32Module = ui32UartModule;

    uint32_t ui32ErrorStatus;

    //
    // Reset the CR register to a known value.
    //
    uint32_t ui32Cr = 0;
    UARTn(ui32Module)->CR = ui32Cr;

    //
    // Start by enabling the clocks, which needs to happen in a critical
    // section.
    //
    ui32Cr = _VAL2FLD(UART0_CR_CLKEN, 1);
    UARTn(ui32Module)->CR = ui32Cr;

    // B0 does not support SYSPLL
    if (( psConfig->eClockSrc > AM_HAL_UART_CLOCK_SRC_SYSPLL ) ||
       (( psConfig->eClockSrc == AM_HAL_UART_CLOCK_SRC_SYSPLL ) && ( APOLLO5_B0 )) )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    psCfgOut->eClkSrc = (psConfig->eClockSrc == AM_HAL_UART_CLOCK_SRC_HFRC) ? AM_HAL_CLKMGR_CLK_ID_HFRC : AM_HAL_CLKMGR_CLK_ID_SYSPLL;

    uint32_t ui32UartCrClockSelVal = 0 ;
    if ( psConfig->ui32BaudRate > 1500000 )
    {
        if ( APOLLO5_GE_B1 )
        {
            // Set D2ASPARE, force uart_gate.clken to be 1, making uart.pclk to be always-on
            MCUCTRL->D2ASPARE |= (MCUCTRL_D2ASPARE_UART0PLL_Msk << ui32Module);
        }

        ui32UartCrClockSelVal = (psCfgOut->eClkSrc == AM_HAL_CLKMGR_CLK_ID_SYSPLL) ? UART0_CR_CLKSEL_PLL_CLK : UART0_CR_CLKSEL_HFRC_48MHZ;
    }
    else
    {
        if ( APOLLO5_GE_B1 )
        {
            // Clear D2ASPARE to save power
            MCUCTRL->D2ASPARE &= ~(MCUCTRL_D2ASPARE_UART0PLL_Msk << ui32Module);
        }
        ui32UartCrClockSelVal = (psCfgOut->eClkSrc == AM_HAL_CLKMGR_CLK_ID_SYSPLL) ? UART0_CR_CLKSEL_PLL_CLK : UART0_CR_CLKSEL_HFRC_24MHZ;
    }
    am_hal_clkmgr_clock_request(psCfgOut->eClkSrc, (am_hal_clkmgr_user_id_e)(AM_HAL_CLKMGR_USER_ID_UART0 + ui32Module));

    ui32Cr |= _VAL2FLD(UART0_CR_CLKSEL, ui32UartCrClockSelVal);

    UARTn(ui32Module)->CR = ui32Cr;

    //
    // Set the baud rate.
    //
    ui32ErrorStatus = config_baudrate(ui32Module,
                                      psConfig->ui32BaudRate,
                                      &(psCfgOut->ui32BaudRate));

    if (ui32ErrorStatus != AM_HAL_STATUS_SUCCESS)
    {
        return ui32ErrorStatus;
    }

    //
    // Set the flow control options
    //
    ui32Cr |= psConfig->eFlowControl;

    //
    // Calculate the parity options.
    //
    uint32_t ui32ParityEnable = 0;
    uint32_t ui32EvenParity = 0;

    switch (psConfig->eParity)
    {
        case AM_HAL_UART_PARITY_ODD:
            ui32ParityEnable = 1;
            break;

        case AM_HAL_UART_PARITY_EVEN:
            ui32ParityEnable = 1;
            ui32EvenParity = 1;
            break;

        case AM_HAL_UART_PARITY_NONE:
            break;
    }

    //
    // Set the data format.
    //
    uint32_t ui32LCRH = _VAL2FLD(UART0_LCRH_BRK, 0) |
        _VAL2FLD(UART0_LCRH_PEN, ui32ParityEnable) |
        _VAL2FLD(UART0_LCRH_EPS, ui32EvenParity) |
        _VAL2FLD(UART0_LCRH_STP2, psConfig->eStopBits) |
        _VAL2FLD(UART0_LCRH_FEN, 1) |
        _VAL2FLD(UART0_LCRH_WLEN, psConfig->eDataBits) |
        _VAL2FLD(UART0_LCRH_SPS, 0) ;

    UARTn(ui32Module)->LCRH = ui32LCRH;

    //
    // Set the FIFO levels.
    //
    UARTn(ui32Module)->IFLS = _VAL2FLD(UART0_IFLS_TXIFLSEL, psConfig->eTXFifoLevel) |
        _VAL2FLD(UART0_IFLS_RXIFLSEL, psConfig->eRXFifoLevel) ;

    //
    // Enable the UART, RX, and TX.
    //
    ui32Cr |= _VAL2FLD(UART0_CR_UARTEN, 1);
    UARTn(ui32Module)->CR = ui32Cr;

    UARTn(ui32Module)->CR = ui32Cr | _VAL2FLD(UART0_CR_RXE, 1) | _VAL2FLD(UART0_CR_TXE, 1);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Handle init function
//
//*****************************************************************************
uint32_t am_hal_uart_stream_initialize(uint32_t ui32Module, void **ppUartHandle)
{
    if (ui32Module >= AM_REG_UART_NUM_MODULES )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    void *pUartHandle = &g_am_hal_uart_stream_state[ui32Module];
    *ppUartHandle = pUartHandle;

    am_hal_uart_stream_state_t *psState = (am_hal_uart_stream_state_t *)pUartHandle;

    if (psState->psActiveState)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    psState->psActiveState      = true;

    psState->prefix.s.bInit     = true;
    psState->prefix.s.magic     = AM_HAL_MAGIC_UART_STREAM;
    psState->ui32Module         = ui32Module;
    psState->sRegState.bValid   = false;
    psState->sTx_params.bEnableTxQueue     = false;
    psState->sRx_params.bEnableRxQueue     = false;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
