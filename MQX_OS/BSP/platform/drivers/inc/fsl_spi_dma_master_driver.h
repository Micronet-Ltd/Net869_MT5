/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__FSL_SPI_MASTER_DMA_DRIVER_H__)
#define __FSL_SPI_DMA_MASTER_DRIVER_H__

#include "fsl_spi_hal.h"
#include "fsl_os_abstraction.h"
#include "fsl_dma_driver.h"

#if FSL_FEATURE_SOC_SPI_COUNT

/*! @addtogroup SPI_DRV_MasterDriver*/
/*! @{*/

/*! @brief Table of base pointers for SPI instances. */
extern SPI_Type * const g_spiBase[SPI_INSTANCE_COUNT];

/*! @brief Table to save SPI IRQ enumeration numbers defined in CMSIS header file. */
extern const IRQn_Type g_spiIrqId[SPI_INSTANCE_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/

enum _spi_dma_timeouts
{
	/*! Waits forever for a transfer to complete.*/
	kSpiDmaWaitForever = 0x7fffffff
};

/*!
 * @brief Information about a device on the SPI bus with DMA.
 */
typedef struct SpiDmaUserConfig {
	uint32_t bitsPerSec;    /*!< SPI baud rate in bits per sec */
	spi_clock_polarity_t polarity;
	spi_clock_phase_t phase;
	spi_shift_direction_t direction;

	/* 16-bit support related members */
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
	spi_data_bitcount_mode_t bitCount;  /*!< Number of bits (8 or 16) in a transfer */
#endif
} spi_dma_master_user_config_t;

/*!
 * @brief Runtime state of the SPI master driver with DMA.
 *
 * This structure holds data that are used by the SPI master peripheral driver to
 * communicate between the transfer function and the interrupt handler. The
 * interrupt handler also uses this information to keep track of its progress.
 */
typedef struct SpiDmaMasterState {
	uint32_t spiSourceClock;              /*!< Module source clock*/
	volatile bool isTransferInProgress;     /*!< True if there is an active transfer.*/
	const uint8_t * sendBuffer;    /*!< The buffer being sent.*/
	uint8_t * receiveBuffer;       /*!< The buffer into which received bytes are placed.*/
	volatile size_t remainingSendByteCount; /*!< Number of bytes remaining to send.*/
	volatile size_t remainingReceiveByteCount; /*!< Number of bytes remaining to receive.*/
	volatile size_t transferredByteCount;   /*!< Number of bytes transferred so far.*/
	volatile bool isTransferBlocking;    /*!< True if transfer is a blocking transaction. */
	semaphore_t irqSync;                    /*!< Used to wait for ISR to complete its business.*/
	bool extraByte;    /*!< Flag used for 16-bit transfers with odd byte count */
	dma_channel_t dmaReceive;    /*!< The DMA channel used for receive */
	dma_channel_t dmaTransmit;   /*!< The DMA channel used for transmit */
	uint32_t transferByteCnt;  /*!< Number of bytes to transfer.*/
} spi_dma_master_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name Initialization and shutdown*/
/*@{*/

/*!
 *
 * @brief Initializes a SPI instance for master mode operation to work with DMA.
 *
 * This function uses a DMA-driven method for transferring data.
 * This function initializes the run-time state structure to track the ongoing
 * transfers, un-gates the clock to the SPI module, resets the SPI module, initializes the module
 * to user defined settings and default settings, configures the IRQ state structure, enables
 * the module-level interrupt to the core, and enables the SPI module.
 *
 * This initialization function also configures the DMA module by requesting channels for DMA
 * operation.
 *
 * @param instance The instance number of the SPI peripheral.
 * @param spiDmaState The pointer to the SPI DMA master driver state structure. The user
 *  must pass the memory for this run-time state structure and the SPI master driver
 *  fills out the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @return kStatus_SPI_Success indicating successful initialization
 */
spi_status_t SPI_DRV_DmaMasterInit(uint32_t instance, spi_dma_master_state_t * spiDmaState);

/*!
 * @brief Shuts down a SPI instance with DMA support.
 *
 * This function resets the SPI peripheral, gates its clock, disables any used interrupts to
 * the core, and releases any used DMA channels.
 *
 * @param instance The instance number of the SPI peripheral.
 * @return kStatus_SPI_Success indicating successful de-initialization
 */
spi_status_t SPI_DRV_DmaMasterDeinit(uint32_t instance);

/*@}*/

/*! @name Bus configuration*/
/*@{*/

/*!
 * @brief Configures the SPI port to access a device on the bus with DMA support.
 *
 * The term "device" is used to indicate the SPI device for which the SPI master is communicating.
 * The user has two options to configure the device parameters: either pass in the
 * pointer to the device configuration structure to the desired transfer function or pass it in to
 * the SPI_DRV_DmaMasterConfigureBus function.  The user can pass in a device structure to the
 * transfer function which contains the parameters for the bus (the transfer function then calls
 * this function). However, the user has the option to call this function directly especially
 * to get the calculated baud rate, at which point they may pass in NULL for the device
 * structure in the transfer function (assuming they have called this configure bus function
 * first).
 *
 * @param instance The instance number of the SPI peripheral.
 * @param device Pointer to the device information structure. This structure contains the settings
 *  for SPI bus configurations.
 * @param calculatedBaudRate The calculated baud rate passed back to the user to determine
 *  if the calculated baud rate is close enough to meet the needs. The baud rate never exceeds
 *  the desired baud rate unless the baud rate requested is less than the absolute minimum in
 *  which case the minimum baud rate will be returned.
 */
void SPI_DRV_DmaMasterConfigureBus(uint32_t instance,
								   const spi_dma_master_user_config_t * device,
								   uint32_t * calculatedBaudRate);

/*@}*/

/*! @name Blocking transfers*/
/*@{*/

/*!
 * @brief Performs a blocking SPI master mode transfer with DMA support.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does return until the transfer is complete.
 *
 * @param instance The instance number of the SPI peripheral.
 * @param device Pointer to the device information structure. This structure contains the settings
 *      for the SPI bus configuration for this transfer. You may pass NULL for this
 *      parameter, in which case the current bus configuration is used unmodified.
 * @param sendBuffer Buffer of data to send. You may pass NULL for this parameter, in which case
 *      bytes with a value of 0 (zero) are sent.
 * @param receiveBuffer Buffer where received bytes are stored. If you pass NULL for this parameter,
 *      the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 * @param timeout A timeout for the transfer in microseconds. If the transfer takes longer than
 *      this amount of time, the transfer is aborted and a #kStatus_SPI_Timeout error is
 *      returned.
 *
 * @return #kStatus_Success The transfer was successful.
 *         #kStatus_SPI_Busy Cannot perform another transfer because one is already in progress.
 *         #kStatus_SPI_Timeout The transfer timed out and was aborted.
 */
spi_status_t SPI_DRV_DmaMasterTransferBlocking(uint32_t instance,
											   const spi_dma_master_user_config_t * device,
											   const uint8_t * sendBuffer,
											   uint8_t * receiveBuffer,
											   size_t transferByteCount,
											   uint32_t timeout);

/*@}*/

/*! @name Non-blocking transfers*/
/*@{*/

/*!
 * @brief Performs a non-blocking SPI master mode transfer with DMA support.
 *
 * This function returns immediately. It is the user's responsibility to check back to
 * ascertain if the transfer is complete (using the SPI_DRV_DmaMasterGetTransferStatus function).
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does return until the transfer is complete.
 *
 * @param instance The instance number of the SPI peripheral.
 * @param device Pointer to the device information structure. This structure contains the settings
 *      for the SPI bus  configuration for this transfer. You may pass NULL for this
 *      parameter, in which case the current bus configuration is used unmodified.
 * @param sendBuffer Buffer of data to send. You may pass NULL for this parameter, in which case
 *      bytes with a value of 0 (zero) is sent.
 * @param receiveBuffer Buffer where received bytes are stored. If you pass NULL for this parameter,
 *      the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 *
 * @return #kStatus_Success The transfer was successful.
 *         #kStatus_SPI_Busy Cannot perform another transfer because one is already in progress.
 *         #kStatus_SPI_Timeout The transfer timed out and was aborted.
 */
spi_status_t SPI_DRV_DmaMasterTransfer(uint32_t instance,
									   const spi_dma_master_user_config_t * device,
									   const uint8_t * sendBuffer,
									   uint8_t * receiveBuffer,
									   size_t transferByteCount);

/*!
 * @brief Returns whether the previous transfer finished with DMA support.
 *
 * When performing an a-sync transfer, the user can call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success). In addition, if the transfer
 * is still in progress, the user can get the number of words that have been
 * transferred up to now.
 *
 * @param instance The instance number of the SPI peripheral.
 * @param bytesTransferred Pointer to a value that is filled in with the number of bytes that
 *      were sent in the active transfer
 *
 * @return kStatus_Success The transfer has completed successfully.
 *         kStatus_SPI_Busy The transfer is still in progress. @a bytesTransferred is filled
 *         with the number of bytes that have been transferred so far.
 */
spi_status_t SPI_DRV_DmaMasterGetTransferStatus(uint32_t instance,
												uint32_t * bytesTransferred);

/*!
 * @brief Terminates an asynchronous transfer early with DMA support.
 *
 * During an a-sync transfer, the user has the option to terminate the transfer early if the transfer
 * is still in progress.
 *
 * @param instance The instance number of the SPI peripheral.
 * @return kStatus_SPI_Success The transfer was successful.
 *         kStatus_SPI_NoTransferInProgress No transfer is currently in progress.
 */
spi_status_t SPI_DRV_DmaMasterAbortTransfer(uint32_t instance);

/*!
 * @brief Interrupt handler for SPI master mode.
 * This handler is used when the extraByte flag is set to retrieve the received last byte.
 *
 * @param instance The instance number of the SPI peripheral.
 */
void SPI_DRV_DmaMasterIRQHandler(uint32_t instance);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_FEATURE_SOC_SPI_COUNT */
#endif /* __FSL_SPI_MASTER_DMA_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
