/*HEADER**********************************************************************
*
* Copyright 2012-2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the source for a simple example of an
*   application that writes and reads the SPI memory using the SPI driver.
*   It's already configured for onboard SPI flash where available.
*
*
*END************************************************************************/


#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <fsl_dspi_master_driver.h>
#include <fsl_dspi_shared_function.h>

#include <fpga_API.h>
#include "spi_settings.h"
#include "W25X40CL.h"

dspi_master_user_config_t 	userConfig;
dspi_master_state_t 		dspiMasterState;

#define CRC32_POLINOMIAL 0xEDB88320
void disable_others(uint32_t WithFpga)
{
  	GPIO_DRV_ClearPinOutput (FPGA_PWR_ENABLE);

	if(WithFpga)
	{
		GPIO_DRV_ClearPinOutput (FPGA_RSTB);//fpga disable
	}
	GPIO_DRV_ClearPinOutput (CAN1_J1708_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput (CAN2_SWC_PWR_ENABLE);

	GPIO_DRV_ClearPinOutput (USB_ENABLE);
	GPIO_DRV_ClearPinOutput (UART_ENABLE);
	//GPIO_DRV_ClearPinOutput (SPKR_LEFT_EN);
	//GPIO_DRV_ClearPinOutput (SPKR_RIGHT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_EXT_EN);
	GPIO_DRV_ClearPinOutput (CPU_MIC_EN);
	
	GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE);
}
void disable_spi(void)
{
	PORT_HAL_SetMuxMode(PORTB,20u,kPortPinDisabled);
	/* Affects register PCS0 */
	PORT_HAL_SetMuxMode(PORTB,21u,kPortPinDisabled);
	/* Affects register MOSI */
	PORT_HAL_SetMuxMode(PORTB,22u,kPortPinDisabled);
	/* Affects register MISO */
	PORT_HAL_SetMuxMode(PORTB,23u,kPortPinDisabled);
	GPIO_DRV_SetPinOutput (FPGA_RSTB);
}
uint32_t crc_32(uint8_t *page, int len) {
   int i, j;
   uint32_t crc, m;

   i = 0;
   crc = 0xFFFFFFFF;
   for (i = 0; i < len; i++) {
	  crc = crc ^ (uint32_t)page[i];
	  for (j = 7; j >= 0; j--) {    // Do eight times.
		 m = -(crc & 1);
		 crc = (crc >> 1) ^ (CRC32_POLINOMIAL & m);
	  }
   }

   return ~crc;
}

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : fpga_addr_to_buffer
* Comments  : Fills in given address into buffer in correct byte order
*
*END*----------------------------------------------------------------------*/
void fpga_addr_to_buffer(uint32_t addr, uint8_t *buffer)
{
	for (int i = SPI_ADDRESS_BYTES; i; i--)
	{
		buffer[i-1] = (uint8_t)(addr & 0xFF);
		addr >>= 8;
	}
}

int32_t fpga_update_init(uint32_t instance)
{
	int32_t IRQNumber = g_dspiIrqId[instance];
//    dspi_master_user_config_t 	userConfig;
//    dspi_master_state_t 		dspiMasterState;
	/* Define bus configuration */
	dspi_device_t spiDevice;
	/* Configure the SPI bus */
	uint32_t calculatedBaudRate;
	/* Configure timing delays 200ns, it is optional, useful for slower peripheral diveces or "fine tune" SPI timings */
	uint32_t delayInNanosec = 200;
	uint32_t calculatedDelay;

	calculatedDelay = GPIO_DRV_ReadPinInput (FPGA_DONE);//temp!!!maybe check - must be 0
	printf ("updater:FPGA_DONE = %d\n", calculatedDelay);

	userConfig.isChipSelectContinuous 	= true;
	userConfig.isSckContinuous 			= false;
	userConfig.pcsPolarity 				= kDspiPcs_ActiveLow;
	userConfig.whichCtar 				= kDspiCtar0;
	userConfig.whichPcs 				= kDspiPcs0;

	// Init the DSPI module. Setting SPI config structure
	dspi_status_t status = DSPI_DRV_MasterInit(instance, &dspiMasterState, &userConfig);
	if(status != kStatus_DSPI_Success)
	{
		printf ("updater: SPI DSPI_DRV_MasterInit Error %d\n", status);
		return status;
	}

	spiDevice.dataBusConfig.bitsPerFrame = 8;
	spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
	spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
	spiDevice.dataBusConfig.direction = kDspiMsbFirst;
	spiDevice.bitsPerSec =  500000;

//	GPIO_DRV_ClearPinOutput (FPGA_RSTB);
	disable_others(1);
	
	status = DSPI_DRV_MasterConfigureBus(instance, &spiDevice, &calculatedBaudRate);
	if(status != kStatus_DSPI_Success)
	{
		printf ("updater: SPI DSPI_DRV_MasterConfigureBus Error %d\n", status);
		return status;
	}

	/* Set pcs to sck delay */
	DSPI_DRV_MasterSetDelay(instance, kDspiPcsToSck, delayInNanosec, &calculatedDelay);
	/* Set last sck to pcs delay */
	DSPI_DRV_MasterSetDelay(instance, kDspiLastSckToPcs, delayInNanosec, &calculatedDelay);
	/* Set delay after transfer */
	DSPI_DRV_MasterSetDelay(instance, kDspiAfterTransfer, delayInNanosec, &calculatedDelay);

	configure_spi_pins(instance);// Configure pins for SPI

	/* SPI use interrupt, must be installed in MQX and file fsl_dspi_irq.c must not be included in project */
	_int_install_isr(IRQNumber, (INT_ISR_FPTR)DSPI_DRV_MasterIRQHandler, (void*)instance);

	return 0;
}
dspi_status_t W25X40CL_init(uint32_t instance)
{
	dspi_status_t spiStatus;
	uint8_t 	sendBuffer[64] = {0};
//	uint8_t 	receiveBuffer[4] = {0};

	sendBuffer[0] = GPIO_DRV_ReadPinInput (FPGA_DONE);
	printf ("FPGA_DONE = %d\n", sendBuffer[0]);

	sendBuffer[0] = (uint8_t)RLPWD;//0xab - Release Power-down
	spiStatus = DSPI_DRV_MasterTransferBlocking(instance, NULL, sendBuffer, 0, 1, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
		printf ("updater: SPI RLPWD Error %d\n", spiStatus);
//		return spiStatus;
	}
	_time_delay(1);

	sendBuffer[0] = WREN;//0x06 - Write Enable
	spiStatus = DSPI_DRV_MasterTransferBlocking(instance, NULL, sendBuffer, 0, 1, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
		printf ("updater: SPI WREN Error %d\n", spiStatus);
		return spiStatus;
	}
//	sendBuffer[0] = WREVSR;//0x50 - Write Enable for Volatile Status Register;
//    spiStatus = DSPI_DRV_MasterTransferBlocking(instance, NULL, sendBuffer, 0, 1, SPI_TRANSFER_TIMEOUT);
//    if(spiStatus != kStatus_DSPI_Success)
//	{
//	 	printf ("updater: SPI WREVSR Error %d\n", spiStatus);
//	}
	return spiStatus;
}
int32_t fpga_init(void)
{
	if(kStatus_DSPI_Success == fpga_update_init(SPI_INSTANCE))
	{
		if(kStatus_DSPI_Success != W25X40CL_init(SPI_INSTANCE))
		{
			return -1;
		}
	}
	return kStatus_DSPI_Success;
}
bool is_fpga_flash_ready(void)
{
	dspi_status_t spiStatus;
	uint8_t bufferTx[3] = {0};
	uint8_t bufferRx[3] = {0};
	status_t status = {0};

	bufferTx[0] = RDSR;

	/* send command to read mem status register */
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, bufferTx, bufferRx, 3, SPI_TRANSFER_TIMEOUT);
	if(spiStatus == kStatus_DSPI_Success)
	{
	  status.as_uint8 = bufferRx[1];
		return (!status.BUSY);
	}

	return false;
}
bool is_fpga_flash_WEL(void)
{
	dspi_status_t spiStatus;
	uint8_t bufferTx[3] = {0};
	uint8_t bufferRx[3] = {0};
	status_t status = {0};

	bufferTx[0] = RDSR;

	/* send command to read mem status register */
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, bufferTx, bufferRx, 3, SPI_TRANSFER_TIMEOUT);
	if(spiStatus == kStatus_DSPI_Success)
	{
	  status.as_uint8 = bufferRx[1];
		return (!!status.WEL);
	}

	return false;
}
/*for debug - not checked
int32_t fpga_read_data(uint32_t addr, uint8_t* buf, uint32_t length)
{
	dspi_status_t spiStatus;
	uint8_t send_buffer[264] = {0};
	buf[0] = READ;

	fpga_addr_to_buffer(addr, &send_buffer[1]);

	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, send_buffer, buf, length, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
		printf ("SPI Error\n");
	}
	return spiStatus;
}
*/
int32_t fpga_erase_sector(uint32_t addr)
{
	dspi_status_t spiStatus;
	uint8_t send_buffer[8] = {0};


	while(!is_fpga_flash_ready());
	send_buffer[0] = WREN;//0x06 - Write Enable
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, send_buffer, 0, 1, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
	  printf ("updater: %s: SPI WREN Error %d\n", __func__, spiStatus);
		return spiStatus;
	}

	send_buffer[0] = SER;//0x20 - sector erase
	fpga_addr_to_buffer(addr, &(send_buffer[1]));
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, send_buffer, 0, 4, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
		printf("updater: %s err %d\n", __func__, spiStatus);
		return -1;
	}
	while(is_fpga_flash_WEL());

	return 0;
}
int32_t fpga_write_data(uint32_t addr, uint8_t* buf, uint32_t length)
{
	uint8_t send_buffer[2] = {0};
	dspi_status_t spiStatus;

	while(!is_fpga_flash_ready());
	send_buffer[0] = WREN;//0x06 - Write Enable
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, send_buffer, 0, 1, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
	  printf ("updater: %s: SPI WREN Error %d\n", __func__, spiStatus);
		return spiStatus;
	}

	buf[0] = PP;//0x2 - page program
	fpga_addr_to_buffer(addr, &(buf[1]));
	spiStatus = DSPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, buf, 0, 4 + length, SPI_TRANSFER_TIMEOUT);
	if(spiStatus != kStatus_DSPI_Success)
	{
		printf("updater: %s err %d\n", __func__, spiStatus);
		return -1;
	}
	while(is_fpga_flash_WEL());
	return 0;
}

void fpga_get_version(char* buf)
{
	uint32_t version = 0;
	if(FPGA_read_version(&version))
	{
		sprintf(buf, "%02X%02X%02X%02X", ((version >> 24) & 0xFF), ((version >> 16) & 0xFF), ((version >> 8) & 0xFF) , (version & 0xFF));
	}
	else
	{
		memset(buf, '0', 8);
		buf[8] = 0;
	}
}
void fpga_deinit(void)
{
	DSPI_DRV_MasterDeinit(SPI_INSTANCE);
	GPIO_DRV_SetPinOutput(FPGA_RSTB);
}
	/* Deinit the SPI */
/*    DSPI_DRV_MasterDeinit(SPI_INSTANCE);

	printf ("updater: \n-------------- End of example --------------\n\n");

GPIO_DRV_SetPinOutput (FPGA_RSTB);
FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); //Green LED
FPGA_write_led_status(LED_RIGHT, LED_DEFAULT_BRIGHTESS, 0, 0, 0xFF); //Blue LED
FPGA_write_led_status(LED_MIDDLE, LED_DEFAULT_BRIGHTESS, 0xFF, 0xFF, 0); // Yellow LED
   _task_block();
}
*/
/* EOF */
