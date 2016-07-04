#ifndef __spi_settings_h__
#define __spi_settings_h__
/*HEADER**********************************************************************
*
* Copyright 2015 Freescale Semiconductor, Inc.
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
*   This file contains definitions for the SPI example.
*
*
*END************************************************************************/


/* SPI instance, 2 for SPI2 */
#define SPI_INSTANCE    SPI2_IDX

/* Timeout in microseconds for calling SPI blocking transfer function */
#define SPI_TRANSFER_TIMEOUT    1000

/* Number of bytes used for addressing within memory */
#define SPI_ADDRESS_BYTES       3



void fpga_get_version(char* buf);
int32_t fpga_init(void);
void fpga_deinit(void);
bool is_fpga_flash_ready(void);
uint32_t crc_32(uint8_t *page, int32_t len);
int32_t fpga_erase_sector(uint32_t addr);
int32_t fpga_write_data(uint32_t addr, uint8_t* buf, uint32_t length);
#endif
