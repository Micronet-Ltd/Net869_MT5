#include <mqx.h>
#include <bsp.h>
#include <stdio.h>
#include "nvm.h"
#include "nvm_configs.h"

#include "SSD_Types.h"

//// Flash Standard Software Driver Structure.
FLASH_SSD_CONFIG flashSSDConfig1 =
{
    FTFx_REG_BASE,          /*! FTFx control register base */
    P_FLASH_BASE,           /*! Base address of PFlash block */
    P_FLASH_SIZE,           /*! Size of PFlash block */
    FLEXNVM_BASE,           /*! Base address of DFlash block */
	FLEXNVM_BLOCK_SIZE * FLEXNVM_BLOCK_NUM,    /*! Size of DFlash block */
    EERAM_BASE,             /*! Base address of EERAM block */
	EERAM_SIZE,                      /*! Size of EEE block */
    DEBUGENABLE,            /*! Background debug mode enable bit */
    NULL_CALLBACK           /*! Pointer to callback function */
};

/*
* @brief Gets called when an error occurs.
* Print error message and trap forever.
*/
void error_trap(void)
{
    printf("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {}
}

void test_serial(void)
{
	char ser[10] = "abcdefghij";
	set_serial_num_NVM(ser);
}

void nvm_init_test(void)
{
	uint32_t result;               /*! Return code from each SSD function */
	uint8_t  securityStatus = 0;   /*! Return protection status */
	// Setup flash SSD structure for device and initialize variables.
	result = FlashInit(&flashSSDConfig1);
	if (FTFx_OK != result)
	{
		error_trap();
	}

	 // print welcome message
	printf("\r\n Flash Example Start \r\n");
	// Print flash information - PFlash.
	printf("\r\n Flash Information: ");
	printf("\r\n Total Flash Size:\t%d KB, Hex: (0x%x)", (P_FLASH_SIZE/ONE_KB), P_FLASH_SIZE);
	//printf("\r\n Flash Sector Size:\t%d KB, Hex: (0x%x) ", (FTFx_PSECTOR_SIZE/ONE_KB), FTFx_PSECTOR_SIZE);
	// Check if DFlash exist on this device.
	if (flashSSDConfig1.DFlashSize)
	{
		printf("\r\n Data Flash Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig1.DFlashSize/ONE_KB), (unsigned int)flashSSDConfig1.DFlashSize);
		printf("\r\n Data Flash Base Address:\t0x%x", (unsigned int)flashSSDConfig1.DFlashBase);
	}
	else
	{
	  printf("\r\n There is no D-Flash (FlexNVM) on this Device.");
	}
	// Check if FlexMemory exist on this device.
	if (flashSSDConfig1.EEESize)
	{
		printf("\r\n Enhanced EEPROM (EEE) Block Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig1.EEESize/ONE_KB), (unsigned int)flashSSDConfig1.EEESize);
		printf("\r\n Enhanced EEPROM (EEE) Base Address:\t0x%x", (unsigned int)flashSSDConfig1.EERAMBase);
	}
	else
	{
	  printf("\r\n There is no Enhanced EEPROM (EEE) on this Device.");
	}

	// Check security status.
	result = FlashGetSecurityState(&flashSSDConfig1, &securityStatus);
	if (FTFx_OK != result)
	{
		error_trap();
	}
	// Print security status.
	switch(securityStatus)
	{
		case FLASH_NOT_SECURE:
			printf("\r\n Flash is UNSECURE!");
			break;
		case FLASH_SECURE_BACKDOOR_ENABLED:
			printf("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
			break;
		case FLASH_SECURE_BACKDOOR_DISABLED:
			printf("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
			break;
		default:
			break;
	}
	printf("\r\n");
	char serial2[10];
//	if (partition_flash(0x30 | FTFL_PDD_EEPROM_DATA_SIZE_512_B, DFLASH_SIZE0)) //K20 Ref Manual pg 636
//	{
//		clearEEPROM(NVM_SIZE);
//		writeFullNVMStruct(0);
//	}
	
	get_serial_num_NVM(serial2);
	printf("serial %s", serial2);
	//test_serial();
	//readFullNVMStruct();
	
	//test_serial();
	//readFullNVMStruct();
	

//	if (partition_flash(0x30 | FTFL_PDD_EEPROM_DATA_SIZE_512_B, DFLASH_SIZE0)) //K20 Ref Manual pg 636
//	{
//		clearEEPROM(NVM_SIZE);
//		writeFullNVMStruct(0);
//	}
}

void nvm_init(void)
{
	if (partition_flash(FTFL_PDD_EEPROM_SPLIT_FACTOR_B_1_2 | FTFL_PDD_EEPROM_SIZE_2048_B, DFLASH_SIZE_64K_EEPROM_SIZE_192K)) //K20 Ref Manual pg 636
	{
		clear_EEPROM(NVM_SIZE);
		write_full_NVM_struct(0);
	}
	nvm_config_init();
}

void clear_EEPROM(uint32_t size)
{
	uint32_t addr_offset = 0;
	for (addr_offset = 0; addr_offset < size/4 ; addr_offset+=4)
	{
		*(uint32_t*)(FLEX_RAM_START_ADDR + addr_offset) = 0;
		while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
	}
}

/********************************************************************/
/* Partition flash routine. This function can be used to setup
 * the flash for enhanced EEPROM operation. In order to guarantee
 * the eEE endurance the partition command should only be used one
 * time (re-partitioning in a different configuration will erase
 * wear-leveling data, so endurance can no longer be guaranteed).
 * This function will test to make sure the flash has not been
 * partitioned already.
 *
 * Parameters:
 * eeprom_size size of the two EEPROM data sets (A and B) defines in flexmem_demo.h
 * dlfash_size amount of dflash memory available after partitioning defines in flexmem_demo.h
 *
 * Returns:
 * 1  partitioning completed successfully
 * 0  partitioning not completed (device is already partitioned)
 */
int partition_flash(uint8_t eeprom_split_and_size, uint8_t dflash_size)
{
      /* Test to make sure the device is not already partitioned. If it
       * is already partitioned, then return with no action performed.
       */
      if ((SIM_FCFG1 & SIM_FCFG1_DEPART(0xF)) != 0x00000F00)
      {
	
    	  //printf("\nDevice is already partitioned.\n");
          //return 0;
      }

      /* Write the FCCOB registers */
      FTFL_FCCOB0 = FTFL_FCCOB0_CCOBn(0x80); // Selects the PGMPART command
      FTFL_FCCOB1 = 0x00;
      FTFL_FCCOB2 = 0x00;
      FTFL_FCCOB3 = 0x00;

      /* FCCOB4 is written with the code for the subsystem sizes (eeprom_size define) */
      //FTFL_FCCOB4 = ((0xC0) | eeprom_split_and_size);
	  FTFL_FCCOB4 = eeprom_split_and_size;

      /* FFCOB5 is written with the code for the Dflash size (dflash_size define) */
      //FTFL_FCCOB5 = (0xF0) | dflash_size;
	  FTFL_FCCOB5 = dflash_size;

      /* All required FCCOBx registers are written, so launch the command */
      FTFL_FSTAT = FTFL_FSTAT_CCIF_MASK;

      /* Wait for the command to complete */
      while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK));

      return 1;
}


/*! @fn uint8_t write8bitToEEPROM(uint32_t addr_offset, uint8_t * data)
    @brief write to flex RAM by providing the address offset and pointer to data
    @return void
*/
void write8bit_to_EEPROM(uint32_t addr_offset, uint8_t * data)
{
	*(uint8_t*)(FLEX_RAM_START_ADDR + addr_offset) = *data;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn void read8bitFromEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read8bit_from_EEPROM(uint32_t addr_offset, uint8_t * data)
{
	uint8_t * pData = (uint8_t*)(FLEX_RAM_START_ADDR + addr_offset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn uint8_t write16bitToEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief write to flex RAM by providing the address offset and pointer to data
    @return void
*/
void write16bit_to_EEPROM(uint32_t addr_offset, uint16_t * data)
{
	*(uint16_t*)(FLEX_RAM_START_ADDR + addr_offset) = *data;
	while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK)); // Wait for command completion
}

/*! @fn void read16bitFromEEPOM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read16bit_from_EEPROM(uint32_t addr_offset, uint16_t * data)
{
	uint16_t * pData = (uint16_t*)(FLEX_RAM_START_ADDR + addr_offset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn uint8_t write32bitToEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief write to flex RAM by providing the address offset and pointer to data
    @return void
*/
void write32bit_to_EEPROM(uint32_t addr_offset, uint32_t * data)
{	
	*(uint32_t*)(FLEX_RAM_START_ADDR + addr_offset) = *data;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn void readFromEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read32bit_from_EEPROM(uint32_t addr_offset, uint32_t * data)
{
	uint32_t * pData = (uint32_t*)(FLEX_RAM_START_ADDR + addr_offset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

int write_block_to_EEPROM(uint32_t addr_offset, void * pdata, uint16_t dataSize)
{
	uint16_t i = 0;
	uint16_t * u16_pdata = (uint16_t *) pdata;

	if ((dataSize > NVM_SIZE) || (addr_offset > NVM_SIZE))
	{
		return -1;
	}
	else
	{
		for (i = 0; i < dataSize; i += 2 )
		{
			//write16bitToEEPROM(addr_offset + i, (uint16_t *)(pData + i));
			/* I needed to do u16_pData + i/2 because the pointer was being incremented by 4 instead of 2  - Abid*/
			write16bit_to_EEPROM((addr_offset + i), (u16_pdata + i/2));
		}
	}
	return 0;
}

int read_block_from_EEPROM(uint32_t addr_offset, void * pdata, uint16_t dataSize)
{
	uint16_t i = 0;
	uint16_t * u16_pdata = (uint16_t *) pdata;

	if ((dataSize > NVM_SIZE) || (addr_offset > NVM_SIZE))
	{
		return -1;
	}
	else
	{
		for (i = 0; i < dataSize; i += 2 )
		{
			/* I needed to do u16_pData + i/2 because the pointer was being incremented by 4 instead of 2  - Abid*/
			read16bit_from_EEPROM(addr_offset + i, u16_pdata + i/2);
		}
	}
	return 0;
}
