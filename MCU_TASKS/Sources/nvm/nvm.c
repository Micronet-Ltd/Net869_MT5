#include "nvm.h"


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


void clearEEPROM(uint32_t size)
{
	uint32_t addrOffset = 0;
	for (addrOffset = 0; addrOffset < size/4 ; addrOffset+=4)
	{
		*(uint32_t*)(FLEX_RAM_START_ADDR + addrOffset) = 0;
		while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
	}
}

int partition_flash(int eeprom_size, int dflash_size)
{
      /* Test to make sure the device is not already partitioned. If it
       * is already partitioned, then return with no action performed.
       */
      if ((SIM_FCFG1 & SIM_FCFG1_DEPART(0xF)) != 0x00000F00)
      {
	
    	  //printf("\nDevice is already partitioned.\n");
          return 0;
      }

      /* Write the FCCOB registers */
      FTFL_FCCOB0 = FTFL_FCCOB0_CCOBn(0x80); // Selects the PGMPART command
      FTFL_FCCOB1 = 0x00;
      FTFL_FCCOB2 = 0x00;
      FTFL_FCCOB3 = 0x00;

      /* FCCOB4 is written with the code for the subsystem sizes (eeprom_size define) */
      FTFL_FCCOB4 = eeprom_size;

      /* FFCOB5 is written with the code for the Dflash size (dflash_size define) */
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
void write8bitToEEPROM(uint32_t addrOffset, uint8_t * data)
{
	*(uint8_t*)(FLEX_RAM_START_ADDR + addrOffset) = *data;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn void read8bitFromEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read8bitFromEEPROM(uint32_t addrOffset, uint8_t * data)
{
	uint8_t * pData = (uint8_t*)(FLEX_RAM_START_ADDR + addrOffset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn uint8_t write16bitToEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief write to flex RAM by providing the address offset and pointer to data
    @return void
*/
void write16bitToEEPROM(uint32_t addrOffset, uint16_t * data)
{
	*(uint16_t*)(FLEX_RAM_START_ADDR + addrOffset) = *data;
	while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK)); // Wait for command completion
}

/*! @fn void read16bitFromEEPOM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read16bitFromEEPROM(uint32_t addrOffset, uint16_t * data)
{
	uint16_t * pData = (uint16_t*)(FLEX_RAM_START_ADDR + addrOffset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn uint8_t write32bitToEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief write to flex RAM by providing the address offset and pointer to data
    @return void
*/
void write32bitToEEPROM(uint32_t addrOffset, uint32_t * data)
{	
	*(uint32_t*)(FLEX_RAM_START_ADDR + addrOffset) = *data;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

/*! @fn void readFromEEPROM(uint32_t addr_offset, uint16_t * data)
    @brief read from flex RAM by providing the address offset and pointer where 
    the data is to be stored
    @return void
*/
void read32bitFromEEPROM(uint32_t addrOffset, uint32_t * data)
{
	uint32_t * pData = (uint32_t*)(FLEX_RAM_START_ADDR + addrOffset);
	*data = *pData;
	while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK)); // Wait for command completion
}

int writeBlockToEEPROM(uint32_t addrOffset, void * pData, uint16_t dataSize)
{
	int i = 0;

	if ((dataSize > 512) || (addrOffset > 512))
	{
		return -1;
	}
	else
	{
		for (i = 0; i < dataSize; i += 2 )
		{
			write16bitToEEPROM(addrOffset + i, (uint16_t *)(pData + i));
		}
	}
	return 0;
}

int readBlockFromEEPROM(uint32_t addrOffset, void * pData, uint16_t dataSize)
{
	int i = 0;

	if ((dataSize > 512) || (addrOffset > 512))
	{
		return -1;
	}
	else
	{
		for (i = 0; i < dataSize; i += 2 )
		{
			read16bitFromEEPROM(addrOffset + i, (uint16_t *)(pData + i));
		}
	}
	return 0;
}
