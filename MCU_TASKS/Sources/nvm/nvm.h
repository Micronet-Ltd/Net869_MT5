/*
 * NVM.h
 *
 *  Created on: Apr 9, 2015
 *      Author: esmail
 */

#ifndef NVM_H_
#define NVM_H_

#include "SSD_FTFx.h"

#define FLEX_RAM_START_ADDR 0x14000000
#define EEPROM32	0x39	  // 32 bytes for k20, 100MHz.
#define DFLASH_SIZE0 0x03	  // 224Kbytes for Data flash and 32kBytes for EEPROM backup from K20, 100Mhz RM (pg 641)

#define DEBUGENABLE               0x00

#define READ_NORMAL_MARGIN        0x00
#define READ_USER_MARGIN          0x01
#define READ_FACTORY_MARGIN       0x02

#define ONE_KB                    1024

#define FTFx_REG_BASE             0x40020000
#define P_FLASH_BASE              0x00000000

// Program Flash block information
#define P_FLASH_SIZE            (FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT)
#define P_BLOCK_NUM             FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT
#define P_SECTOR_SIZE           FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
// Data Flash block information
#define FLEXNVM_BASE            FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS
#define FLEXNVM_SECTOR_SIZE     FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE
#define FLEXNVM_BLOCK_SIZE      FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SIZE
#define FLEXNVM_BLOCK_NUM       FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_COUNT

// Flex Ram block information
#define EERAM_BASE              FSL_FEATURE_FLASH_FLEX_RAM_START_ADDRESS
#define EERAM_SIZE              FSL_FEATURE_FLASH_FLEX_RAM_SIZE

/* EEPROM size constants */
#define FTFL_PDD_EEPROM_DATA_SIZE_0_B     0xFU   /**< No EEPROM */
#define FTFL_PDD_EEPROM_DATA_SIZE_32_B    0x9U   /**< 32 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_64_B    0x8U   /**< 64 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_128_B   0x7U   /**< 128 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_256_B   0x6U   /**< 256 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_512_B   0x5U   /**< 512 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_1024_B  0x4U   /**< 1024 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_SIZE_2048_B       0x3U   /**< 2048 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_4096_B  0x2U   /**< 4096 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_8192_B  0x1U   /**< 8192 bytes of EEPROM data */
#define FTFL_PDD_EEPROM_DATA_SIZE_16384_B 0U     /**< 16384 bytes of EEPROM data */

void nvm_init(void);
void clearEEPROM(uint32_t size);
int partition_flash(int eeprom_size, int dflash_size);
void write8bitToEEPROM(uint32_t addrOffset, uint8_t * data);
void read8bitFromEEPROM(uint32_t addrOffset, uint8_t * data);
void write16bitToEEPROM(uint32_t addrOffset, uint16_t * data);
void write32bitToEEPROM(uint32_t addrOffset, uint32_t * data);
void read16bitFromEEPROM(uint32_t addrOffset, uint16_t * data);
void read32bitFromEEPROM(uint32_t addrOffset, uint32_t * data);
int writeBlockToEEPROM(uint32_t addrOffset, void * pData, uint16_t dataSize);
int readBlockFromEEPROM(uint32_t addrOffset, void * pData, uint16_t dataSize);

#endif /* NVM_H_ */
