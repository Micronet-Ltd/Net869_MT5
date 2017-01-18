/*
 * NVM.h
 *
 *  Created on: Apr 9, 2015
 *      Author: esmail
 */

#ifndef NVM_H_
#define NVM_H_

#include "SSD_FTFx.h"
#include "MK20D10_features.h"

#define FLEX_RAM_START_ADDR 0x14000000
#define DFLASH_SIZE_64K_EEPROM_SIZE_192K 0x12	  // 0b1100: 224Kbytes for Data flash and 32kBytes for EEPROM backup from K20, 100Mhz RM (pg 641)

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

#define FTFL_PDD_EEPROM_SPLIT_FACTOR_A_1_8 	0x00U  /* Subsystem A: EESIZE*1/8, Subsystem B: EESIZE*7/8 */
#define FTFL_PDD_EEPROM_SPLIT_FACTOR_A_1_4 	0x10U  /* Subsystem A: EESIZE*1/4, Subsystem B: EESIZE*3/4 */
#define FTFL_PDD_EEPROM_SPLIT_FACTOR_A_1_2 	0x20U  /* Subsystem A: EESIZE*1/2, Subsystem B: EESIZE*1/2 */
#define FTFL_PDD_EEPROM_SPLIT_FACTOR_B_1_2 	0x30U  /* Subsystem A: EESIZE*1/2, Subsystem B: EESIZE*1/2 */

void nvm_init(void);
void clear_EEPROM(uint32_t size);
int partition_flash(uint8_t eeprom_split_and_size, uint8_t dflash_size);
void write8bit_to_EEPROM(uint32_t addr_offset, uint8_t * data);
void read8bit_from_EEPROM(uint32_t addr_offset, uint8_t * data);
void write16bit_to_EEPROM(uint32_t addr_offset, uint16_t * data);
void write32bit_to_EEPROM(uint32_t addr_offset, uint32_t * data);
void read16bit_from_EEPROM(uint32_t addr_offset, uint16_t * data);
void read32bit_from_EEPROM(uint32_t addr_offset, uint32_t * data);
int write_block_to_EEPROM(uint32_t addr_offset, void * p_data, uint16_t data_size);
int read_block_from_EEPROM(uint32_t addr_offset, void * p_data, uint16_t data_size);

#endif /* NVM_H_ */
