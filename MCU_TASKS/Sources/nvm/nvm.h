/*
 * NVM.h
 *
 *  Created on: Apr 9, 2015
 *      Author: esmail
 */

#ifndef NVM_H_
#define NVM_H_

#include "Cpu.h"

#define FLEX_RAM_START_ADDR 0x14000000
#define EEPROM32	0x39	  // 32 bytse for k2072 mHz.
#define DFLASH_SIZE0 0x03	  // 0  Kbyte  for D flash for K2072Mhz

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
