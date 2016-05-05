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

#include "flash_funcs.h"
//#include <string.h>
#include <stdio.h>

FLASH_SSD_CONFIG flashSSDConfig =
{
    FTFx_REG_BASE,          /* FTFx control register base */
    P_FLASH_BASE, //PFLASH_BASE,      /* base address of PFlash block */
    P_FLASH_SIZE,//PBLOCK_SIZE,            /* size of PFlash block */
    FLEXNVM_BASE,     /* base address of DFlash block */
    0,				        /* size of DFlash block */
    EERAM_BASE,       		/* base address of EERAM block */
    0,                      /* size of EEE block */
    0,            			/* background debug mode enable bit */
    NULL_CALLBACK           /* pointer to callback function */
};

/////////
uint32_t EraseSectors(int32_t sid)
{
    uint32_t ret;          /* Return code from each SSD function */
    uint32_t destination;         /* Address of the target location */
    uint32_t size;

	switch(sid)
	{
		case FROM_P_FLASH:  
		{	destination = PFLASH_BASE;//flashSSDConfig.PFlashBase;
			size		= PBLOCK_SIZE;//flashSSDConfig.PFlashSize;
		}
		break;
		case FROM_FLEX_NVM:
		{
			destination = flashSSDConfig.DFlashBase;
			size = DEBLOCK_SIZE_M;	//flashSSDConfig.DFlashSize;
		}
		break;
//		case FROM_BOOT_REG:
//		{
//		  	destination = BOOT_REGISTERS_START_ADDR;
//			size 		= FLEXNVM_SECTOR_SIZE;//BOOT_REGISTERS_SIZE;
//		}
		break;
		default: 
		{
		  	printf("updater: EraseSection wrong parameter 0x%x\n", sid);
			return -1;
		}
	}

	ret = FlashEraseSector(&flashSSDConfig, destination, size, FlashCommandSequence);
//    if (FTFx_OK != ret)
//    {
//        ErrorTrap(ret);
//    }
	
	return ret;
}
uint32_t Flash_Init(void)
{
    CACHE_DISABLE
	  
	return	FlashInit(&flashSSDConfig);
}
uint32_t Flash_Program(uint32_t dest, uint32_t size, uint8_t* pData) 
{
   return FlashProgram(&flashSSDConfig, dest, size, pData, FlashCommandSequence);
}
uint32_t Update_Reg(int32_t sid)
{
	uint32_t ret;
	ret = EraseSectors(FROM_BOOT_REG);
	if(0 == ret)
	{
  		ret = Flash_Program(BOOT_REGISTERS_START_ADDR, 4, (uint8_t*)&sid);
	}
	return ret;
}
//////////
              
/* end of file */
