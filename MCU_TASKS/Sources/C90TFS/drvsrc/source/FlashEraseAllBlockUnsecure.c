/*****************************************************************************
 (c) Copyright 2015 Freescale Semiconductor, Inc.
 ALL RIGHTS RESERVED.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the <organization> nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************

*****************************************************************************
*                                                                           *
*        Standard Software Flash Driver For FTFx                            *
*                                                                           *
* FILE NAME     :  SSD_FTFx.h                                               *
* DATE          :  Jun 15, 2015                                             *
*                                                                           *
* AUTHOR        :  ROM Team                                                 *
* E-mail        :  b46522@freescale.com                                     *
*                                                                           *
*****************************************************************************/

/************************** CHANGES *************************************
1.3.GA      06.23.2015      ROM Team      First version of  FlashEraseAllBlockUnsecure()

*************************************************************************/

/* include the header files */
#include "SSD_FTFx.h"

/************************************************************************
*
*  Function Name    : FlashEraseAllBlockUnsecure.c
*  Description      : The Erase All Blocks Unsecure operation will erase all Flash
					  blocks, verify all memory contents, program security byte
					  to unsecure state, then release MCU security.
*  Arguments        : PFLASH_SSD_CONFIG, pFLASHCOMMANDSEQUENCE
*  Return Value     : uint32_t
*
*************************************************************************/

/* Enable size optimization */
#if(ARM_CORTEX_M != CPU_CORE)
#pragma optimize_for_size on
#pragma optimization_level 4
#endif /* End of CPU_CORE */

uint32_t SIZE_OPTIMIZATION FlashEraseAllBlockUnsecure (PFLASH_SSD_CONFIG pSSDConfig, \
					   pFLASHCOMMANDSEQUENCE pFlashCommandSequence)
{
	uint32_t ret;         /* return code variable */
	uint32_t temp;        /* temporary variable */

	/* clear RDCOLERR & ACCERR & FPVIOL flag in flash status register. Write 1 to clear*/
	temp = pSSDConfig->ftfxRegBase + FTFx_SSD_FSTAT_OFFSET;
	REG_WRITE(temp, FTFx_SSD_FSTAT_ERROR_BITS);

	/* passing parameter to the command */
	temp = pSSDConfig->ftfxRegBase + FTFx_SSD_FCCOB0_OFFSET;
	REG_WRITE(temp, FTFx_ERASE_ALL_BLOCK_UNSECURE);

	/* calling flash command sequence function to execute the command */

	ret = pFlashCommandSequence(pSSDConfig);

#if C90TFS_ENABLE_DEBUG
	/* Enter Debug state if enabled */
	if (TRUE == (pSSDConfig->DebugEnable))
	{
		ENTER_DEBUG_MODE;
	}
#endif

	return(ret);
}

/* End of file */
