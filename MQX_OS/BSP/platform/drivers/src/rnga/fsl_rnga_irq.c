/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include <stdint.h>
#include <stdbool.h>
#include "fsl_rnga_driver.h"
#if FSL_FEATURE_SOC_RNG_COUNT

/*******************************************************************************
 * Variables
 ******************************************************************************/
#define RNGA_INSTANCE      0U

/******************************************************************************
 * Code
 *****************************************************************************/
#if FSL_RTOS_MQX
	/* MQX RTOS require instalation of interrupt handlers by application. */
	void MQX_RNGA0_IRQHandler(void)
	{
		RNGA_DRV_IRQHandler(RNGA_INSTANCE);
	}
#else /* FSL_RTOS_MQX */
	#if defined(KM34Z7_SERIES)
	/* RNGA_IRQHandler IRQ handler that would cover the same name's APIs in startup code */
	void RNGA0_IRQHandler(void)
	{
		RNGA_DRV_IRQHandler(RNGA_INSTANCE);
	}
	#else
	/* RNGA_IRQHandler IRQ handler that would cover the same name's APIs in startup code */
	void RNG0_IRQHandler(void)
	{
		RNGA_DRV_IRQHandler(RNGA_INSTANCE);
	}
	#endif
#endif

#endif /* FSL_FEATURE_SOC_RNG_COUNT */
/******************************************************************************
 * EOF
 *****************************************************************************/
