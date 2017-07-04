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

#include <assert.h>
#include <string.h>
#include "fsl_uart_driver.h"
#include "fsl_interrupt_manager.h"
#include "nio_serial.h"
#include "updater.h"
#include "uart_configuration.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Pointer to uart runtime state structure */
extern void * g_uartStatePtr[UART_INSTANCE_COUNT];

#ifdef DEB_PRINT_FIRST_IRQS
uint8_t 	s1[16] 		= {0};
uint8_t 	rcfifo[16] 	= {0};
uint8_t 	sfifo[16] 	= {0};
uint8_t 	ch[16] 		= {0};
TIME_STRUCT tt[16] 		= {0};
#endif
/*FUNCTION**********************************************************************
 *
 * Function Name : UART_DRV_IRQHandler
 * Description   : Interrupt handler for UART.
 * This handler uses the buffers stored in the uart_state_t structs to transfer
 * data. This is not a public API as it is called whenever an interrupt occurs.
 *
 *END**************************************************************************/
void UART_DRV_IRQHandler_spec(uint32_t instance)
{
	uart_state_t * uartState = (uart_state_t *)g_uartStatePtr[instance];
	UART_Type * base = g_uartBase[instance];

#ifdef DEB_PRINT_FIRST_IRQS
	static uint32_t debug_cnt = 0;
#endif
	uint8_t irq_en_mask;
	uint8_t irq_stat, rx_stat, tx_stat;
	uint8_t rxofes;

	static uint32_t busy = 0;

	if (busy)
		return;
	INT_SYS_DisableIRQ(UART3_RX_TX_IRQn);
	busy = 1;

	irq_en_mask = (UART_C2_REG(base) & (UART_C2_RIE_MASK | UART_C2_TIE_MASK));
	irq_stat = irq_en_mask;

	rxofes = (UART_S1_REG(base) & (UART_S1_PF_MASK | UART_S1_FE_MASK | UART_S1_NF_MASK | UART_S1_OR_MASK)) |
				(UART_SFIFO_REG(base) & UART_SFIFO_RXOF_MASK);

#ifdef DEB_PRINT_FIRST_IRQS
	if(16 > debug_cnt)
	{
		_time_get(&tt[debug_cnt]);
		s1[debug_cnt] = UART_S1_REG(base);
		rcfifo[debug_cnt] = UART_RD_RCFIFO(base);
		sfifo[debug_cnt] = UART_RD_SFIFO(base);
	}
#endif
	while(rxofes)
	{
		UART_HAL_Getchar(base, uartState->rxBuff);
		UART_HAL_FlushRxFifo(base);
		if(UART_SFIFO_REG(base) & UART_SFIFO_RXOF_MASK)
			UART_SFIFO_REG(base) |= UART_SFIFO_RXOF_MASK;
		asm("nop");
		asm("nop");
		asm("nop");
		rxofes = (UART_S1_REG(base) & (UART_S1_PF_MASK | UART_S1_FE_MASK | UART_S1_NF_MASK | UART_S1_OR_MASK)) |
				(UART_SFIFO_REG(base) & UART_SFIFO_RXOF_MASK);
	}
	rx_stat = (UART_S1_REG(base) & UART_S1_RDRF_MASK);
//	tx_stat = (UART_S1_REG(base) & UART_S1_TDRE_MASK);
	irq_stat |= rx_stat; // | tx_stat;
	while(irq_stat)
	{
	/* Handle receive data register full interrupt, if rx data register full
	 * interrupt is enabled AND there is data available. */

//    if((UART_BRD_C2_RIE(base)) && (UART_BRD_S1_RDRF(base)))
		if (irq_stat & rx_stat)
		{
			/* Read out all data from RX FIFO */
			while(UART_HAL_GetRxDatawordCountInFifo(base))
			{
				/* Get data and put into receive buffer */
				UART_HAL_Getchar(base, uartState->rxBuff);
#ifdef DEB_PRINT_FIRST_IRQS
				if(16 > debug_cnt)
				{
					ch[debug_cnt] = *uartState->rxBuff;
				}
#endif
					//++uartState->rxBuff;
					//--uartState->rxSize;
					//++g_RxCounter;
				if (uartState->rxCallback != NULL)
				{
					uartState->rxCallback(instance, uartState);
				}
			}
		}

	/* Handle transmit data register empty interrupt, if tx data register empty
	 * interrupt is enabled AND tx data register is currently empty. */
//    if((UART_BRD_C2_TIE(base)) && (UART_BRD_S1_TDRE(base)))
/*		if (irq_stat & tx_stat)
		{
			// Check to see if there are any more bytes to send
			if (uartState->txSize)
			{
				uint8_t emptyEntryCountInFifo;
		#if FSL_FEATURE_UART_HAS_FIFO
				emptyEntryCountInFifo = uartState->txFifoEntryCount -
										UART_HAL_GetTxDatawordCountInFifo(base);
		#else
				emptyEntryCountInFifo = uartState->txFifoEntryCount;
		#endif
				while(emptyEntryCountInFifo--)
				{
					// Transmit data and update tx size/buff
					UART_HAL_Putchar(base, *(uartState->txBuff));

					// Invoke callback if there is one
					if (uartState->txCallback != NULL)
					{
					   // The callback MUST set the txSize to 0 if the
						//	 transmit is ended.
					   uartState->txCallback(instance, uartState);
					}
					else
					{
						++uartState->txBuff;
						--uartState->txSize;
					}

					// Check and see if this was the last byte
					if (uartState->txSize == 0U)
					{
						UART_DRV_CompleteSendData(instance);
						break;
					}
				}
			}
		}
*/
		rx_stat = (UART_S1_REG(base) & UART_S1_RDRF_MASK);
//		tx_stat = (UART_S1_REG(base) & UART_S1_TDRE_MASK);
		irq_stat &= ~(UART_C2_RIE_MASK);// | UART_C2_TIE_MASK);
		irq_stat |= rx_stat;//    | tx_stat;
//		rxofes = (UART_S1_REG(base) & (UART_S1_PF_MASK | UART_S1_FE_MASK | UART_S1_NF_MASK | UART_S1_OR_MASK));

	}//while any

#ifdef DEB_PRINT_FIRST_IRQS
	debug_cnt++;
#endif
	busy = 0;
	INT_SYS_EnableIRQ(UART3_RX_TX_IRQn);
}

int32_t	Set_IRQHandler_spec(void)
{
	uart_state_t * uartState = (uart_state_t *)g_uartStatePtr[UART_UPDATE_FW_IDX];

	INT_SYS_DisableIRQ(UART3_RX_TX_IRQn);
	OSA_InstallIntHandler (UART3_RX_TX_IRQn, MQX_UART3_RX_TX_IRQHandler);
//	uartState->rxCallback = &nio_serial_uart_rxcallback_spec;

	INT_SYS_EnableIRQ(UART3_RX_TX_IRQn);

	return 0;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
