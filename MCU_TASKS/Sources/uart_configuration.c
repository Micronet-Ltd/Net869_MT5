#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "uart_configuration.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void UART_DRV_IRQHandler(uint32_t instance);

static uart_state_t  uartState [UART_INSTANCE_COUNT];

void UART_Enable  (uint8_t port, const uart_user_config_t *uartConfig) 
{
	switch (port) {
		case UART0_IDX:	NVIC_SetPriority      (UART0_RX_TX_IRQn, 6U);
						OSA_InstallIntHandler (UART0_RX_TX_IRQn, MQX_UART0_RX_TX_IRQHandler);
						break;

		case UART1_IDX:	NVIC_SetPriority      (UART1_RX_TX_IRQn, 6U);
						OSA_InstallIntHandler (UART1_RX_TX_IRQn, MQX_UART1_RX_TX_IRQHandler);
						break;
						
		default:		printf("\nUART Enable - illeagal port\n");
						return;
	}
	
	UART_DRV_Init (port, &uartState[port], uartConfig);
	printf("\nUART %d Enabled\n", port);
}

void UART_Disable (uint8_t port) 
{
	UART_DRV_Deinit (port);
	printf("\nUART %d Disabled\n", port);
}

void UART_Reset  (uint8_t port, const uart_user_config_t *uartConfig)
{
	UART_Disable (port);
	UART_Enable  (port, uartConfig);
}

void MQX_UART0_RX_TX_IRQHandler (void)		{ UART_DRV_IRQHandler (0); }
void MQX_UART1_RX_TX_IRQHandler (void)		{ UART_DRV_IRQHandler (1); }
