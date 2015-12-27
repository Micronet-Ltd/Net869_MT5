#ifndef _MIC_UART_CONFIGURATION_H
#define _MIC_UART_CONFIGURATION_H

#include "fsl_uart_hal.h"
#include "fsl_uart_driver.h"

void UART_Enable   (uint8_t port, const uart_user_config_t *uartConfig);
void UART_Disable  (uint8_t port);
void UART_Reset    (uint8_t port, const uart_user_config_t *uartConfig);

extern void MQX_UART0_RX_TX_IRQHandler (void);
extern void MQX_UART1_RX_TX_IRQHandler (void);

#endif /* _MIC_UART_CONFIGURATION_H */

