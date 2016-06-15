/*
 * UART.h
 *
 *  Created on: 29 בדצמ 2015
 *      Author: dovt
 */

#ifndef DRIVERS_INCLUDES_UPDATER_H_
#define DRIVERS_INCLUDES_UPDATER_H_

#include <stdint.h>
#include <stdbool.h>
#include <mqx.h>
#include <bsp.h>
#include <lwmsgq.h>

#define BAUD_115200 115200
#define NUM_MESSAGES  	1//2
#define UPD_MSG_SIZE   	8 //type + addr + size + 256 bytes + 1

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

extern uint32_t client_queue[];
extern void	set_result(uint32_t res);
  
#ifdef __cplusplus
extern "C"
}
#endif

#endif /* DRIVERS_INCLUDES_UPDATER_H_ */
