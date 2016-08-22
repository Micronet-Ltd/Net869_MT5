/*
 * watchdog_mgmt.h
 *
 *  Created on: July 7, 2016
 *      Author: abid.esmail
 */

#ifndef _WATCHDOG_WATCHDOG_MGMT_H_
#define _WATCHDOG_WATCHDOG_MGMT_H_

#include <mqx.h>
#include <watchdog.h>

#define WATCHDOG_MCU_MAX_TIME 10000//ms
#define WDG_RESET_MCU() WDOG_UNLOCK = 0xd928;  WDOG_UNLOCK = 0xc520;

#define WATCHDOG_A8_CPU_WATCHDOG_BIT (1 << 0)
#define WATCHDOG_A8_USB_PINGING_BIT (1 << 1)

typedef struct watchdog_a8_s
{
	MQX_TICK_STRUCT prev_ticks;
	MQX_TICK_STRUCT curr_ticks;
	uint32_t count;
}watchdog_a8_t;

void handle_mcu_watchdog_expiry(void *td_ptr);
bool watchdog_mcu_init(void);
void a8_watchdog_init(void);
void pet_a8_watchdog_isr(void);
void check_a8_watchdog_expiry_isr(void);

#endif /* _WATCHDOG_WATCHDOG_MGMT_H_ */