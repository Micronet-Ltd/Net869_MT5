/*
 * watchdog_mgmt.c
 *
 *  Created on: July 7, 2016
 *      Author: abid.esmail
 */
#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <lwtimer.h>
#include <event.h>
#include "watchdog_mgmt.h"

#define MS_PER_TICK	5
#define WATCHDOG_MCU_MAX_TICKS 10000/MS_PER_TICK /* 10000ms */
#define WATCHDOG_A8_MAX_TICKS 60000/MS_PER_TICK /* 60000ms */
#define WATCHDOG_A8_PET_TICKS 5000/MS_PER_TICK /* 5000ms */

static watchdog_a8_t watchdog_a8_g;
static LWTIMER_PERIOD_STRUCT lwtimer_period_a8_pet_g, lwtimer_period_a8_check_g;
static LWTIMER_STRUCT lwtimer_a8_pet_g, lwtimer_a8_check_g;
void *a8_watchdog_event_g;

static inline void delay_1s(void)
{
	unsigned long i = 0;
	for (i = 0; i< 20000000; i++)
	{
		__asm("nop");	
	}
}

/*FUNCTION*------------------------------------------------------
*
* Function Name  : handle_mcu_watchdog_expiry
* Returned Value : none
* Comments       :
*     This function is called when a watchdog has expired.
*END*-----------------------------------------------------------*/

/* only use this fx from hardfault handler because it resets FPGA */
static inline void blink_red_led(uint8_t count)
{
	uint8_t i = 0;
	GPIO_Type * gpioBasePortA = g_gpioBase[GPIOA_IDX];
	
	GPIO_HAL_SetPinOutput(gpioBasePortA, 11);	// turn on 5V0 power rail
	/* Shut off power to the accelerometer */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 19); //ACC_VIB_ENABLE

	/* Reset and Disable FPGA */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 12); //FPGA_RSTB
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 13); //FPGA_PWR_ENABLE

	/* Set all LEDs off */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 4); //LED_RED
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 7); //LED_BLUE
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 5); //LED_GREEN

	/* Blink LED */
	for (i = 0; i < count; i++)
	{
		delay_1s();
		GPIO_HAL_TogglePinOutput(gpioBasePortA, 4); //LED_RED
	}	
}


void handle_mcu_watchdog_expiry(void * td_ptr)
{
	printf("\r\n MCU Watchdog Expired, resetting MCU! \r\n");
	blink_red_led(10);
	/* on hard fault reset the system */
	WDG_RESET_MCU();
}

bool watchdog_mcu_init()
{
	_mqx_uint result;
	result = _watchdog_create_component(BSP_SYSTIMER_INTERRUPT_VECTOR,
                                       handle_mcu_watchdog_expiry);
    
	if (result != MQX_OK) 
	{
		printf("%s: Error creating watchdog component.", __func__);
		_task_block();
    }
	return TRUE;
}

void a8_watchdog_init(void)
{
	_event_create ("event.WATCHDOG");
	_event_open   ("event.WATCHDOG", &a8_watchdog_event_g);
	_lwtimer_create_periodic_queue(&lwtimer_period_a8_pet_g, WATCHDOG_A8_PET_TICKS, 0);
	_lwtimer_create_periodic_queue(&lwtimer_period_a8_check_g, WATCHDOG_A8_MAX_TICKS, 0);
	_lwtimer_add_timer_to_queue(&lwtimer_period_a8_pet_g, &lwtimer_a8_pet_g, 0, \
		(LWTIMER_ISR_FPTR)pet_a8_watchdog_isr, 0);
	_lwtimer_add_timer_to_queue(&lwtimer_period_a8_check_g, &lwtimer_a8_check_g, \
		0, (LWTIMER_ISR_FPTR)check_a8_watchdog_expiry_isr, 0);
	_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
	_time_get_elapsed_ticks(&watchdog_a8_g.curr_ticks);
}

/* pet_a8_watchdog_isr() is serviced every WATCHDOG_A8_PET_TIME */
void pet_a8_watchdog_isr(void)
{
	_mqx_uint event_bits;
	
	/* Only check the USB iodriver pings if the USB is in device mode */
	if (GPIO_DRV_ReadPinInput (OTG_ID) == 1)
	{
		/* Pet the A8 watchdog if cpu_status_pin is toggling and we are getting ping messages */
		if (_event_get_value(a8_watchdog_event_g, &event_bits) == MQX_OK) 
		{ 
			if (event_bits == (WATCHDOG_A8_USB_PINGING_BIT | WATCHDOG_A8_CPU_WATCHDOG_BIT))
			{
				_event_clear(a8_watchdog_event_g, WATCHDOG_A8_USB_PINGING_BIT | WATCHDOG_A8_CPU_WATCHDOG_BIT);
				_time_get_elapsed_ticks(&watchdog_a8_g.curr_ticks);
				watchdog_a8_g.count++;
			}
		}
	}
	else
	{
		_time_get_elapsed_ticks(&watchdog_a8_g.curr_ticks);
		watchdog_a8_g.count++;
	}
}

/* check_a8_watchdog_expiry_isr() is serviced every WATCHDOG_A8_CHECK_TICKS */
void check_a8_watchdog_expiry_isr(void)
{
	MQX_TICK_STRUCT time_diff_ticks;
	_mqx_uint res;

	res = _time_diff_ticks(&watchdog_a8_g.curr_ticks, 
		&watchdog_a8_g.prev_ticks, &time_diff_ticks);

	if (res == MQX_OK)
	{
		if (time_diff_ticks.TICKS[0] < WATCHDOG_A8_MAX_TICKS)
		{
			_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
		}
		else
		{
			_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
			printf("%s: A8 Watchdog Expired, resetting MCU! \r\n", __func__);
			handle_mcu_watchdog_expiry(NULL);
		}
	}
}