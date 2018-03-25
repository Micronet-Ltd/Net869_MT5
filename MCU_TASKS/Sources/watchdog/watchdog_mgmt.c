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
#include "gpio_pins.h"
#include "board.h"
#include "fsl_wdog_driver.h"

//#define WATCHDOG_DEBUG

#define WATCHDOG_MCU_MAX_TICKS 10000/MS_PER_TICK /* 10000ms */
#define WATCHDOG_A8_MAX_TICKS 300000/MS_PER_TICK /* 5 minutes */
#define WATCHDOG_A8_CHECK_TICKS 30000/MS_PER_TICK /* 30 seconds */
#define WATCHDOG_A8_PET_TICKS 5000/MS_PER_TICK /* 5000ms */

static watchdog_a8_t watchdog_a8_g;
static LWTIMER_PERIOD_STRUCT lwtimer_period_a8_pet_g, lwtimer_period_a8_check_g;
static LWTIMER_STRUCT lwtimer_a8_pet_g, lwtimer_a8_check_g;
void *a8_watchdog_event_g;

static inline void delay_1s(void)
{
	unsigned long i = 0;
	/* change count based on clock frequency: Loop takes 3 cyles, but experimental results showed 6 worked better */
	unsigned long count = SystemCoreClock/6;
	for (i = 0; i< count; i++)
	{
		__asm("nop");	
	}
}

/*FUNCTION*------------------------------------------------------
*
* Function Name  : blink_led
* Arg 1		 : count: Number of times to blink LED
* Arg 2		 : color: LED_RED = 4, 
* Returned Value : NONE
* Comments       : Do not call this fx from a task as it has a delay loop
*     Blinks the LED, 
*END*-----------------------------------------------------------*/

/* only use this fx from hardfault handler because it resets FPGA */
static inline void blink_led(uint8_t count, uint8_t color)
{
	uint8_t i = 0;
	GPIO_Type * gpioBasePortA = g_gpioBase[GPIOA_IDX];

	GPIO_HAL_SetPinOutput(gpioBasePortA, 11);	// turn on 5V0 power rail(If off)

	/* Set all LEDs off */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, LED_RED_GPIO_NUM); //LED_RED
	GPIO_HAL_ClearPinOutput(gpioBasePortA, LED_BLUE_GPIO_NUM); //LED_BLUE
	GPIO_HAL_ClearPinOutput(gpioBasePortA, LED_GREEN_GPIO_NUM); //LED_GREEN

	/* Blink LED */
	for (i = 0; i < count; i++)
	{
		delay_1s();
		GPIO_HAL_TogglePinOutput(gpioBasePortA, color);
	}
}

void shutdown_fpga_accel(void)
{
	GPIO_Type * gpioBasePortA = g_gpioBase[GPIOA_IDX];
	
	GPIO_HAL_SetPinOutput(gpioBasePortA, 11);	// turn on 5V0 power rail
	/* Shut off power to the accelerometer */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 19); //ACC_VIB_ENABLE

	/* Reset and Disable FPGA */
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 12); //FPGA_RSTB
	GPIO_HAL_ClearPinOutput(gpioBasePortA, 13); //FPGA_PWR_ENABLE
}

void handle_watchdog_expiry(void * td_ptr)
{
	uint8_t i, color;
	
	/* if in Debug mode, do not reset the MCU */
	if (GPIO_DRV_ReadPinInput (OTG_ID) == 0)
	{
		printf("\r\n watchdog Expired, but NOT resetting because in debug mode! \r\n");
		_watchdog_start(WATCHDOG_MCU_MAX_TIME);
		return;
	}
	
	printf("\r\n watchdog Expired, resetting MCU! \r\n");
	shutdown_fpga_accel();

#ifdef WATCHDOG_DEBUG
	if (td_ptr == NULL)
	{
		blink_led(10, LED_RED_GPIO_NUM);
	}
	else
	{
		color = *((uint8_t * )(td_ptr));
		if (color == LED_RED_GPIO_NUM ||
			color == LED_BLUE_GPIO_NUM ||
			color == LED_GREEN_GPIO_NUM)
			blink_led(10, color);
		else
			blink_led(10, LED_RED_GPIO_NUM);
	}
#endif
	
	enable_msm_power(FALSE);		// turn off 5V0 power rail
	
	WDG_RESET_MCU();
}

bool watchdog_mcu_init()
{
	const wdog_config_t watchdog_config = {.wdogEnable = 1,
											.winEnable = 0,
											.workMode = 0,
											.updateEnable = 1,
											.timeoutValue = 20000}; //20 sec.
	return (kStatus_WDOG_Success == WDOG_DRV_Init(&watchdog_config));
}

bool watchdog_rtos_init()
{
	_mqx_uint result;
	result = _watchdog_create_component(BSP_SYSTIMER_INTERRUPT_VECTOR,
                                       handle_watchdog_expiry);
    
	if (result != MQX_OK) 
	{
		printf("%s: Error creating watchdog component.", __func__);
		_task_block();
    }
	return TRUE;
}

void a8_watchdog_init(void)
{
	if (_event_create ("event.WATCHDOG") != MQX_OK)
	{
		printf("%s: event.WATCHODOG create failed\n", __func__);
		_mqx_exit(0);
	}
	_event_open   ("event.WATCHDOG", &a8_watchdog_event_g);
	_lwtimer_create_periodic_queue(&lwtimer_period_a8_pet_g, WATCHDOG_A8_PET_TICKS, 0);
	_lwtimer_create_periodic_queue(&lwtimer_period_a8_check_g, WATCHDOG_A8_CHECK_TICKS, 0);
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
	int32_t time_diff_ticks;
	bool overflow;
	uint8_t color = LED_BLUE_GPIO_NUM;
	
	/* if we are getting pings, diff will be -ve, if not getting pings, diff will
	be positive and keep incrementing */
	time_diff_ticks = _time_diff_ticks_int32(&watchdog_a8_g.prev_ticks, 
		&watchdog_a8_g.curr_ticks, &overflow);
	
	/* we should not be getting an overflow, but in case it happens, just reset counts */
	if (overflow)
	{
		_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
		_time_get_elapsed_ticks(&watchdog_a8_g.curr_ticks);
		time_diff_ticks = 0;
	}
	
	if (time_diff_ticks < WATCHDOG_A8_MAX_TICKS)
	{
		_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
	}
	else
	{
		_time_get_elapsed_ticks(&watchdog_a8_g.prev_ticks);
		printf("%s: A8 Watchdog Expired, resetting MCU! \r\n", __func__);
		handle_watchdog_expiry(&color);
	}
}

void cancel_a8_timers(void)
{
	_lwtimer_cancel_timer(&lwtimer_a8_check_g);
	_lwtimer_cancel_period(&lwtimer_period_a8_check_g);
	_lwtimer_cancel_timer(&lwtimer_a8_pet_g);
	_lwtimer_cancel_period(&lwtimer_period_a8_pet_g);
}
