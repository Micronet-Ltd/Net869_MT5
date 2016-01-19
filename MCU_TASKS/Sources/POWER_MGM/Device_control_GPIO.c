#include <stdio.h>
#include <mqx.h>

#include "device_control_gpio.h"
#include "gpio_pins.h"

#define DEVICE_CONTROL_TIME_ON_TH		3000
#define DEVICE_CONTROL_TIME_OFF_TH		3000
#define DEVICE_CONTROL_TIME_RESET_TH	500

typedef struct {
	uint32_t time_threshold;
	uint32_t time;
	uint32_t delay_period;
	bool     enable;
} DEVICE_CONTROL_GPIO_t;

DEVICE_CONTROL_GPIO_t device_control_gpio;


void Device_control_GPIO_init (uint32_t delay_period)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio.delay_period   = delay_period;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = false;
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);
}

void Device_turn_on  (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_turn_off (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_OFF_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_reset (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_RESET_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_control_GPIO (void)
{
	if (device_control_gpio.enable == false)
		return;
		
	if (device_control_gpio.time <	device_control_gpio.time_threshold) {
		GPIO_DRV_SetPinOutput   (LED_BLUE);
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		device_control_gpio.time += device_control_gpio.delay_period;
	} else {
		GPIO_DRV_ClearPinOutput (LED_BLUE);
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
		device_control_gpio.time = 0;
		device_control_gpio.enable = false;		
	}
}
