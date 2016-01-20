#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "device_control_gpio.h"
#include "wiggle_sensor.h"
#include "tasks_list.h"

#define POWER_MGM_TIME_DELAY		100

void Power_MGM_task (uint32_t initial_data )
{
	Wiggle_sensor_init       (POWER_MGM_TIME_DELAY, 0);
	Device_control_GPIO_init (POWER_MGM_TIME_DELAY   );

	printf("\nPower Management Task: Start \n");

	while ( 1 ) {
		// check amount of vibrations sensed by the wiggle sensor
		// if amount of vibrations is more than TH, it will turn on the device
		// and stop the interrupts for better running efficiency
		Wiggle_sensor_update ();
		if (Wiggle_sensor_cross_TH ()) {
			Device_turn_on ();
			Wiggle_sensor_stop ();
			_task_create(0, ACC_TASK, 0);
		}
		Device_control_GPIO ();

		_time_delay (POWER_MGM_TIME_DELAY);
	}
}
