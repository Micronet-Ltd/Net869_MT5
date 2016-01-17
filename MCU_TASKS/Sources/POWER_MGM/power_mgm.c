#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "device_control_gpio.h"
#include "wiggle_sensor.h"

#define POWER_MGM_TIME_DELAY		100

void Power_MGM_task (uint32_t initial_data )
{
	Wiggle_sensor_init       (POWER_MGM_TIME_DELAY, 0);
	Device_control_GPIO_init (POWER_MGM_TIME_DELAY   );

	printf("\nPower Managment Task: Start \n");

	while ( 1 ) {
		Wiggle_sensor_update ();
		if (Wiggle_sensor_cross_TH ())
			Device_turn_on ();

		Device_control_GPIO ();

		_time_delay (POWER_MGM_TIME_DELAY);
	}
}
