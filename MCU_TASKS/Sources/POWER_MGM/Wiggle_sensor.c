#include <stdio.h>

#include "wiggle_sensor.h"

#define WIGGLE_SENSOR_SAMPLE_PERIOD		1000
#define WIGGLE_SENSOR_VIBRATION_TH		100

typedef struct {
	uint32_t vibration_threshold ;
	uint32_t delay_period        ;
	uint32_t time                ;
	bool     status              ;
	bool     enable              ;
} WIGGLE_SENSOR_t;

uint32_t wiggle_sensor_cnt = 0;

WIGGLE_SENSOR_t sensor;

void Wiggle_sensor_init (uint32_t delay_period, uint32_t vibration_threshold)
{
	wiggle_sensor_cnt           = 0;
	
	Wiggle_sensor_set_vibration_TH (vibration_threshold);
	sensor.delay_period        = delay_period;
	sensor.time                = 0;
	sensor.status              = false;
	sensor.enable              = true;
}

void Wiggle_sensor_start    (void) {sensor.enable = true ;}
void Wiggle_sensor_stop     (void) {sensor.enable = false;}
bool Wiggle_sensor_cross_TH (void) {return sensor.status ;}

void Wiggle_sensor_set_vibration_TH  (uint32_t vibration_threshold)
{
	if (vibration_threshold == 0)
		sensor.vibration_threshold = WIGGLE_SENSOR_VIBRATION_TH;
	else
		sensor.vibration_threshold = vibration_threshold;
}

void Wiggle_sensor_update (void)
{
	if (sensor.enable == false)
		return;
	
	if (sensor.time < WIGGLE_SENSOR_SAMPLE_PERIOD) {
		sensor.time  += sensor.delay_period;
		sensor.status = false;
		return;
	}
		
	sensor.status     = (wiggle_sensor_cnt > sensor.vibration_threshold);
	sensor.time       = 0;
	wiggle_sensor_cnt = 0;
}
