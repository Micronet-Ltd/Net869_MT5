/**************************************************************************************
* The wiggle sensor is a Tilt sensor that generates a normally closed switch which    *
* chatters open and closed as it is tilted or vibrated. When at rest, it normally     *
* settles in a closed state (however, this is not guaranteed). When in motion, it     *
* will produce continuous on/off contact, and therefore the function looks for edge   *
* edge transitions rather than an open or closed state of the switch.                 *
*                                                                                     *
* The function implements the following algorithm as recommended by the manufacturer: *
* On interrupt signal                                                                 *
* 	Disable further interrupts for ½ second                                           *
* 	If Pulse_Count < (Max_Count – Inc_val )                                           *
* 		Pulse_Count = Pulse_Count + Inc_val                                           *
*                                                                                     *
* Do every ½ second                                                                   *
* 	Re-enable interrupts                                                              *
* 	If (Pulse_Count > Dec_val)                                                        *
* 		Pulse_Count = Pulse_Count – Dec_val                                           *
*                                                                                     *
* 	If (Pulse_Count > Thresh)                                                         *
* 		Trigger = True                                                                *
* 	else                                                                              *
* 		Trigger = False                                                               *
*                                                                                     *
* number of vibrations and motion period are configurable via API                     *
**************************************************************************************/

#include <stdio.h>
#include <gpio_pins.h>
#include "fsl_port_hal.h"

#include "wiggle_sensor.h"

#define WIGGLE_SENSOR_SAMPLE_PERIOD					500

#define WIGGLE_SENSOR_MOTION_DETECTION_PERIOD_TH	5000
#define WIGGLE_SENSOR_VIBRATION_TH					25

#define WIGGLE_SENSOR_MAX_VALUE						50
#define WIGGLE_SENSOR_INC_VALUE						5
#define WIGGLE_SENSOR_DEC_VALUE						1


typedef struct {
	uint32_t vibration_threshold ;
	uint32_t duration_threshold  ;
	uint32_t delay_period        ;
	uint32_t movement_period     ;
	uint32_t sample_period       ;
	uint32_t wiggle_sensor_cnt   ;
	bool     status              ;
//	bool     enable              ;
} WIGGLE_SENSOR_t;


void *g_WIGGLE_SENSOR_event_h = NULL;


WIGGLE_SENSOR_t sensor;

void Wiggle_sensor_init (uint32_t delay_period)
{
	if (g_WIGGLE_SENSOR_event_h == NULL) {
		_event_create ("event.WIGGLE_SENSOR");
		_event_open   ("event.WIGGLE_SENSOR", &g_WIGGLE_SENSOR_event_h);
	}

	sensor.delay_period = delay_period;
	Wiggle_sensor_set_vibration_TH (0, 0);
	Wiggle_sensor_restart ();
}

void Wiggle_sensor_restart (void)
{
	sensor.movement_period     = 0;
	sensor.sample_period       = 0;
	sensor.wiggle_sensor_cnt   = 0;
	sensor.status              = false;
	//	sensor.enable              = true;
}

void Wiggle_sensor_start (void)
{
    uint32_t port = GPIO_EXTRACT_PORT(VIB_SENS);
    uint32_t pin  = GPIO_EXTRACT_PIN(VIB_SENS);
    PORT_Type *portBase = g_portBase[port];

//	sensor.enable = true;
	PORT_HAL_SetPinIntMode (portBase, pin, kPortIntFallingEdge);
}

void Wiggle_sensor_stop (void)
{
    uint32_t port = GPIO_EXTRACT_PORT(VIB_SENS);
    uint32_t pin  = GPIO_EXTRACT_PIN(VIB_SENS);
    PORT_Type *portBase = g_portBase[port];

//	sensor.enable = false;
	PORT_HAL_SetPinIntMode (portBase, pin, kPortIntDisabled);
}

bool Wiggle_sensor_cross_TH (void)
{
	return sensor.status;
}

void Wiggle_sensor_set_vibration_TH  (uint32_t vibration_threshold, uint32_t duration_threshold)
{
	if (vibration_threshold == 0)
		sensor.vibration_threshold = WIGGLE_SENSOR_VIBRATION_TH;
	else
		sensor.vibration_threshold = vibration_threshold;


	if (duration_threshold == 0)
		sensor.duration_threshold = WIGGLE_SENSOR_MOTION_DETECTION_PERIOD_TH;
	else
		sensor.duration_threshold = duration_threshold;
}

void Wiggle_sensor_update (void)
{
#if 0
	sensor.status = false;

	if (sensor.enable == false)
		return;
	
	if (sensor.movement_period < sensor.duration_threshold) {
		sensor.movement_period  += sensor.delay_period;
		return;
	}
		
	sensor.status     = (wiggle_sensor_cnt > sensor.vibration_threshold);
	sensor.movement_period = 0;
	wiggle_sensor_cnt = 0;
#else
	uint32_t g_WIGGLE_SENSOR_event_bit;

	sensor.movement_period += sensor.delay_period;

	// check if interrupt occurred
	_event_get_value (g_WIGGLE_SENSOR_event_h, &g_WIGGLE_SENSOR_event_bit);
	if (g_WIGGLE_SENSOR_event_bit == EVENT_WIGGLE_SENSOR_IRQ) {
		_event_clear    (g_WIGGLE_SENSOR_event_h, EVENT_WIGGLE_SENSOR_IRQ);

		// reset sampling timer and by that disable interrupts for SAMPLE_PERIOD_TH
		sensor.sample_period = 0;

		if (sensor.wiggle_sensor_cnt < WIGGLE_SENSOR_MAX_VALUE - WIGGLE_SENSOR_INC_VALUE)
			sensor.wiggle_sensor_cnt += WIGGLE_SENSOR_INC_VALUE;
	}

	// wait 500 mili-seconds
	if (sensor.sample_period < WIGGLE_SENSOR_SAMPLE_PERIOD) {
		sensor.sample_period += sensor.delay_period;
		return;
	}

	// re-enable interrupts
	sensor.sample_period = 0;
	Wiggle_sensor_start ();

	// decrements counter when no motion (without underflow)
	// if counter reaches 0, clear main timer - no movement
	if (sensor.wiggle_sensor_cnt > WIGGLE_SENSOR_DEC_VALUE)
		sensor.wiggle_sensor_cnt -= WIGGLE_SENSOR_DEC_VALUE;
	else
		sensor.movement_period = 0;

	// if duration has passed, update status
	if (sensor.movement_period >= sensor.duration_threshold)
		sensor.status = (sensor.wiggle_sensor_cnt > sensor.vibration_threshold);
#endif
}
