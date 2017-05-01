#ifndef _POWER_MGM_H
#define _POWER_MGM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "fsl_power_manager.h"

#define CABLE_TYPE_VOLTAGE					(3300 / 2)

#define POWER_IN_TURN_ON_TH					7200
#define POWER_IN_SHUTDOWN_TH				7000//6000
#define POWER_IN_SUPERCAP_DISCHARGE_TH		POWER_IN_SHUTDOWN_TH
#define MCU_MIN_OPERATING_VOLTAGE           2000

#define IGNITION_TURN_ON_TH_DEFAULT			POWER_IN_TURN_ON_TH

/* Conversion from Temperature(c) to mV => 		mV = (temp * 10) + 500 */
#define TEMPERATURE_SHUTDOWN_MIN_TH		250			 /* mV = (-25 * 10) + 500 */
#define TEMPERATURE_SHUTDOWN_MAX_TH		1450		 /* mV = (95 * 10) + 500 */
#define TEMPERATURE_TURNON_MIN_TH		250			 /* mV = (-25 * 10) + 500 */
#define TEMPERATURE_TURNON_MAX_TH		1250		 /* mV = (75 * 10) + 500 */
	
#define EVENT_CPU_STATUS_HIGH 1
#define EVENT_CPU_STATUS_LOW 2

typedef struct tick_measure_s{
	MQX_TICK_STRUCT start_ticks;
	MQX_TICK_STRUCT end_ticks;
	int32_t time_diff; //ms
}tick_measure_t;

void get_ignition_threshold(uint32_t * p_ignition_threshold);
void set_ignition_threshold(uint32_t ignition_threshold);
void switch_power_mode(power_manager_modes_t mode);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _POWER_MGM_H */
