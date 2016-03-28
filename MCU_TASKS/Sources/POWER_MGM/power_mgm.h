#ifndef _POWER_MGM_H
#define _POWER_MGM_H

#ifdef __cplusplus
extern "C"
{
#endif

#define CABLE_TYPE_VOLTAGE					(3300 / 2)

#define POWER_IN_TURN_ON_TH					7200
#define POWER_IN_SHUTDOWN_TH				6000
#define POWER_IN_SUPERCAP_DISCHARGE_TH		POWER_IN_SHUTDOWN_TH

#define IGNITION_TURN_ON_TH_DEFAULT			POWER_IN_TURN_ON_TH

#define TEMPERATURE_MIN_TH					(-20)
#define TEMPERATURE_MAX_TH					(80)

void get_ignition_threshold(uint32_t * p_ignition_threshold);
void set_ignition_threshold(uint32_t ignition_threshold);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _POWER_MGM_H */ 
