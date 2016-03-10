#ifndef _DEVICE_CONTROL_GPIO_H
#define _DEVICE_CONTROL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

typedef enum {
	DEVICE_STATE_OFF = 0,
	DEVICE_STATE_TURNING_ON,
	DEVICE_STATE_ON,
	DEVICE_STATE_BACKUP_RECOVERY,
	DEVICE_STATE_BACKUP_POWER,
	DEVICE_STATE_TURN_OFF,
	DEVICE_STATE_RESET,
} DEVICE_STATE_t;

#define POWER_MGM_DEVICE_ON_IGNITION_TRIGGER	(1 << 0)
#define POWER_MGM_DEVICE_ON_WIGGLE_TRIGGER		(1 << 1)

void Device_init         (uint32_t delay_period);
void Device_update_state (void);
void Device_turn_on      (void);
void Device_turn_off     (void);
void Device_reset        (void);

DEVICE_STATE_t Device_get_status   (void);

#ifdef __cplusplus
}
#endif

#endif /* _DEVICE_CONTROL_GPIO_H */
