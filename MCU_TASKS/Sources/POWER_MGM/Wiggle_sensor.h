#ifndef _WIGGLE_SENSOR_H
#define _WIGGLE_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#define EVENT_WIGGLE_SENSOR_IRQ		(1 << 0)

extern void *g_WIGGLE_SENSOR_event_h;

void Wiggle_sensor_init     (uint32_t delay_period);
void Wiggle_sensor_restart  (void);
void Wiggle_sensor_start    (void);
void Wiggle_sensor_stop     (void);
bool Wiggle_sensor_cross_TH (void);
void Wiggle_sensor_update   (void);
void Wiggle_sensor_set_vibration_TH  (uint32_t vibration_threshold, uint32_t duration_threshold);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _WIGGLE_SENSOR_H */
