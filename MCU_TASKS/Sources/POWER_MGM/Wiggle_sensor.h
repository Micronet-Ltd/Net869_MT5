#ifndef _WIGGLE_SENSOR_H
#define _WIGGLE_SENSOR_H

void Wiggle_sensor_init     (uint32_t delay_period, uint32_t vibration_threshold);
void Wiggle_sensor_start    (void);
void Wiggle_sensor_stop     (void);
bool Wiggle_sensor_cross_TH (void);
void Wiggle_sensor_update   (void);
void Wiggle_sensor_set_vibration_TH  (uint32_t vibration_threshold);

#endif /* _WIGGLE_SENSOR_H */