#ifndef _WIGGLE_SENSOR_H
#define _WIGGLE_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#define EVENT_WIGGLE_SENSOR_IRQ		(1 << 0)

typedef struct {
	uint32_t vibration_threshold ;
	uint32_t duration_threshold  ;
	uint32_t delay_period        ;
	uint32_t movement_period     ;
	uint32_t sample_period       ;
	volatile uint32_t wiggle_sensor_cnt   ;
	bool     status              ;
	bool     enable              ;
} WIGGLE_SENSOR_t;

extern void *g_WIGGLE_SENSOR_event_h;

void Wiggle_sensor_init     (uint32_t delay_period);
void Wiggle_sensor_restart  (void);
void Wiggle_sensor_start    (void);
void Wiggle_sensor_stop     (void);
bool Wiggle_sensor_cross_TH (void);
void Wiggle_sensor_update (uint32_t * time_diff);
void Wiggle_sensor_set_vibration_TH  (uint32_t vibration_threshold, uint32_t duration_threshold);
void Wiggle_sensor_get_vibration_TH  (uint32_t * vibration_threshold, uint32_t *duration_threshold);
uint32_t get_wiggle_sensor_count(void);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _WIGGLE_SENSOR_H */
