#ifndef _DEVICE_CONTROL_GPIO_H
#define _DEVICE_CONTROL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif


extern void Device_turn_on      (void);
extern void Device_turn_off     (void);
extern void Device_reset        (void);
extern void Device_control_GPIO (void);

extern void Device_control_GPIO_init (uint32_t delay_period);

#ifdef __cplusplus
}
#endif

#endif /* _DEVICE_CONTROL_GPIO_H */
