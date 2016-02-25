#ifndef _OBC_ADC_H
#define _OBC_ADC_H

#include <stdio.h>

typedef enum {
	kADC_POWER_IN	= 0,
	kADC_POWER_VCAP	   ,
	kADC_TEMPERATURE   ,
	kADC_CABLE_TYPE	   ,

	// TELEMETRY INPUTS (ANALOG INPUTS)
	kADC_ANALOG_IN1    ,
	kADC_GPIO_IN1      ,
	kADC_GPIO_IN2      ,
	kADC_GPIO_IN3      ,
	kADC_GPIO_IN4      ,
	kADC_GPIO_IN5      ,
	kADC_GPIO_IN6      ,
	kADC_GPIO_IN7      ,
	kADC_CHANNELS
} KADC_CHANNELS_t;


#ifdef __cplusplus
extern "C"
{
#endif


void ADC_init (void);
void ADC_sample_input (KADC_CHANNELS_t channel);
uint32_t ADC_get_value (KADC_CHANNELS_t channel);


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _OBC_ADC_H */
	
