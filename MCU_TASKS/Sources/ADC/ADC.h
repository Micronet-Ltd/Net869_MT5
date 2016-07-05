#ifndef _OBC_ADC_H
#define _OBC_ADC_H

#include <stdio.h>

typedef enum {
	// TELEMETRY INPUTS (ANALOG INPUTS)
	kADC_ANALOG_IN1    = 0,
	kADC_GPIO_IN1      ,
	kADC_GPIO_IN2      ,
	kADC_GPIO_IN3      ,
	kADC_GPIO_IN4      ,
	kADC_GPIO_IN5      ,
	kADC_GPIO_IN6      ,
	kADC_GPIO_IN7      ,

	kADC_POWER_IN	   ,
	kADC_POWER_VCAP	   ,
	kADC_TEMPERATURE   ,
	kADC_CABLE_TYPE	   ,
    
    kADC_POWER_IN_ISR   ,
	kADC_CHANNELS
} KADC_CHANNELS_t;


#ifdef __cplusplus
extern "C"
{
#endif


void ADC_init (void);
void ADC_Set_IRQ_TH (KADC_CHANNELS_t channel, uint16_t low_thrashold, uint16_t high_thrashold);
void ADC_Compare_enable (KADC_CHANNELS_t channel);
void ADC_Compare_disable (KADC_CHANNELS_t channel);

void ADC_sample_input (KADC_CHANNELS_t channel);
uint32_t ADC_get_value (KADC_CHANNELS_t channel);


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _OBC_ADC_H */
	
