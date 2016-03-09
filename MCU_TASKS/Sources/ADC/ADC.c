
#include "ADC.h"
#include "gpio_pins.h"

#include "fsl_adc16_driver.h"
#include "fsl_adc16_hal.h"

#define ADC16_INSTANCE              1

#define ADC16_CHN_GROUP_0			0
#define ADC16_CHN_GROUP_1			1

#define ADC_VREF					3300

typedef struct {
	uint32_t				value;
	uint32_t				sample;
	uint32_t				factor_offset;
	uint32_t				factor_mul;
	uint32_t				factor_div;
	adc16_chn_config_t 		chnConfig;
} ADC_INTPUT_t;


#ifdef __cplusplus
extern "C"
{
#endif

void ADC_convert_sample_to_value (ADC_INTPUT_t *adc_intput);
void calibrateParams(void);

void ADC_channel_init (
		ADC_INTPUT_t *adc_intput,
		adc16_chn_t   adc_index,
		uint32_t      factor_offset,
		uint32_t      factor_mul,
		uint32_t      factor_div,
		bool          diff_mode);
		
ADC_INTPUT_t adc_input	[kADC_CHANNELS];
adc16_converter_config_t adcConfig;   // structure for user config

void ADC_init (void)
{
    // Initialization ADC for 16bit resolution.
    // interrupt mode and HW trigger disabled,
    // normal convert speed, VREFH/L as reference,
    // disable continuous convert mode.
    ADC16_DRV_StructInitUserConfigDefault (&adcConfig);    
    ADC16_DRV_Init (ADC16_INSTANCE, &adcConfig);

	// initalize all channels
	//                         channel                   index         factor  factor  factor    diff
	//                          name                     number        offset    mul     div     pair
	ADC_channel_init (&adc_input[kADC_POWER_IN   ], ADC_POWER_IN   ,     0,     105,      5,    false);
	ADC_channel_init (&adc_input[kADC_POWER_VCAP ], ADC_POWER_VCAP ,     0,       3,      1,    false);
	ADC_channel_init (&adc_input[kADC_TEMPERATURE], ADC_TEMPERATURE,   500,       1,     10,    false);
	ADC_channel_init (&adc_input[kADC_CABLE_TYPE ], ADC_CABLE_TYPE ,     0,       1,      1,    false);
	ADC_channel_init (&adc_input[kADC_ANALOG_IN1 ], ADC_ANALOG_IN1 ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN1   ], ADC_GPIO_IN1   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN2   ], ADC_GPIO_IN2   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN3   ], ADC_GPIO_IN3   ,     0,     110,     10,    true );
	ADC_channel_init (&adc_input[kADC_GPIO_IN4   ], ADC_GPIO_IN4   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN5   ], ADC_GPIO_IN5   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN6   ], ADC_GPIO_IN6   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN7   ], ADC_GPIO_IN7   ,     0,     110,     10,    false);

	calibrateParams ();
}

void ADC_sample_input (KADC_CHANNELS_t channel)
{
	int16_t sample1;
	uint16_t sample2;

	assert (channel < kADC_CHANNELS);

	ADC16_DRV_ConfigConvChn (ADC16_INSTANCE, ADC16_CHN_GROUP_0, &adc_input[channel].chnConfig);				// trigger the conversion
	ADC16_DRV_WaitConvDone  (ADC16_INSTANCE, ADC16_CHN_GROUP_0);												// Wait for the conversion to be done
	sample1 = ADC16_DRV_GetConvValueSigned (ADC16_INSTANCE, ADC16_CHN_GROUP_0); 								// get value

	if (adc_input[channel].chnConfig.diffConvEnable) {
		// in case of differential pair sample positive input and calculate the value of negative pad
		adc_input[channel].chnConfig.diffConvEnable = false;
		ADC16_DRV_ConfigConvChn (ADC16_INSTANCE, ADC16_CHN_GROUP_0, &adc_input[channel].chnConfig);			// trigger the conversion
		ADC16_DRV_WaitConvDone  (ADC16_INSTANCE, ADC16_CHN_GROUP_0);											// Wait for the conversion to be done
		sample2 = ADC16_DRV_GetConvValueSigned (ADC16_INSTANCE, ADC16_CHN_GROUP_0);								// get value
		sample1 = sample2 - (int32_t) (sample1 << 1) ;

		// the result can't be negative number
		if (sample1 < 0)
			sample1 = 0;
		adc_input[channel].chnConfig.diffConvEnable = true;
	}
	
	// KDS 3.0.0 has a compiler bug, where casting from int16_t to uint32_t is not implemented correctly.
	// the actual casting is from int16_t to int32_t, therefore a workaround is implemented
	sample2 = (uint16_t) sample1;
	adc_input[channel].sample = (uint32_t) sample2;

	ADC_convert_sample_to_value (&adc_input[channel]);
}	

	
void ADC_channel_init (
		ADC_INTPUT_t *adc_intput,
		adc16_chn_t   adc_index,
		uint32_t      factor_offset,
		uint32_t      factor_mul,
		uint32_t      factor_div,
		bool          diff_mode)
{
	adc_intput->value         = 0;
	adc_intput->sample        = 0;
	adc_intput->factor_offset = factor_offset;
	adc_intput->factor_mul    = factor_mul;
	adc_intput->factor_div    = factor_div;

	adc_intput->chnConfig.chnIdx                 = adc_index;
	adc_intput->chnConfig.diffConvEnable         = diff_mode;
	adc_intput->chnConfig.convCompletedIntEnable = false;
}

void ADC_convert_sample_to_value (ADC_INTPUT_t *adc_intput)
{
	// calculate ADC voltage level based on the sampled value
	adc_intput->value  = adc_intput->sample * ADC_VREF;
	adc_intput->value  = adc_intput->value >> FSL_FEATURE_ADC16_MAX_RESOLUTION;

	// calculate voltage before voltage divider
	adc_intput->value -= adc_intput->factor_offset;
	adc_intput->value *= adc_intput->factor_mul;
	adc_intput->value /= adc_intput->factor_div;	
}


void calibrateParams(void)
{
#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    adc16_hw_average_config_t userHwAverageConfig;
#endif

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration
    adc16_calibration_param_t adcCalibraitionParam;
    ADC16_DRV_GetAutoCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC16_INSTANCE, &adcCalibraitionParam);
#endif // FSL_FEATURE_ADC16_HAS_CALIBRATION.


#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
    // Use hardware average to increase stability of the measurement.
    userHwAverageConfig.hwAverageEnable = true;
    userHwAverageConfig.hwAverageCountMode = kAdc16HwAverageCountOf32;
    ADC16_DRV_ConfigHwAverage(ADC16_INSTANCE, &userHwAverageConfig);
#endif // FSL_FEATURE_ADC16_HAS_HW_AVERAGE

}

uint32_t ADC_get_value (KADC_CHANNELS_t channel)
{
	assert (channel < kADC_CHANNELS);
	return adc_input[channel].value;
}


#ifdef __cplusplus
extern "C"
}
#endif
