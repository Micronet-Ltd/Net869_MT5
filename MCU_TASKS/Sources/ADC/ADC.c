#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <lwmsgq.h>

#include "fsl_adc16_driver.h"
#include "fsl_adc16_hal.h"

#include "ADC.h"
#include "gpio_pins.h"

#include "mic_typedef.h"

#define ADC16_INSTANCE0              0
#define ADC16_INSTANCE1              1

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

typedef struct {
	uint32_t				min_value;
	uint32_t				max_value;
} ADC_IRQ_TH_t;

#ifdef __cplusplus
extern "C"
{
#endif

void ADC_convert_sample_to_value (ADC_INTPUT_t *adc_intput);
void calibrateParams(uint32_t adc16_interface);
void MQX_ADC0_IRQHandler (void);

void ADC_channel_init (
		ADC_INTPUT_t *adc_intput,
		adc16_chn_t   adc_index,
		uint32_t      factor_offset,
		uint32_t      factor_mul,
		uint32_t      factor_div,
		bool          diff_mode);

ADC_INTPUT_t adc_input	[kADC_CHANNELS];
ADC_IRQ_TH_t adc_irq_th;


void ADC_init (void)
{
	adc16_converter_config_t adc0_config;   // structure for user config
	adc16_converter_config_t adc1_config;   // structure for user config
	adc16_hw_average_config_t adc1_hw_aver_config;
	adc16_hw_average_config_t adc0_hw_aver_config;

	adc_irq_th.max_value = 0;
	adc_irq_th.min_value = 0;

	// Initialization ADC1 for 16bit resolution.
	// interrupt mode and HW trigger disabled,
	// high convert speed, VREFH/L as reference,
	// disable continuous convert mode.
	ADC16_DRV_StructInitUserConfigDefault (&adc1_config);
	adc1_config.longSampleTimeEnable = true;
	adc1_config.highSpeedEnable = true;
	adc1_config.clkDividerMode = kAdc16ClkDividerOf2;
	ADC16_DRV_Init (ADC16_INSTANCE1, &adc1_config);

	ADC16_DRV_StructInitUserConfigDefault (&adc0_config);
	adc0_config.longSampleTimeEnable = false;
	adc0_config.highSpeedEnable = true;
	adc0_config.continuousConvEnable = true;
	adc0_config.clkDividerMode = kAdc16ClkDividerOf2;
	ADC16_DRV_Init (ADC16_INSTANCE0, &adc0_config);

	// initalize all channels
	//                         channel                   index         factor  factor  factor    diff
	//                          name                     number        offset    mul     div     pair
	ADC_channel_init (&adc_input[kADC_ANALOG_IN1 ], ADC_ANALOG_IN1 ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN1   ], ADC_GPIO_IN1   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN2   ], ADC_GPIO_IN2   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN3   ], ADC_GPIO_IN3   ,     0,     110,     10,    true );
	ADC_channel_init (&adc_input[kADC_GPIO_IN4   ], ADC_GPIO_IN4   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN5   ], ADC_GPIO_IN5   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN6   ], ADC_GPIO_IN6   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_GPIO_IN7   ], ADC_GPIO_IN7   ,     0,     110,     10,    false);
	ADC_channel_init (&adc_input[kADC_POWER_IN   ], ADC_POWER_IN   ,     0,     105,      5,    false);
	ADC_channel_init (&adc_input[kADC_POWER_VCAP ], ADC_POWER_VCAP ,     0,       3,      1,    false);
	ADC_channel_init (&adc_input[kADC_TEMPERATURE], ADC_TEMPERATURE,     0,       1,      1,    false);
	ADC_channel_init (&adc_input[kADC_CABLE_TYPE ], ADC_CABLE_TYPE ,     0,       1,      1,    false);

	ADC_channel_init (&adc_input[kADC_POWER_IN_ISR],ADC_POWER_IN_ISR,    0,     105,      5,    false);

	NVIC_SetPriority(ADC0_IRQn, ADC_NVIC_IRQ_Priority);
	_int_install_kernel_isr(ADC0_IRQn, MQX_ADC0_IRQHandler);

	calibrateParams (ADC16_INSTANCE0);
	adc0_hw_aver_config.hwAverageEnable = false;
	adc0_hw_aver_config.hwAverageCountMode = kAdc16HwAverageCountOf4;
	ADC16_DRV_ConfigHwAverage(ADC16_INSTANCE0, &adc0_hw_aver_config);

	calibrateParams (ADC16_INSTANCE1);
	adc1_hw_aver_config.hwAverageEnable = true;
	adc1_hw_aver_config.hwAverageCountMode = kAdc16HwAverageCountOf16;
	ADC16_DRV_ConfigHwAverage(ADC16_INSTANCE1, &adc1_hw_aver_config);
}

void ADC_Set_IRQ_TH (KADC_CHANNELS_t channel, uint16_t low_threshold, uint16_t high_threshold)
{
	ADC_Type *base = g_adcBase[ADC16_INSTANCE0];
	uint32_t val;

	val  = low_threshold;
	val *= adc_input[channel].factor_div;
	val /= adc_input[channel].factor_mul;
	val += adc_input[channel].factor_offset;
	val  = val << FSL_FEATURE_ADC16_MAX_RESOLUTION;
	val /= ADC_VREF;
	adc_irq_th.min_value = val;


	val  = high_threshold;
	val *= adc_input[channel].factor_div;
	val /= adc_input[channel].factor_mul;
	val += adc_input[channel].factor_offset;
	val  = val << FSL_FEATURE_ADC16_MAX_RESOLUTION;
	val /= ADC_VREF;
	adc_irq_th.max_value = val;

	ADC_WR_CV1(base, adc_irq_th.min_value);
	ADC_WR_SC2_ACFGT (base, 0);
}

void ADC_Compare_enable (KADC_CHANNELS_t channel)
{
	ADC_Type *base = g_adcBase[ADC16_INSTANCE0];

	ADC_WR_SC2_ACFE  (base, 1);
	ADC_WR_SC1_AIEN (base, ADC16_CHN_GROUP_0, 1);

	// trigger the conversion with IRQ enable
	adc_input[channel].chnConfig.convCompletedIntEnable = true;
	ADC16_DRV_ConfigConvChn (ADC16_INSTANCE0, ADC16_CHN_GROUP_0, &adc_input[channel].chnConfig);
}

void ADC_Compare_disable (KADC_CHANNELS_t channel)
{
	ADC_Type *base = g_adcBase[ADC16_INSTANCE0];

	ADC_WR_SC2_ACFE (base, 0);
	ADC_WR_SC1_AIEN (base, ADC16_CHN_GROUP_0, 0);
	adc_input[channel].chnConfig.convCompletedIntEnable = false;
}

void ADC_sample_input (KADC_CHANNELS_t channel)
{
	   assert (channel < kADC_CHANNELS);

	   ADC16_DRV_ConfigConvChn (ADC16_INSTANCE1, ADC16_CHN_GROUP_0, &adc_input[channel].chnConfig);         // trigger the conversion
	   ADC16_DRV_WaitConvDone  (ADC16_INSTANCE1, ADC16_CHN_GROUP_0);                                        // Wait for the conversion to be done

	   if (adc_input[channel].chnConfig.diffConvEnable)
	   {
			  // in case of differential channel read differential value represented in signed 16bit.
			  // In order to convert it to signed 32 bit range, the value needs to be multiply by 2
			  int32_t   sample_diff_chn = (int32_t) (ADC16_DRV_GetConvValueSigned (ADC16_INSTANCE1, ADC16_CHN_GROUP_0) << 1);
			  uint32_t  sample_pos_chn;

			  // sample positive input and calculate the value of negative pad
			  adc_input[channel].chnConfig.diffConvEnable = false;
			  ADC16_DRV_ConfigConvChn (ADC16_INSTANCE1, ADC16_CHN_GROUP_0, &adc_input[channel].chnConfig);   // trigger the conversion
			  ADC16_DRV_WaitConvDone  (ADC16_INSTANCE1, ADC16_CHN_GROUP_0);                                  // Wait for the conversion to be done
			  adc_input[channel].chnConfig.diffConvEnable = true;

			  sample_pos_chn = ADC16_DRV_GetConvValueRAW (ADC16_INSTANCE1, ADC16_CHN_GROUP_0);               // get value of positive channel
			  adc_input[channel].sample = sample_pos_chn - sample_diff_chn;                                 // calculate differential value
	   }
	   else
	   {
			  adc_input[channel].sample = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE1, ADC16_CHN_GROUP_0);     // get  value for single ended channel
	   }

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


void calibrateParams(uint32_t adc16_interface)
{
#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
	adc16_hw_average_config_t userHwAverageConfig;
#endif

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
	// Auto calibration
	adc16_calibration_param_t adcCalibraitionParam;
	ADC16_DRV_GetAutoCalibrationParam(adc16_interface, &adcCalibraitionParam);
	ADC16_DRV_SetCalibrationParam(adc16_interface, &adcCalibraitionParam);
#endif // FSL_FEATURE_ADC16_HAS_CALIBRATION.


#if FSL_FEATURE_ADC16_HAS_HW_AVERAGE
	// Use hardware average to increase stability of the measurement.
	userHwAverageConfig.hwAverageEnable = true;
	userHwAverageConfig.hwAverageCountMode = kAdc16HwAverageCountOf32;
	ADC16_DRV_ConfigHwAverage(adc16_interface, &userHwAverageConfig);
#endif // FSL_FEATURE_ADC16_HAS_HW_AVERAGE

}

uint32_t ADC_get_value (KADC_CHANNELS_t channel)
{
	assert (channel < kADC_CHANNELS);
	return adc_input[channel].value;
}

void MQX_ADC0_IRQHandler (void)
{
	ADC_Type *base  = g_adcBase[ADC16_INSTANCE0];
	uint32_t sample;

	GPIO_DRV_SetPinOutput (POWER_DISCHARGE_ENABLE);
	sample = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE0, ADC16_CHN_GROUP_0);     // get  value for single ended channel
	adc_input[kADC_POWER_IN].sample = sample;
	ADC_convert_sample_to_value (&adc_input[kADC_POWER_IN]);
	if (sample < adc_irq_th.min_value)
	{
		GPIO_DRV_SetPinOutput (POWER_DISCHARGE_ENABLE);
		// this IRQ routine is called when POWER INPUT voltage level is less than threshold and enables supercap discharge
		ADC_WR_CV1(base, adc_irq_th.max_value);
		ADC_WR_SC2_ACFGT (base, 1);
	}
	else
	{
		GPIO_DRV_ClearPinOutput (POWER_DISCHARGE_ENABLE);
		ADC_WR_CV1(base, adc_irq_th.min_value);
		ADC_WR_SC2_ACFGT (base, 0);
	}
	ADC_WR_SC1_AIEN  (base, ADC16_CHN_GROUP_0, 1);
}

#ifdef __cplusplus
extern "C"
}
#endif
