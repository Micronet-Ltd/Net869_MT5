#include "board_type.h"
#include "fsl_adc16_driver.h"
#include "fsl_adc16_hal.h"
#include "gpio_pins.h"
#include "ADC.h"

#define CONFIG_FULL 			 'A'
#define CONFIG_WINDSHIELD_OBC	 'D' 	/* Orbcomm device */
#define CONFIG_FULL_SMART_CRADLE 'E'
#define CONFIG_UNDER_DASH_OBC 	 'F' 	/* Orbcomm device */
#define CONFIG_SMART_CRADLE 	 'G'
#define CONFIG_INVALID			 'Z'

#define ADC_VREF 			3300 /* mV */
#define ADC16_INSTANCE0 	0
#define ADC16_CHN_GROUP_0	0
#define DELTA 				40 /* mV */

static uint32_t board_rev = 0;
static char board_config = 'O';

uint32_t get_board_adc_value(adc16_chn_t channel){
	uint32_t raw_adc_value = 0;	
	uint32_t adc_value = 0;
	
	adc16_chn_config_t chn_config = {.chnIdx = channel, .convCompletedIntEnable = false, .diffConvEnable = false };
	
	ADC_Compare_disable (kADC_POWER_IN_ISR);

	ADC16_DRV_ConfigConvChn (ADC16_INSTANCE0, ADC16_CHN_GROUP_0, &chn_config);     /* trigger the conversion */
	ADC16_DRV_WaitConvDone  (ADC16_INSTANCE0, ADC16_CHN_GROUP_0);                  /* Wait for the conversion to be done */
	raw_adc_value = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE0, ADC16_CHN_GROUP_0); /* get  value for single ended channel */
	adc_value = (raw_adc_value * ADC_VREF)>>FSL_FEATURE_ADC16_MAX_RESOLUTION;
	
	ADC_Compare_enable (kADC_POWER_IN_ISR);
	
	return adc_value;
}

/* get_board_revision: returns the board number read via ADC0_DM1
*						Below Rev 6 = 0.30V to 0.36V (Floating)
*						Rev 6 = 1V
*						Rev 7 = 1.5V
*						Returns 0xff on no match
*/
uint8_t get_board_revision(void)
{
	uint32_t board_adc_val = 0;
	
	board_adc_val = get_board_adc_value(ADC_BOARD_VER);
	
	if (board_adc_val > 1500 && board_adc_val < 1800)  /* pin floating */
	{
		board_rev = 4; /* board V3 has the same value since the pin is also floating */
	}
	else if (((board_adc_val > (1000-DELTA)) && (board_adc_val < (1000+DELTA)))
			 || (board_adc_val > (100-DELTA)) && (board_adc_val < (100+DELTA))) /* special case for the first few REV6 boards that were produced */
	{
		board_rev = 6;
	}
	else if ((board_adc_val > (2000-DELTA)) && (board_adc_val < (2000+DELTA)))
	{
		board_rev = 7;	
	}
	else{
		board_rev = 0xff;	
	}
	return board_rev;
}

/* get_board_configuration: returns how the board has been populated(BOM)
*							A - FULL - 3.3V
*							D - WINDSHIELD OBC for Orbcomm - 0.30V to 0.36V (Floating)
*							E - FULL SMART CRADLE - 0.2V
*							F - UNDER DASH OBC for Orbcomm - 0.1V
*							G - SMART CRADLE - 0.3V
*							Z - no match/invalid ADC range
*/
char get_board_configuration(void)
{
	uint32_t board_adc_val = 0;
	
	uint8_t board_ver = get_board_revision();
	
	if (board_ver == 4) /* since board_ver has a floating ADC_BOARD_CONFIG pin, we automatically assume it is CONFIG_OBC5 */
	{
		board_config = CONFIG_WINDSHIELD_OBC;	
	}
	else
	{
		board_adc_val = get_board_adc_value(ADC_BOARD_CONFIG);

		if (board_adc_val < 50)
		{
			board_config = CONFIG_WINDSHIELD_OBC;
		}
		else if ((board_adc_val > (100-DELTA)) && (board_adc_val < (100+DELTA)))
		{
			board_config = CONFIG_UNDER_DASH_OBC;
		}
		else if ((board_adc_val > (200-DELTA)) && (board_adc_val < (200+DELTA)))
		{
			board_config = CONFIG_FULL_SMART_CRADLE;
		}
		else if ((board_adc_val > (300-DELTA)) && (board_adc_val < (300+DELTA)))
		{
			board_config = CONFIG_SMART_CRADLE;
		}
		else if ((board_adc_val > (3300-DELTA)) && (board_adc_val < (3300+DELTA)))
		{
			board_config = CONFIG_FULL;
		}
		else
		{
			board_config = CONFIG_INVALID;
		}
	}
	return board_config;
}

uint8_t get_saved_board_revision(void)
{
	if (board_rev == 0){
		get_board_revision();
	}
	return board_rev;
}

char get_saved_board_configuration(void)
{
	if (board_config == 'O'){
		get_board_configuration();	
	}
	return board_config;	
}

