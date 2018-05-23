#include "board_type.h"
#include "fsl_adc16_driver.h"
#include "fsl_adc16_hal.h"
#include "gpio_pins.h"

#define CONFIG_FULL 			 'A'
#define CONFIG_OBC5 			 'D' 	/* Orbcomm device */
#define CONFIG_FULL_SMART_CRADLE 'E'
#define CONFIG_UNDER_DASH_OBC 	 'F' 	/* Orbcomm device */
#define CONFIG_SMART_CRADLE 	 'G'
#define CONFIG_INVALID			 'Z'

#define ADC_VREF 			3300 /* mV */
#define ADC16_INSTANCE0 	0
#define ADC16_CHN_GROUP_0	0
#define DELTA 				30 /* mV */

uint32_t get_board_adc_value(adc16_chn_t channel){
	uint32_t raw_adc_value = 0;	
	uint32_t adc_value = 0;
	
	adc16_chn_config_t chn_config = {.chnIdx = channel, .convCompletedIntEnable = false, .diffConvEnable = false };

	ADC16_DRV_ConfigConvChn (ADC16_INSTANCE0, ADC16_CHN_GROUP_0, &chn_config);     /* trigger the conversion */
	ADC16_DRV_WaitConvDone  (ADC16_INSTANCE0, ADC16_CHN_GROUP_0);                  /* Wait for the conversion to be done */
	raw_adc_value = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE0, ADC16_CHN_GROUP_0); /* get  value for single ended channel */
	adc_value = (raw_adc_value * ADC_VREF)>>FSL_FEATURE_ADC16_MAX_RESOLUTION;
	return adc_value;
}

/* get_board_revision: returns the board number read via ADC0_DM1
*						Below Rev 6 = 0.30V to 0.36V (Floating)
*						Rev 6 = 0.1V
*						Rev 7 = 0.5V
*						Each multiple of 0.1 increments the revision 
*						Returns 0xff on no match
*/
int get_board_revision(void)
{
	uint32_t board_rev = 0;
	uint32_t board_adc_val = 0;
	
	board_adc_val = get_board_adc_value(ADC_BOARD_VER);
	
	if ((board_adc_val > (330-70)) && (board_adc_val < (330+70)))  /* Delta of 70 was determined through trial and error  - pin floating */
	{
		board_rev = 4; /* board V3 has the same value since the pin is also floating */
	}
	else if ((board_adc_val > (100-DELTA)) && (board_adc_val < (100+DELTA)))
	{
		board_rev = 6;
	}
	else if ((board_adc_val > (500-DELTA)) && (board_adc_val < (500+DELTA))) //TODO: verify when this board is produced
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
*							D - OBC5 for Orbcomm - 0.30V to 0.36V (Floating)
*							E - FULL SMART CRADLE - 0.2V
*							F - UNDER DASH OBC for Orbcomm - 0.1V
*							G - SMART CRADLE - 0.3V
*							Z - no match/invalid ADC range
*/
char get_board_configuration(void)
{
	char board_config;
	uint32_t board_adc_val = 0;
	
	board_adc_val = get_board_adc_value(ADC_BOARD_CONFIG);
	
	if ((board_adc_val > (3300-DELTA)) && (board_adc_val < (3300+DELTA)))
	{
		board_config = CONFIG_FULL;
	}
	else if ((board_adc_val > (330-70)) && (board_adc_val < (330+70))) /* Delta of 70 was determined through trial and error  - pin floating */
	{
		board_config = CONFIG_OBC5;
	}
	else if ((board_adc_val > (200-DELTA)) && (board_adc_val < (200+DELTA)))
	{
		board_config = CONFIG_FULL_SMART_CRADLE;
	}
	else if ((board_adc_val > (100-DELTA)) && (board_adc_val < (100+DELTA)))
	{
		board_config = CONFIG_UNDER_DASH_OBC;
	}
	else if ((board_adc_val > (300-DELTA)) && (board_adc_val < (300+DELTA))) //TODO: this probably needs to change coz CONFIG_OBC5(floating) is 330mV
	{
		board_config = CONFIG_SMART_CRADLE;
	}
	else
	{
		board_config = CONFIG_INVALID;
	}
	return board_config;
}

