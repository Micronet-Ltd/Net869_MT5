#include "board_type.h"
#include "fsl_adc16_driver.h"
#include "fsl_adc16_hal.h"
#include "gpio_pins.h"
#include "ADC.h"

#define CONFIG_FULL 			 'A'
#define CONFIG_WINDSHIELD_OBC	 'D' 	/* Orbcomm device */
// Not sipported more
// #define CONFIG_FULL_SMART_CRADLE 'E'
//
#define CONFIG_UNDER_DASH_OBC 	 'F' 	/* Orbcomm device */
#define CONFIG_SMART_CRADLE 	 'G'
#define CONFIG_TAB8_ECRADLE 	 'H'
#define CONFIG_INVALID			 'Z'

#define ADC_VREF 			3300 /* mV */
#define ADC16_INSTANCE0 	0
#define ADC16_CHN_GROUP_0	0
#define BRD_REV6_LOW		900     /* mV */
#define BRD_REV6_HIGH		1200    /* mV */
#define BRD_REV7_LOW		1400    /* mV */
#define BRD_REV7_HIGH		1600    /* mV */
#define BRD_REV8_LOW		2100    /* mV */
#define BRD_REV8_HIGH		2300    /* mV */

#define BRD_CFGA_LOW		3000    /* mV */
#define BRD_CFGA_HIGH		3350    /* mV */
#define BRD_CFGD_LOW		0       /* mV */
#define BRD_CFGD_HIGH		30      /* mV */
// Not sipported more
// #define BRD_CFGE_LOW		180     /* mV */
// #define BRD_CFGE_HIGH	220     /* mV */
//
#define BRD_CFGG_LOW		260     /* mV */
#define BRD_CFGG_HIGH		340     /* mV */
#define BRD_CFGF_LOW		50      /* mV */
#define BRD_CFGF_HIGH		150     /* mV */
#define BRD_CFGH_LOW		360     /* mV */
#define BRD_CFGH_HIGH		440     /* mV */

static uint32_t board_rev = 0;
static char board_config = 'O';

uint32_t get_board_adc_value(adc16_chn_t channel){
	uint32_t raw_adc_value = 0;	
	uint32_t adc_value = 0;
	uint8_t i = 0, mi = 0;
	uint32_t adc_sum = 0, adc_avar = 0;
	
	adc16_chn_config_t chn_config = {.chnIdx = channel, .convCompletedIntEnable = false, .diffConvEnable = 0 };
	
	ADC_Compare_disable (kADC_POWER_IN_ISR);
	for (i = 0; i < 4; i++) {
		_time_delay(50);
		ADC16_DRV_ConfigConvChn (ADC16_INSTANCE0, ADC16_CHN_GROUP_0, &chn_config);     /* trigger the conversion */
		ADC16_DRV_WaitConvDone  (ADC16_INSTANCE0, ADC16_CHN_GROUP_0);                  /* Wait for the conversion to be done */
		raw_adc_value = ADC16_DRV_GetConvValueRAW(ADC16_INSTANCE0, ADC16_CHN_GROUP_0); /* get  value for single ended channel */
		adc_value = (raw_adc_value * ADC_VREF)>>FSL_FEATURE_ADC16_MAX_RESOLUTION;
		//printf("%s: i=%d, adc_value %d\n", __func__, i, adc_value);
        if (0 == adc_sum) {
			adc_sum += adc_value;
			mi = 1;
			adc_avar = adc_sum/mi;
        } else {
          if (adc_avar > adc_value - 100 && adc_avar < adc_value + 100) {
              adc_sum += adc_value;
              mi++;
              adc_avar = adc_sum/mi;
          } else {
              adc_sum = 0;
              mi = 0;
          }
        }
	}
	
	ADC_Compare_enable (kADC_POWER_IN_ISR);
	return adc_avar;
}

/* 
* get_board_revision: returns the board number read via ADC0_DM1
*						Below Rev 6 = not supported more (Floating)
*						Rev 6 = 1V
*						Rev 7 = 1.5V
*						Rev 8 = 2.2V
*/
uint32_t board_adc_val_g = 0;
uint8_t get_board_revision(void)
{
	uint32_t board_adc_val = 0;
    uint32_t config_adc_val = 0;
	uint16_t i = 0;

	board_adc_val_g = board_adc_val = get_board_adc_value(ADC_BOARD_VER);
    config_adc_val = get_board_adc_value(ADC_BOARD_CONFIG);
	//printf("\n%s: i=%d, board_rev_adc_val %d\n", __func__, i, board_adc_val);
	
	if (board_adc_val < BRD_REV6_HIGH) {
		board_rev = 6;
	} else if (board_adc_val > BRD_REV8_LOW && board_adc_val < BRD_REV8_HIGH) {
		board_rev = 8;
	} else {
		board_rev = 7;
	}

	return board_rev;
}

/* get_board_configuration: returns how the board has been populated(BOM)
*							A - FULL - 3.3V
*							D - WINDSHIELD OBC for Orbcomm - 0.30V to 0.36V (Floating)
*							E - FULL SMART CRADLE - 0.2V
*							F - UNDER DASH OBC for Orbcomm - 0.1V
*							G - SMART CRADLE - 0.3V
*							H - TAB8 SMART CRADLE - 0.4V
*							Z - no match/invalid ADC range
*/
uint32_t config_adc_val_g  = 0;
char get_board_configuration(void)
{
	uint32_t board_adc_val = 0;
	
	uint8_t board_ver = get_saved_board_revision();
	
    config_adc_val_g = board_adc_val = get_board_adc_value(ADC_BOARD_CONFIG);
    printf("\n%s: board_config_adc_val %d\n", __func__, board_adc_val);

    if (board_adc_val < BRD_CFGD_HIGH) {
        board_config = CONFIG_WINDSHIELD_OBC;
    } else if (board_adc_val < BRD_CFGF_HIGH) {
        board_config = CONFIG_UNDER_DASH_OBC;
    } else if (board_adc_val < BRD_CFGG_HIGH) {
        board_config = CONFIG_SMART_CRADLE;
    } else if (board_adc_val < BRD_CFGH_HIGH) {
        board_config = CONFIG_TAB8_ECRADLE;
    } else if (board_adc_val > BRD_CFGA_LOW) {
        board_config = CONFIG_FULL;
    } else {
        board_config = CONFIG_INVALID;
    }

	return board_config;
}

uint8_t get_saved_board_revision(void)
{
	if (board_rev == 0) {
		get_board_revision();
	}

	return board_rev;
}

char get_saved_board_configuration(void)
{
	if (board_config == 'O') {
		get_board_configuration();	
	}

	return board_config;	
}

