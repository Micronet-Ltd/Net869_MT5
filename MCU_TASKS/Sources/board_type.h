#ifndef _BOARD_TYPE_H_
#define _BOARD_TYPE_H_
#include <stdint.h>
#include <fsl_adc16_hal.h>

uint32_t get_board_adc_value(adc16_chn_t channel);
uint8_t get_board_revision(void);
char get_board_configuration(void);

#endif /* _BOARD_TYPE_H_ */