#ifndef _EXT_GPIO_H
#define _EXT_GPIO_H

#include <stdint.h>
#include <mqx.h>
#include <event.h>


#include "ADC.h"

#define GPIO_LOGIC_LOW_TH			10000//2500 (value in mV)
#define GPIO_LOGIC_HIGH_TH			11000//7000 (value in mV)

typedef enum {
	GPIO_IN_LOGIC_LOW = 0,
	GPIO_IN_LOGIC_HIGH   ,
	GPIO_IN_LOGIC_3STATE
} KINPUT_LOGIC_LEVEL;

typedef enum {
	kANALOG_EXT_IN = 0,
	kGPIO_IN1,
	kGPIO_IN2,
	kGPIO_IN3,
	kGPIO_IN4,
	kGPIO_IN5,
	kGPIO_IN6,
	kGPIO_IN7,
	NUM_OF_INPUT_GPIOS
} KGPIOS_INPUT_CHANNELS;

typedef enum {
	GPIO_OUT_LOW = 0,
	GPIO_OUT_OPEN_DRAIN
} KOUTPUT_LEVEL;

typedef enum {
	kGPIO_OUT1 = 0,
	kGPIO_OUT2,
	kGPIO_OUT3,
	kGPIO_OUT4,
	NUM_OF_OUTPUT_GPIOS
} KGPIOS_OUTPUT_CHANNELS;

#define EVENT_GPIO_IN(x)		(1 << (x))

#ifdef __cplusplus
extern "C"
{
#endif

extern void *g_GPIO_event_h;

void GPIO_sample_all (void);
KINPUT_LOGIC_LEVEL GPIO_INPUT_get_logic_level   (KGPIOS_INPUT_CHANNELS gpio_input);
uint32_t           GPIO_INPUT_get_voltage_level (KGPIOS_INPUT_CHANNELS gpio_input);

void    GPIO_OUTPUT_set_level        (KGPIOS_OUTPUT_CHANNELS gpio_output, KOUTPUT_LEVEL level);
void 	gpio_set_output (KGPIOS_OUTPUT_CHANNELS gpo_num, KOUTPUT_LEVEL level);
void 	gpio_set_multiple_outputs(uint8_t * mask, uint8_t * value);

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* _EXT_GPIO_H */
