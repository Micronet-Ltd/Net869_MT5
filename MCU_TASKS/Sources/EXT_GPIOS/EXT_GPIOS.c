/**************************************************************************************
* The OBC contains 8 analog General Purpose Inputs and 4 Open Drain General Purpose   *
* outputs.                                                                            *
*                                                                                     *
* Each input can be referred as analog input or as 3-State digital input.             *
* The GPIOs are sampled once in a while by the main task. each input generates an     *
* event when its status is changed. There are 3 Digital Status levels - Logic high,   *
* Logic low and 3-State.                                                              *
*                                                                                     *
* Each output can be set to become either LOW or Open Drain.                          *
**************************************************************************************/

#include "EXT_GPIOS.h"
#include <assert.h>

#include "gpio_pins.h"
#include "Uart_debugTerminal.h"

#define NUM_OF_GPIOS	8

typedef struct {
	const KADC_CHANNELS_t adc_channel;
	      uint32_t        gpio_input_voltage;
} GPIO_INPUT;


#ifdef __cplusplus
extern "C"
{
#endif

GPIO_INPUT  gpio_inputs [NUM_OF_INPUT_GPIOS] = {	{kADC_ANALOG_IN1, 0}, {kADC_GPIO_IN1, 0}, {kADC_GPIO_IN2, 0}, {kADC_GPIO_IN3, 0},
													{kADC_GPIO_IN4,   0}, {kADC_GPIO_IN5, 0}, {kADC_GPIO_IN6, 0}, {kADC_GPIO_IN7, 0}};

void *g_GPIO_event_h;

KINPUT_LOGIC_LEVEL GPIO_INPUT_convert_voltage_to_level (uint32_t voltage);


void GPIO_sample_all (void)
{
	uint32_t gpio_event = 0;
	KGPIOS_INPUT_CHANNELS i;
	KINPUT_LOGIC_LEVEL state_prev, state_current;

	for (i = 0; i < NUM_OF_GPIOS; i++) {
		state_prev = GPIO_INPUT_get_logic_level (i);
		ADC_sample_input (gpio_inputs[i].adc_channel);
		gpio_inputs[i].gpio_input_voltage = ADC_get_value (gpio_inputs[i].adc_channel);
		state_current = GPIO_INPUT_get_logic_level (i);
		
		if (state_current != state_prev) {
			MIC_DEBUG_UART_PRINTF ("GPIO_IN %d level became %d\n", i, state_prev);
			gpio_event |= (1 << i);
		}
	}

	_event_set(g_GPIO_event_h, gpio_event);
}


KINPUT_LOGIC_LEVEL GPIO_INPUT_convert_voltage_to_level (uint32_t voltage)
{
	if (voltage < GPIO_LOGIC_LOW_TH)		return GPIO_IN_LOGIC_LOW;
	else if (voltage > GPIO_LOGIC_HIGH_TH)	return GPIO_IN_LOGIC_HIGH;
	else 									return GPIO_IN_LOGIC_3STATE;
}

KINPUT_LOGIC_LEVEL GPIO_INPUT_get_logic_level   (KGPIOS_INPUT_CHANNELS gpio_input)
{
	assert(gpio_input < NUM_OF_INPUT_GPIOS);

	return GPIO_INPUT_convert_voltage_to_level (gpio_inputs[gpio_input].gpio_input_voltage);
}

uint32_t GPIO_INPUT_get_voltage_level (KGPIOS_INPUT_CHANNELS gpio_input)
{
	assert(gpio_input < NUM_OF_INPUT_GPIOS);

	return gpio_inputs[gpio_input].gpio_input_voltage;
}

void GPIO_OUTPUT_set_level (KGPIOS_OUTPUT_CHANNELS gpio_output, KOUTPUT_LEVEL level)
{
	switch (gpio_output) {
		case (kGPIO_OUT1) : if (level == GPIO_OUT_LOW)	GPIO_DRV_SetPinOutput (GPIO_OUT1);  else GPIO_DRV_ClearPinOutput (GPIO_OUT1); break;
		case (kGPIO_OUT2) : if (level == GPIO_OUT_LOW)	GPIO_DRV_SetPinOutput (GPIO_OUT2);  else GPIO_DRV_ClearPinOutput (GPIO_OUT2); break;
		case (kGPIO_OUT3) : if (level == GPIO_OUT_LOW)	GPIO_DRV_SetPinOutput (GPIO_OUT3);  else GPIO_DRV_ClearPinOutput (GPIO_OUT3); break;
		case (kGPIO_OUT4) : if (level == GPIO_OUT_LOW)	GPIO_DRV_SetPinOutput (GPIO_OUT4);  else GPIO_DRV_ClearPinOutput (GPIO_OUT4); break;
		default           : MIC_DEBUG_UART_PRINTF ("ERROR: Illegal GPIO_OUT %d\n", gpio_output); return;
	}
	MIC_DEBUG_UART_PRINTF ("GPIO_OUT %d level is set to %s\n", gpio_output,  (level == GPIO_OUT_LOW) ? "LOW" : "OPEN DRAIN");
}

#ifdef __cplusplus
extern "C"
}
#endif
