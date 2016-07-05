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

#include "control_task.h"
#include "protocol.h"

typedef struct {
	const KADC_CHANNELS_t adc_channel;
		  uint32_t        gpio_input_voltage;
} GPIO_INPUT;

#ifdef __cplusplus
extern "C"
{
#endif

static GPIO_INPUT  gpio_inputs [NUM_OF_INPUT_GPIOS] = {
		[kANALOG_EXT_IN] = {kADC_ANALOG_IN1, 0},
		[kGPIO_IN1]      = {kADC_GPIO_IN1  , 0},
		[kGPIO_IN2]      = {kADC_GPIO_IN2  , 0},
		[kGPIO_IN3]      = {kADC_GPIO_IN3  , 0},
		[kGPIO_IN4]      = {kADC_GPIO_IN4  , 0},
		[kGPIO_IN5]      = {kADC_GPIO_IN5  , 0},
		[kGPIO_IN6]      = {kADC_GPIO_IN6  , 0},
		[kGPIO_IN7]      = {kADC_GPIO_IN7  , 0},
};

static const uint32_t gp_out_mapping[] = {
		[kGPIO_OUT1] = GPIO_OUT1,
		[kGPIO_OUT2] = GPIO_OUT2,
		[kGPIO_OUT3] = GPIO_OUT3,
		[kGPIO_OUT4] = GPIO_OUT4,
};

void *g_GPIO_event_h;

KINPUT_LOGIC_LEVEL GPIO_INPUT_convert_voltage_to_level (uint32_t voltage);

//void GPIO_sample_all (void)
void GPIO_sample_all (KGPIOS_INPUT_CHANNELS i)
{
	uint32_t gpio_event = 0;
//	KGPIOS_INPUT_CHANNELS i;
	KINPUT_LOGIC_LEVEL state_prev, state_current, state_temp;

//	for (i = kANALOG_EXT_IN; i < NUM_OF_GPI; i++) {
		state_prev = GPIO_INPUT_get_logic_level (i);
		ADC_sample_input (gpio_inputs[i].adc_channel);
		gpio_inputs[i].gpio_input_voltage = ADC_get_value (gpio_inputs[i].adc_channel);
		state_temp = GPIO_INPUT_get_logic_level (i);

		/* Do not update the state if it is in between low and high */
		//TODO: verify this with Eyal, -Abid
		if (state_temp == GPIO_IN_LOGIC_3STATE){
			state_current = state_prev;
		}
		else{
			state_current = state_temp;
		}


		if (state_current != state_prev) {
			printf ("GPIO_IN %d level became %d\n", i, state_current);
			gpio_event |= (1 << i);
		}
//	}

	//TODO: for now just send the control message from within the GPIO driver
	if (gpio_event != 0){
		send_gpi_change((uint8_t *)&gpio_event);
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
		default           : printf ("ERROR: Illegal GPIO_OUT %d\n", gpio_output); return;
	}
	printf ("GPIO_OUT %d level is set to %s\n", gpio_output,  (level == GPIO_OUT_LOW) ? "LOW" : "OPEN DRAIN");
}

void send_gpi_change(uint8_t * gpio_mask)
{
	packet_t gpi_msg;
	gpi_msg.pkt_type = GPIO_INT_STATUS;
	uint8_t val = 0;

	KGPIOS_INPUT_CHANNELS gpi_num = kANALOG_EXT_IN;
	/* loop through the mask, if true set the value bit */
	for (gpi_num = kANALOG_EXT_IN; gpi_num < NUM_OF_INPUT_GPIOS; gpi_num++)
	{
		if (*gpio_mask>>gpi_num & 0x1)
		{
			val |= GPIO_INPUT_get_logic_level(gpi_num)<<gpi_num;
		}
	}

	gpi_msg.data[0] = *gpio_mask;
	gpi_msg.data[1] = val;
	send_control_msg(&gpi_msg, 2);

}

void gpio_set_output (KGPIOS_OUTPUT_CHANNELS gpo_num, KOUTPUT_LEVEL level)
{
	GPIO_DRV_WritePinOutput(gp_out_mapping[gpo_num], level );
	printf("set GPIO num: %d to val %d \n", gpo_num, level);
}

void gpio_set_multiple_outputs(uint8_t * mask, uint8_t * value)
{
	KGPIOS_OUTPUT_CHANNELS gpo_num = kGPIO_OUT1;
	/* loop through the mask, if true set the level */
	for (gpo_num = kGPIO_OUT1; gpo_num < NUM_OF_OUTPUT_GPIOS; gpo_num++)
	{
		if (*mask>>gpo_num & 0x1)
		{
			gpio_set_output(gpo_num, (KOUTPUT_LEVEL)(*value>>gpo_num & 0x1));
		}
	}
}

#ifdef __cplusplus
extern "C"
}
#endif
