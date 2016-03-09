#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "MK20D10_extension.h"
#include "fsl_power_manager.h"

#include "tasks_list.h"

#include "power_mgm.h"
#include "Device_control_GPIO.h"
#include "ADC.h"
#include "EXT_GPIOS.h"

#include "Uart_debugTerminal.h"

#define POWER_MGM_TIME_DELAY		 100
#define TEMPERATURE_TIME_DELAY		5000
#define IGNITION_TIME_DELAY			1000
#define CABLE_TYPE_TIME_DELAY		1000
#define EXT_GPIO_TIME_DELAY			1000

#define CABLE_TYPE_MIN_VOLTAGE		(CABLE_TYPE_VOLTAGE *  90/100)
#define CABLE_TYPE_MAX_VOLTAGE		(CABLE_TYPE_VOLTAGE * 110/100)

#define BOOST_ENABLE

#ifdef BOOST_ENABLE
#define SUPERCAP_MIN_TH		3600
#define SUPERCAP_MAX_TH		3800
#else
#define SUPERCAP_MIN_TH		5000
#define SUPERCAP_MAX_TH		5300
#endif

void Supercap_charge_state (void);
void Supercap_discharge_state (void);

void Power_MGM_task (uint32_t initial_data )
{
	uint32_t current_time          = 0;
	uint32_t temperature_last_time = 0;
	uint32_t ignition_last_time    = 0;
	uint32_t ext_gpio_last_time    = 0;
	uint32_t cable_type_last_time  = 0;
	uint32_t cable_type_voltage    = 0;

	Device_init (POWER_MGM_TIME_DELAY);
	ADC_init ();

	MIC_DEBUG_UART_PRINTF ("\nPower Management Task: Start \n");

	while (1) {
		ADC_sample_input (kADC_POWER_IN);
		ADC_sample_input (kADC_POWER_VCAP);

		if ((current_time - ignition_last_time) >= IGNITION_TIME_DELAY) {
			ignition_last_time = current_time;
			ADC_sample_input (kADC_ANALOG_IN1);
		}

		if ((current_time - temperature_last_time) >= TEMPERATURE_TIME_DELAY) {
			temperature_last_time = current_time;
			ADC_sample_input (kADC_TEMPERATURE);
		}

		// check cable type - report if not detected (tamper) or not as expected
		if ((current_time - cable_type_last_time) >= CABLE_TYPE_TIME_DELAY) {
			cable_type_last_time = current_time;
			ADC_sample_input (kADC_CABLE_TYPE);

			cable_type_voltage = ADC_get_value(kADC_CABLE_TYPE);
			if ((cable_type_voltage < CABLE_TYPE_MIN_VOLTAGE) ||
				(cable_type_voltage > CABLE_TYPE_MAX_VOLTAGE) ) {
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: WARNING: CABLE TYPE is not as expected (current voltage %d mV - expected %d mV\n", cable_type_voltage, CABLE_TYPE_VOLTAGE);
			}
		}

		if ((current_time - ext_gpio_last_time) >= EXT_GPIO_TIME_DELAY) {
			ext_gpio_last_time = current_time;
			GPIO_sample_all ();
		}

		Device_update_state      ();
		Supercap_charge_state    ();
		Supercap_discharge_state ();
		_time_delay (POWER_MGM_TIME_DELAY);
		current_time += POWER_MGM_TIME_DELAY;
	}
}

// charge supercap if supercap voltage level is below threshold.
// continue till supercap voltage level reaches upper threshold.
void Supercap_charge_state (void)
{
	uint32_t supercap_voltaeg = ADC_get_value (kADC_POWER_VCAP);

	if (supercap_voltaeg >= SUPERCAP_MAX_TH) {
		// send a message only once
		if (GPIO_DRV_ReadPinInput (POWER_CHARGE_ENABLE) == 1)
			MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: SUPERCAP full Charged %d mV\n", supercap_voltaeg);

		GPIO_DRV_ClearPinOutput (POWER_CHARGE_ENABLE);
	} else if (supercap_voltaeg <= SUPERCAP_MIN_TH) {
		// send a message only once
		if (GPIO_DRV_ReadPinInput (POWER_CHARGE_ENABLE) == 0)
			MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: SUPERCAP low %d mV - Start Charging\n", supercap_voltaeg);

		GPIO_DRV_SetPinOutput (POWER_CHARGE_ENABLE);
	}
}


// discharge supercap if input power is below threshold
void Supercap_discharge_state (void)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN);

	if (power_in_voltage <= POWER_IN_SHUTDOWN_TH) {
		// send a message only once
		if (GPIO_DRV_ReadPinInput (POWER_DISCHARGE_ENABLE) == 0)
			MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: SUPERCAP start DisCharged (power in voltage is: %d mV)\n", power_in_voltage);

		GPIO_DRV_SetPinOutput (POWER_DISCHARGE_ENABLE);
	} else {
		// send a message only once
		if (GPIO_DRV_ReadPinInput (POWER_DISCHARGE_ENABLE) == 1)
			MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: SUPERCAP stop DisCharged (power in voltage is: %d mV)\n", power_in_voltage);

		GPIO_DRV_ClearPinOutput (POWER_DISCHARGE_ENABLE);
	}
}
