/**************************************************************************************
 * Device Control file controls the devices status - turning it on \ off or reseting. *
 *                                                                                    *
 * Device has the following states:                                                   *
 *   - DEVICE_STATE_OFF -                                                             *
 *           device is off and in low power mode.                                     *
 *           MCU clock is internal slow clock                                         *
 *           CPU is powered by 4.2V but is OFF                                        *
 *           state changes only if power is valid and temperature is OK               *
 *           Next state - DEVICE_STATE_TURNING_ON                                     *
 *                                                                                    *
 *   - DEVICE_STATE_TURNING_ON  -                                                     *
 *           Pulse is generated to turn on the CPU                                    *
 *           MCU clock is internal slow clock                                         *
 *           CPU is powered by 4.2V and boots                                         *
 *           state changes when pulse duration ends                                   *
 *           Next state - DEVICE_STATE_ON                                             *
 *                                                                                    *
 *   - DEVICE_STATE_ON                                                                *
 *           device is on and operates normally                                       *
 *           MCU clock is highest based on external clock                             *
 *           CPU is powered by 4.2V and runs                                          *
 *           state changes when power drops (critical) or when                        *
 *                              temperature is out of range                           *
 *           Next state - DEVICE_STATE_BACKUP_RECOVERY when power drops               *
 *                        DEVICE_STATE_TURNING_OFF when temperature is out of range   *
 *                                                                                    *
 *   - DEVICE_STATE_BACKUP_RECOVERY                                                   *
 *           device is on and runs normally, giving a chance to power come back       *
 *           MCU clock is highest based on external clock                             *
 *           CPU is powered by 4.2V based on supercap                                 *
 *           state changes when power returns or when                                 *
 *                              timeout is reached                                    *
 *           Next state - DEVICE_STATE_ON if power returns back                       *
 *                        DEVICE_STATE_BACKUP_POWER if timeout reaches                *
 *                                                                                    *
 *   - DEVICE_STATE_BACKUP_POWER                                                      *
 *           device is on and in low power mode.                                      *
 *           MCU clock is internal slow clock                                         *
 *           CPU is powered by 4.2V based on supercap                                 *
 *           state changes when timeout is reached (supercap energy is almost lost)   *
 *           Next state - DEVICE_STATE_TURNING_OFF                                    *
 *                                                                                    *
 *   - DEVICE_STATE_TURNING_OFF                                                       *
 *           Pulse is generated to turn off the CPU                                   *
 *           MCU clock is internal slow clock                                         *
 *           CPU is powered by 4.2V and shuts down                                    *
 *           state changes when pulse duration ends                                   *
 *           Next state - DEVICE_STATE_OFF                                            *
 *                                                                                    *
 *   - DEVICE_STATE_RESET                                                             *
 *           Not supported yet                                                        *
 *                                                                                    *
 * It also controls the MCU POWER STATUS LED as following:                            *
 *   - OFF             no input power. Device is off or shutting down by supercap     *
 *   - Blinking RED    while device is off                                            *
 *   - stable   GREEN  while device is on                                             *
 *   - stable   BLUE   while device is turning on \ off \ reseting                    *
 *   - stable   YELLOW while device is shutting down if power return after power drop *
 *                                                                                    *
 * when device is OFF, all peripherals are disabled                                   *
**************************************************************************************/

#include "Device_control_GPIO.h"
#include "power_mgm.h"
#include "wiggle_sensor.h"
#include "ADC.h"
#include "gpio_pins.h"
#include "protocol.h"

#include "fpga_api.h"

#define DEVICE_CONTROL_TIME_ON_TH				 3000		// number of mili-seconds pulse for turning device on
#define DEVICE_CONTROL_TIME_OFF_TH				 3000		// number of mili-seconds pulse for turning device off
#define DEVICE_CONTROL_TIME_RESET_TH			  500		// number of mili-seconds pulse for reseting device

#define BACKUP_RECOVER_TIME_TH					 1000		// number of mili-seconds to try power failure overcome (device is powered by supercap)
#define BACKUP_POWER_TIME_TH					10000		// number of mili-seconds to power device by supercap

typedef struct {
	uint32_t time_threshold;								// pulse time period
	uint32_t time;											// time counter
	uint32_t delay_period;									// time duration that needs to be added to counter
	bool     enable;										// pulse enable control
	bool     status;										// pulse status - TRUE as long as pulse is generated
} DEVICE_CONTROL_GPIO_t;


DEVICE_CONTROL_GPIO_t device_control_gpio;
DEVICE_STATE_t        device_state;

uint32_t backup_power_cnt = 0 ;
uint8_t ledBlinkCnt       = 0 ;

void Device_control_GPIO        (void);
bool Device_control_GPIO_status (void);
void peripherals_enable         (void);
void peripherals_disable        (void);


void Device_update_state (void)
{
#if 1
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN   );
	uint32_t ingition_voltage  = ADC_get_value (kADC_ANALOG_IN1 );
	int32_t  temperature       = ADC_get_value (kADC_TEMPERATURE);
	bool     turn_on_condition = FALSE;



	Device_control_GPIO ();

	switch (device_state) {
		case DEVICE_STATE_OFF:
			// blink RED LED when device is off
			if      (++ledBlinkCnt == 8)		GPIO_DRV_SetPinOutput   (LED_RED);
			else if (  ledBlinkCnt < 10)		GPIO_DRV_ClearPinOutput (LED_RED);
			else       ledBlinkCnt = 0;

			// check amount of vibrations sensed by the wiggle sensor
			// if amount of vibrations is more than TH, it will turn on the device
			// and stop the interrupts for better running efficiency
			Wiggle_sensor_update ();

			if ((power_in_voltage < POWER_IN_TURN_ON_TH ) ||
				(temperature      < TEMPERATURE_MIN_TH  ) ||
				(temperature      > TEMPERATURE_MAX_TH  )  )
				break;

			if (ingition_voltage >= IGNITION_TURN_ON_TH) {
				printf ("\nPOWER_MGM: TURNING ON DEVICE with ignition\n");
				turn_on_condition = TRUE;
				// TO DO: set event or send message
			}

			if (Wiggle_sensor_cross_TH ()) {
				printf ("\nPOWER_MGM: TURNING ON DEVICE with wiggle sensor \n");
				turn_on_condition = TRUE;
				// TO DO: set event or send message
			}

			if (turn_on_condition) {
				turn_on_condition = FALSE;
				ledBlinkCnt = 0;
				Wiggle_sensor_stop ();						// disable interrupt
				peripherals_enable ();
				Device_turn_on     ();
				GPIO_DRV_ClearPinOutput (LED_RED);
				device_state = DEVICE_STATE_TURNING_ON;
			}
			break;

		case DEVICE_STATE_TURNING_ON:
			GPIO_DRV_SetPinOutput (POWER_3V3_ENABLE);
			GPIO_DRV_SetPinOutput (FPGA_PWR_ENABLE);		// enables CPU GPIOs power

			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status()) {
				Board_SetFastClk ();
				device_state = DEVICE_STATE_ON;
				printf ("\nPOWER_MGM: DEVICE RUNNING\n");
			}
			break;

		case DEVICE_STATE_ON:
			GPIO_DRV_SetPinOutput (LED_GREEN);
			GPIO_DRV_SetPinOutput (POWER_3V3_ENABLE);
			GPIO_DRV_SetPinOutput (FPGA_PWR_ENABLE);

			// if power drops below threshold - shutdown
			if (power_in_voltage < POWER_IN_SHUTDOWN_TH) {
				printf ("\nPOWER_MGM: WARNING: INPUT POWER LOW %d - SHUTING DOWN !!! \n", power_in_voltage);
				device_state = DEVICE_STATE_BACKUP_RECOVERY;
				break;
			}

			// if temperature is out of range - turn off device
			if ((temperature < TEMPERATURE_MIN_TH)   ||
				(temperature > TEMPERATURE_MAX_TH)    ) {
				printf ("\nPOWER_MGM: TEMPERATURE OUT OF RANGE %d - SHUTING DOWN !!! \n", temperature);
				GPIO_DRV_ClearPinOutput (LED_GREEN);
				Device_turn_off  ();
				Board_SetSlowClk ();
				device_state = DEVICE_STATE_TURN_OFF;
			}
			break;

		case DEVICE_STATE_BACKUP_RECOVERY:
			backup_power_cnt += device_control_gpio.delay_period;

			// if recovery period has passed - generate power loss interrupt to CPU and close peripherals
			if (backup_power_cnt > BACKUP_RECOVER_TIME_TH) {
				GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
				peripherals_disable ();
				Board_SetSlowClk ();
				device_state = DEVICE_STATE_BACKUP_POWER;
				printf ("\nPOWER_MGM: Recovery period is over\n");
				break;
			}

			// if power is back during recovery period - return to DEVICE_ON state, like nothing happen
			if (power_in_voltage >= POWER_IN_TURN_ON_TH) {
				printf ("\nPOWER_MGM: INPUT POWER OK %d\n", power_in_voltage);
				backup_power_cnt = 0;
				device_state = DEVICE_STATE_ON;
			}
			break;

		case DEVICE_STATE_BACKUP_POWER:
			// if power is back during backup period - turn on a YELLOW LED
			if (power_in_voltage >= POWER_IN_TURN_ON_TH) {
				GPIO_DRV_SetPinOutput   (LED_RED);
				GPIO_DRV_SetPinOutput   (LED_GREEN);
			}

			backup_power_cnt += device_control_gpio.delay_period;
			if (backup_power_cnt > BACKUP_POWER_TIME_TH) {
				GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
				GPIO_DRV_ClearPinOutput (LED_RED);
				GPIO_DRV_ClearPinOutput (LED_GREEN);
				backup_power_cnt = 0;
				ledBlinkCnt = 0;
				Device_turn_off ();
				device_state = DEVICE_STATE_TURN_OFF;
				printf ("\nPOWER_MGM: backup period is over - shutting down\n");

			}
			break;

		case DEVICE_STATE_TURN_OFF:
			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status()) {
				printf ("\nPOWER_MGM: DEVICE IS OFF\n");
				device_state = DEVICE_STATE_OFF;
				Wiggle_sensor_restart ();
				peripherals_disable ();
				Wiggle_sensor_start ();									// enable interrupt
			}
			break;

		default:
			printf ("\nPOWER_MGM: UNKNOWN STATE %d\n", device_state );
			device_state = DEVICE_STATE_OFF;
			Wiggle_sensor_restart ();
			break;
	}
#endif
}


void Device_init (uint32_t delay_period)
{
	device_state = DEVICE_STATE_OFF;

	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio.delay_period   = delay_period;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = false;
	device_control_gpio.status         = false;
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);

	Wiggle_sensor_init (delay_period);
}

void Device_turn_on  (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_turn_off (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_OFF_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_reset (void)
{
	device_control_gpio.time_threshold = DEVICE_CONTROL_TIME_RESET_TH;
	device_control_gpio.time           = 0;
	device_control_gpio.enable         = true;	
}

void Device_control_GPIO (void)
{
	if (device_control_gpio.enable == false)
		return;
		
	if (device_control_gpio.time <	device_control_gpio.time_threshold) {
		GPIO_DRV_SetPinOutput   (LED_BLUE);
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		device_control_gpio.status = true;
		device_control_gpio.time  += device_control_gpio.delay_period;
	} else {
		GPIO_DRV_ClearPinOutput (LED_BLUE);
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
		device_control_gpio.status = false;
		device_control_gpio.time   = 0;
		device_control_gpio.enable = false;		
	}
}

bool Device_control_GPIO_status  (void) {return device_control_gpio.status;}
DEVICE_STATE_t Device_get_status (void) {return device_state;}

void peripherals_enable (void)
{
	GPIO_DRV_ClearPinOutput (FPGA_RSTB);

    GPIO_DRV_SetPinOutput   (POWER_3V3_ENABLE);	// turn on 3V3 power rail
    GPIO_DRV_SetPinOutput   (POWER_5V0_ENABLE);	// turn on 5V0 power rail
    GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE);	// FPGA Enable
    FPGA_init ();
    GPIO_DRV_SetPinOutput   (FPGA_RSTB);

	GPIO_DRV_SetPinOutput   (CAN_ENABLE);			// Enable CAN

    GPIO_DRV_ClearPinOutput (USB_OTG_SEL);		// Connect D1 <-> D MCU or HUB
//  GPIO_DRV_SetPinOutput(USB_OTG_SEL);			// Connect D2 <-> D A8 OTG
    GPIO_DRV_ClearPinOutput (USB_OTG_OE);		//Enable OTG/MCU switch

    // Enable USB for DEBUG
    GPIO_DRV_SetPinOutput   (USB_HUB_RSTN);
    GPIO_DRV_SetPinOutput   (USB_ENABLE);

    GPIO_DRV_SetPinOutput   (UART_ENABLE);			// Enable UART
    GPIO_DRV_SetPinOutput   (FTDI_RSTN);

    // wait till FPGA is loaded
    while (GPIO_DRV_ReadPinInput (FPGA_DONE) == 0)
    	asm ("nop");
    J1708_enable  (7);

	{
		uint8_t Br, R,G,B;
		R = G = B = 255;
		Br = 10;
		FPGA_write_led_status (LED_RIGHT , &Br, &R, &G, &B);
		FPGA_write_led_status (LED_MIDDLE, &Br, &R, &G, &B);
	}

}


void peripherals_disable (void)
{
	GPIO_DRV_ClearPinOutput (POWER_3V3_ENABLE);
	GPIO_DRV_ClearPinOutput (LED_RED);
	GPIO_DRV_ClearPinOutput (LED_GREEN);
	GPIO_DRV_ClearPinOutput (LED_BLUE);
	GPIO_DRV_ClearPinOutput (FPGA_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput (CAN_ENABLE);
	GPIO_DRV_ClearPinOutput (USB_ENABLE);
	GPIO_DRV_ClearPinOutput (UART_ENABLE);
	GPIO_DRV_ClearPinOutput (SPKR_INT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_EXT_EN);
	GPIO_DRV_ClearPinOutput (CPU_MIC_EN);
}




