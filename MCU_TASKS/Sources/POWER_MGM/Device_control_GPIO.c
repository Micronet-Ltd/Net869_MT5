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
#include "ADC.h"
#include "power_mgm.h"
#include "wiggle_sensor.h"

#include "board.h"
#include "gpio_pins.h"

#include "control_task.h"

// TODO: need to be removed after tasks enable will be set fro USB protocol
#include "fpga_api.h"
#include "J1708_task.h"

#include "Uart_debugTerminal.h"

#define DEVICE_CONTROL_TIME_ON_TH				 3000		// number of mili-seconds pulse for turning device on
#define DEVICE_CONTROL_TIME_OFF_TH				 3000		// number of mili-seconds pulse for turning device off
#define DEVICE_CONTROL_TIME_RESET_TH			  500		// number of mili-seconds pulse for reseting device

#define BACKUP_RECOVER_TIME_TH					 1000		// number of mili-seconds to try power failure overcome (device is powered by supercap)
#define BACKUP_POWER_TIME_TH					10000		// number of mili-seconds to power device by supercap

#define CPU_OFF_CHECK_TIME						1000		// time between checks for CPU/A8 off
#define MAX_CPU_OFF_CHECK_TIME					30000		// Max time to wait before shutting off the unit by killing the power

#define MCU_AND_CPU_BOARD_CONNECTED

typedef struct
{
	uint32_t time_threshold;								// pulse time period
	uint32_t time;											// time counter
	uint32_t delay_period;									// time duration that needs to be added to counter
	bool     enable;										// pulse enable control
	bool     status;										// pulse status - TRUE as long as pulse is generated
} DEVICE_CONTROL_GPIO_t;


DEVICE_CONTROL_GPIO_t device_control_gpio_g;
DEVICE_STATE_t        device_state_g;
static uint8_t turn_on_condition_g = 0;

uint32_t backup_power_cnt_g = 0 ;
uint8_t led_blink_cnt_g     = 0 ;
extern uint32_t ignition_threshold_g;
extern volatile uint32_t cpu_watchdog_count_g;

void Device_control_GPIO        (void);
bool Device_control_GPIO_status (void);
void peripherals_enable         (void);
void peripherals_disable        (void);
void send_power_change          (uint8_t *power_mask);


void Device_update_state (void)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN   );
	uint32_t ignition_voltage  = ADC_get_value (kADC_ANALOG_IN1 );
	int32_t  temperature       = ADC_get_value (kADC_TEMPERATURE);

	Device_control_GPIO ();

	switch (device_state_g)
	{
		case DEVICE_STATE_OFF:
			turn_on_condition_g = 0;
			// blink RED LED when device is off
			if      (++led_blink_cnt_g == 8)		GPIO_DRV_SetPinOutput   (LED_RED);
			else if (  led_blink_cnt_g < 10)		GPIO_DRV_ClearPinOutput (LED_RED);
			else       led_blink_cnt_g = 0;

			// check amount of vibrations sensed by the wiggle sensor
			// if amount of vibrations is more than TH, it will turn on the device
			// and stop the interrupts for better running efficiency
			Wiggle_sensor_update ();

			if ((power_in_voltage < POWER_IN_TURN_ON_TH ) ||
				(temperature      < TEMPERATURE_MIN_TH  ) ||
				(temperature      > TEMPERATURE_MAX_TH  )  )
				break;

			if (ignition_voltage >= ignition_threshold_g)
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: TURNING ON DEVICE with ignition\n");
				turn_on_condition_g |= POWER_MGM_DEVICE_ON_IGNITION_TRIGGER;
			}

			if (Wiggle_sensor_cross_TH ())
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: TURNING ON DEVICE with wiggle sensor \n");
				turn_on_condition_g |= POWER_MGM_DEVICE_ON_WIGGLE_TRIGGER;
			}

			if (turn_on_condition_g != 0)
			{
				led_blink_cnt_g = 0;
				Wiggle_sensor_stop ();						// disable interrupt
				peripherals_enable ();
				Device_turn_on     ();
				//send_power_change  (&turn_on_condition);
				GPIO_DRV_ClearPinOutput (LED_RED);
				device_state_g = DEVICE_STATE_TURNING_ON;
			}
			break;

		case DEVICE_STATE_TURNING_ON:
			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status())
			{
				Board_SetFastClk ();
				device_state_g = DEVICE_STATE_ON;
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: DEVICE RUNNING\n");
			}
			break;

		case DEVICE_STATE_ON:
			GPIO_DRV_SetPinOutput (LED_GREEN);

			// if power drops below threshold - shutdown
			if (power_in_voltage < POWER_IN_SHUTDOWN_TH)
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: WARNING: INPUT POWER LOW %d - SHUTING DOWN !!! \n", power_in_voltage);
				device_state_g = DEVICE_STATE_BACKUP_RECOVERY;
				break;
			}

			// if temperature is out of range - turn off device
			if ((temperature < TEMPERATURE_MIN_TH)   ||
				(temperature > TEMPERATURE_MAX_TH)    )
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: TEMPERATURE OUT OF RANGE %d - SHUTING DOWN !!! \n", temperature);
				GPIO_DRV_ClearPinOutput (LED_GREEN);
				Device_turn_off  ();
				Board_SetSlowClk ();
				device_state_g = DEVICE_STATE_TURN_OFF;
			}
			break;

		case DEVICE_STATE_BACKUP_RECOVERY:
			backup_power_cnt_g += device_control_gpio_g.delay_period;

			// if recovery period has passed - generate power loss interrupt to CPU and close peripherals
			if (backup_power_cnt_g > BACKUP_RECOVER_TIME_TH)
			{
				GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
				peripherals_disable ();
				Board_SetSlowClk ();
				device_state_g = DEVICE_STATE_BACKUP_POWER;
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: Recovery period is over\n");
				break;
			}

			// if power is back during recovery period - return to DEVICE_ON state, like nothing happen
			if (power_in_voltage >= POWER_IN_TURN_ON_TH)
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: INPUT POWER OK %d\n", power_in_voltage);
				backup_power_cnt_g = 0;
				device_state_g = DEVICE_STATE_ON;
			}
			break;

		case DEVICE_STATE_BACKUP_POWER:
			// if power is back during backup period - turn on a YELLOW LED
			if (power_in_voltage >= POWER_IN_TURN_ON_TH)
			{
				GPIO_DRV_SetPinOutput   (LED_RED);
				GPIO_DRV_SetPinOutput   (LED_GREEN);
			}

			backup_power_cnt_g += device_control_gpio_g.delay_period;
			if (backup_power_cnt_g > BACKUP_POWER_TIME_TH)
			{
				GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
				GPIO_DRV_ClearPinOutput (LED_RED);
				GPIO_DRV_ClearPinOutput (LED_GREEN);
				backup_power_cnt_g = 0;
				led_blink_cnt_g = 0;
				Device_turn_off ();
				device_state_g = DEVICE_STATE_TURN_OFF;
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: backup period is over - shutting down\n");
			}
			break;

		case DEVICE_STATE_TURN_OFF:
			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status())
			{
				MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: DEVICE IS OFF\n");
				device_state_g = DEVICE_STATE_OFF;
				Wiggle_sensor_restart ();
				peripherals_disable ();
				Wiggle_sensor_start ();									// enable interrupt
			}
			break;

		default:
			MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: ERROR: UNKNOWN STATE %d\n", device_state_g );
			device_state_g = DEVICE_STATE_OFF;
			Wiggle_sensor_restart ();
			break;
	}
}

void Device_get_turn_on_reason(uint8_t * turn_on_reason)
{
	*turn_on_reason = turn_on_condition_g;
}

void Device_off_req(uint8_t wait_time)
{
	uint32_t cpu_off_wait_time = 0;
	uint8_t cpu_status_pin = 0;

	_time_delay(wait_time*1000);
	Device_turn_off  ();

#ifdef MCU_AND_CPU_BOARD_CONNECTED
	/* monitor CPU_STATUS stop signal for MAX_CPU_OFF_CHECK_TIME */
	while (cpu_off_wait_time < MAX_CPU_OFF_CHECK_TIME)
	{
		_time_delay(CPU_OFF_CHECK_TIME);
		cpu_off_wait_time += CPU_OFF_CHECK_TIME;
		cpu_status_pin = GPIO_DRV_ReadPinInput (CPU_STATUS);
		if (cpu_status_pin == 0)
		{
			MIC_DEBUG_UART_PRINTF ("Device_off_req: CPU_status pin %d, wait_time %d ms\n", cpu_status_pin, cpu_off_wait_time);
			break;
		}
	}

	/* if the CPU/A8 does not end up being powered off kill the power by manually
	 * turning off the 5V rail. Note, this ends up turning off the LED and Audio
	 * power
	 */
	if (cpu_off_wait_time >= MAX_CPU_OFF_CHECK_TIME)
	{
		MIC_DEBUG_UART_PRINTF ("Device_off_req: WARNING, TURNED OFF 5V0 power rail coz cpu_off_time expired\n");
		GPIO_DRV_ClearPinOutput   (POWER_5V0_ENABLE);	// turn off 5V0 power rail
	}
#endif
	backup_power_cnt_g = 0;
	led_blink_cnt_g = 0;
	GPIO_DRV_ClearPinOutput (LED_GREEN);
	GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	//Board_SetSlowClk ();
	MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: DEVICE IS OFF through Device_off_req\n");
	device_state_g = DEVICE_STATE_OFF;
	Wiggle_sensor_restart ();
	peripherals_disable ();
	Wiggle_sensor_start ();
}

void Device_init (uint32_t delay_period)
{
	device_state_g = DEVICE_STATE_OFF;

	device_control_gpio_g.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.delay_period   = delay_period;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = false;
	device_control_gpio_g.status         = false;
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);

	Wiggle_sensor_init (delay_period);
}

void Device_turn_on  (void)
{
	device_control_gpio_g.time_threshold = DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;	
}

void Device_turn_off (void)
{
	device_control_gpio_g.time_threshold = DEVICE_CONTROL_TIME_OFF_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;	
}

void Device_reset (void)
{
	device_control_gpio_g.time_threshold = DEVICE_CONTROL_TIME_RESET_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;	
}

void Device_control_GPIO (void)
{
	if (device_control_gpio_g.enable == false)
		return;
		
	if (device_control_gpio_g.time <	device_control_gpio_g.time_threshold)
	{
		GPIO_DRV_SetPinOutput   (LED_BLUE);
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = true;
		device_control_gpio_g.time  += device_control_gpio_g.delay_period;
	}
	else
	{
		GPIO_DRV_ClearPinOutput (LED_BLUE);
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = false;
		device_control_gpio_g.time   = 0;
		device_control_gpio_g.enable = false;		
	}
}

bool Device_control_GPIO_status  (void) {return device_control_gpio_g.status;}
DEVICE_STATE_t Device_get_status (void) {return device_state_g;}

void peripherals_enable (void)
{
	uint32_t i;

	GPIO_DRV_ClearPinOutput (FPGA_RSTB);

    GPIO_DRV_SetPinOutput   (POWER_5V0_ENABLE);	// turn on 5V0 power rail
    GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE);	// FPGA Enable
    GPIO_DRV_SetPinOutput   (FPGA_RSTB);

	//GPIO_DRV_SetPinOutput   (CAN1_J1708_PWR_ENABLE);		// Enable CAN1 and J1708
	//GPIO_DRV_SetPinOutput   (CAN2_SWC_PWR_ENABLE);		// Enable CAN2 and SWC

//    GPIO_DRV_ClearPinOutput (USB_OTG_SEL);		// Connect D1 <-> D MCU or HUB
//  GPIO_DRV_SetPinOutput(USB_OTG_SEL);			// Connect D2 <-> D A8 OTG
    GPIO_DRV_ClearPinOutput (USB_OTG_OE);		//Enable OTG/MCU switch

    // Enable USB for DEBUG
    GPIO_DRV_SetPinOutput   (USB_HUB_RSTN);
    GPIO_DRV_SetPinOutput   (USB_ENABLE);

    GPIO_DRV_SetPinOutput   (UART_ENABLE);			// Enable UART
    GPIO_DRV_SetPinOutput   (FTDI_RSTN);

	GPIO_DRV_SetPinOutput (SPKR_LEFT_EN);
	GPIO_DRV_SetPinOutput (SPKR_RIGHT_EN);
//	GPIO_DRV_SetPinOutput (SPKR_EXT_EN);
//	GPIO_DRV_SetPinOutput (CPU_MIC_EN);

    // wait till FPGA is loaded
    for (i = 0; i < 100000; i++)
    {
    	if (GPIO_DRV_ReadPinInput (FPGA_DONE) == 1)
    	{
    		MIC_DEBUG_UART_PRINTF ("\nPOWER_MGM: INFO: FPGA is loaded\n");
    		break;
    	}
    }

    // TODO: need to be removed after tasks enable will be set fro USB protocol
    J1708_enable  (7);

}


void peripherals_disable (void)
{
	GPIO_DRV_ClearPinOutput (LED_RED);
	GPIO_DRV_ClearPinOutput (LED_GREEN);
	GPIO_DRV_ClearPinOutput (LED_BLUE);
	GPIO_DRV_ClearPinOutput (FPGA_PWR_ENABLE);

	//GPIO_DRV_ClearPinOutput (CAN1_J1708_PWR_ENABLE);
	//GPIO_DRV_ClearPinOutput (CAN2_SWC_PWR_ENABLE);

	GPIO_DRV_ClearPinOutput (USB_ENABLE);
	GPIO_DRV_ClearPinOutput (UART_ENABLE);
	GPIO_DRV_ClearPinOutput (SPKR_LEFT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_RIGHT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_EXT_EN);
	GPIO_DRV_ClearPinOutput (CPU_MIC_EN);
}


void send_power_change (uint8_t *power_mask)
{
	packet_t pwm_msg;						// power management message
	pwm_msg.pkt_type = POWER_MGM_STATUS;

	pwm_msg.data[0] = *power_mask;
	send_control_msg(&pwm_msg, 1);
}
