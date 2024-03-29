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
#include "rtc.h"

#include "board.h"
#include "gpio_pins.h"

#include "control_task.h"

// TODO: need to be removed after tasks enable will be set fro USB protocol
#include "fpga_api.h"
#include "J1708_task.h"

#include "Uart_debugTerminal.h"
#include "acc_task.h"
#include "fsl_power_manager.h"
#include "fsl_clock_manager.h"
#include "mqx_prv.h"
#include "watchdog_mgmt.h"
#include <lwtimer.h>
#include "EXT_GPIOS.h"

#define DEVICE_CONTROL_TIME_ON_TH				 3200		// number of mili-seconds pulse for turning device on
#define DEVICE_CONTROL_TIME_OFF_TH				 3200		// number of mili-seconds pulse for turning device off
#define DEVICE_CONTROL_TIME_RESET_TH			  500		// number of mili-seconds pulse for reseting device

#define BACKUP_RECOVER_TIME_TH					 2000		// number of mili-seconds to try power failure overcome (device is powered by supercap)
//NOTE: The BACKUP_POWER_TIME_TH needs to be greater than the OS 'off_delay� timeout which is defaulted to 19sec in OS 21.0 
#define BACKUP_POWER_TIME_TH					24000		// number of mili-seconds to power device by supercap

#define CPU_OFF_CHECK_TIME						1000		// time between checks for CPU/A8 off
#define MAX_CPU_OFF_CHECK_TIME					15000		// Max time to wait before shutting off the unit by killing the power

#define MAX_CPU_TICKS_TAKEN_TO_BOOT				45000/MS_PER_TICK  //maximum time to wait for the CPU_STATUS signal to be received from the A8

#define MCU_AND_CPU_BOARD_CONNECTED

#define TEMPERATURE_DBG_PRINT

typedef struct
{
    // 64-bit time_threshold works for 300 thousands years since MCU started 
	uint64_t 	time_threshold;								// pulse time period
	uint64_t 	time;											// time er
	uint64_t	delay_period;									// time duration that needs to be added to counter
	bool    	enable;										// pulse enable control
	bool    	status;										// pulse status - TRUE as long as pulse is generated
	bool		spare[6];
} DEVICE_CONTROL_GPIO_t;


DEVICE_CONTROL_GPIO_t device_control_gpio_g;
DEVICE_STATE_t        device_state_g;
static uint8_t turn_on_condition_g = 0;

uint32_t backup_power_cnt_g = 0 ;
uint8_t led_blink_cnt_g     = 0 ;
extern uint32_t ignition_threshold_g;
extern volatile uint32_t a8_watchdog_count_g;
extern void * power_up_event_g;
extern void * g_a8_pwr_state_event;
extern void * g_GPIO_event_h;

LWTIMER_PERIOD_STRUCT lwtimer_period_a8_turn_on_g;
LWTIMER_STRUCT lwtimer_a8_turn_on_g;

void Device_control_GPIO        (uint32_t * time_diff);
bool Device_control_GPIO_status (void);
void send_power_change          (uint8_t *power_mask);

void enable_peripheral_clocks(void)
{
  	uint8_t i;
	printf ("%s: enable ports clocks\n", __func__);
	
  for (i = 1; i < PORT_INSTANCE_COUNT; i++)
  {
	CLOCK_SYS_EnablePortClock (i);
  }
}

/* Disable All peripheral clocks except Port A, which is used by Wiggle Sense */
void disable_peripheral_clocks(void)
{
  uint8_t i;
  for (i = 1; i < PORT_INSTANCE_COUNT; i++)
  {
	CLOCK_SYS_DisablePortClock (i);
  }
}
 
/*disables alarm polling before shutting down in case date has already passed 
  this function should be used only in this file so it will stay inlined and won't slow the system*/
void rtc_clear_outdated_alarm()
{
   rtc_get_flags();
   if(rtc_alarm1_is_trigered())
   {
        //in case the alarm has been triggered already 
        //make sure the alarm will be set off from now on by setting the time to 0 
        uint8_t zero_month = 0;
        uint8_t cmd_buff = 0XA;//RTC_ALRM1_MONTH_ADDR

    	if(!rtc_send_data(&cmd_buff,1 ,&zero_month, 1))
    	{
    		printf("rtc_set_alarm1: ERROR: could not update the bytes related to alarm1\n");
    	}
   }
   _event_clear(rtc_flags_g, ALARM1_ACTIVATE_BIT); 
}

void device_state_stringify(DEVICE_STATE_t device_state, char * dev_state_str )
{
	switch 	(device_state)
	{
		case DEVICE_STATE_OFF:
			sprintf(dev_state_str, "DEVICE_STATE_OFF");
			break;
		case DEVICE_STATE_TURNING_ON:
			sprintf(dev_state_str, "DEVICE_STATE_TURNING_ON");
			break;
		case DEVICE_STATE_ON:
			sprintf(dev_state_str, "DEVICE_STATE_ON");
			break;
		case DEVICE_STATE_BACKUP_RECOVERY:
			sprintf(dev_state_str, "DEVICE_STATE_BACKUP_RECOVERY");
			break;
		case DEVICE_STATE_BACKUP_POWER:
			sprintf(dev_state_str, "DEVICE_STATE_BACKUP_POWER");
			break;
		case DEVICE_STATE_TURN_OFF:
			sprintf(dev_state_str, "DEVICE_STATE_TURN_OFF");
			break;
		case DEVICE_STATE_RESET:
			sprintf(dev_state_str, "DEVICE_STATE_RESET");
			break;
		default: 
			sprintf(dev_state_str, "DEVICE_STATE_UNDEFINED");
	}
}

void change_device_state(DEVICE_STATE_t device_state)
{
	char device_state_str[30];
	device_state_stringify(device_state, device_state_str);
	printf("%s: %llu ms, %s\n", __func__, ms_from_start(), device_state_str);
	device_state_g = device_state;
}

uint8_t get_turn_on_reason(uint32_t * ignition_voltage)
{
	uint8_t turn_on_condition = 0;
	if (*ignition_voltage >= ignition_threshold_g)
	{
		printf ("\n%s: POWER_MGM: TURNING ON DEVICE with ignition\n", __func__);
		turn_on_condition |= POWER_MGM_DEVICE_ON_IGNITION_TRIGGER;
	}

	if (Wiggle_sensor_cross_TH ())
	{
		printf ("\n%s: POWER_MGM: TURNING ON DEVICE with wiggle sensor \n", __func__);
		turn_on_condition |= POWER_MGM_DEVICE_ON_WIGGLE_TRIGGER;
	}

	if(RCM_BRD_SRS1_LOCKUP((RCM_Type*)RCM_BASE))
	{
		printf ("\n%s: POWER_MGM: TURNING ON DEVICE due to ARM LOCKUP \n", __func__);
		turn_on_condition |= POWER_MGM_DEVICE_ARM_LOCKUP;
	}

	if(RCM_BRD_SRS0_WDOG((RCM_Type*)RCM_BASE))
	{
		printf ("\n%s: POWER_MGM: TURNING ON DEVICE due to WATCHDOG RESET \n", __func__);
		turn_on_condition |= POWER_MGM_DEVICE_WATCHDOG_RESET;
	}

    if(rtc_alarm1_is_trigered())
    {
        printf ("\n%s: POWER_MGM: TURNING ON DEVICE due to wakeup ALARM TRIGGERED  \n", __func__);
        turn_on_condition |= POWER_MGM_DEVICE_TIMER_ALARM;
    }

	//if(RCM_BRD_SRS1_SW((RCM_Type*)RCM_BASE))//SYSRESETREQ)
	//{
	//  	turn_on_condition |= POWER_MGM_DEVICE_SW_RESET_REQ;
	//}
	return turn_on_condition;
}

void Device_update_state (uint32_t * time_diff)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN   );
	uint32_t ignition_voltage  = ADC_get_value (kADC_ANALOG_IN1 );
	int32_t  temperature       = ADC_get_value (kADC_TEMPERATURE);
	uint32_t supercap_voltage;
	static bool printed_temp_error = FALSE;
	static bool print_backup_power = FALSE;
	uint32_t a8_s;
	uint32_t gpio_event;

	Device_control_GPIO(time_diff);

#ifdef TEMPERATURE_DBG_PRINT
	static uint16_t time_since_temp_print = 0;
	time_since_temp_print += *time_diff;
	if ((device_state_g == DEVICE_STATE_OFF && time_since_temp_print > 45000) //about 10 seconds, coz running @ slower clock
		|| (device_state_g != DEVICE_STATE_OFF && time_since_temp_print > 30000)) // 30 seconds
	{
		printf("%s: Time: %llu ms, dev_state:%d, power_in=%d\n",
			   __func__, ms_from_start(), device_state_g, power_in_voltage);
		_time_delay(5);
		printf("%s: ign_vol=%d, tempx10=%d\n",
			   __func__, ignition_voltage, temperature - 500);
		time_since_temp_print = 0;
	}
#endif

	if (MQX_OK != _event_get_value(g_a8_pwr_state_event, &a8_s))
		a8_s = EVENT_A8_PWR_DOWN;

    if (a8_s&EVENT_A8_PWR_DOWN) {
		uint64_t reset_timeout = ms_from_start() + 1000;
		printf("%s: WARNING EVENT_A8_PWR_DOWN 1\n", __func__);
        while (ms_from_start() < reset_timeout) {
			// WDOG reset
        }
		Device_off_req(1, 0);
		return;
    }

    switch (device_state_g)
	{
		case DEVICE_STATE_OFF:
			turn_on_condition_g = 0;
			// blink RED LED when device is off
			//if      (++led_blink_cnt_g == 8)		GPIO_DRV_SetPinOutput   (LED_RED);
			//else if (  led_blink_cnt_g < 10)		GPIO_DRV_ClearPinOutput (LED_RED);
			//else       led_blink_cnt_g = 0;

			// check amount of vibrations sensed by the wiggle sensor
			// if amount of vibrations is more than TH, it will turn on the device
			// and stop the interrupts for better running efficiency
			Wiggle_sensor_update (time_diff);

			if (MQX_OK == _event_get_value(g_GPIO_event_h, &gpio_event))
			{
				/* To see MCU debug output in DEVICE_OFF state, 
				to help debug, UART is enabled if GPIO7 is high */
				if (gpio_event&(1<<kGPIO_IN7))
				{	
					printf("%s:gpio:%x\n",__func__, gpio_event);
					if (GPIO_INPUT_get_logic_level(kGPIO_IN7))
					{
						GPIO_DRV_SetPinOutput(UART_ENABLE);
						printf("%s:enabling UART_ENABLE\n",__func__);
					}
					else
					{
						printf("%s:disabling UART_ENABLE\n",__func__);
						GPIO_DRV_ClearPinOutput(UART_ENABLE);
					}
					_event_clear(g_GPIO_event_h, gpio_event);
				}	
			}

			if (power_in_voltage < POWER_IN_TURN_ON_TH )
			{
				break;
			}

			if ((temperature < TEMPERATURE_TURNON_MIN_TH  ) ||
				(temperature > TEMPERATURE_TURNON_MAX_TH  )  )
			{
				if (printed_temp_error == FALSE)
				{
					printf("%s: temperature alert! temp: %d mV", __func__,temperature);
					/* To avoid printing the error over and over, we use a flag */
					printed_temp_error = TRUE;
				}
				break;
			}

			turn_on_condition_g = get_turn_on_reason(&ignition_voltage);
			if (turn_on_condition_g != 0)
			{
				led_blink_cnt_g = 0;
				Wiggle_sensor_stop ();						// disable interrupt
				//send_power_change  (&turn_on_condition);
				printf("%s: device turn on temperature: %d mV %llu\n", __func__, temperature, ms_from_start());
				printed_temp_error = FALSE;
				change_device_state(DEVICE_STATE_TURNING_ON);
			}
			break;

		case DEVICE_STATE_TURNING_ON:
			// wait while pulse is still generated (time period didn't reach threshold)
            printf ("%s: DEVICE_STATE_TURNING_ON %llu\n", __func__, ms_from_start());
            if (power_in_voltage < POWER_IN_SHUTDOWN_TH) {
                printf ("%s: DEVICE_STATE_TURNING_ON a8 already/yet down - restart %llu\n", __func__, ms_from_start());
                backup_power_cnt_g = 0;
                led_blink_cnt_g = 0;
                Device_off_req(TRUE, 0);
                break;
            }
			if (!Device_control_GPIO_status())
			{
				switch_power_mode(kPowerManagerRun);
				_bsp_MQX_tick_timer_init ();
				enable_peripheral_clocks();
				peripherals_enable ();
				//Board_SetFastClk ();
				Device_turn_on     ();
				change_device_state(DEVICE_STATE_ON);
				printf ("\nPOWER_MGM: DEVICE RUNNING %llu\n", ms_from_start());
				FPGA_init ();
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); /*Green LED */
				GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
				_event_set(power_up_event_g, 1);
			}
			break;

		case DEVICE_STATE_ON:

			// if power drops below threshold - shutdown
			if (power_in_voltage < POWER_IN_SHUTDOWN_TH)
			{
				printf ("\nPOWER_MGM: WARNING: INPUT POWER LOW %d - going to BACKUP_RECOVERY_STATE !!! \n", power_in_voltage);
				change_device_state(DEVICE_STATE_BACKUP_RECOVERY);
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0, 0xFF); /*Blue LED */
				break;
			}

			// if temperature is out of range - turn off device
			if ((temperature < TEMPERATURE_SHUTDOWN_MIN_TH)   ||
				(temperature > TEMPERATURE_SHUTDOWN_MAX_TH)    )
			{
				printf ("\nPOWER_MGM: TEMPERATURE OUT OF RANGE %d - SHUTING DOWN !!! \n", temperature);
				change_device_state(DEVICE_STATE_OFF);
				Device_off_req(FALSE, 0);
			}
			break;

		case DEVICE_STATE_BACKUP_RECOVERY:
			backup_power_cnt_g += *time_diff;

			// if recovery period has passed - generate power loss interrupt to CPU and close peripherals
			if (backup_power_cnt_g > BACKUP_RECOVER_TIME_TH) {
                GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
                _time_delay(1);
				GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0xFF, 0); /* Yellow LED*/
				//peripherals_disable ();
				//switch_power_mode(kPowerManagerVlpr);
				//Board_SetVerySlowClk ();
				change_device_state(DEVICE_STATE_BACKUP_POWER);
				printf ("\nPOWER_MGM: Recovery period is over, going to BACKUP_POWER_STATE\n");
				break;
			}

			if (!(a8_s&EVENT_A8_PWR_UP)) {
				printf("%s: WARNING !EVENT_A8_PWR_UP 2\n", __func__);
				backup_power_cnt_g = 0;
				led_blink_cnt_g = 0;
				Device_off_req(1, 0);
				break;
			}
			// if power is back during recovery period - return to DEVICE_ON state, like nothing happen
			if (power_in_voltage >= POWER_IN_TURN_ON_TH)
			{
				printf ("\nPOWER_MGM: INPUT POWER OK %d, going to ON_STATE\n", power_in_voltage);
				backup_power_cnt_g = 0;
				change_device_state(DEVICE_STATE_ON);
                FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); /*Green LED */
			}
			break;

		case DEVICE_STATE_BACKUP_POWER:
			if (power_in_voltage >= POWER_IN_TURN_ON_TH && !print_backup_power)
			{
				printf ("\nPOWER_MGM: DEVICE_STATE_BACKUP_POWER state, power returned\n");
				print_backup_power = TRUE; /*only print power returned once */
			}

			if (!(a8_s&EVENT_A8_PWR_UP)) {
				printf("%s: WARNING !EVENT_A8_PWR_UP 3\n", __func__);
				backup_power_cnt_g = 0;
				led_blink_cnt_g = 0;
				Device_off_req(1, 0);
				break;
			}
            backup_power_cnt_g += *time_diff;
			if (backup_power_cnt_g > BACKUP_POWER_TIME_TH) {
//				GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
				backup_power_cnt_g = 0;
				led_blink_cnt_g = 0;
				Device_turn_off ();
				change_device_state(DEVICE_STATE_TURN_OFF);
				supercap_voltage = ADC_get_value (kADC_POWER_VCAP);
				printf ("\nPOWER_MGM: backup period is over - shutting down, supercap voltage = %d\n", supercap_voltage);
			}
			break;

	case DEVICE_STATE_TURN_OFF:
			// wait while pulse is still generated (time period didn't reach threshold)

            // device should be off w/o unconditionally, moreover pulse will produce inside off request
			//if (!Device_control_GPIO_status())
			{
				printf ("\nPOWER_MGM: DEVICE IS about to turn OFF\n");
				Device_off_req(FALSE, 0);
				change_device_state(DEVICE_STATE_OFF);
				//Wiggle_sensor_restart ();
				//peripherals_disable ();
				//Wiggle_sensor_start ();	// enable interrupt
				//disable_peripheral_clocks();
			}
			break;

		default:
			printf ("\nPOWER_MGM: ERROR: UNKNOWN STATE %d\n", device_state_g );
			change_device_state(DEVICE_STATE_OFF);
			Wiggle_sensor_restart ();
			enable_peripheral_clocks();

			break;
	}
}

void Device_get_turn_on_reason(uint8_t * turn_on_reason)
{
	*turn_on_reason = turn_on_condition_g;
}

void Device_off_req_immediate(bool clean_reset)
{
	uint32_t cpu_off_wait_time = 0;
	uint8_t cpu_status_pin = 0;

    rtc_clear_outdated_alarm();

	/* Disable the Accelerometer and RTC because we were seeing I2C issues where 
	SCL line was stuck on boot up */
	AccDisable();
	/* Shut off power to the accelerometer */
	GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);
	backup_power_cnt_g = 0;
	led_blink_cnt_g = 0;
	/* Shut off power to the accelerometer */
	GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);

	enable_msm_power(FALSE);		// turn off 5V0 power rail
    GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);

	if (clean_reset)
	{
		NVIC_SystemReset();
	}
	else
	{
		WDG_RESET_MCU();
	}
}

void Device_off_req(bool skip_a8_shutdown, uint8_t wait_time)
{
	uint32_t cpu_off_wait_time = 0;
	uint8_t cpu_status_pin = 0;
	volatile static bool device_off_req_in_progress = 0;

    //shut down alarm if it is already outdated
    rtc_clear_outdated_alarm();

    printf("%s: [%d, %d] %llu\n", __func__, skip_a8_shutdown, wait_time, ms_from_start());
    /* If this command is called from a different thread, it will not execute again while it is already being performed */
    while (device_off_req_in_progress) {
    	_time_delay(1000);
    }

	device_off_req_in_progress = TRUE;

//    GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0, 0); /* Red LED*/
	_time_delay(wait_time*1000);
	
	/* Disable the Accelerometer and RTC because we were seeing I2C issues where 
	SCL line was stuck on boot up */
	AccDisable();
	/* Shut off power to the accelerometer */
	GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);

    // only one shutdown way should be selecteed
    // by power button or power loss indication
    // the defuault power lost configuration of a8 is 20 secs, it will fail w/o shutdown in any case
    //
	if (skip_a8_shutdown) {
        GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
    } else {
#ifdef MCU_AND_CPU_BOARD_CONNECTED
    	printf ("%s: shutting down A8\n", __func__);
    	/* Turn A8 device off */
    	GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
    	_time_delay (DEVICE_CONTROL_TIME_OFF_TH);
    	GPIO_DRV_SetPinOutput(CPU_ON_OFF);

        /* monitor CPU_STATUS stop signal for MAX_CPU_OFF_CHECK_TIME */
        while (cpu_off_wait_time < MAX_CPU_OFF_CHECK_TIME) {
            _time_delay(CPU_OFF_CHECK_TIME);
            cpu_off_wait_time += CPU_OFF_CHECK_TIME;
            cpu_status_pin = GPIO_DRV_ReadPinInput (CPU_STATUS);
            if (cpu_status_pin == 0)
            {
                printf ("%s: CPU_status pin %d, wait_time %d ms\n", __func__, cpu_status_pin, cpu_off_wait_time);
                break;
            }
        }
#endif
	}

	if (cpu_off_wait_time >= MAX_CPU_OFF_CHECK_TIME)
    {
        printf ("%s: WARNING, cpu_off_wait_time expired %d\n", __func__, cpu_off_wait_time);
    }

    printf ("%s: TURNED OFF 5V0 power rail\n", __func__);

    // a8 power should be cut in any case
    enable_msm_power(FALSE);		// turn off 5V0 power rail

    backup_power_cnt_g = 0;
	led_blink_cnt_g = 0;
	GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	printf ("\nPOWER_MGM: DEVICE IS OFF through Device_off_req\n");

    // for debug outpt only, should be removed
    //_time_delay(100);

	NVIC_SystemReset();

    // this function shouldn't return;
    while (1) {
    }

    device_off_req_in_progress = FALSE;
}

// delay in msec
void Device_reset_req(int32_t wait_time)
{
	int32_t cpu_off_wait_time;
	uint8_t cpu_status_pin = 0;

    GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
    _time_delay(1);
	GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
	FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0, 0); /* Red LED*/

    rtc_clear_outdated_alarm();

	cpu_off_wait_time = CPU_OFF_CHECK_TIME*((wait_time + (CPU_OFF_CHECK_TIME >> 1))/CPU_OFF_CHECK_TIME);
	if (cpu_off_wait_time > MAX_CPU_OFF_CHECK_TIME)
	  cpu_off_wait_time = MAX_CPU_OFF_CHECK_TIME;
	
	_time_delay(cpu_off_wait_time);

#ifdef MCU_AND_CPU_BOARD_CONNECTED
	cpu_status_pin = GPIO_DRV_ReadPinInput (CPU_STATUS);
	while (cpu_off_wait_time) {
	  	/* monitor CPU_STATUS stop signal for MAX_CPU_OFF_CHECK_TIME */
		if (cpu_status_pin != GPIO_DRV_ReadPinInput (CPU_STATUS)) {
			printf ("Device_off_req: CPU_status pin %d, wait_time %d ms\n", cpu_status_pin, cpu_off_wait_time);
			_time_delay(200);
			cpu_status_pin = 2;
			break;
		}
		
		_time_delay(5);
		cpu_off_wait_time -= 5;
	}
#endif
	
	// turning off the 5V rail always.
	// printf ("Device_off_req: WARNING, TURNED OFF 5V0 power rail coz cpu_off_time expired\n");
	// Turn device off
	if (2 != cpu_status_pin) {
		printf ("%s: try shutdown A8 by power button\n", __func__);
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		_time_delay (DEVICE_CONTROL_TIME_OFF_TH);
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
	}
	
	enable_msm_power(FALSE);		// turn off 5V0 power rail

	GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	printf ("\nPOWER_MGM: DEVICE reset through Device_reset_req\n");
	peripherals_disable (1);
	change_device_state(DEVICE_STATE_OFF);

	// Vladimir
	// TODO: should be investigated influence of GPIOA11 and GPIOE25 pins during reset
	// default states
	// 	output register: 		0
	//	direction register:		inputs
	// PTA11: disabled
	// PTE25: ADC0SE18
	WDG_RESET_MCU();
	
}

void Device_init (uint32_t delay_period)
{
	change_device_state(DEVICE_STATE_OFF);
	TIME_STRUCT ticks_now;

	_time_get_elapsed(&ticks_now);

	device_control_gpio_g.time_threshold = (uint64_t)1000*ticks_now.SECONDS + ticks_now.MILLISECONDS + DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.delay_period   = delay_period;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = false;
	device_control_gpio_g.status         = false;
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);

	Wiggle_sensor_init (delay_period);
}

void Device_turn_on  (void)
{
	TIME_STRUCT ticks_now;

	_time_get_elapsed(&ticks_now);

	device_control_gpio_g.time_threshold = (uint64_t)1000*ticks_now.SECONDS + ticks_now.MILLISECONDS + DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;

	/* Create a timer that calls a watchdog reset if the A8 does NOT turn ON in MAX_CPU_TICKS_TAKEN_TO_BOOT */
	/* NOTE: This timer NEEDs to be cancelled if a successful bootup happens */
	_lwtimer_create_periodic_queue(&lwtimer_period_a8_turn_on_g, MAX_CPU_TICKS_TAKEN_TO_BOOT, MAX_CPU_TICKS_TAKEN_TO_BOOT);
	_lwtimer_add_timer_to_queue(&lwtimer_period_a8_turn_on_g, &lwtimer_a8_turn_on_g, 0, \
		(LWTIMER_ISR_FPTR)handle_watchdog_expiry, 0);
}

void Device_turn_off (void)
{
	TIME_STRUCT ticks_now;

	_time_get_elapsed(&ticks_now);

	device_control_gpio_g.time_threshold = (uint64_t)1000*ticks_now.SECONDS + ticks_now.MILLISECONDS + DEVICE_CONTROL_TIME_OFF_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;
}

void Device_reset (void)
{
	TIME_STRUCT ticks_now;

	_time_get_elapsed(&ticks_now);

	device_control_gpio_g.time_threshold = (uint64_t)1000*ticks_now.SECONDS + ticks_now.MILLISECONDS + DEVICE_CONTROL_TIME_RESET_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;
}

void Device_control_GPIO (uint32_t * time_diff)
{
	TIME_STRUCT ticks_now;
	uint64_t ms;

	_time_get_elapsed(&ticks_now);
	ms = (uint64_t)1000*ticks_now.SECONDS + ticks_now.MILLISECONDS;

	if (device_control_gpio_g.enable == false)
		return;

	if (ms < device_control_gpio_g.time_threshold) {
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = true;
		device_control_gpio_g.time  += *time_diff;
	} else {
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = false;
		device_control_gpio_g.time   = 0;
		device_control_gpio_g.enable = false;
        printf("%s: power pulse done %d ms\n", __func__, ms);
	}
}

bool Device_control_GPIO_status  (void) {return device_control_gpio_g.status;}
DEVICE_STATE_t Device_get_status (void) {return device_state_g;}

void peripherals_enable (void)
{
	uint32_t i;
	uint32_t total_wait_time = 0;

	GPIO_DRV_ClearPinOutput (FPGA_RSTB);

//	enable_msm_power(TRUE);		// turn on 5V0 power rail
	GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE);	// FPGA Enable
	GPIO_DRV_SetPinOutput   (FPGA_RSTB);

	//AccEnable();
	/* keep CAN1, J1708, CAN2 and SWC disabled by default. A software command
	will enable it once the OS boots up */
	GPIO_DRV_ClearPinOutput(CAN1_J1708_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput(CAN1_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput(J1708_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput(CAN2_SWC_PWR_ENABLE);

//    GPIO_DRV_ClearPinOutput (USB_OTG_SEL);		// Connect D1 <-> D MCU or HUB
//  GPIO_DRV_SetPinOutput(USB_OTG_SEL);			// Connect D2 <-> D A8 OTG
	GPIO_DRV_ClearPinOutput (USB_OTG_OE);		//Enable OTG/MCU switch

	// Enable USB for DEBUG
	GPIO_DRV_SetPinOutput   (USB_HUB_RSTN);
	GPIO_DRV_SetPinOutput   (USB_ENABLE);

	GPIO_DRV_SetPinOutput   (UART_ENABLE);			// Enable UART
	GPIO_DRV_SetPinOutput   (FTDI_RSTN);

	/* Only enable the speakers if CPU_SPKR_EN is set */
	if (GPIO_DRV_ReadPinInput(CPU_SPKR_EN))
	{
		GPIO_DRV_SetPinOutput (SPKR_RIGHT_EN);
		GPIO_DRV_SetPinOutput (SPKR_LEFT_EN);
		GPIO_DRV_SetPinOutput (SPKR_EXT_EN);
	}
	else
	{
		GPIO_DRV_ClearPinOutput(SPKR_RIGHT_EN);
		GPIO_DRV_ClearPinOutput(SPKR_LEFT_EN);
		GPIO_DRV_ClearPinOutput(SPKR_EXT_EN);
	}
	
//	GPIO_DRV_SetPinOutput (SPKR_EXT_EN);
//	GPIO_DRV_SetPinOutput (CPU_MIC_EN);
	GPIO_DRV_SetPinOutput (EXT_GPS_EN);

	// wait till FPGA is loaded
	printf("%s: wait for fpga ready %d ms\n", __func__, total_wait_time);
	while(1)
	{
		if (GPIO_DRV_ReadPinInput (FPGA_DONE) == 1)
		{
			printf ("%s: FPGA is loaded on %d ms\n", __func__, total_wait_time );
			break;
		}
		_time_delay(1);
		total_wait_time++;
		/* Measured to take 71ms typically (tested on 1 board, 10 reboot cycles) - Abid */
		if(total_wait_time > 10000)
		{
            PORT_HAL_SetPinIntMode (PORTC, GPIO_EXTRACT_PIN(FPGA_GPIO0), kPortIntDisabled);
            GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
			printf("%s: FPGA FAILED TO LOAD!... %d ms\n", total_wait_time);
			break;
		}
	}

	printf("%s: enable acc interrupts %d ms\n", __func__, total_wait_time);
	PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntLogicZero);
	GPIO_DRV_ClearPinIntFlag(ACC_INT);
	// TODO: need to be removed after tasks enable will be set fro USB protocol
	//J1708_enable  (7);

}


void peripherals_disable (uint32_t WithFpga)
{
  	// disable FPGA based resources
	PORT_HAL_SetPinIntMode (PORTC, GPIO_EXTRACT_PIN(FPGA_GPIO0), kPortIntDisabled);
	GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
	GPIO_DRV_ClearPinIntFlag(ACC_INT);
	PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntDisabled);
	GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);

	if(WithFpga)
	{  
		// Down FPGA
		GPIO_DRV_ClearPinOutput(FPGA_RSTB);
		GPIO_DRV_ClearPinOutput(FPGA_PWR_ENABLE);
	}
	
	GPIO_DRV_ClearPinOutput (CAN1_J1708_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput (J1708_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput(CAN1_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput (CAN2_SWC_PWR_ENABLE);

    GPIO_DRV_ClearPinOutput (USB_HUB_RSTN);
	GPIO_DRV_ClearPinOutput (FTDI_RSTN);
	GPIO_DRV_ClearPinOutput (USB_ENABLE);
	GPIO_DRV_ClearPinOutput (UART_ENABLE);
	GPIO_DRV_ClearPinOutput (SPKR_LEFT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_RIGHT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_EXT_EN);
	GPIO_DRV_ClearPinOutput (CPU_MIC_EN);
	GPIO_DRV_ClearPinOutput (EXT_GPS_EN);
	//AccDisable();
    GPIO_DRV_SetPinOutput (USB_OTG_OE);		//Disable OTG/MCU switch
}


void send_power_change (uint8_t *power_mask)
{
	packet_t pwm_msg;						// power management message
	pwm_msg.pkt_type = POWER_MGM_STATUS;

	pwm_msg.data[0] = *power_mask;
	send_control_msg(&pwm_msg, 1);
}
