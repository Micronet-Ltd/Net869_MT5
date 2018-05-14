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
//#include "wiggle_sensor.h"

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
#include "main_tasks.h"
#include "mic_typedef.h"

#define DEVICE_CONTROL_TIME_ON_TH				 3200		// number of mili-seconds pulse for turning device on
#define DEVICE_CONTROL_TIME_OFF_TH				 3200		// number of mili-seconds pulse for turning device off
#define DEVICE_CONTROL_TIME_RESET_TH			  500		// number of mili-seconds pulse for reseting device

#define BACKUP_RECOVER_TIME_TH					 2000		// number of mili-seconds to try power failure overcome (device is powered by supercap)
//NOTE: The BACKUP_POWER_TIME_TH needs to be greater than the OS 'off_delay’ timeout which is defaulted to 19sec in OS 21.0 
#define BACKUP_POWER_TIME_TH					24000		// number of mili-seconds to power device by supercap

#define CPU_OFF_CHECK_TIME						1000		// time between checks for CPU/A8 off
#define MAX_CPU_OFF_CHECK_TIME					15000		// Max time to wait before shutting off the unit by killing the power

#define MAX_CPU_TICKS_TAKEN_TO_BOOT				45000/MS_PER_TICK  //maximum time to wait for the CPU_STATUS signal to be received from the A8

#define MCU_AND_CPU_BOARD_CONNECTED

#define TEMPERATURE_DBG_PRINT
//#define SUSPEND_DEBUG
void Pulse_Device(uint32_t period);

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
static uint32_t turn_on_condition_g = 0;

uint32_t backup_power_cnt_g = 0 ;
uint8_t led_blink_cnt_g     = 0 ;
extern uint32_t ignition_threshold_g;
extern volatile uint32_t a8_watchdog_count_g;
extern void * power_up_event_g;

LWTIMER_PERIOD_STRUCT lwtimer_period_a8_turn_on_g;
LWTIMER_STRUCT lwtimer_a8_turn_on_g;

extern void * cpu_int_suspend_event_g;

extern int32_t g_MT5_present;

void Device_control_GPIO        (uint32_t * time_diff);
bool Device_control_GPIO_status (void);
void send_power_change          (uint8_t *power_mask);

extern void configure_otg_for_host_or_device(int);
//extern uint64_t g_wd_fall_time;
//extern uint64_t g_wd_rise_time;
extern uint64_t g_last_state_time;
extern uint64_t g_last_rf_int_time;

int32_t	g_on_flag = 0;

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

uint32_t get_turn_on_reason(uint32_t * ignition_voltage)
{
	uint32_t turn_on_condition = 0;
	if (*ignition_voltage >= ignition_threshold_g)
	{
		//printf ("\nPOWER_MGM: TURNING ON DEVICE with ignition\n");
		turn_on_condition |= POWER_MGM_DEVICE_ON_IGNITION_TRIGGER;
	}

//	if (Wiggle_sensor_cross_TH ())
//	{
//		printf ("\nPOWER_MGM: TURNING ON DEVICE with wiggle sensor \n");
//		turn_on_condition |= POWER_MGM_DEVICE_ON_WIGGLE_TRIGGER;
//	}

	if(RCM_BRD_SRS1_LOCKUP((RCM_Type*)RCM_BASE))
	{
		printf ("\nPOWER_MGM: TURNING ON DEVICE due to ARM LOCKUP \n");
//		turn_on_condition |= POWER_MGM_DEVICE_ARM_LOCKUP;
	}

//	
	if(RCM_BRD_SRS0_WDOG((RCM_Type*)RCM_BASE))
	{
//		printf ("\nPOWER_MGM: TURNING ON DEVICE due to WATCHDOG RESET \n");
//		turn_on_condition |= POWER_MGM_DEVICE_WATCHDOG_RESET; //
	}

	//if(RCM_BRD_SRS1_SW((RCM_Type*)RCM_BASE))//SYSRESETREQ)
	//{
	//  	turn_on_condition |= POWER_MGM_DEVICE_SW_RESET_REQ;
	//}

	return turn_on_condition;
}
void set_run_mode(int32_t fOn, int32_t fForce)
{
	uint32_t freq = 0;
	static int32_t last_tst = 0;
	if( (fOn == last_tst) && !fForce)
		return;
	
	printf("%s: %d - %d\n", __func__, fOn, fForce);//temp
	if(fOn)
	{
		switch_power_mode(kPowerManagerRun);
		_bsp_MQX_tick_timer_init ();
		enable_peripheral_clocks();
		peripherals_enable ();
		FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); /*Green LED */
	}
	else
	{
//		configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_BYPASS);
		configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_DISABLE);

		switch_power_mode(kPowerManagerVlpr);
		_bsp_MQX_tick_timer_init ();
		if(fOn	== last_tst)
			FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0xFF); /*Green Blue LED */
		peripherals_disable(1);
		disable_peripheral_clocks();

		CLOCK_SYS_EnablePortClock (PORTB_IDX); //Enable PortB clock so we can still read the WD signal in MSM suspend
		CLOCK_SYS_EnablePortClock (PORTC_IDX);//to read RF_KILL
		// Enable power to the vibration sensor and accelerometer
		//GPIO_DRV_SetPinOutput(ACC_VIB_ENABLE);
		//Wiggle_sensor_start();
		//Wiggle_sensor_restart();
	}
	last_tst = fOn;					
}
void set_otg_id(int32_t fOn, int32_t fForce)
{
	static int32_t last = -1;
	if(last != fOn || fForce)
	{
		if(fOn)
			GPIO_DRV_SetPinOutput (CPU_OTG_ID);
		else
			GPIO_DRV_ClearPinOutput (CPU_OTG_ID);

		printf("%s: %d\n", __func__, fOn);
		last = fOn;
	}
}
void set_credle_detect(int fOn, int fForce)
{
	static int32_t last = -1;
	if(last != fOn || fForce)
	{
		if(fOn)
		{
			GPIO_DRV_SetPinDir(CPU_INT, kGpioDigitalOutput);
			GPIO_DRV_WritePinOutput(CPU_INT, 0);
		}
		else
		{
			GPIO_DRV_SetPinDir(CPU_INT, kGpioDigitalInput);
		}
		printf("%s: %d\n", __func__, fOn);
		last = fOn;
	}
}
				
void Device_update_state (uint32_t * time_diff)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN   );
	uint32_t ignition_voltage  = ADC_get_value (kADC_ANALOG_IN1 );
	int32_t  temperature       = ADC_get_value (kADC_TEMPERATURE);
	uint32_t supercap_voltage;
	static bool printed_temp_error = FALSE;
	static bool print_backup_power = FALSE;
	_mqx_uint event_bits = 0;
	_mqx_uint event_result = MQX_OK;
	uint32_t freq = 0;
	static uint64_t wait_on_timeout = 0;
	static uint64_t wait_off_timeout = 0;
	static uint64_t start_timeout = 0;
	
	static uint64_t now_time = 0;
	
	static int	start_count = 0;
	int dev_present = 0;
//	static uint32_t last_wiggle_condition = 0;
	int32_t MT5_present;
	static int32_t last_MT5_present = 0;
	static int fCritBattery = 0;	
	
#ifdef TEMPERATURE_DBG_PRINT
	static uint16_t time_since_temp_print = 0;
	time_since_temp_print += *time_diff;
	if ((device_state_g == DEVICE_STATE_OFF && time_since_temp_print > 45000) //about 10 seconds, coz running @ slower clock
		|| (device_state_g != DEVICE_STATE_OFF && time_since_temp_print > 30000)) // 30 seconds
	{
		printf("temp x 10 : %d c \n", temperature-500);
		time_since_temp_print = 0;
	}
#endif

	MT5_present = g_MT5_present;
	now_time = ms_from_start();
	
	
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
//			Wiggle_sensor_update ();

			if (power_in_voltage < POWER_IN_TURN_ON_TH )
			{
				printf("%s: power_in_voltage alert! %d\n", __func__, power_in_voltage);
				break;
			}

			if ((temperature < TEMPERATURE_TURNON_MIN_TH  ) ||
				(temperature > TEMPERATURE_TURNON_MAX_TH  )  )
			{
				if (printed_temp_error == FALSE)
				{
					printf("%s: temperature alert! temp: %d mV\n", __func__,temperature);
					/* To avoid printing the error over and over, we use a flag */
					printed_temp_error = TRUE;
				}
				break;
			}
			
			if(now_time < wait_off_timeout)// wait for stability 
			{
				break;
			}
			
			turn_on_condition_g = get_turn_on_reason(&ignition_voltage);

			dev_present = GPIO_DRV_ReadPinInput(CPU_RF_KILL);
///
			if(fCritBattery)
			{
				if(MT5_out == MT5_present)
				{
					if(now_time < start_timeout)
						break;
				}
				else
				{
					start_timeout = 0;
					fCritBattery = 0;
					printf("%s: exit from critical [%llu]: present %d [%d]\n", __func__, ms_from_start(), dev_present, MT5_present);
				}
			}			   
///			
			if( (now_time < wait_on_timeout) && (MT5_active_on != MT5_present) )
			{
				break;
			}

//			if( (0 == dev_present) && (MT5_inside != MT5_present) )//undefined intermediate state
//			{
//				break;///do nothing - wait for changes - maybe reset after timeout
//			}
			
			if((turn_on_condition_g != 0) && (0 == start_count) && MT5_inside == MT5_present)
			{				
				printf("%s: short pulse on[%llu]: present %d [%d], condition %d\n", __func__, 
					   ms_from_start(), dev_present, MT5_present, turn_on_condition_g);

				Pulse_Device(40);//short pulse for suspended device
				wait_on_timeout = ms_from_start() + 6500;
//				last_wiggle_condition = (turn_on_condition_g & POWER_MGM_DEVICE_ON_WIGGLE_TRIGGER);//store
				start_count = 1;
                //enable_msm_power(0, 0);//temp!!!???
				break;
			}
			
//			turn_on_condition_g |= last_wiggle_condition;
//			last_wiggle_condition = 0;

			start_count = 0;
			
			if( fCritBattery || ((turn_on_condition_g != 0) && (MT5_out != MT5_present)) || (MT5_active_on == MT5_present) )
			{			
				if( (fCritBattery && (MT5_out == MT5_present) && (1 == dev_present)) ||
				   ((MT5_inside == MT5_present) && (0 == dev_present) && ((now_time - g_last_rf_int_time) > 4000) ) )
				{
					printf("%s: pulse on - delta rf %llu [%llu]\n", __func__, now_time - g_last_rf_int_time, ms_from_start());
					Pulse_Device(DEVICE_CONTROL_TIME_ON_TH);
				}
				else if((0 == turn_on_condition_g) && 
						((0 == dev_present) || ((now_time - g_last_state_time) < 4000)) )//suspend - wait to condition
				{
					//printf("%s: do nothing - delta state %llu [%llu]\n", __func__, now_time - g_last_state_time, ms_from_start());
					break;
				}
				printf("%s: device turn on: present %d [%d], condition %d [%llu]\n", __func__, dev_present, MT5_present, turn_on_condition_g, ms_from_start());
				led_blink_cnt_g = 0;
//				Wiggle_sensor_stop ();
				printed_temp_error = FALSE;
				device_state_g = DEVICE_STATE_TURNING_ON;				
			}
			else if( (MT5_out == MT5_present) && (1 == dev_present) && !fCritBattery)
			{
				set_otg_id(1, 0);
				enable_msm_power(0, 0);
				set_credle_detect(1, 0);				
			}
			break;

		case DEVICE_STATE_TURNING_ON:
			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status())
			{
				set_run_mode(1, 0);
				_event_set(power_up_event_g, 1);//for the 1st on only
				device_state_g = DEVICE_STATE_ON;
				printf ("\nPOWER_MGM: DEVICE RUNNING %llu ms\n", ms_from_start());
				wait_on_timeout = ms_from_start() + 21000;
				fCritBattery = 0;
			}
			break;

		case DEVICE_STATE_ON:
		{	
			// if power drops below threshold - shutdown
			if (power_in_voltage < POWER_IN_SHUTDOWN_TH)
			{
				printf ("\nPOWER_MGM: WARNING: INPUT POWER LOW %d - SHUTING DOWN !!! \n", power_in_voltage);
				device_state_g = DEVICE_STATE_BACKUP_RECOVERY;
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0, 0xFF); /*Blue LED */
				break;
			}

			// if temperature is out of range - turn off device
			if ((temperature < TEMPERATURE_SHUTDOWN_MIN_TH)   ||
				(temperature > TEMPERATURE_SHUTDOWN_MAX_TH)    )
			{
				printf ("\nPOWER_MGM: TEMPERATURE OUT OF RANGE %d - SHUTING DOWN !!! \n", temperature);
				device_state_g = DEVICE_STATE_OFF;
				Device_off_req(FALSE, 0);
			}
			if(MT5_active_on == MT5_present)
			{
				if(wait_on_timeout)//1st
				{
					uint64_t cur_time =  ms_from_start();
					printf("%s: from power on %llu [%llu]\n", __func__, (cur_time - (wait_on_timeout - 21000)), cur_time);
					wait_on_timeout = 0;
					g_on_flag = 1;
				}
			}
			else// if(MT5_out == MT5_present || MT5_inside == MT5_present) or CPU_RF_KILL == 0
			{
				if(ms_from_start() > wait_on_timeout) 
				{
					wait_off_timeout = ms_from_start() + 2000;
					if( (0 == g_on_flag) )//&& (MT5_inside == MT5_present) )// maybe critical 
					{
						fCritBattery = 1;
						enable_msm_power(1, 0);
						start_timeout = ms_from_start() + (1 * 60 * 1000);//delay befor next On 
						printf("%s: not started!!! - present %d [%d] to %llu\n", __func__, (GPIO_DRV_ReadPinInput(CPU_RF_KILL)), MT5_present, start_timeout);
					}
					else
					{
						printf("%s: dev off - present %d [%d]. delay to %llu\n", __func__, (GPIO_DRV_ReadPinInput(CPU_RF_KILL)), MT5_present, wait_off_timeout );
					}						
					g_on_flag = 0;
					device_state_g = DEVICE_STATE_OFF;
					set_run_mode(0, 0);

					break;
				}
			}
//			break;
//		}	
			event_result = _event_get_value(cpu_int_suspend_event_g, &event_bits)  ;
			if (event_result == MQX_OK)
			{

#ifdef SUSPEND_DEBUG
				static uint32_t time_in_on_state = 0;
				time_in_on_state += *time_diff;
				if ((event_bits & EVENT_CPU_INT_SUSPEND_HIGH) || (time_in_on_state > 10000))
					time_in_on_state = 0;
#else
				if ((event_bits & EVENT_CPU_INT_SUSPEND_HIGH) )	
#endif
				{
					_event_clear(cpu_int_suspend_event_g, EVENT_CPU_INT_SUSPEND_HIGH);
					_event_clear(cpu_int_suspend_event_g, EVENT_CPU_INT_SUSPEND_LOW);
					//Changing the device_state_g to OS suspended early because other tasks use this flag to go to a dormant state
					device_state_g = DEVICE_STATE_ON_OS_SUSPENDED;
					printf("%s: cpu_int_suspend_event_g high \n", __func__);
					printf("\n%s: Switched to DEVICE_STATE_ON_OS_SUSPENDED  \n", __func__);

					/* Disable OS watchdog */

					/* Pause tasks that capture data */
					/* Go into lower power mode */
					CLOCK_SYS_GetFreq(kCoreClock, &freq);
					switch_power_mode(kPowerManagerVlpr);
					/* Start off with the peripherals disabled */
					FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0xFF); /*Green Blue LED */
					disable_peripheral_clocks();
					//CLOCK_SYS_EnablePortClock (PORTB_IDX); //Enable PortB clock so we can still read the WD signal in MSM suspend
					CLOCK_SYS_EnablePortClock (PORTC_IDX); //Enable PortB clock so we can still read the WD signal in MSM suspend
					peripherals_disable (false);
					_bsp_MQX_tick_timer_init ();
					/* Enable power to the vibration sensor and accelerometer */
					//GPIO_DRV_SetPinOutput(ACC_VIB_ENABLE);

					/* Enable Wake Source monitoring */
//					Wiggle_sensor_start();
//					Wiggle_sensor_restart();
					configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_NONE); /* Needs to be done after changing state */
				}
			}
			break;
		}
		case DEVICE_STATE_ON_OS_SUSPENDED: /* has DEVICE_STATE_ON and DEVICE_STATE_OFF code */
			turn_on_condition_g = 0;
//			Wiggle_sensor_update();

			// if temperature is out of range - turn off device
			if ((temperature < TEMPERATURE_SHUTDOWN_MIN_TH)   ||
				(temperature > TEMPERATURE_SHUTDOWN_MAX_TH)    )
			{
				printf ("\nPOWER_MGM: TEMPERATURE OUT OF RANGE %d - SHUTING DOWN !!! \n", temperature);
				device_state_g = DEVICE_STATE_OFF;
				Device_off_req(FALSE, 0);
			}

			turn_on_condition_g = get_turn_on_reason(&ignition_voltage);

			// if power drops below threshold - get out of suspend and follow normal shutdown process
			if (power_in_voltage < POWER_IN_SHUTDOWN_TH)
			{
				printf ("\nPOWER_MGM: WARNING: INPUT POWER LOW %d - waking up from suspend !!! \n", power_in_voltage);
			}

			event_result = _event_get_value(cpu_int_suspend_event_g, &event_bits)  ;
			if (event_result == MQX_OK)
			{
				if (event_bits & EVENT_CPU_INT_SUSPEND_LOW)
				{
					_event_clear(cpu_int_suspend_event_g, EVENT_CPU_INT_SUSPEND_LOW);
					printf("%s: cpu_int_suspend_event_g low \n", __func__);
					turn_on_condition_g |= POWER_MGM_DEVICE_MSM_WAKEUP ;
					/* If we don't get EVENT_CPU_INT_SUSPEND_LOW event after a wakeup request we should reset the device */
				}
			}

			if ((turn_on_condition_g != 0) || (power_in_voltage < POWER_IN_SHUTDOWN_TH))
			{
				led_blink_cnt_g = 0;
//				Wiggle_sensor_stop ();						// disable interrupt

				/* Go into full power mode */
				switch_power_mode(kPowerManagerRun);
				enable_peripheral_clocks();
				peripherals_enable ();
				_bsp_MQX_tick_timer_init ();
				FPGA_init ();

				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); /*Green LED */
				device_state_g = DEVICE_STATE_ON;
				configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_NONE); /* Needs to be done after changing state */
				printf("\n%s: Switched to DEVICE_STATE_ON  \n", __func__);
			}
			break;

		case DEVICE_STATE_BACKUP_RECOVERY:
			backup_power_cnt_g += *time_diff;

			// if recovery period has passed - generate power loss interrupt to CPU and close peripherals
			if (backup_power_cnt_g > BACKUP_RECOVER_TIME_TH)
			{
				GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0xFF, 0); /* Yellow LED*/
				//peripherals_disable ();
				//switch_power_mode(kPowerManagerVlpr);
				//Board_SetVerySlowClk ();
				device_state_g = DEVICE_STATE_BACKUP_POWER;
				printf ("\nPOWER_MGM: Recovery period is over\n");
				break;
			}

			// if power is back during recovery period - return to DEVICE_ON state, like nothing happen
			if (power_in_voltage >= POWER_IN_TURN_ON_TH)
			{
				printf ("\nPOWER_MGM: INPUT POWER OK %d\n", power_in_voltage);
				backup_power_cnt_g = 0;
				device_state_g = DEVICE_STATE_ON;
				FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0, 0xFF, 0); /*Green LED */
			}
			break;

		case DEVICE_STATE_BACKUP_POWER:
			if (power_in_voltage >= POWER_IN_TURN_ON_TH && !print_backup_power)
			{
				printf ("\nPOWER_MGM: DEVICE_STATE_BACKUP_POWER state, power returned\n");
				print_backup_power = TRUE; /*only print power returned once */
			}

			backup_power_cnt_g += *time_diff;
			if (backup_power_cnt_g > BACKUP_POWER_TIME_TH)
			{
				GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
				backup_power_cnt_g = 0;
				led_blink_cnt_g = 0;
				Device_turn_off ();
				device_state_g = DEVICE_STATE_TURN_OFF;
				supercap_voltage = ADC_get_value (kADC_POWER_VCAP);
				printf ("\nPOWER_MGM: backup period is over - shutting down, supercap voltage = %d\n", supercap_voltage);
			}
			break;

		case DEVICE_STATE_TURN_OFF:
			// wait while pulse is still generated (time period didn't reach threshold)
			if (!Device_control_GPIO_status())
			{
				printf ("\nPOWER_MGM: DEVICE IS about to turn OFF\n");
				Device_off_req(FALSE, 0);
				device_state_g = DEVICE_STATE_OFF;
				//Wiggle_sensor_restart ();
				//peripherals_disable ();
				//Wiggle_sensor_start ();	// enable interrupt
				//disable_peripheral_clocks();
			}
			break;

		default:
			printf ("\nPOWER_MGM: ERROR: UNKNOWN STATE %d\n", device_state_g );
			device_state_g = DEVICE_STATE_OFF;
//			Wiggle_sensor_restart ();
			enable_peripheral_clocks();

			break;
	}

	last_MT5_present = MT5_present;
	Device_control_GPIO(time_diff);
}

void Device_get_turn_on_reason(uint8_t * turn_on_reason)
{
	*turn_on_reason = turn_on_condition_g;
}

void Device_off_req_immediate(bool clean_reset)
{
	uint32_t cpu_off_wait_time = 0;
	uint8_t cpu_status_pin = 0;

	/* Disable the Accelerometer and RTC because we were seeing I2C issues where 
	SCL line was stuck on boot up */
	//AccDisable();
	/* Shut off power to the accelerometer */
	//GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);
	backup_power_cnt_g = 0;
	led_blink_cnt_g = 0;
	GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	/* Shut off power to the accelerometer */
	//GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);

	enable_msm_power(0, 0);		// turn off 5V0 power rail

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
	volatile static bool device_off_req_in_progress;

	/* If this command is called from a different thread, it will not execute
	again while it is already being performed */
	if (device_off_req_in_progress == TRUE)
	{
		return;
	}

	device_off_req_in_progress = TRUE;

	GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
	FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0, 0); /* Red LED*/
	_time_delay(wait_time*1000);
	
	/* Disable the Accelerometer and RTC because we were seeing I2C issues where 
	SCL line was stuck on boot up */
	//AccDisable();
	/* Shut off power to the accelerometer */
	//GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);
	if (!skip_a8_shutdown)
	{
#ifdef MCU_AND_CPU_BOARD_CONNECTED
	printf ("Device_off_req: shutting down A8\n");
	/* Turn A8 device off */
	GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
	_time_delay (DEVICE_CONTROL_TIME_OFF_TH);
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);
	/* monitor CPU_STATUS stop signal for MAX_CPU_OFF_CHECK_TIME */
	while (cpu_off_wait_time < MAX_CPU_OFF_CHECK_TIME)
	{
		_time_delay(CPU_OFF_CHECK_TIME);
		cpu_off_wait_time += CPU_OFF_CHECK_TIME;
		cpu_status_pin = GPIO_DRV_ReadPinInput (CPU_STATUS);
		if (cpu_status_pin == 0)
		{
			printf ("Device_off_req: CPU_status pin %d, wait_time %d ms\n", cpu_status_pin, cpu_off_wait_time);
			break;
		}
	}

	/* if the CPU/A8 does not end up being powered off kill the power by manually
	 * turning off the 5V rail. Note, this ends up turning off the LED and Audio
	 * power
	 */
	if (cpu_off_wait_time >= MAX_CPU_OFF_CHECK_TIME)
	{
		printf ("Device_off_req: WARNING, TURNED OFF 5V0 power rail coz cpu_off_time expired\n");
		enable_msm_power(0, 0);		// turn off 5V0 power rail
	}
#endif
	}
	backup_power_cnt_g = 0;
	led_blink_cnt_g = 0;
	GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	/* Shut off power to the accelerometer */
	//GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);
	//Board_SetSlowClk ();
	printf ("\nPOWER_MGM: DEVICE IS OFF through Device_off_req\n");
	NVIC_SystemReset();
	device_off_req_in_progress = FALSE;
}

// delay in msec
void Device_reset_req(int32_t wait_time)
{
	int32_t cpu_off_wait_time;
	uint8_t cpu_status_pin = 0;

	GPIO_DRV_SetPinOutput (CPU_POWER_LOSS);
	FPGA_write_led_status(LED_LEFT, LED_DEFAULT_BRIGHTESS, 0xFF, 0, 0); /* Red LED*/

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
	
	enable_msm_power(0, 0);		// turn off 5V0 power rail

	//GPIO_DRV_ClearPinOutput (CPU_POWER_LOSS);
	printf ("\nPOWER_MGM: DEVICE reset through Device_reset_req\n");
	peripherals_disable (1);
	device_state_g = DEVICE_STATE_OFF;

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
	device_state_g = DEVICE_STATE_OFF;

	device_control_gpio_g.time_threshold = ms_from_start() + DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.delay_period   = delay_period;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = false;
	device_control_gpio_g.status         = false;
	GPIO_DRV_SetPinOutput(CPU_ON_OFF);

//	Wiggle_sensor_init (delay_period);
}

void Pulse_Device(uint32_t period)
{
	device_control_gpio_g.time_threshold = ms_from_start() + period;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;

}
void Device_turn_on  (void)
{
	device_control_gpio_g.time_threshold = ms_from_start() + DEVICE_CONTROL_TIME_ON_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;

	/* Create a timer that calls a watchdog reset if the A8 does NOT turn ON in MAX_CPU_TICKS_TAKEN_TO_BOOT */
	/* NOTE: This timer NEEDs to be cancelled if a successful bootup happens */
//	_lwtimer_create_periodic_queue(&lwtimer_period_a8_turn_on_g, MAX_CPU_TICKS_TAKEN_TO_BOOT, MAX_CPU_TICKS_TAKEN_TO_BOOT);
//	_lwtimer_add_timer_to_queue(&lwtimer_period_a8_turn_on_g, &lwtimer_a8_turn_on_g, 0, \
//		(LWTIMER_ISR_FPTR)handle_watchdog_expiry, 0);
}

void Device_turn_off (void)
{
	device_control_gpio_g.time_threshold = ms_from_start() + DEVICE_CONTROL_TIME_OFF_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;
}

void Device_reset (void)
{

	device_control_gpio_g.time_threshold = ms_from_start() + DEVICE_CONTROL_TIME_RESET_TH;
	device_control_gpio_g.time           = 0;
	device_control_gpio_g.enable         = true;
}

void Device_control_GPIO (uint32_t * time_diff)
{
	uint64_t ms;

	if (device_control_gpio_g.enable == false)
		return;

	ms = ms_from_start();

	if (ms < device_control_gpio_g.time_threshold) {
		GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = true;
		device_control_gpio_g.time  += *time_diff;
	} else {
		GPIO_DRV_SetPinOutput(CPU_ON_OFF);
		device_control_gpio_g.status = false;
		device_control_gpio_g.time   = 0;
		device_control_gpio_g.enable = false;
	}
	//printf("%s: ms %llu [%llu]\n", __func__, ms, device_control_gpio_g.time_threshold); 
}

bool Device_control_GPIO_status  (void) {return device_control_gpio_g.status;}
DEVICE_STATE_t Device_get_status (void) {return device_state_g;}

void peripherals_enable (void)
{
	uint32_t i;
	uint32_t total_wait_time = 0;

	GPIO_DRV_ClearPinOutput (FPGA_RSTB);

//	enable_msm_power(1, 0);		// turn on 5V0 power rail
	GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE);	// FPGA Enable
	GPIO_DRV_SetPinOutput   (FPGA_RSTB);

	//AccEnable();
	/* keep CAN1, J1708, CAN2 and SWC disabled by default. A software command
	will enable it once the OS boots up */
	GPIO_DRV_ClearPinOutput(CAN1_J1708_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput(CAN1_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput(CAN2_SWC_PWR_ENABLE);

//    GPIO_DRV_ClearPinOutput (USB_OTG_SEL);		// Connect D1 <-> D MCU or HUB
//  GPIO_DRV_SetPinOutput(USB_OTG_SEL);			// Connect D2 <-> D A8 OTG
//	GPIO_DRV_ClearPinOutput (USB_OTG_OE);		//Enable OTG/MCU switch

	// Enable USB for DEBUG
//	GPIO_DRV_SetPinOutput   (USB_HUB_RSTN);
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
//	GPIO_DRV_SetPinOutput (EXT_GPS_EN);

	// wait till FPGA is loaded
	printf("%s: wait for fpga ready\n", __func__);
	while(1)
	{
		if (GPIO_DRV_ReadPinInput (FPGA_DONE) == 1)
		{
			printf ("%s: FPGA is loaded on %d ms\n", __func__, total_wait_time );
			
			FPGA_init();
			J1708_enable(7);
            PORT_HAL_SetPinIntMode (PORTC, GPIO_EXTRACT_PIN(FPGA_GPIO0), kPortIntRisingEdge);
            GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
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

//	printf("%s: enable acc interrupts %d ms\n", __func__, total_wait_time);
//	PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntLogicZero);
//	GPIO_DRV_ClearPinIntFlag(ACC_INT);
	// TODO: need to be removed after tasks enable will be set fro USB protocol
	//J1708_enable  (7);

}


void peripherals_disable (uint32_t WithFpga)
{
  	// disable FPGA based resources
	PORT_HAL_SetPinIntMode (PORTC, GPIO_EXTRACT_PIN(FPGA_GPIO0), kPortIntDisabled);
	GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
//	GPIO_DRV_ClearPinIntFlag(ACC_INT);
//	PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntDisabled);
//	GPIO_DRV_ClearPinOutput(ACC_VIB_ENABLE);

	if(WithFpga)
	{  
		// Down FPGA
		GPIO_DRV_ClearPinOutput(FPGA_RSTB);
		GPIO_DRV_ClearPinOutput(FPGA_PWR_ENABLE);
	}

	GPIO_DRV_ClearPinOutput (CAN1_J1708_PWR_ENABLE);
	GPIO_DRV_ClearPinOutput(CAN1_PWR_EN); //0n NET869V6 and greater boards, the J1708 and CAN1 power were split
	GPIO_DRV_ClearPinOutput (CAN2_SWC_PWR_ENABLE);

//    GPIO_DRV_ClearPinOutput (USB_HUB_RSTN);
	GPIO_DRV_ClearPinOutput (FTDI_RSTN);
	GPIO_DRV_ClearPinOutput (USB_ENABLE);
	GPIO_DRV_ClearPinOutput (UART_ENABLE);
	//GPIO_DRV_ClearPinOutput   (FTDI_RSTN);
	GPIO_DRV_ClearPinOutput (SPKR_LEFT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_RIGHT_EN);
	GPIO_DRV_ClearPinOutput (SPKR_EXT_EN);
	GPIO_DRV_ClearPinOutput (CPU_MIC_EN);
//	GPIO_DRV_ClearPinOutput (EXT_GPS_EN);
	//AccDisable();
//    GPIO_DRV_SetPinOutput (USB_OTG_OE);		//Disable OTG/MCU switch
}


void send_power_change (uint8_t *power_mask)
{
	packet_t pwm_msg;						// power management message
	pwm_msg.pkt_type = POWER_MGM_STATUS;

	pwm_msg.data[0] = *power_mask;
	send_control_msg(&pwm_msg, 1);
}
