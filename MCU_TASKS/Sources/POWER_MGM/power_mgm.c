#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <lwmsgq.h>
#include <lwtimer.h>
#include <mqx_prv.h>

#include "MK20D10_extension.h"
#include "fsl_power_manager.h"

#include "tasks_list.h"

#include "power_mgm.h"
#include "Device_control_GPIO.h"
#include "ADC.h"
#include "EXT_GPIOS.h"

#include "Uart_debugTerminal.h"
#include <event.h>
#include "board.h"
#include "fsl_uart_driver.h"
#include "fsl_lptmr_driver.h"

#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_pmc_hal.h"
#include "fsl_smc_hal.h"
#include "fsl_os_abstraction.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr_driver.h"
#include "fsl_interrupt_manager.h"
#include "board.h"
#include "bsp.h"
#include "mic_typedef.h"
#include "watchdog_mgmt.h"

#define SUPERCAP_CHRG_DISCHRG_ENABLE   1
//#define DEBUGGING_ENABLED                1

#define POWER_MGM_TIME_DELAY		 50
#define TEMPERATURE_TIME_DELAY		5000
#define IGNITION_TIME_DELAY			1000
#define CABLE_TYPE_TIME_DELAY		1000
#define EXT_GPIO_TIME_DELAY			1000

#define CABLE_TYPE_MIN_VOLTAGE		(CABLE_TYPE_VOLTAGE *  90/100)
#define CABLE_TYPE_MAX_VOLTAGE		(CABLE_TYPE_VOLTAGE * 110/100)

#define SUPERCAP_MIN_TH_WHEN_DEV_OFF	5000//4000
#define SUPERCAP_MAX_TH_WHEN_DEV_OFF	5300//4300

#define SUPERCAP_MIN_TH_WHEN_DEV_ON		5000//4500
#define SUPERCAP_MAX_TH_WHEN_DEV_ON		5300//4800

#define CPU_STATUS_SHUTDOWN_DURATION 50 //ms
#define CPU_STATUS_TURNON_DURATION 150  //ms

/* The LPTMR instance used for LPTMR */
#define PM_RTOS_DEMO_LPTMR_FUNC_INSTANCE                0
#define TICK_PER_SEC            1000

#define SYSTICK_CM4_CSR_REG (*((volatile unsigned long *)0xE000E010))
#define SYSTICK_CM4_RVR_REG (*((volatile unsigned long *)0xE000E014))
#define SYSTICK_CM4_CVR_REG (*((volatile unsigned long *)0xE000E018))
#define SYSTICK_CM4_CALIB_REG (*((volatile unsigned long *)0xE000E01C))

#define SYSTICK_DISABLE()       (SYSTICK_CM4_CSR_REG &= (~1))
#define SYSTICK_ENABLE()        (SYSTICK_CM4_CSR_REG |= 1)
#define SYSTICK_RELOAD(tps)     (SYSTICK_CM4_RVR_REG = tps)

#define CHECK_RET_VAL(ret, mode) \
if (ret != kPowerManagerSuccess) \
{ \
    PRINTF("POWER_SYS_SetMode(%u) returned unexpected status : %u\r\n",mode,ret); \
}

clock_manager_error_code_t dbg_console_cm_callback(clock_notify_struct_t *notify, void* callbackData);
power_manager_error_code_t rtos_pm_callback(power_manager_notify_struct_t * notify,  power_manager_callback_data_t * dataPtr);
clock_manager_error_code_t rtos_cm_callback(clock_notify_struct_t *notify, void* callbackData);
extern void MQX_PORTA_IRQHandler(void);

typedef enum demo_power_modes {
	kDemoMin  = 0,
	kDemoRun,           // Normal RUN mode
	kDemoWait,
	kDemoStop,
	kDemoVlpr,
	kDemoVlpw,
	kDemoVlps,
	kDemoLls,
	kDemoVlls1,
	kDemoVlls2,
	kDemoVlls3,
	KDemoADC,
	kDemoMax
} demo_power_modes_t;

// callback type for power manager user callback data.
// each callback message has this variables for counting events
typedef struct {
	uint32_t counter;
	uint32_t status;
	uint32_t err;
} callback_data_t;

// callback type which is used for power manager user callback
typedef struct {
	callback_data_t before;
	callback_data_t after;
	power_manager_notify_t lastType;
	uint32_t err;
} user_callback_data_t;

//lptmrStructure_t cmCallbackData;

typedef enum wakeUpSource
{
	wakeUpSourceErr = 0,
	wakeUpSourceRtc = 1,
	wakeUpSourceLptmr = 2,
	wakeUpSourceSwBtn =  4,
} wakeUpSource_t;

/*--------------------------------MODE---------------------------------------*/
power_manager_user_config_t const vlprConfig =
{
	.mode = kPowerManagerVlpr,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};

power_manager_user_config_t const vlpwConfig =
{
	.mode = kPowerManagerVlpw,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
};

power_manager_user_config_t const vlls1Config =
{
	.mode = kPowerManagerVlls1,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const vlls2Config =
{
	.mode = kPowerManagerVlls2,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const vlls3Config =
{
	.mode = kPowerManagerVlls3,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const llsConfig =
{
	// classic LLS mode retains all ram content too
	.mode = kPowerManagerLls,

	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const vlpsConfig =
{
	.mode = kPowerManagerVlps,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const waitConfig =
{
	.mode = kPowerManagerWait,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const stopConfig =
{
	.mode = kPowerManagerStop,
	.lowPowerWakeUpOnInterruptValue = kSmcLpwuiDisabled,
	.sleepOnExitValue = false,
};
power_manager_user_config_t const runConfig =
{
	.mode = kPowerManagerRun
};

/* keep the order of these configs the same as _power_manager_modes
 * only K20 supported power modes have valid pointers.
 * NOTE: If an ifdef is changed in the k20 header file, the pointers could be null and cause issues */
power_manager_user_config_t const *powerConfigs[] =
{
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
	null,            /*!< High-speed run mode. All Kinetis chips. @internal gui name="High speed run mode" */
#endif
	&runConfig,
	&vlprConfig,
	&waitConfig,
	&vlpwConfig,
	&stopConfig,
	&vlpsConfig,
#if FSL_FEATURE_SMC_HAS_PSTOPO
	null,           /*!< Partial stop 1 mode. Chip-specific. @internal gui name="Partial stop 1 mode" */
	null,           /*!< Partial stop 2 mode. Chip-specific. @internal gui name="Partial stop 2 mode" */
#endif
	&llsConfig,
#if FSL_FEATURE_SMC_HAS_LLS_SUBMODE
	null,             /*!< Low leakage stop 2 mode. Chip-specific. @internal gui name="Low leakage stop 2 mode" */
	null,             /*!< Low leakage stop 3 mode. Chip-specific. @internal gui name="Low leakage stop 3 mode" */
#endif
#if FSL_FEATURE_SMC_HAS_STOP_SUBMODE0
	null,            /*!< Very low leakage stop 0 mode. All Kinetis chips. @internal gui name="Very low leakage stop 0 mode" */
#endif
	&vlls1Config,
	&vlls2Config,
	&vlls3Config,
};

/*--------------------------------Callback---------------------------------------*/
clock_manager_callback_user_config_t dbg_console_cm_callback_tbl_data =
{
	.callback     = dbg_console_cm_callback,
	.callbackType = kClockManagerCallbackBeforeAfter,
	.callbackData = NULL
};

clock_manager_callback_user_config_t rtos_cm_callback_tbl_data =
{
	.callback     = rtos_cm_callback,
	.callbackType = kClockManagerCallbackBeforeAfter,
//    .callbackData = &cmCallbackData,
};

clock_manager_callback_user_config_t *cm_callback_tbl[] =
{
	&rtos_cm_callback_tbl_data,
	&dbg_console_cm_callback_tbl_data,
};

power_manager_callback_user_config_t rtos_pm_callback_tbl_data =
{
	.callback     = rtos_pm_callback,
	.callbackType = kPowerManagerCallbackBeforeAfter,
	.callbackData = NULL,
};

power_manager_callback_user_config_t *pm_callback_tbl[] =
{
//    &adc16_pm_callback_tbl_data,
	&rtos_pm_callback_tbl_data
};

size_t const cm_callback_tbl_size = sizeof(cm_callback_tbl)/sizeof(clock_manager_callback_user_config_t *);
size_t const powerConfigsSize = sizeof(powerConfigs)/sizeof(power_manager_user_config_t *);
size_t const pm_callback_tbl_size = sizeof(pm_callback_tbl)/sizeof(power_manager_callback_user_config_t *);

void Supercap_charge_state (void);
void check_supercap_voltage (void);
//void Supercap_discharge_state (void);

void * power_up_event_g;
void * cpu_status_event_g;
void * cpu_int_suspend_event_g;

uint32_t ignition_threshold_g = IGNITION_TURN_ON_TH_DEFAULT;

extern const clock_manager_user_config_t g_defaultClockConfigRun;
extern const clock_manager_user_config_t g_defaultClockConfigVlpr;
extern DEVICE_STATE_t        device_state_g;
extern LWTIMER_PERIOD_STRUCT lwtimer_period_a8_turn_on_g;
extern LWTIMER_STRUCT lwtimer_a8_turn_on_g;

extern tick_measure_t cpu_status_time_g;
bool a8_booted_up_correctly_g = false;

const clock_manager_user_config_t * g_defaultClockConfigurations[] =
{
	NULL,
	&g_defaultClockConfigVlpr,
	&g_defaultClockConfigRun,
};

/*!
 * @brief wait uart finished.
 *
 */
void wait_finish_uart(void)
{
	uint32_t bytesRemaining = 0;
	uint32_t wait_time = 0;
	volatile bool isLastByteTranmistComplete = false;
	do
	{
		UART_DRV_GetTransmitStatus(BOARD_DEBUG_UART_INSTANCE, &bytesRemaining);
		isLastByteTranmistComplete = UART_HAL_GetStatusFlag(BOARD_DEBUG_UART_BASEADDR,kUartTxComplete);
		if ((bytesRemaining != 0) || (!isLastByteTranmistComplete))
		{
			_time_delay(10);
			wait_time += 10;
			if (wait_time > 5000)
			{
				break;
			}
		}	
	} while ((bytesRemaining != 0) || (!isLastByteTranmistComplete));
	//printf("%s: wait_time = %d ms", __func__, wait_time);
}

/* Update clock.*/
void update_clock_mode(uint8_t cmConfigMode)
{
	CLOCK_SYS_UpdateConfiguration(cmConfigMode, kClockManagerPolicyForcible);
}

/*FUNCTION*********************************************************************
 *
 * Function Name : dbg_console_cm_callback
 * Description   : debug console callback for change event from power manager
 *
 *END*************************************************************************/
clock_manager_error_code_t dbg_console_cm_callback(clock_notify_struct_t *notify,
	 void* callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:     // Received "pre" message
			DbgConsole_DeInit();
			break;

		case kClockManagerNotifyRecover: // Received "recover" message
		case kClockManagerNotifyAfter:    // Received "post" message
			//DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_LOW_POWER_UART_BAUD, kDebugConsoleUART);
			DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUD, kDebugConsoleUART);
			break;

		default:
			result = kClockManagerError;
			break;
	}
	return result;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : rtos_cm_callback
 * Description   : rtos callback for change event from clock manager
 *
 *END*************************************************************************/
clock_manager_error_code_t rtos_cm_callback(clock_notify_struct_t *notify,
	 void* callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;
	//lptmrStructure_t *lptmrStr = (lptmrStructure_t*)callbackData;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:     // Received "pre" message
			wait_finish_uart();
			SYSTICK_DISABLE();
			break;

		case kClockManagerNotifyRecover: // Received "recover" message
			SYSTICK_DISABLE();
			SYSTICK_RELOAD((CLOCK_SYS_GetCoreClockFreq()/TICK_PER_SEC)-1UL);
			SYSTICK_ENABLE();
			break;
		case kClockManagerNotifyAfter:    // Received "post" message
			/* Caculate prescaler clock frequency */
//            if ( kLptmrTimerModeTimeCounter == lptmrStr->lptmrUserConfig.timerMode)
//            {
//                lptmrStr->lptmrState.prescalerClockHz = CLOCK_SYS_GetLptmrFreq(lptmrStr->instance,
//                        lptmrStr->lptmrUserConfig.prescalerClockSource);
//
//                if (lptmrStr->lptmrUserConfig.prescalerEnable)
//                {
//                    lptmrStr->lptmrState.prescalerClockHz = (lptmrStr->lptmrState.prescalerClockHz >> ((uint32_t)(lptmrStr->lptmrUserConfig.prescalerValue+1)));
//                }
//            }
			SYSTICK_RELOAD((CLOCK_SYS_GetCoreClockFreq()/TICK_PER_SEC)-1UL);
			SYSTICK_ENABLE();
			break;

		default:
			result = kClockManagerError;
			break;
	}
	return result;
}

/*FUNCTION*********************************************************************
 *
 * Function Name : rtos_pm_callback
 * Description   : rtos callback for change event from power manager
 *
 *END*************************************************************************/
power_manager_error_code_t rtos_pm_callback(power_manager_notify_struct_t * notify,
	power_manager_callback_data_t * dataPtr)
{
	power_manager_error_code_t result = kPowerManagerSuccess;
	power_manager_modes_t crr_mode = POWER_SYS_GetCurrentMode();
	switch (notify->notifyType)
	{
		case kPowerManagerNotifyRecover:
			/* TODO */
			/* Add code here. */
			break;

		case kPowerManagerNotifyBefore:
			/* TODO */
			/* Add code here. */
			if(notify->targetPowerConfigPtr->mode == kPowerManagerRun
			|| notify->targetPowerConfigPtr->mode == kPowerManagerVlpr
			)
			{
				/* if in run mode, do nothing. */
			}
			else
			{
				wait_finish_uart();
				//disable_unused_pins();
				/* Disable systick in all other mode. */
				/* Noticed that if I disable SYSTICK in VLPR mode the RTOS power
				   powermanagement task does not run, but interrupts work - Abid
				*/
				//SYSTICK_DISABLE();
			}
			break;

		case kPowerManagerNotifyAfter:
			/* TODO */
			/* Add code here. */
			if( crr_mode == kPowerManagerRun
			 || crr_mode == kPowerManagerVlpr
			 )
			{
#if (defined FSL_RTOS_BM)
				// With BM, we must let LPTMR continues counting for OSA_GetMsec function work.
				// Disable interrupt.
				//LPTMR_HAL_SetIntCmd(g_lptmrBase[PM_RTOS_DEMO_LPTMR_FUNC_INSTANCE],false);
#else
				/* Stop lptmr. */
				//LPTMR_DRV_Stop(PM_RTOS_DEMO_LPTMR_FUNC_INSTANCE);
#endif
				//enable_unused_pins();
				/* Enable systick in all run mode. */
				SYSTICK_ENABLE();
			}
			else
			{
				/* if in other mode, do nothing. */
			}
			break;

		default:
			result = kPowerManagerError;
			break;
	}

	return result;
}

void llwuDisableWakeUp(void)
{
	LLWU_HAL_SetInternalModuleCmd(LLWU_BASE_PTR, kLlwuWakeupModule5, false);
	LLWU_HAL_SetInternalModuleCmd(LLWU_BASE_PTR, kLlwuWakeupModule0, false);
}

void gpioDisableWakeUp(void)
{
	// disables interrupt
	PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntDisabled);
	INT_SYS_DisableIRQ(BOARD_SW_LLWU_IRQ_NUM);

	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE_PTR, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
	LLWU_HAL_SetExternalInputPinMode(LLWU_BASE_PTR,kLlwuExternalPinDisabled, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

void gpioEnableWakeUp(void)
{
	// enables falling edge interrupt for switch SWx
	//PORT_HAL_SetPinIntMode(BOARD_SW_LLWU_BASE, BOARD_SW_LLWU_PIN, kPortIntFallingEdge);
	//INT_SYS_EnableIRQ(BOARD_SW_LLWU_IRQ_NUM);

	//LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE_PTR, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
	//LLWU_HAL_SetExternalInputPinMode(LLWU_BASE_PTR,kLlwuExternalPinFallingEdge, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);

	uint32_t port = GPIO_EXTRACT_PORT(VIB_SENS);
	uint32_t pin  = GPIO_EXTRACT_PIN(VIB_SENS);
	PORT_Type *portBase = g_portBase[port];
	PORT_HAL_SetPinIntMode (portBase, pin, kPortIntFallingEdge);
	INT_SYS_EnableIRQ(PORTA_IRQn);

	NVIC_SetPriority(PORTA_IRQn, PORT_NVIC_IRQ_Priority);
	OSA_InstallIntHandler(PORTA_IRQn, MQX_PORTA_IRQHandler);

	//LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE_PTR, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
	//LLWU_HAL_SetExternalInputPinMode(LLWU_BASE_PTR,kLlwuExternalPinFallingEdge, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

void setWakeUpSource(wakeUpSource_t wus)
{
	uint8_t timeout;

	if((wus & (wakeUpSourceRtc | wakeUpSourceLptmr)) != 0)
	{
		gpioDisableWakeUp();
		// Wake up on timer's interrupt
		//timeout = setWakeUpTimeOut(wus);
	}
	else if((wus & wakeUpSourceSwBtn) != 0)
	{
		llwuDisableWakeUp();
		// Wake up on gpio interrupt from button(s)
		gpioEnableWakeUp();
	}
	else
	{
		printf("Unknown error.\r\n");
	}
}

void switch_power_mode(power_manager_modes_t mode)
{
	uint32_t ret;
	switch (mode)
	{
		case kPowerManagerVlpr:
			/* Going into VLPRun power mode */
			printf("Going to VLPRun mode \n ");
			if (POWER_SYS_GetCurrentMode() != kPowerManagerVlpr)
			{
				update_clock_mode(CLOCK_VLPR);
				ret = POWER_SYS_SetMode(kPowerManagerVlpr, kPowerManagerPolicyAgreement); //kPowerManagerPolicyForcible
				CHECK_RET_VAL(ret, mode);
			}
			else
			{
				printf("VLPR already active\n");
			}

			break;

		case kPowerManagerVlpw:
			/* Going into VLPWait power mode */
			printf("Going to VLPWait mode \n ");
			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				printf("\nPower Management Task: cannot go from RUN to VLPW directly \n");
			}

			setWakeUpSource(wakeUpSourceSwBtn);

			ret = POWER_SYS_SetMode(kPowerManagerVlpw, kPowerManagerPolicyAgreement);

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				/* update Clock Mode to Run */
				update_clock_mode(CLOCK_RUN);
			}
			CHECK_RET_VAL(ret, mode);
			break;

		case kPowerManagerVlps:
			/* Going into VLPStop power mode */
			printf("Going into VLPStop mode \n ");
			setWakeUpSource(wakeUpSourceSwBtn);
			ret = POWER_SYS_SetMode(kPowerManagerVlps, kPowerManagerPolicyAgreement);

			if (POWER_SYS_GetCurrentMode() == kPowerManagerRun)
			{
				/* update Clock Mode to Run */
				update_clock_mode(CLOCK_RUN);
			}
			CHECK_RET_VAL(ret, mode);
			break;

		case kPowerManagerRun:
			/* Going back to run mode */
			printf("Going to run mode \n ");
			ret = POWER_SYS_SetMode(kPowerManagerRun, kPowerManagerPolicyAgreement);
			if (ret != kPowerManagerSuccess)
			{
				printf("POWER_SYS_SetMode(%u) returned unexpected status : %u\r\n",mode, ret);
			}
			else
			{
				update_clock_mode(CLOCK_RUN);
			}
			break;
	}
}

void Power_MGM_task (uint32_t initial_data )
{
	KADC_CHANNELS_t adc_input = kADC_ANALOG_IN1;
	KADC_CHANNELS_t i = kADC_ANALOG_IN1;
	uint32_t ret;
	uint8_t mode;
	uint8_t cmConfigMode = CLOCK_RUN;
	uint32_t freq = 0;
	MQX_TICK_STRUCT ticks_now;
	MQX_TICK_STRUCT ticks_prev;
	int32_t time_diff_milli = 0;
	uint32_t time_diff_milli_u = 0;
	int32_t time_diff_micro = 0;
	bool time_diff_overflow = false;
    _mqx_uint event_bits;
	_mqx_uint event_result;
	
	_time_get_elapsed_ticks_fast(&ticks_prev);

	event_result = _event_create("event.PowerUpEvent");
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not create PowerUp event \n");
	}

	event_result = _event_open("event.PowerUpEvent", &power_up_event_g);
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not open PowerUp event \n");
	}

	event_result = _event_create("event.cpuStatusEvent");
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not create cpuStatusEvent \n");
	}

	event_result = _event_open("event.cpuStatusEvent", &cpu_status_event_g);
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not open cpuStatusEvent \n");
	}

	event_result = _event_create("event.cpuIntSuspendEvent");
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not create cpuIntSuspendEvent \n");
	}
	event_result = _event_open("event.cpuIntSuspendEvent", &cpu_int_suspend_event_g);
	if(MQX_OK != event_result){
		printf("Power_MGM_task: Could not open cpuIntSuspendEvent \n");
	}

	Device_init (POWER_MGM_TIME_DELAY);
	ADC_init ();
	/* Get all the initial ADC values */
	for (i = kADC_ANALOG_IN1; i < (kADC_CHANNELS-1); i++)
	{
		ADC_sample_input (i);
	}

	ADC_Set_IRQ_TH (kADC_POWER_IN_ISR, POWER_IN_SUPERCAP_DISCHARGE_TH, POWER_IN_TURN_ON_TH);
	ADC_Compare_enable (kADC_POWER_IN_ISR);

	CLOCK_SYS_Init(g_defaultClockConfigurations,
				   CLOCK_NUMBER_OF_CONFIGURATIONS,
				   cm_callback_tbl,
				   cm_callback_tbl_size);

	CLOCK_SYS_UpdateConfiguration(cmConfigMode, kClockManagerPolicyForcible);

	/* initialize power manager driver */
	POWER_SYS_Init(powerConfigs, powerConfigsSize, pm_callback_tbl, pm_callback_tbl_size);

	// Enables LowLeakageWakeupUnit interrupt
	//INT_SYS_EnableIRQ(LLWU_IRQn);

	CLOCK_SYS_GetFreq(kCoreClock, &freq);
#ifdef DEBUGGING_ENABLED
	switch_power_mode(kPowerManagerRun);
#else
	switch_power_mode(kPowerManagerVlpr);
#endif

	/* Start off with the peripherals disabled */
	disable_peripheral_clocks();
	peripherals_disable (true);
	_bsp_MQX_tick_timer_init ();
	/* Enable power to the vibration sensor and accelerometer */
	GPIO_DRV_SetPinOutput(ACC_VIB_ENABLE);

	printf ("\nPower Management Task: Start \n");

	while (1)
	{
		ADC_sample_input (adc_input);
		if (++adc_input >= (kADC_CHANNELS - 1))
		{
			//printf ("\nPOWER_MGM: WARNING: CABLE TYPE is not as expected (current voltage %d mV - expected %d mV\n", cable_type_voltage, CABLE_TYPE_VOLTAGE);
			GPIO_all_check_for_change();
			adc_input = kADC_ANALOG_IN1;
		}
		
		/* update the supercap voltage at a higher frequency compared to all the other ADCs */
		if ((0 == adc_input%3) && (adc_input != kADC_POWER_VCAP) )
		{
			ADC_sample_input(kADC_POWER_VCAP);
		}


#ifdef SUPERCAP_CHRG_DISCHRG_ENABLE
		Supercap_charge_state    ();
		check_supercap_voltage   ();
#endif
		_time_delay (POWER_MGM_TIME_DELAY);

		_time_get_elapsed_ticks_fast(&ticks_now);
		time_diff_milli = _time_diff_milliseconds(&ticks_now, &ticks_prev, &time_diff_overflow);
		_time_get_elapsed_ticks_fast(&ticks_prev);

		if (time_diff_milli < 0)
		{
			printf("Power_MGM_task: timediff -ve!");
		}
		time_diff_milli_u = (uint32_t) time_diff_milli;

		Device_update_state(&time_diff_milli_u);
		
        if (_event_get_value(cpu_status_event_g, &event_bits) == MQX_OK) 
        {
            if (event_bits & EVENT_CPU_STATUS_HIGH) 
            {
				_event_clear(cpu_status_event_g, 1);
				printf("%s: cpu_status_event_g high \n", __func__);
            }
			if (event_bits & EVENT_CPU_STATUS_LOW) 
            {
				_event_clear(cpu_status_event_g, 2);
				cpu_status_time_g.time_diff = (int32_t)((cpu_status_time_g.end_ticks.TICKS[0] - cpu_status_time_g.start_ticks.TICKS[0])* MS_PER_TICK);
				printf("%s: cpu_status_event_g low, high time %d \n", __func__, cpu_status_time_g.time_diff);
				
				/* A8 sent a shutdown request */
				if (cpu_status_time_g.time_diff > (CPU_STATUS_SHUTDOWN_DURATION - 10) && cpu_status_time_g.time_diff < (CPU_STATUS_SHUTDOWN_DURATION + 30))
				{
					printf ("\n%s : DEVICE shutdown request by A8\n", __func__);
					_time_delay(100); /* give some time for the print statement */
					Device_off_req_immediate(true);
				}
				/* A8 booted up correctly, cancel the incorrect-a8-bootup timer */
				else if (cpu_status_time_g.time_diff > (CPU_STATUS_TURNON_DURATION - 10) && cpu_status_time_g.time_diff < (CPU_STATUS_TURNON_DURATION + 30))
				{
					_lwtimer_cancel_timer(&lwtimer_a8_turn_on_g);
					_lwtimer_cancel_period(&lwtimer_period_a8_turn_on_g);
					a8_booted_up_correctly_g = true;
				}
            }
        }
	}
}

void supercap_charge_discarge(uint32_t min_threshold, uint32_t max_threshold)
{
	uint32_t supercap_voltage  = ADC_get_value (kADC_POWER_VCAP);
	if (supercap_voltage >= max_threshold)
	{
		/* send a message only once */
		if (GPIO_DRV_ReadPinInput (POWER_CHARGE_ENABLE) == 1)
		{
			printf ("\nPOWER_MGM: SUPERCAP full Charged %d mV\n", supercap_voltage);
		}
		GPIO_DRV_ClearPinOutput (POWER_CHARGE_ENABLE);
	}
	else if (supercap_voltage <= min_threshold)
	{
		/* send a message only once */
		if (GPIO_DRV_ReadPinInput (POWER_CHARGE_ENABLE) == 0)
		{
			printf ("\nPOWER_MGM: SUPERCAP low %d mV - Start Charging\n", supercap_voltage);
		}
		GPIO_DRV_SetPinOutput (POWER_CHARGE_ENABLE);
	}
}

/* charge supercap if supercap voltage level is below threshold.
 continue till supercap voltage level reaches upper threshold. */
void Supercap_charge_state (void)
{
	uint32_t supercap_voltage = ADC_get_value (kADC_POWER_VCAP);

	/* The supercap manufucturer informed us that charging the supercap to 5V at temperatures
	above 70c degrades the supercap. Since the device is mostly in the off state when the 
	temperature is high, we are reducing the voltage we charge to in the off state */
	if (device_state_g == DEVICE_STATE_OFF)
	{
		supercap_charge_discarge(SUPERCAP_MIN_TH_WHEN_DEV_OFF, SUPERCAP_MAX_TH_WHEN_DEV_OFF);
	}
	else
	{
		supercap_charge_discarge(SUPERCAP_MIN_TH_WHEN_DEV_ON, SUPERCAP_MAX_TH_WHEN_DEV_ON);
	}

}

/* Disable the supercap if we do not have enough power to run the MCU */
void check_supercap_voltage (void)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN);
	uint32_t supercap_voltage = ADC_get_value (kADC_POWER_VCAP);

	/* we do not want to try to use the supercap when there isn't enough power to run the MCU */
	if ((power_in_voltage < POWER_IN_SHUTDOWN_TH) && (supercap_voltage < MCU_MIN_OPERATING_VOLTAGE)
		&& (device_state_g != DEVICE_STATE_OFF))
	{
		/* send a message only once */
		if (GPIO_DRV_ReadPinInput (POWER_DISCHARGE_ENABLE) == 1)
		{
			printf ("\nPOWER_MGM: SUPERCAP stop DisCharged (power in voltage is: %d mV, supercap voltage is %d mV)\n", power_in_voltage, supercap_voltage);
		}
		/* Instead of disabling supercap power we put the MCU in low power mode when we don't have enough power */
		//GPIO_DRV_ClearPinOutput (POWER_DISCHARGE_ENABLE);
		Device_off_req(TRUE, 0);
	}
}

#if 0
/* discharge supercap if input power is below threshold */
void Supercap_discharge_state (void)
{
	uint32_t power_in_voltage  = ADC_get_value (kADC_POWER_IN);
	uint32_t supercap_voltage = ADC_get_value (kADC_POWER_VCAP);

	/* we do not want to try to use the supercap when there isn't enough power to run the MCU */
	if ((power_in_voltage <= POWER_IN_SHUTDOWN_TH) && (supercap_voltage >= MCU_MIN_OPERATING_VOLTAGE))
	{
		/* send a message only once */
		if (GPIO_DRV_ReadPinInput (POWER_DISCHARGE_ENABLE) == 0)
		{
		  printf ("\nPOWER_MGM: SUPERCAP start DisCharged (power in voltage is: %d mV, supercap voltage is: %d mV)\n", power_in_voltage, supercap_voltage);
		}
		GPIO_DRV_SetPinOutput (POWER_DISCHARGE_ENABLE);
	}
	else
	{
		/* send a message only once */
		if (GPIO_DRV_ReadPinInput (POWER_DISCHARGE_ENABLE) == 1)
		{
			printf ("\nPOWER_MGM: SUPERCAP stop DisCharged (power in voltage is: %d mV, supercap voltage is %d mV)\n", power_in_voltage, supercap_voltage);
		}
		GPIO_DRV_ClearPinOutput (POWER_DISCHARGE_ENABLE);
	}
}
#endif

void get_ignition_threshold(uint32_t * p_ignition_threshold)
{
	*p_ignition_threshold = ignition_threshold_g;
}

void set_ignition_threshold(uint32_t ignition_threshold)
{
	ignition_threshold_g = ignition_threshold;
}
