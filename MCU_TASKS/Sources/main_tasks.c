
#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include <fsl_flexcan_driver.h>
#include <fsl_flexcan_hal.h>
#include <lwmsgq.h>
#include <mutex.h>

#include "fsl_i2c_master_driver.h"

#include "tasks_list.h"
#include "gpio_pins.h"
#include "J1708_task.h"
#include "fpga_api.h"
#include "ADC.h"
#include "EXT_GPIOS.h"
#include "wiggle_sensor.h"
#include "rtc.h"
#include "mic_typedef.h"
#include "Uart_debugTerminal.h"
#include "FlexCanDevice.h"
#include "Wiggle_sensor.h"
#include "Device_control_GPIO.h"
#include "watchdog_mgmt.h"
#include "power_mgm.h"
#include "version.h"
#include "fsl_wdog_driver.h"
#include "board_type.h"

//#define DEBUG_BLINKING_RIGHT_LED 1
//#define MCU_HARD_FAULT_DEBUG 1
#define DEBUG_A8_WATCHOG_DISABLED 1

//void MQX_I2C0_IRQHandler (void);
//void MQX_I2C1_IRQHandler (void);
void MQX_PORTA_IRQHandler(void);
void MQX_PORTB_IRQHandler(void);
void MQX_PORTC_IRQHandler(void);
void MQX_PORTE_IRQHandler(void);
void configure_otg_for_host_or_device(int);

void HardFault_Handler_asm(void);

#define	MAIN_TASK_SLEEP_PERIOD	1000			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))
#define	OTG_CTLEP_RECOVERY_TO	30000

//static i2c_master_state_t i2c0_master;
//static i2c_master_state_t i2c1_master;

//_pool_id   g_out_message_pool;
_pool_id   g_in_message_pool;

_task_id   g_TASK_ids[NUM_TASKS] = { 0 };
extern WIGGLE_SENSOR_t sensor_g;

extern void * g_acc_event_h;
extern void * power_up_event_g;
extern void * a8_watchdog_event_g;
extern void * cpu_status_event_g;
tick_measure_t cpu_status_time_g = {0, 0, 0};

//TEST CANFLEX funtion
void _test_CANFLEX( void );

MUTEX_STRUCT g_i2c0_mutex;
MUTEX_STRUCT g_i2c1_mutex;

extern uint8_t g_flag_Exit;

uint8_t g_board_rev;
char g_board_config;


/* induce_hard_fault: Induce divide by zero hard fault(used for debugging) */
void induce_hard_fault(void)
{
	volatile uint8_t j = 0;
	volatile uint8_t i = 0;

	SCB->CCR |= 0x10;
	for (i=0;i<15;i++)
	{
		if (i == 10)
		{
			printf("i=10\n");
		}
		j = i/(i-10);
	}
	printf("j=%d\n", j);
}

#ifdef MCU_HARD_FAULT_DEBUG
/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
volatile unsigned long stacked_r0 ;
volatile unsigned long stacked_r1 ;
volatile unsigned long stacked_r2 ;
volatile unsigned long stacked_r3 ;
volatile unsigned long stacked_r12 ;
volatile unsigned long stacked_lr ;
volatile unsigned long stacked_pc ;
volatile unsigned long stacked_psr ;
volatile unsigned long _CFSR ;
volatile unsigned long _HFSR ;
volatile unsigned long _DFSR ;
volatile unsigned long _AFSR ;
volatile unsigned long _BFAR ;
volatile unsigned long _MMAR ;
void HardFault_HandlerC(unsigned long *hardfault_args)
{
	stacked_r0 = ((unsigned long)hardfault_args[0]) ;
	stacked_r1 = ((unsigned long)hardfault_args[1]) ;
	stacked_r2 = ((unsigned long)hardfault_args[2]) ;
	stacked_r3 = ((unsigned long)hardfault_args[3]) ;
	stacked_r12 = ((unsigned long)hardfault_args[4]) ;
	stacked_lr = ((unsigned long)hardfault_args[5]) ;
	stacked_pc = ((unsigned long)hardfault_args[6]) ;
	stacked_psr = ((unsigned long)hardfault_args[7]) ;

	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

	// Hard Fault Status Register
	_HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

	// Debug Fault Status Register
	_DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
	// Bus Fault Address Register
	_BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

	//printf("\n %s: r0: %x \r r1: %x \r r2: %x \r r3: %x \r r12: %x \r " ,
	//	   __func__, stacked_r0, stacked_r1, stacked_r2, stacked_r3, stacked_r12);

	//printf("\n %s: lr: %x \r pc %x \r psr: %x\r",
	//	   __func__, stacked_lr, stacked_pc, stacked_psr);

	//printf("\n %s: CFSR: %x \r HFSR %x \r DFSR: %x \r AFSR: %x \r",
	//	   __func__, _CFSR, _HFSR, _DFSR, _AFSR);


	__asm("BKPT #0\n") ; // Break into the debugger
}
#endif

void HardFault_Handler_asm()//(Cpu_ivINT_Hard_Fault)
{
	//printf("ERROR: HardFault_Handler_asm HIT \n");
	 /*
         * Get the appropriate stack pointer, depending on our mode,
         * and use it as the parameter to the C handler. This function
         * will never return
         */
	__asm volatile (
	" movs r0,#4       \n"
	" movs r1, lr      \n"
	" tst r0, r1       \n"
	" beq _MSP         \n"
	" mrs r0, psp      \n"
	" b _HALT          \n"
	"_MSP:               \n"
	" mrs r0, msp      \n"
	"_HALT:              \n"
	" ldr r1,[r0,#20]  \n"
#ifdef MCU_HARD_FAULT_DEBUG
	" b HardFault_HandlerC \n"
#endif
	);
	WDG_RESET_MCU();
}

int g_a8_sw_reboot = -1;
int g_otg_ctl_port_active = 0;
extern void * g_a8_pwr_state_event;

void Main_task( uint32_t initial_data ) {

	_queue_id  main_qid;    //, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
	MUTEX_ATTR_STRUCT mutexattr;
    uint32_t FPGA_version = 0;
    _mqx_uint event_result;
    _mqx_uint result;
    _mqx_uint event_bits;
	uint32_t cdc_recovery_count = -1;
	uint64_t otg_reset_time;
	uint64_t otg_check_time;
	uint32_t pet_count = 0;
	uint8_t zero_date[5] = {0};

#if (!_DEBUG)
	watchdog_mcu_init();
#endif
	printf("\n%s: Start\n", __func__);
#if 0
	PinMuxConfig ();
	ADCInit      ();
	USBInit      ();
	I2CInit      ();
	registerInit ();
#endif

    // board Initialization
    post_bsp_hardware_init ();
    OSA_Init();
#if (!_DEBUG)
	watchdog_rtos_init();
	_watchdog_start(WATCHDOG_MCU_MAX_TIME);
#endif
    GPIO_Config();
//    ADC_init ();

	/* Initialize mutex attributes: */
	if (_mutatr_init(&mutexattr) != MQX_OK)
	{
		printf("Initializing mutex attributes failed.\n");
		_task_block();
	}
	/* Initialize the mutex: */
	if (_mutex_init(&g_i2c0_mutex, &mutexattr) != MQX_OK)
	{
		printf("Initializing i2c0 mutex failed.\n");
		_task_block();
	}
	
	/* Initialize the mutex: */
	if (_mutex_init(&g_i2c1_mutex, &mutexattr) != MQX_OK)
	{
		printf("Initializing i2c1 mutex failed.\n");
		_task_block();
	}
	
	OSA_InstallIntHandler(HardFault_IRQn, HardFault_Handler_asm);

	NVIC_SetPriority(PORTA_IRQn, PORT_NVIC_IRQ_Priority);
	OSA_InstallIntHandler(PORTA_IRQn, MQX_PORTA_IRQHandler);

//    // I2C0 Initialization
//    NVIC_SetPriority(I2C0_IRQn, I2C_NVIC_IRQ_Priority);
//    OSA_InstallIntHandler(I2C0_IRQn, MQX_I2C0_IRQHandler);
//    I2C_DRV_MasterInit(I2C0_IDX, &i2c0_master);
//
//    // I2C1 Initialization
//	NVIC_SetPriority(I2C1_IRQn, I2C_NVIC_IRQ_Priority);
//	OSA_InstallIntHandler(I2C1_IRQn, MQX_I2C1_IRQHandler);
//	I2C_DRV_MasterInit(I2C1_IDX, &i2c1_master);

	Virtual_Com_MemAlloc(); // Allocate USB out buffers

	//rtc init is using TimerEvent so it should be initiated here
	if(MQX_OK !=  _event_create("event.TimerEvent")){
		printf("rtc_check_alarm_working: Could not create event.TimerEvent \n");
	}

	if(MQX_OK != _event_open("event.TimerEvent", &rtc_flags_g)){
		printf("rtc_check_alarm_working: Could not open event.TimerEvent \n");
	}
    /*Note: rtc_init() is intentionally placed before accelerometer init 
    because I was seeing  i2c arbitration errors - Abid */
    rtc_init();

	g_TASK_ids[POWER_MGM_TASK] = _task_create(0, POWER_MGM_TASK   , 0 );
	if (g_TASK_ids[POWER_MGM_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create POWER_MGM_TASK\n");
	}

	g_in_message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE_T), NUM_CLIENTS, 0, 0);
	if (g_in_message_pool == MSGPOOL_NULL_POOL_ID)
	{
		printf("\nCould not create a g_in_message_pool message pool\n");
		_task_block();
	}

	main_qid = _msgq_open(MAIN_QUEUE, 0);

	event_result = _event_open("event.PowerUpEvent", &power_up_event_g);
	if(MQX_OK != event_result){
			printf("Main_task: Could not open PowerUp event \n");
	}

    printf("%s: wait for power on event\n", __func__);
    while (1)
    {
#if (!_DEBUG)
		WDOG_DRV_Refresh();
        result = _watchdog_start(WATCHDOG_MCU_MAX_TIME);
#endif
        /* We are waiting until a wiggle event happens before starting everything up
           The main reason for this is to stay below 5mA at 12V */
        _event_wait_all(power_up_event_g, 1, WATCHDOG_MCU_MAX_TIME/2);
        if (_event_get_value(power_up_event_g, &event_bits) == MQX_OK) 
        {
            if (event_bits & 0x01) 
            {
              _event_clear(power_up_event_g, 1);
#if (!_DEBUG)
              _watchdog_start(WATCHDOG_MCU_MAX_TIME);
#endif
              break;
            }
        }
		//printf("%s: Petting Watchdog count %d", __func__, pet_count++);
    }
    printf("%s: power event got, power on A8\n", __func__);

	NVIC_SetPriority(PORTC_IRQn, PORT_NVIC_IRQ_Priority);
	OSA_InstallIntHandler(PORTC_IRQn, MQX_PORTC_IRQHandler);
	NVIC_SetPriority(PORTB_IRQn, PORT_NVIC_IRQ_Priority);
	OSA_InstallIntHandler(PORTB_IRQn, MQX_PORTB_IRQHandler);
//
//	ADC_Compare_disable (kADC_POWER_IN_ISR);
//	g_board_rev = get_board_revision();
//	g_board_config = get_board_configuration();
//	printf("board_rev = %u, board_config= %c\n", g_board_rev, g_board_config);
//	ADC_Compare_enable (kADC_POWER_IN_ISR);

	// turn on device
	enable_msm_power(TRUE);		// turn on 5V0 power rail

//	//Enable UART
////	GPIO_DRV_SetPinOutput(UART_ENABLE);	
////	GPIO_DRV_SetPinOutput(FTDI_RSTN);
//	
//	//GPIO_DRV_SetPinOutput(RS485_ENABLE);
//
//	GPIO_DRV_SetPinOutput(ACC_VIB_ENABLE);
//	_time_delay (1000);
//
//	FlexCanDevice_InitHW();
//
////	FPGA_init ();
//
//	J1708_enable  (7);
//
//	if (GPIO_DRV_ReadPinInput (FPGA_DONE)) {
//		PORT_HAL_SetPinIntMode (PORTC, GPIO_EXTRACT_PIN(FPGA_GPIO0), kPortIntRisingEdge);
//		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
//	}
//
	
//    otg_check_time = OTG_CTLEP_RECOVERY_TO + ms_from_start();
//    do {
//        _time_delay (1000);
//        if (otg_check_time < ms_from_start()) {
//            printf("%s: failure to power up A8\n", __func__);
//            Device_off_req(1, 0);
//        }
//#if (!_DEBUG)
//        WDOG_DRV_Refresh();
//        _watchdog_start(WATCHDOG_MCU_MAX_TIME);
//#endif
//        if (MQX_OK != _event_get_value(g_a8_pwr_state_event, &result))
//            result &= ~EVENT_A8_BOOTED;
//    } while (!(result & EVENT_A8_BOOTED));
	
    _event_clear(g_a8_pwr_state_event, EVENT_A8_BOOTED);
   //make sure the alarm will be set off from now on by setting the time to 0 
	poll_timeout_g = 0;
//	rtc_set_alarm1(zero_date);
//
//    g_TASK_ids[USB_TASK] = _task_create(0, USB_TASK, 0);
//    if ( g_TASK_ids[USB_TASK] == MQX_NULL_TASK_ID ) {
//            printf("\nMain Could not create USB_TASK\n");
//    }
//
//	g_TASK_ids[J1708_TX_TASK] = _task_create(0, J1708_TX_TASK, 0 );
//	if (g_TASK_ids[J1708_TX_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create J1708_TX_TASK\n");
//	}
//
//	g_TASK_ids[J1708_RX_TASK] = _task_create(0, J1708_RX_TASK, 0 );
//	if (g_TASK_ids[J1708_RX_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create J1708_RX_TASK\n");
//	}
//
//	g_TASK_ids[FPGA_UART_RX_TASK] = _task_create(0, FPGA_UART_RX_TASK, 0 );
//	if (g_TASK_ids[FPGA_UART_RX_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create FPGA_UART_RX_TASK\n");
//	}
//
//	//Start CAN RX TX Tasks
//	g_TASK_ids[CAN_TASK_RX_0] = _task_create(0, CAN_TASK_RX_0, (uint32_t)(can_Device_0));
//	if ( g_TASK_ids[CAN_TASK_RX_0] == MQX_NULL_TASK_ID ) {
//		printf("Could not create CAN_TASK_RX for inst %u \n", can_Device_0->instance);
//	}
//
//	g_TASK_ids[CAN_TASK_RX_1] = _task_create(0, CAN_TASK_RX_1, (uint32_t)(can_Device_1));
//	if ( g_TASK_ids[CAN_TASK_RX_1] == MQX_NULL_TASK_ID ) {
//		printf("Could not create CAN_TASK_RX for inst %u \n", can_Device_1->instance);
//	}
//
//	g_TASK_ids[CAN_TASK_TX_0] = _task_create(0, CAN_TASK_TX_0, (uint32_t)(can_Device_0));
//	if ( g_TASK_ids[CAN_TASK_TX_0] == MQX_NULL_TASK_ID ) {
//		printf("Could not create CAN_TASK_TX for inst %u \n", can_Device_0->instance);
//	}
//
//	g_TASK_ids[CAN_TASK_TX_1] = _task_create(0, CAN_TASK_TX_1, (uint32_t)(can_Device_1));
//	if ( g_TASK_ids[CAN_TASK_TX_1] == MQX_NULL_TASK_ID ) {
//			printf("Could not create CAN_TASK_TX for inst %u \n", can_Device_1->instance);
//	}
//
//	g_TASK_ids[ACC_TASK] = _task_create(0, ACC_TASK, 0);
//	if (g_TASK_ids[ACC_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create ACC_TASK\n");
//	}
//
//	g_TASK_ids[UPDATER_TASK] = _task_create(0, UPDATER_TASK, 0);
//	if (g_TASK_ids[UPDATER_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create UPDATER_TASK\n");
//	}
//	else
//		printf("\nMain UPDATER_TASK created\n");
//
//	g_TASK_ids[CONTROL_TASK] = _task_create(0, CONTROL_TASK, 0);
//	if (g_TASK_ids[CONTROL_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create CONTROL_TASK\n");
//	}
//	
//	g_TASK_ids[ONE_WIRE_TASK] = _task_create(0, ONE_WIRE_TASK, 0);
//	if (g_TASK_ids[ONE_WIRE_TASK] == MQX_NULL_TASK_ID)
//	{
//		printf("\nMain Could not create 1-wire task\n");
//	}
//
//	_time_delay(20); /* short delay to Allow UART to initialize and prints to work */
//	FPGA_read_version(&FPGA_version);
//	printf("\n%s: FPGA version, %x\n", __func__, FPGA_version);
//	printf("%s: MCU version, %x.%x.%x.%x\n", __func__, FW_VER_BTLD_OR_APP, FW_VER_MAJOR, FW_VER_MINOR, FW_VER_BUILD );
//	printf("board_rev = %u, board_config= %c\n", g_board_rev, g_board_config);
//
//#ifndef DEBUG_A8_WATCHOG_DISABLED 
//#if (!_DEBUG)
//	a8_watchdog_init();
//#endif
//#endif
	printf("\nMain Task: Loop \n");
	configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_BYPASS);
	otg_reset_time = ms_from_start() + OTG_CTLEP_RECOVERY_TO;
	
    while ( 1 ) 
    {
#if (!_DEBUG)
		WDOG_DRV_Refresh();
    	//TODO: only pet watchdog if all other MCU tasks are running fine -Abid
        result = _watchdog_start(WATCHDOG_MCU_MAX_TIME);
#endif
        _time_delay(MAIN_TASK_SLEEP_PERIOD);
		// MCU starts with OTG ID disabled for monitoring
		// Main task starts monitor this one only after Power task powers on the A8 and gets PON pulse from it
		// The Main task monitors OTG ID and switch USB according it.
		// If MCU usb routed to A8 and CDC driver is not getting messages about control lines change during timeout the main task routs the USB to bypass, resets all devices connected to USB (HUB, FTDI...) and stops monitor for recover time that eq 10 main task periods (10 secs), after this main task back to OTG ID monitoring.
		// The USB sub-system also goes to recovery if SW reboot/WD of A8 occurs.
		// This workaround completely debugged and tested by 48 hours resets and A8 always connects to MCU. Sure it will retested by QA together with functional tests. Moreover I want that Roman will include also some stress tests
//		if (g_a8_sw_reboot > 0) {
//		  	if (-1 == cdc_recovery_count) {
//				if (GPIO_DRV_ReadPinInput(OTG_ID)) {
//			  		cdc_recovery_count = 10;
//				} else {
//					g_a8_sw_reboot = 0;
//				}
//			} else if (cdc_recovery_count) {
//			  	cdc_recovery_count--;
//			} else {
//			  	cdc_recovery_count = -1;
//				g_a8_sw_reboot = 0;
//				otg_reset_time = ms_from_start() + OTG_CTLEP_RECOVERY_TO;
//				if (GPIO_DRV_ReadPinInput(OTG_ID)) {
//					GPIO_DRV_SetPinOutput (USB_HUB_RSTN);
//					configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_MCU_A8);
//				}
//			}
//		} else {
//			otg_check_time = ms_from_start();
//			if (!g_flag_Exit && (g_a8_sw_reboot == 0)) {
//				if (otg_reset_time < otg_check_time) {
//					if (GPIO_DRV_ReadPinInput(OTG_ID)) {
//						if (!g_otg_ctl_port_active) {
//							configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_BYPASS);
//							GPIO_DRV_ClearPinOutput (USB_HUB_RSTN);
//							g_a8_sw_reboot = 1;
//							otg_reset_time = otg_check_time;
//							continue;
//						}
//					}
//					otg_reset_time = otg_check_time + OTG_CTLEP_RECOVERY_TO;
//				}
//				configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_NONE);
//			} else {
//				otg_reset_time = otg_check_time + OTG_CTLEP_RECOVERY_TO;
//			}
//		}
#undef DEBUG_BLINKING_RIGHT_LED
#ifdef DEBUG_BLINKING_RIGHT_LED
	if(!g_flag_Exit)
	{
		static int bri = 0;
		
		bri = (bri)?0:LED_DEFAULT_BRIGHTESS;
		FPGA_write_led_status(LED_MIDDLE, bri, 0, 0, 0xFF); /*Blue LED */
//		FPGA_write_led_status(LED_MIDDLE, LED_DEFAULT_BRIGHTESS, 0, 0, 0xFF); /*Blue LED */

//		_time_delay(MAIN_TASK_SLEEP_PERIOD);
//		FPGA_write_led_status(LED_MIDDLE, 0, 0, 0, 0); /*Blue LED */
	}
#endif
	}

	printf("\nMain Task: End \n");
	_task_block();       // should never get here

}

#if 0
void OTG_CONTROL (void)
{
	uint8_t user_switch_status =  (GPIO_DRV_ReadPinInput (SWITCH2) << 1) + GPIO_DRV_ReadPinInput (SWITCH1);

	if (user_switch_status == user_switch)
		return;

	user_switch = user_switch_status;
	GPIO_DRV_SetPinOutput (USB_OTG_OE);
	_time_delay (1000);

	// disable OTG Switch

	case (user_switch) {
		OTG_CPU_CONNECTION :


			// select channel

			break;

		OTG_HUB_CONNECTION :
			break;

		default            : break;

		// enable OTG Switch
		GPIO_DRV_ClearPinOutput (USB_OTG_OE);
	}
}
#endif

void configure_otg_for_host_or_device(int force)
{
	static bool prev_otg_id_state = true;
	bool curr_otg_id_state = false;

	curr_otg_id_state = GPIO_DRV_ReadPinInput (OTG_ID);

	if (curr_otg_id_state != prev_otg_id_state || force){
		if (OTG_ID_CFG_FORCE_MCU_A8 == force) {
			curr_otg_id_state = 1;
		} else if (OTG_ID_CFG_FORCE_BYPASS == force) {
			curr_otg_id_state = 0;
		}
		prev_otg_id_state =  curr_otg_id_state;
		if (curr_otg_id_state == true)
		{
			/* Connect D1 <-> D MCU or HUB */
			printf("connect D1 to MCU/hub ie clear USB_OTG_SEL\n");
			GPIO_DRV_ClearPinOutput (USB_OTG_SEL);

			GPIO_DRV_SetPinOutput 	(FTDI_RSTN);
			GPIO_DRV_ClearPinOutput (USB_OTG_OE);
			GPIO_DRV_ClearPinOutput (CPU_OTG_ID);
		}
		else
		{
			/* Connect D2 <-> D A8 OTG */
			printf("connect D2 to A8 OTG ie set USB_OTG_SEL\n");
			GPIO_DRV_SetPinOutput (USB_OTG_SEL);

			GPIO_DRV_ClearPinOutput (USB_OTG_OE);
			GPIO_DRV_SetPinOutput 	(CPU_OTG_ID);
			GPIO_DRV_ClearPinOutput (FTDI_RSTN);
			g_otg_ctl_port_active = 0;
		}
	}
}

void MQX_PORTA_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (VIB_SENS)) {
		GPIO_DRV_ClearPinIntFlag(VIB_SENS);
		//Wiggle_sensor_stop ();
		sensor_g.wiggle_sensor_cnt++;
		//_event_set(g_WIGGLE_SENSOR_event_h, EVENT_WIGGLE_SENSOR_IRQ);
        if (sensor_g.wiggle_sensor_cnt > 1000) {
			PORT_HAL_SetPinIntMode(PORTA, GPIO_EXTRACT_PIN(VIB_SENS), kPortIntDisabled);
        }
	}

	if (GPIO_DRV_IsPinIntPending (ACC_INT)) {
		GPIO_DRV_ClearPinIntFlag(ACC_INT);
		PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntDisabled);
		// Signal main task to read acc
		_event_set(g_acc_event_h, 1);
	}

}

void MQX_PORTB_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (CPU_WATCHDOG))
	{
		GPIO_DRV_ClearPinIntFlag(CPU_WATCHDOG);
		_event_set(a8_watchdog_event_g, WATCHDOG_A8_CPU_WATCHDOG_BIT); 
	}

	if (GPIO_DRV_IsPinIntPending(CPU_SPKR_EN))
	{
		GPIO_DRV_ClearPinIntFlag(CPU_SPKR_EN);
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
	}
}


void MQX_PORTC_IRQHandler(void)
{
//	int64_t h,l;
	
	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO0)) 
	{
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
		_event_set(g_J1708_event_h, EVENT_J1708_RX);
	}
	if (GPIO_DRV_IsPinIntPending (CPU_STATUS)) 
	{
		GPIO_DRV_ClearPinIntFlag(CPU_STATUS);
		if (GPIO_DRV_ReadPinInput(CPU_STATUS))
		{
			_time_get_elapsed_ticks_fast(&(cpu_status_time_g.start_ticks));
//			_time_get_elapsed_ticks(&(cpu_status_time_g.start_ticks));
			_event_set(cpu_status_event_g, EVENT_CPU_STATUS_HIGH);
		}
		else
		{
			_time_get_elapsed_ticks_fast(&(cpu_status_time_g.end_ticks));
//			_time_get_elapsed_ticks(&(cpu_status_time_g.end_ticks));
			_event_set(cpu_status_event_g, EVENT_CPU_STATUS_LOW);
		}
//		h = (((uint64_t)cpu_status_time_g.start_ticks.TICKS[1]) << 32) + cpu_status_time_g.start_ticks.TICKS[0];
//		l = (((uint64_t)cpu_status_time_g.end_ticks.TICKS[1]) << 32) + cpu_status_time_g.end_ticks.TICKS[0];
		
//		printf("%s: CPU staus change %llu[%llu] ms  \n", __func__, h, l);
	}
}

void MQX_PORTE_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (OTG_ID))
	{
		GPIO_DRV_ClearPinIntFlag(OTG_ID);
		configure_otg_for_host_or_device(OTG_ID_CFG_FORCE_NONE);
	}
}

void _test_CANFLEX( ) {
#if 0
	//	ids[CAN_TASK_RX_0] = _task_create(0, CAN_TASK_RX_0, BSP_CAN_DEVICE_0);
	//    if ( ids[USB_TASK] == MQX_NULL_TASK_ID ) {
	//        printf("\nMain Could not create CAN_TASK_RX_0\n");
	//    }


	flexcandevice_initparams_t initCan0, initCan1;
	flexcan_device_status_t ret;

	initCan0.flexcanMode        = fdFlexCanNormalMode;
	initCan0.instanceBitrate    = fdBitrate_125_kHz;
	initCan0.is_rx_fifo_needed  = false;
	initCan0.max_num_mb         = MAX_MB_NUMBER;
	initCan0.num_id_filters     = kFlexCanRxFifoIDFilters_8;
	initCan0.RX_queue_num       = RX_FLEXCAN_MSGQ_MESAGES;
	initCan0.TX_queue_num       = TX_FLEXCAN_MSGQ_MESAGES;

	initCan1.flexcanMode        = fdFlexCanNormalMode;
	initCan1.instanceBitrate    = fdBitrate_125_kHz;
	initCan1.is_rx_fifo_needed  = false;
	initCan1.max_num_mb         = MAX_MB_NUMBER;
	initCan1.num_id_filters     = kFlexCanRxFifoIDFilters_8;
	initCan1.RX_queue_num       = RX_FLEXCAN_MSGQ_MESAGES;
	initCan1.TX_queue_num       = TX_FLEXCAN_MSGQ_MESAGES;

	ret = FlexCanDevice_Init(&initCan0, &initCan1);

	printf("FlexCanDevice_Init( ) return %d\n", ret);

	ret = FlexCanDevice_Start(can_Device_0);
	printf("FlexCanDevice_Start( ) return %d\n", ret);

	ret = FlexCanDevice_Start(can_Device_1);
	printf("FlexCanDevice_Start( ) return %d\n", ret);

	ret = FlexCanDevice_SetTermination(can_Device_0, true);
	printf("FlexCanDevice_SetTermination( ) return %d\n", ret);

	ret = FlexCanDevice_SetTermination(can_Device_1, false);
	printf("FlexCanDevice_SetTermination( ) return %d\n", ret);

	ret = FlexCanDevice_SetRxMaskType(can_Device_0, false);
	printf("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

	ret = FlexCanDevice_SetRxMaskType(can_Device_1, false);
	printf("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

//        ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdStd, 8, 0x7FF, true);
//        printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 5, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 6, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 7, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 8, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

//        ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdStd, 9, 0x7FF, true);
//        printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 9, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 10, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 11, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 12, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	//CAN1
	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 5, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 6, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 7, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 8, 0x3, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 9, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 10, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 11, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 12, 0xC, true);
	printf("FlexCanDevice_setMailbox( ) return %d\n", ret);
#endif

}
