
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
#include "mic_typedef.h"

#include "Uart_debugTerminal.h"

#include "FlexCanDevice.h"
#include "rtc.h"
#include "Wiggle_sensor.h"

//#define DEBUG_BLINKING_RIGHT_LED 1

//void MQX_I2C0_IRQHandler (void);
//void MQX_I2C1_IRQHandler (void);
void MQX_PORTA_IRQHandler(void);
void MQX_PORTB_IRQHandler(void);
void MQX_PORTC_IRQHandler(void);
void MQX_PORTE_IRQHandler(void);
void configure_otg_for_host_or_device(void);

#define	MAIN_TASK_SLEEP_PERIOD	1000			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))

//static i2c_master_state_t i2c0_master;
//static i2c_master_state_t i2c1_master;

//_pool_id   g_out_message_pool;
_pool_id   g_in_message_pool;

_task_id   g_TASK_ids[NUM_TASKS] = { 0 };
volatile uint32_t cpu_watchdog_count_g = 0;
extern WIGGLE_SENSOR_t sensor_g;

extern void * g_acc_event_h;
extern void * power_up_event_g;

//TEST CANFLEX funtion
void _test_CANFLEX( void );

MUTEX_STRUCT g_i2c0_mutex;

void Main_task( uint32_t initial_data ) {

    _queue_id  main_qid;    //, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
	MUTEX_ATTR_STRUCT mutexattr;

    uint32_t FPGA_version = 0;
    _mqx_uint event_result;

    _time_delay (10);

	printf("\n%s: Start\n", __func__);
#if 0
    PinMuxConfig ();
    ADCInit      ();
    USBInit      ();
    I2CInit      ();
    registerInit ();
#endif

    // board Initialization
    hardware_init();
    OSA_Init();
    GPIO_Config();
    ADC_init ();
	
	

	NVIC_SetPriority(PORTA_IRQn, 6U);
	OSA_InstallIntHandler(PORTA_IRQn, MQX_PORTA_IRQHandler);
	NVIC_SetPriority(PORTC_IRQn, 6U);
	OSA_InstallIntHandler(PORTC_IRQn, MQX_PORTC_IRQHandler);

	NVIC_SetPriority(PORTB_IRQn, 6U);
	OSA_InstallIntHandler(PORTB_IRQn, MQX_PORTB_IRQHandler);

//    // I2C0 Initialization
//    NVIC_SetPriority(I2C0_IRQn, 6U);
//    OSA_InstallIntHandler(I2C0_IRQn, MQX_I2C0_IRQHandler);
//    I2C_DRV_MasterInit(I2C0_IDX, &i2c0_master);
//
//    // I2C1 Initialization
//	NVIC_SetPriority(I2C1_IRQn, 6U);
//	OSA_InstallIntHandler(I2C1_IRQn, MQX_I2C1_IRQHandler);
//	I2C_DRV_MasterInit(I2C1_IDX, &i2c1_master);

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

    Virtual_Com_MemAlloc(); // Allocate USB out buffers
	
    g_TASK_ids[POWER_MGM_TASK] = _task_create(0, POWER_MGM_TASK   , 0 );
    if (g_TASK_ids[POWER_MGM_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create POWER_MGM_TASK\n");
    }

/*    
    g_out_message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE_T), NUM_CLIENTS, 0, 0);
    if (g_out_message_pool == MSGPOOL_NULL_POOL_ID)
    {
        printf("\nCould not create a g_out_message_pool message pool\n");
        _task_block();
    }
*/
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

    printf("\nbefore power on event\n");
    /* We are waiting until a wiggle event happens before starting everything up
       The main reason for this is to stay below 5mA at 12V */
    _event_wait_all(power_up_event_g, 1, 0);
    _event_clear(power_up_event_g, 1);

    printf("\nAfter power on event\n");

    // turn on device
    GPIO_DRV_SetPinOutput(POWER_5V0_ENABLE);


    
    g_TASK_ids[USB_TASK] = _task_create(0, USB_TASK, 0);
    if ( g_TASK_ids[USB_TASK] == MQX_NULL_TASK_ID ) {
            printf("\nMain Could not create USB_TASK\n");
    }


    GPIO_DRV_SetPinOutput(ACC_ENABLE);

    //Enable UART
    GPIO_DRV_SetPinOutput(UART_ENABLE);
    GPIO_DRV_SetPinOutput(FTDI_RSTN);

    _time_delay (1000);

    FlexCanDevice_InitHW();

    FPGA_init ();

    J1708_enable  (7);

    rtc_init();

    g_TASK_ids[J1708_TX_TASK] = _task_create(0, J1708_TX_TASK, 0 );
    if (g_TASK_ids[J1708_TX_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create J1708_TX_TASK\n");
    }

    g_TASK_ids[J1708_RX_TASK] = _task_create(0, J1708_RX_TASK, 0 );
    if (g_TASK_ids[J1708_RX_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create J1708_RX_TASK\n");
    }

    g_TASK_ids[FPGA_UART_RX_TASK] = _task_create(0, FPGA_UART_RX_TASK, 0 );
    if (g_TASK_ids[FPGA_UART_RX_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create FPGA_UART_RX_TASK\n");
    }

    //Start CAN RX TX Tasks
    g_TASK_ids[CAN_TASK_RX_0] = _task_create(0, CAN_TASK_RX_0, (uint32_t)(can_Device_0));
    if ( g_TASK_ids[CAN_TASK_RX_0] == MQX_NULL_TASK_ID ) {
        printf("Could not create CAN_TASK_RX for inst %u \n", can_Device_0->instance);
    }

    g_TASK_ids[CAN_TASK_RX_1] = _task_create(0, CAN_TASK_RX_1, (uint32_t)(can_Device_1));
    if ( g_TASK_ids[CAN_TASK_RX_1] == MQX_NULL_TASK_ID ) {
        printf("Could not create CAN_TASK_RX for inst %u \n", can_Device_1->instance);
    }

    g_TASK_ids[CAN_TASK_TX_0] = _task_create(0, CAN_TASK_TX_0, (uint32_t)(can_Device_0));
    if ( g_TASK_ids[CAN_TASK_TX_0] == MQX_NULL_TASK_ID ) {
        printf("Could not create CAN_TASK_TX for inst %u \n", can_Device_0->instance);
    }

    g_TASK_ids[CAN_TASK_TX_1] = _task_create(0, CAN_TASK_TX_1, (uint32_t)(can_Device_1));
    if ( g_TASK_ids[CAN_TASK_TX_1] == MQX_NULL_TASK_ID ) {
            printf("Could not create CAN_TASK_TX for inst %u \n", can_Device_1->instance);
    }

    g_TASK_ids[ACC_TASK] = _task_create(0, ACC_TASK, 0);
    if (g_TASK_ids[ACC_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create ACC_TASK\n");
    }

	g_TASK_ids[UPDATER_TASK] = _task_create(0, UPDATER_TASK, 0);
	if (g_TASK_ids[UPDATER_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create UPDATER_TASK\n");
	}
	else
		printf("\nMain UPDATER_TASK created\n");
	  

	g_TASK_ids[CONTROL_TASK] = _task_create(0, CONTROL_TASK, 0);
    if (g_TASK_ids[CONTROL_TASK] == MQX_NULL_TASK_ID)
    {
        printf("\nMain Could not create CONTROL_TASK\n");
    }

    configure_otg_for_host_or_device();
    NVIC_SetPriority(PORTE_IRQn, 6U);
    OSA_InstallIntHandler(PORTE_IRQn, MQX_PORTE_IRQHandler);

    _event_create ("event.EXTERNAL_GPIOS");
    _event_open   ("event.EXTERNAL_GPIOS", &g_GPIO_event_h);

    FPGA_read_version(&FPGA_version);
    printf("\n FPGA version, %x", FPGA_version);

    printf("\nMain Task: Loop \n");

    while ( 1 ) 
    {
        _time_delay(MAIN_TASK_SLEEP_PERIOD);
#ifdef DEBUG_BLINKING_RIGHT_LED
        FPGA_write_led_status(LED_RIGHT, LED_DEFAULT_BRIGHTESS, 0, 0, 0xFF); /*Blue LED */

        _time_delay(MAIN_TASK_SLEEP_PERIOD);
        FPGA_write_led_status(LED_RIGHT, 0, 0, 0, 0); /*Blue LED */
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

void configure_otg_for_host_or_device(void)
{
	if (GPIO_DRV_ReadPinInput (OTG_ID) == 0)
	{
		/* Connect D1 <-> D MCU or HUB */
		printf("/r/n connect D1 to MCU/hub ie clear USB_OTG_SEL");
		GPIO_DRV_ClearPinOutput (USB_OTG_SEL);
                
		GPIO_DRV_ClearPinOutput (USB_OTG_OE);
		GPIO_DRV_ClearPinOutput   (CPU_OTG_ID);
	}
	else
	{
		/* Connect D2 <-> D A8 OTG */
		printf("/r/n connect D2 to A8 OTG ie set USB_OTG_SEL");
		GPIO_DRV_SetPinOutput   (USB_OTG_SEL);
                
		GPIO_DRV_ClearPinOutput (USB_OTG_OE);
		GPIO_DRV_SetPinOutput (CPU_OTG_ID);
	}
}

void MQX_PORTA_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (VIB_SENS)) {
		GPIO_DRV_ClearPinIntFlag(VIB_SENS);
		//Wiggle_sensor_stop ();
		sensor_g.wiggle_sensor_cnt++;
		//_event_set(g_WIGGLE_SENSOR_event_h, EVENT_WIGGLE_SENSOR_IRQ);
	}

	if (GPIO_DRV_IsPinIntPending (ACC_INT)) {
		GPIO_DRV_ClearPinIntFlag(ACC_INT);
		// Signal main task to read acc
		_event_set(g_acc_event_h, 1);
	}

}

void MQX_PORTB_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (CPU_WATCHDOG)) 
    {
		GPIO_DRV_ClearPinIntFlag(CPU_WATCHDOG);
		cpu_watchdog_count_g++;
	}
    
    if (GPIO_DRV_IsPinIntPending(CPU_SPKR_EN)) 
    {
        GPIO_DRV_ClearPinIntFlag(CPU_SPKR_EN);
        if (GPIO_DRV_ReadPinInput(CPU_SPKR_EN))
        {
            GPIO_DRV_SetPinOutput (SPKR_RIGHT_EN);
            GPIO_DRV_SetPinOutput (SPKR_LEFT_EN);       
        }   
        else 
        {
            GPIO_DRV_ClearPinOutput(SPKR_RIGHT_EN);
            GPIO_DRV_ClearPinOutput(SPKR_LEFT_EN);
        }
    }
}

void MQX_PORTC_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO0)) {
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
		_event_set(g_J1708_event_h, EVENT_J1708_RX);
	}
}

void MQX_PORTE_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (OTG_ID))
	{
		GPIO_DRV_ClearPinIntFlag(OTG_ID);
		configure_otg_for_host_or_device();
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