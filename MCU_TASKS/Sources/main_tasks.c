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

#include "mic_typedef.h"

#include "Uart_debugTerminal.h"

#include "FlexCanDevice.h"

void MQX_I2C0_IRQHandler (void);
void MQX_PORTA_IRQHandler(void);
void MQX_PORTC_IRQHandler(void);

#define	MAIN_TASK_SLEEP_PERIOD	10			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))

static i2c_master_state_t i2c_master;

_pool_id   g_out_message_pool;
_pool_id   g_in_message_pool;

extern uint32_t wiggle_sensor_cnt;

_task_id   g_TASK_ids[NUM_TASKS] = { 0 };

//TEST CANFLEX funtion
void _test_CANFLEX( void );

MUTEX_STRUCT g_i2c0_mutex;

void Main_task( uint32_t initial_data ) {

    _queue_id  main_qid;    //, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
	_queue_id  j1708_rx_qid;
	//APPLICATION_MESSAGE_T *msg;
	uint32_t gpios_event_bit;
	uint32_t gpio_sample_timer = 0;
	uint32_t i;
	MUTEX_ATTR_STRUCT mutexattr;

    uint8_t u8mainTaskLoopCnt = 0;
    uint32_t FPGA_version = 0;
    _mqx_uint ret = MQX_OK;

    wiggle_sensor_cnt = 0;
    _time_delay (10);


    MIC_DEBUG_UART_PRINTF("\nMain Task: Start \n");
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

    // I2C0 Initialization
    NVIC_SetPriority(I2C0_IRQn, 6U);
    OSA_InstallIntHandler(I2C0_IRQn, MQX_I2C0_IRQHandler);
    I2C_DRV_MasterInit(I2C0_IDX, &i2c_master);

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

    // turn on device
    GPIO_DRV_SetPinOutput(POWER_3V3_ENABLE);
    GPIO_DRV_SetPinOutput(POWER_5V0_ENABLE);
//	GPIO_DRV_ClearPinOutput(ACC_ENABLE       );
	GPIO_DRV_SetPinOutput(ACC_ENABLE       );

    // FPGA Enable
    GPIO_DRV_SetPinOutput(FPGA_PWR_ENABLE);

//	BOARD_InitOsc0();
//	CLOCK_SetBootConfig_Run ();

	//Enable CAN
	GPIO_DRV_SetPinOutput(CAN_ENABLE);

    // Enable USB for DEBUG
    GPIO_DRV_ClearPinOutput(USB_ENABLE);
    GPIO_DRV_ClearPinOutput(USB_HUB_RSTN);

    GPIO_DRV_ClearPinOutput(USB_OTG_SEL);    // Connect D1 <-> D MCU or HUB
    //GPIO_DRV_SetPinOutput(USB_OTG_SEL);    // Connect D2 <-> D A8 OTG
    GPIO_DRV_ClearPinOutput(USB_OTG_OE); //Enable OTG/MCU switch

    _time_delay(10);
    GPIO_DRV_SetPinOutput(USB_HUB_RSTN);
    GPIO_DRV_SetPinOutput(USB_ENABLE);

    _time_delay(20);
    g_TASK_ids[USB_TASK] = _task_create(0, USB_TASK, 0);
	if ( g_TASK_ids[USB_TASK] == MQX_NULL_TASK_ID ) {
		MIC_DEBUG_UART_PRINTF("\nMain Could not create USB_TASK\n");
	}

    //Enable UART
    GPIO_DRV_SetPinOutput(UART_ENABLE);
    GPIO_DRV_SetPinOutput(FTDI_RSTN);

	g_out_message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE_T), NUM_CLIENTS, 0, 0);
	if (g_out_message_pool == MSGPOOL_NULL_POOL_ID)
	{
		printf("\nCould not create a g_out_message_pool message pool\n");
		_task_block();
	}

	g_in_message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE_T), NUM_CLIENTS, 0, 0);
	if (g_in_message_pool == MSGPOOL_NULL_POOL_ID)
	{
		printf("\nCould not create a g_in_message_pool message pool\n");
		_task_block();
	}

	main_qid = _msgq_open(MAIN_QUEUE, 0);

	_time_delay (1000);

	FPGA_init ();

	J1708_enable  (7);


#if 1
	/* Simulate a power on button press on the A8 */
	GPIO_DRV_SetPinOutput   (LED_BLUE);

    GPIO_DRV_ClearPinOutput(CPU_ON_OFF);
    _time_delay (3000);
    GPIO_DRV_SetPinOutput(CPU_ON_OFF);

    GPIO_DRV_ClearPinOutput   (LED_BLUE);
#else
    _time_delay (1000);
#endif

	{
//		uint8_t Br, R,G,B;
//		R = G = B = 255;
//		Br = 10;
//		FPGA_write_led_status (LED_RIGHT , &Br, &R, &G, &B);
//		_time_delay (10);
//		FPGA_write_led_status (LED_MIDDLE, &Br, &R, &G, &B);
//		_time_delay (10);
	}

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

#if 0
	g_TASK_ids[POWER_MGM_TASK] = _task_create(0, POWER_MGM_TASK   , 0 );
	if (g_TASK_ids[POWER_MGM_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create POWER_MGM_TASK\n");
	}
#endif
	
	g_TASK_ids[ACC_TASK] = _task_create(0, ACC_TASK, 0);
	if (g_TASK_ids[ACC_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create ACC_TASK\n");
	}

	g_TASK_ids[CONTROL_TASK] = _task_create(0, CONTROL_TASK, 0);
	if (g_TASK_ids[CONTROL_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create CONTROL_TASK\n");
	}

    //Disable CAN termination
    GPIO_DRV_ClearPinOutput(CAN1_TERM_ENABLE);
    GPIO_DRV_ClearPinOutput(CAN2_TERM_ENABLE);

    //Initialize CAN sample
    configure_can_pins(0);
    configure_can_pins(1);

    _time_delay (1000);

    _test_CANFLEX();

	if (GPIO_DRV_ReadPinInput (SWITCH1) == 1)
	{
		/* Connect D1 <-> D MCU or HUB */
		printf("/r/n connect D1 to MCU/hub ie clear USB_OTG_SEL");
	    GPIO_DRV_ClearPinOutput(USB_OTG_SEL);
	}
	else
	{
		/* Connect D2 <-> D A8 OTG */
		printf("/r/n connect D2 to A8 OTG ie set USB_OTG_SEL");
	    GPIO_DRV_SetPinOutput(USB_OTG_SEL);
	}

	_event_create ("event.EXTERNAL_GPIOS");
	_event_open   ("event.EXTERNAL_GPIOS", &g_GPIO_event_h);

	FPGA_read_version(&FPGA_version);
	printf("\n FPGA version, %x", FPGA_version);

    printf("\nMain Task: Loop \n");

    while ( 1 ) {
#if 0

        u8InputVoltageStatus_Prev = u8InputVoltageStatus;
        u8InputVoltageStatus      = IntputVoltageSample (&inputVoltage);

        // sleep till ADC completed
        while ((_lwadc_read_average(&pot, 8, &inputVoltage) ) == FALSE);
        InputVoltageUpdateStatus (&inputVoltage);
        u8InputVoltageStatus = InputVoltageGetStatus (&inputVoltage);

        if (u8InputVoltageStatus == INPUT_VOLTAGE_STATUS_NORMAL) {
            GPIOClear	(POWER_DISCHARGE_ENABLE);
            GPIOClear	(CPU_POWER_LOSS);
            if (u8InputVoltageStatus_Prev != u8InputVoltageStatus)
            MIC_DEBUG_UART_PRINTF ("MAIN TASK: INPUT VOLTAGE NORMAL\n");
        } else {
            GPIOSet     (CPU_POWER_LOSS);
            GPIOSet     (POWER_DISCHARGE_ENABLE);
            if (u8InputVoltageStatus_Prev != u8InputVoltageStatus)
            MIC_DEBUG_UART_PRINTF ("MAIN TASK: INPUT VOLTAGE OUT OF RANGE\n");
        }

        u8SupercapStatus_Prev     = u8SupercapStatus;
        u8SupercapStatus          = SupercapVoltageSample (&supercap    );
        // sleep till ADC completed

        if (u8SupercapStatus == SUPERCAP_STATUS_LOW)
        GPIOSet		(POWER_CHARGE_ENABLE);
        else
        GPIOClear	(POWER_CHARGE_ENABLE);


        switch (u8mainTaskLoopCnt)
        0 : temperatureStatus = TemperatureVoltageSample(&temperature);
        4 : cableTypeStatus   = TelemetryVoltageSample  (&cableType  );
        7 : DebugSwitchControl (&u8LastSwitchStatus);
        8 : u8mainTaskLoopCnt = 0;


    _time_delay(MAIN_TASK_SLEEP_PERIOD);            // contact switch
#endif

#if 0
		GPIO_DRV_ClearPinOutput (LED_RED);
		GPIO_DRV_ClearPinOutput (LED_GREEN);
		GPIO_DRV_ClearPinOutput (LED_BLUE);
		_time_delay (1000);

		GPIO_DRV_SetPinOutput   (LED_RED);
		_time_delay (1000);

		GPIO_DRV_SetPinOutput   (LED_GREEN);
		_time_delay (1000);

		GPIO_DRV_SetPinOutput   (LED_BLUE);
		_time_delay (1000);
#endif

		gpio_sample_timer  += MAIN_TASK_SLEEP_PERIOD;
		if (gpio_sample_timer >= TIME_ONE_SECOND_PERIOD) {
			gpio_sample_timer = 0;
			GPIO_sample_all ();
		}

#if 0
		_event_wait_any  (	g_GPIO_event_h,
							EVENT_GPIO_IN(0) | EVENT_GPIO_IN(1) | EVENT_GPIO_IN(2) | EVENT_GPIO_IN(3) |
							EVENT_GPIO_IN(4) | EVENT_GPIO_IN(5) | EVENT_GPIO_IN(6) | EVENT_GPIO_IN(7) ,
							10);
		_event_get_value (g_GPIO_event_h, &gpios_event_bit);

		for (i = 0; i < 8; i++) {
			int g;
			if (gpios_event_bit & (1<<i)) {
				g = GPIO_INPUT_get_logic_level (i);
				_event_clear    (g_GPIO_event_h, (1<<i));
			}
		}
#endif

	    _time_delay(MAIN_TASK_SLEEP_PERIOD);            // context switch
    }

    MIC_DEBUG_UART_PRINTF("\nMain Task: End \n");
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

void MQX_PORTA_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (VIB_SENS)) {
		GPIO_DRV_ClearPinIntFlag(VIB_SENS);
		wiggle_sensor_cnt++;
	}
}

void MQX_PORTC_IRQHandler(void)
{
	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO0)) {
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
		_event_set(g_J1708_event_h, EVENT_J1708_RX);
	}
}

void _test_CANFLEX( ) {
#if 1
    //	ids[CAN_TASK_RX_0] = _task_create(0, CAN_TASK_RX_0, BSP_CAN_DEVICE_0);
    //    if ( ids[USB_TASK] == MQX_NULL_TASK_ID ) {
    //        MIC_DEBUG_UART_PRINTF("\nMain Could not create CAN_TASK_RX_0\n");
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

	MIC_DEBUG_UART_PRINTF("FlexCanDevice_Init( ) return %d\n", ret);

	ret = FlexCanDevice_Start(can_Device_0);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_Start( ) return %d\n", ret);

	ret = FlexCanDevice_Start(can_Device_1);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_Start( ) return %d\n", ret);

	ret = FlexCanDevice_SetTermination(can_Device_0, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_SetTermination( ) return %d\n", ret);

	ret = FlexCanDevice_SetTermination(can_Device_1, false);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_SetTermination( ) return %d\n", ret);

	ret = FlexCanDevice_SetRxMaskType(can_Device_0, false);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

	ret = FlexCanDevice_SetRxMaskType(can_Device_1, false);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_SetRxMaskType( ) return %d\n", ret);

//        ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdStd, 8, 0x7FF, true);
//        MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 5, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 6, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 7, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 8, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

//        ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdStd, 9, 0x7FF, true);
//        MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 9, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 10, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 11, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_0, kFlexCanMsgIdExt, 12, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	//CAN1
	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 5, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 6, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 7, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 8, 0x3, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 9, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 10, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 11, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);

	ret = FlexCanDevice_setMailbox(can_Device_1, kFlexCanMsgIdExt, 12, 0xC, true);
	MIC_DEBUG_UART_PRINTF("FlexCanDevice_setMailbox( ) return %d\n", ret);
#endif

}

