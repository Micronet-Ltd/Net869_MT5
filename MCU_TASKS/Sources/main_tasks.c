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


#include "Uart_debugTerminal.h"

#include "FlexCanDevice.h"

void MQX_I2C0_IRQHandler( void );
void MQX_PORTC_IRQHandler(void);

#define	MAIN_TASK_SLEEP_PERIOD	10			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))

static i2c_master_state_t i2c_master;
_pool_id   message_pool;


//TEST CANFLEX funtion
void _test_CANFLEX( void );

void Main_task( uint32_t initial_data ) {

    _task_id   ids[NUM_TASKS] = { 0 };
    _queue_id  main_qid;    //, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
	_queue_id  j1708_rx_qid;
	APPLICATION_MESSAGE_T *msg;

    uint8_t u8mainTaskLoopCnt = 0;
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

    OSA_InstallIntHandler(PORTC_IRQn, MQX_PORTC_IRQHandler);

    // I2C0 Initialization
    NVIC_SetPriority(I2C0_IRQn, 6U);
    OSA_InstallIntHandler(I2C0_IRQn, MQX_I2C0_IRQHandler);
    I2C_DRV_MasterInit(I2C0_IDX, &i2c_master);

    // turn on device
    GPIO_DRV_SetPinOutput(POWER_3V3_ENABLE);
    GPIO_DRV_SetPinOutput(POWER_5V0_ENABLE);
	GPIO_DRV_ClearPinOutput(ACC_ENABLE       );

    // FPGA Enable
    GPIO_DRV_SetPinOutput(FPGA_PWR_ENABLE);

//	BOARD_InitOsc0();
//	CLOCK_SetBootConfig_Run ();

	//Enable CAN
	GPIO_DRV_SetPinOutput(CAN_ENABLE);

    // Enable USB for DEBUG
    GPIO_DRV_ClearPinOutput(USB_ENABLE);
    GPIO_DRV_ClearPinOutput(USB_HUB_RSTN);
    GPIO_DRV_ClearPinOutput(USB_OTG_OE); //Enable OTG/MCU switch
    GPIO_DRV_ClearPinOutput(USB_OTG_SEL);    // Connect D1 <-> D MCU or HUB
    //GPIO_DRV_SetPinOutput(USB_OTG_SEL);    // Connect D2 <-> D A8 OTG


    _time_delay(10);
    GPIO_DRV_SetPinOutput(USB_HUB_RSTN);
    GPIO_DRV_SetPinOutput(USB_ENABLE);

    _time_delay(20);
    ids[USB_TASK] = _task_create(0, USB_TASK, 0);
	if ( ids[USB_TASK] == MQX_NULL_TASK_ID ) {
		MIC_DEBUG_UART_PRINTF("\nMain Could not create USB_TASK\n");
	}

    //Enable UART
    GPIO_DRV_SetPinOutput(UART_ENABLE);
    GPIO_DRV_SetPinOutput(FTDI_RSTN);
	message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE_T), NUM_CLIENTS, 0, 0);
	main_qid = _msgq_open(MAIN_QUEUE, 0);

	_time_delay (1000);

	FPGA_init ();

	J1708_enable  (7);

	{
		uint8_t Br, R,G,B;
		R = G = B = 255;
		Br = 10;
		FPGA_write_led_status (LED_RIGHT , &Br, &R, &G, &B);
		FPGA_write_led_status (LED_MIDDLE, &Br, &R, &G, &B);
	}


	_task_create(0, J1708_TX_TASK, 0 );
	_task_create(0, J1708_RX_TASK, 0 );
	_task_create(0, FPGA_UART_RX_TASK, 0 );

	ids[ACC_TASK] = _task_create(0, ACC_TASK, 0);
	if (ids[ACC_TASK] == MQX_NULL_TASK_ID)
	{
		printf("\nMain Could not create ACC_TASK\n");
	}


    //Disable CAN termination
    GPIO_DRV_ClearPinOutput(CAN1_TERM_ENABLE);
    GPIO_DRV_ClearPinOutput(CAN2_TERM_ENABLE);

    //Initialize CAN sample
    configure_can_pins(0);
    configure_can_pins(1);

    _test_CANFLEX();

    //OSA_Start();

#if 0
    ids[0] = _task_create(0, USB_TASK  , 0 );
    ids[1] = _task_create(0, CAN_TASK  , 1 );
    ids[2] = _task_create(0, CAN_TASK  , 2 );
    ids[3] = _task_create(0, J1708_TASK, 0 );
    ids[4] = _task_create(0, ACC_TASK  , 0 );
    ids[5] = _task_create(0, REG_TASK  , 0 );
#endif


    MIC_DEBUG_UART_PRINTF("\nMain Task: Loop \n");
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


    }
    _time_delay(MAIN_TASK_SLEEP_PERIOD);            // contact switch
#else

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
	    _time_delay(MAIN_TASK_SLEEP_PERIOD);            // context switch
#endif
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

void MQX_I2C0_IRQHandler( void ) {
    I2C_DRV_MasterIRQHandler(0);
}

void MQX_PORTC_IRQHandler(void)
{
	APPLICATION_MESSAGE_T *msg;
	_queue_id J1708_rx_qid = _msgq_get_id (0, J1708_RX_QUEUE);
	_queue_id J1708_tx_qid = _msgq_get_id (0, J1708_TX_QUEUE);

	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO0)) {
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
		msg = _msg_alloc (message_pool);
		msg->header.SOURCE_QID = J1708_rx_qid;
		msg->header.TARGET_QID = J1708_rx_qid;
		_msgq_send (msg);
	}

#if 0
	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO1)) {
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO1);
		msg = _msg_alloc (message_pool);
		msg->HEADER.SOURCE_QID = J1708_tx_qid;
		msg->HEADER.TARGET_QID = J1708_rx_qid;
		_msgq_send (msg);
	}
#endif
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

