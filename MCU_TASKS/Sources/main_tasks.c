#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "fsl_i2c_master_driver.h"

#include "tasks_list.h"
#include "gpio_pins.h"
#include "J1708_task.h"
#include "fpga_api.h"

void MQX_I2C0_IRQHandler(void);
void MQX_PORTC_IRQHandler(void);

#define	MAIN_TASK_SLEEP_PERIOD	10			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))

static i2c_master_state_t i2c_master;
_pool_id   message_pool;

void Main_task (uint32_t initial_data)
{
//	_task_id   ids[10] = {0};
	_queue_id  main_qid;	//, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
	_queue_id  j1708_rx_qid;
	APPLICATION_MESSAGE *msg;

	uint8_t u8mainTaskLoopCnt = 0;
	_time_delay (10);

	printf("\nMain Task: Start \n");
#if 0
	PinMuxConfig ();
	ADCInit      ();
	USBInit      ();
	I2CInit      ();
	registerInit ();
#endif

	// board Initialization
//  hardware_init();
//	OSA_Init();
	GPIO_Config();

	OSA_InstallIntHandler(PORTC_IRQn, MQX_PORTC_IRQHandler);

	// I2C0 Initialization
	NVIC_SetPriority(I2C0_IRQn, 6U);
	OSA_InstallIntHandler(I2C0_IRQn, MQX_I2C0_IRQHandler);
	I2C_DRV_MasterInit   (I2C0_IDX, &i2c_master);

	// turn on device
	GPIO_DRV_SetPinOutput   (POWER_3V3_ENABLE );
	GPIO_DRV_SetPinOutput   (POWER_5V0_ENABLE );
	GPIO_DRV_SetPinOutput   (ACC_ENABLE       );

	// FPGA Enable
	GPIO_DRV_SetPinOutput   (FPGA_PWR_ENABLE  );

//	BOARD_InitOsc0();
//	CLOCK_SetBootConfig_Run ();

	// Enable USB for DEBUG
	GPIO_DRV_ClearPinOutput (USB_ENABLE       );
	GPIO_DRV_ClearPinOutput (USB_HUB_RSTN     );
	GPIO_DRV_SetPinOutput   (USB_OTG_OE       );	// disable USB MUX
	GPIO_DRV_SetPinOutput   (USB_OTG_SEL      );	// disable USB MUX


	GPIO_DRV_SetPinOutput   (USB_HUB_RSTN     );
	GPIO_DRV_SetPinOutput   (USB_ENABLE       );

	message_pool = _msgpool_create (sizeof(APPLICATION_MESSAGE), NUM_CLIENTS, 0, 0);
	main_qid = _msgq_open(MAIN_QUEUE, 0);

//	_task_create(0, ACC_TASK  , 0 );
//	_task_create(0, USB_TASK  , 0 );
	_time_delay (1000);

	FPGA_init ();

	GPIO_DRV_SetPinOutput   (CAN_ENABLE       );

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

#if 0
	ids[0] = _task_create(0, USB_TASK  , 0 );
	ids[1] = _task_create(0, CAN_TASK  , 1 );
	ids[2] = _task_create(0, CAN_TASK  , 2 );
	ids[3] = _task_create(0, J1708_TASK, 0 );
	ids[4] = _task_create(0, ACC_TASK  , 0 );
	ids[5] = _task_create(0, REG_TASK  , 0 );
#endif


	printf("\nMain Task: Loop \n");
	while (1) {
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
				printf ("MAIN TASK: INPUT VOLTAGE NORMAL\n");
		} else {
			GPIOSet     (CPU_POWER_LOSS);
			GPIOSet     (POWER_DISCHARGE_ENABLE);
			if (u8InputVoltageStatus_Prev != u8InputVoltageStatus)
				printf ("MAIN TASK: INPUT VOLTAGE OUT OF RANGE\n");
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
		_time_delay(MAIN_TASK_SLEEP_PERIOD);			// contact switch
#else
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
	}

	printf("\nMain Task: End \n");
   _task_block();		// should never get here

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

void MQX_I2C0_IRQHandler(void)
{
	I2C_DRV_MasterIRQHandler(0);
}


void MQX_PORTC_IRQHandler(void)
{
	APPLICATION_MESSAGE *msg;
	_queue_id J1708_rx_qid = _msgq_get_id (0, J1708_RX_QUEUE);
	_queue_id J1708_tx_qid = _msgq_get_id (0, J1708_TX_QUEUE);

	if (GPIO_DRV_IsPinIntPending (FPGA_GPIO0)) {
		GPIO_DRV_ClearPinIntFlag(FPGA_GPIO0);
		msg = _msg_alloc (message_pool);
		msg->HEADER.SOURCE_QID = J1708_rx_qid;
		msg->HEADER.TARGET_QID = J1708_rx_qid;
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
