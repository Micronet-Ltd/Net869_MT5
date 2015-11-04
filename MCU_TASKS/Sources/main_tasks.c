#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "fsl_i2c_master_driver.h"

#include "tasks_list.h"
#include "gpio_pins.h"

void MQX_I2C0_IRQHandler(void);

#define	MAIN_TASK_SLEEP_PERIOD	10			// 10 mSec sleep
#define TIME_ONE_SECOND_PERIOD	((int) (1000 / MAIN_TASK_SLEEP_PERIOD))

static i2c_master_state_t i2c_master;

void Main_task (uint32_t initial_data)
{
//	_task_id   ids[10] = {0};
	_queue_id  main_qid;	//, usb_qid, can1_qid, can2_qid, j1708_qid, acc_qid, reg_qid;
//	APPLICATION_MESSAGE *msg;

	uint8_t u8mainTaskLoopCnt = 0;


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


	_time_delay (10);
	GPIO_DRV_SetPinOutput   (USB_HUB_RSTN     );
	GPIO_DRV_SetPinOutput   (USB_ENABLE       );

//	_msgpool_create (sizeof(APPLICATION_MESSAGE), NUM_CLIENTS, 0, 0);
//	main_qid = _msgq_open(MAIN_QUEUE, 0);

	_task_create(0, ACC_TASK  , 0 );
	_task_create(0, USB_TASK  , 0 );

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
		GPIO_DRV_ClearPinOutput (LED_GREEN);
		_time_delay (1000);

		GPIO_DRV_SetPinOutput   (LED_GREEN);
		_time_delay (1000);
#endif
	}

	printf("\nMain Task: End \n");
   _task_block();		// should never get here

}

void MQX_I2C0_IRQHandler(void)
{
	I2C_DRV_MasterIRQHandler(0);
}
