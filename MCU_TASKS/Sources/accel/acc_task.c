#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <mutex.h>
//#include <sem.h>

#include <event.h>

//extern const gpio_input_pin_user_config_t inputPins *;
#include "acc_task.h"
#include "gpio_pins.h"
#include "tasks_list.h"

#include "fsl_i2c_master_driver.h"

#include "mic_typedef.h"
#include "i2c_configuration.h"

#include "frame.h"

#define ACC_DEVICE_ADDRESS 			0x1D
#define	I2C_BAUD_RATE				400
#define ACC_TIME_OUT				200		//  in milliseconds

#define	ACC_I2C_PORT				I2C0_IDX

#define ACC_VALUE_ID				0x4A
#define ACC_VALUE_STATUS_WATERMARK	0x3F
#define ACC_VALUE_RESET_CMD			0x40

#define ACC_REG_STATUS				0x00
#define ACC_REG_FIFO_SAMPLES		0x01
#define ACC_REG_F_SETUP				0x09
#define ACC_REG_XYZ_DATA_CFG		0x0E
#define ACC_REG_WHO_AM_I			0x0D
#define ACC_REG_CTRL_REG1			0x2A
#define ACC_REG_CTRL_REG2			0x2B
#define ACC_REG_CTRL_REG4			0x2D
#define ACC_REG_CTRL_REG5			0x2E
#define ACC_REG_MAX_ADDR			0x32

#define MAX_FIFO_SIZE				192
#define ACC_MAX_POOL_SIZE			10
#define ACC_XYZ_PKT_SIZE			6

#define TIME_OUT					200		//  in miliseconds

#define DEBUG						TRUE

static i2c_device_t acc_device_g = {.address = ACC_DEVICE_ADDRESS,    .baudRate_kbps = I2C_BAUD_RATE};
bool acc_enabled_g = false;

extern MUTEX_STRUCT g_i2c0_mutex;

__packed typedef struct{
	uint64_t timestamp;
	uint8_t buff[ACC_XYZ_PKT_SIZE * ACC_MAX_POOL_SIZE];
}acc_data_messg, *pacc_data_messg;

/**************************************************************************************
* The accelerometer is connected to MCU I2C interface. When device is powered, the    *
* accelerometer is accessed every 1.25mSec (800 Hz), reading 32 samples of each axis. *
* When the device is off, accelerometer is not accessed at all and configured to be   *
* in low power mode.                                                                  *
*                                                                                     *
* The MCU resets and reconfigure the accelerometer every power up. accelerometer data *
* is read from its internal FIFO.
**************************************************************************************/
void ISR_accIrq    (void* param);
bool acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size);

void * g_acc_event_h;

#if 0
void acc_irq(void)
{
	GPIO_DRV_ClearPinIntFlag(ACC_INT);
	// Signal main task to read acc
	_event_set(g_acc_event_h, 1);
}

void AccIntEnable()
{
	uint32_t port;

	const gpio_input_pin_user_config_t inputPin =
			{
					  .pinName = ACC_INT,
					  .config.isPullEnable = false,
					  .config.pullSelect = kPortPullUp,
					  .config.isPassiveFilterEnabled = false,
					  .config.interrupt = kPortIntFallingEdge,
			  };

	// FIXME: hardcoded index
	port = GPIO_EXTRACT_PORT(ACC_INT);
	NVIC_SetPriority(g_portIrqId[port], ACC_NVIC_IRQ_Priority);
	OSA_InstallIntHandler(g_portIrqId[port], acc_irq);

	GPIO_DRV_InputPinInit(&inputPin);
}
#endif

//#define MIC_LED_TEST
uint32_t acc_time_diff_zero = 0;
uint32_t acc_time_diff_non_zero = 0;

void Acc_task (uint32_t initial_data)
{
	TIME_STRUCT                 time;
	TIME_STRUCT                 new_time;
	uint64_t                    time_diff;
	acc_data_messg              acc_data_buff;
	pcdc_mic_queue_element_t    pqMemElem;

	//APPLICATION_MESSAGE_T *msg;
	//const _queue_id acc_qid        = _msgq_open ((_queue_number)ACC_QUEUE, 0);
	//const _queue_id power_mgmt_qid  = _msgq_open ((_queue_number)POWER_MGM_QUEUE, 0);

	//uint8_t acc_good_count = 0;
	//uint32_t acc_bad_count = 0;

	/* */
	_mqx_uint event_result;


	event_result = _event_create("event.AccInt");
	if(MQX_OK != event_result){
		printf("ACC Task: Could not create acc. event \n");
	}

	event_result = _event_open("event.AccInt", &g_acc_event_h);
	if(MQX_OK != event_result){
		printf("ACC Task: Could not open acc. event \n");
	}

	printf("\nACC Task: Start \n");

//	GPIO_DRV_SetPinOutput   (ACC_VIB_ENABLE       );
	_time_delay(100);
//	AccIntEnable();
	// try to initialize accelerometer every 10 seconds
	while (accInit () == false)
	{
		_time_delay (10000);
	}

	AccEnable();

	//TODO: Remote Test acc message
	//test_acc_msg.header.SOURCE_QID = acc_qid;
	//test_acc_msg.header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
	//test_acc_msg.header.SIZE = sizeof(MESSAGE_HEADER_STRUCT) + strlen((char *)msg->data) + 1;
	//acc_msg = &test_acc_msg;
	while (0 == g_flag_Exit)
	{
        _event_wait_all(g_acc_event_h, 1, 0);
		_event_clear(g_acc_event_h, 1);

		_time_get(&new_time);
		time_diff = ((new_time.SECONDS * 1000) +  new_time.MILLISECONDS) - ((time.SECONDS * 1000) +  time.MILLISECONDS);
		/* Add delay on back to back reads to avoid overwhelming the USB */
		if (time_diff == 0)
		{
			_time_delay (1);
		}

		if(acc_fifo_read (acc_data_buff.buff, (uint8_t)(ACC_XYZ_PKT_SIZE * ACC_MAX_POOL_SIZE)))
		{
			_time_get(&time);
			acc_data_buff.timestamp = time.SECONDS * 1000 + time.MILLISECONDS;

			pqMemElem = GetUSBWriteBuffer (MIC_CDC_USB_2);
			if (NULL == pqMemElem)
			{
				printf("%s: Error get mem for USB drop\n", __func__);
				continue;
			}

			pqMemElem->send_size = frame_encode((uint8_t*)&acc_data_buff, (const uint8_t*)(pqMemElem->data_buff), sizeof(acc_data_buff) );

			if (!SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_2))
			{
				printf("%s: Error send data to CDC1\n", __func__);
			}
		}
		else
		{
		  	_time_delay(1000);//delay after read error 
		}
	}

	// should never get here
	printf("\nACC Task: End \n");
	_task_block();
}

bool acc_receive_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status ;
	_mqx_uint ret = MQX_OK;

	if (*cmd > ACC_REG_MAX_ADDR)
	{
		printf("acc_receive_data: invalid command address, address %d \n", *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
	if ((ret =_mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("acc_receive_data: i2c mutex lock failed, ret %d \n", ret);
		_task_block();
	}

	if ((i2c_status = I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device_g, cmd,  cmd_size, data, data_size, ACC_TIME_OUT*data_size)) != kStatus_I2C_Success)
	{
		printf ("acc_receive_data: ERROR: Could not receive command 0x%X (I2C error code %d)\n", *cmd, i2c_status);
	}

	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}

bool acc_send_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status ;
	_mqx_uint ret = MQX_OK;

	if (*cmd > ACC_REG_MAX_ADDR)
	{
		printf("accel_send_data: invalid command address, address %d \n", *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("accel_send_data: i2c mutex lock failed, ret %d \n", ret);
		_task_block();
	}

	if ((i2c_status = I2C_DRV_MasterSendDataBlocking (ACC_I2C_PORT, &acc_device_g, cmd,  cmd_size, data, data_size, ACC_TIME_OUT)) != kStatus_I2C_Success)
		printf ("accel_send_data: ERROR: Could not send command 0x%X (I2C error code %d)\n", *cmd, i2c_status);

	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}


bool accInit (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	_mqx_uint ret = MQX_OK;
	i2c_status_t ret_i2c;

	I2C_Enable(ACC_I2C_PORT); /* Note:Both accelerometer and accel are on the same bus*/

	// read recognition device ID
	write_data[0] = ACC_REG_WHO_AM_I   ;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))
	{
		printf("accInit: i2c read ACC_REG_WHO_AM_I failed\n");
		goto _ACC_CONFIG_FAIL;
	}
	if (read_data == ACC_VALUE_ID)
	{
		printf ("ACC Task: Device detected\n");
	}
	else
	{
		printf ("ACC Task: Device NOT detected\n");
		goto _ACC_CONFIG_FAIL;
	}

	// reset device
	write_data[0] = ACC_REG_CTRL_REG2   ;
	write_data[1] = ACC_VALUE_RESET_CMD ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;
	OSA_TimeDelay(1);

	// set CTRL_REG1 to STANDBY Normal mode with 1.56Hz sample rates reads at SLEEP mode and 800Hz at ACTIVE mode
	write_data[0] = ACC_REG_CTRL_REG1   ;
	write_data[1] = 0x00 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_CTRL_REG1   ;
	write_data[1] = 0xC0 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	// configure device to Low power in Sleep mode and Normal power mode at Active
	write_data[0] = ACC_REG_CTRL_REG2   ;
	write_data[1] = 0x18 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	// configure interrupt source to FIFO interrupt on INT1 pin with water mark of 10 samples
	// when FIFO sample count exceeding the water mark event does not stop the FIFO from accepting new data
	// FIFO always contains the most recent samples when overflowed (FMODE = 01)
	write_data[0] = ACC_REG_CTRL_REG5   ;
	write_data[1] = 0x40 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_CTRL_REG4   ;
	write_data[1] = 0x40 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_F_SETUP   ;
	write_data[1] = 0x4A ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	// configure output buffer data format using 8g scale range
	write_data[0] = ACC_REG_XYZ_DATA_CFG   ;
	write_data[1] = 0x02 ;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_CONFIG_FAIL;

	printf ("ACC Task: Device Configured \n");
	_mutex_unlock(&g_i2c0_mutex);
	return true;

_ACC_CONFIG_FAIL:
	printf ("ACC Task: ERROR: Device NOT Configured \n");
	_mutex_unlock(&g_i2c0_mutex);
	return false;
}

void AccEnable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};

	// read recognition register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_ENABLE_FAIL;

	write_data[1] = read_data |= 0x1;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_ENABLE_FAIL;
	printf ("ACC Task: Accelerometer Enabled \n");
	acc_enabled_g = TRUE;
	return;

_ACC_ENABLE_FAIL:
	printf ("ACC Task: ERROR: Accelerometer NOT enabled \n");

}

void AccDisable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};

	// read control register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_DISABLE_FAIL;

	write_data[1] = read_data &= ~0x1;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_DISABLE_FAIL;

		// read control register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_DISABLE_FAIL;

	printf ("ACC Task: Accelerometer Disabled \n");
	acc_enabled_g = FALSE;
	return;

_ACC_DISABLE_FAIL:
	printf ("ACC Task: ERROR: Accelerometer NOT disabled \n");
}

/* AccReadRegister: Helper function to read 1 byte from a register */
void AccReadRegister(uint8_t address, uint8_t * read_data)
{
	if (!acc_receive_data(&address, 1, read_data, 1))
	{
		printf("%s: read address: %d failed\n", __func__, address);
	}
}

/* AccWriteRegister: Helper function to read 1 byte from a register */
void AccWriteRegister(uint8_t address, uint8_t write_data)
{
	if (!acc_send_data(&address, 1, &write_data, 1))
	{
		printf("%s: write address: %d with data %d failed\n", __func__, address, write_data);
	}
}

bool acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size)
{
	uint8_t u8ByteCnt      =  0 ;
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};

	// read status register
	write_data[0] = ACC_REG_STATUS;
	//if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT*10) != kStatus_I2C_Success)		goto _ACC_FIFO_READ_FAIL;
	//TODO: Why was timout set to 100*10 milliseconds??? Abid
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_FIFO_READ_FAIL;

	u8ByteCnt  = (read_data & ACC_VALUE_STATUS_WATERMARK);				// get amount of samples in FIFO
	u8ByteCnt *= 6;														// read 2 Bytes per Sample (each sample is 12 bits): Max 192 Samples

	if (u8ByteCnt > max_buffer_size)
		u8ByteCnt = max_buffer_size;

	write_data[0] = ACC_REG_FIFO_SAMPLES;
	//if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, buffer, u8ByteCnt, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_FIFO_READ_FAIL;
	if (!acc_receive_data(&write_data[0], 1, buffer, u8ByteCnt))		goto _ACC_FIFO_READ_FAIL;

	return TRUE;

_ACC_FIFO_READ_FAIL:
	printf ("ACC Task: ERROR: Accelerometer read failure \n");
	return FALSE;
}

#if 0
void ISR_accIrq (void* param)
{
	LWGPIO_STRUCT_PTR gpio = (LWGPIO_STRUCT_PTR) param;
	APPLICATION_MESSAGE *msg;

	// TODO: change to event instead of message
	if ((msg = _msg_alloc_system (sizeof(*msg))) != NULL)
		 msg->header.SOURCE_QID =  _msgq_get_id(0, ACC_QUEUE);
		 msg->header.TARGET_QID =  _msgq_get_id(0, ACC_QUEUE);
		 _msgq_send (msg);
	}
	lwgpio_int_clear_flag (gpio);
}
#endif
