
#include "tasks_list.h"

#include "fsl_i2c_master_driver.h"

#define ACC_DEVICE_ADDRESS 			0x1D
#define	I2C_BAUD_RATE				400

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

#define TIME_OUT					100		//  in miliseconds

static i2c_device_t acc_device = {.address = ACC_DEVICE_ADDRESS,    .baudRate_kbps = I2C_BAUD_RATE};

/**************************************************************************************
* The accelerometer is connected to MCU I2C interface. When device is powered, the    *
* accelerometer is accessed every 1.25mSec (800 Hz), reading 32 samples of each axis. *
* When the device is off, accelerometer is not accessed at all and configured to be   *
* in low power mode.                                                                  *
*                                                                                     *
* The MCU resets and reconfigure the accelerometer every power up. accelerometer data *
* is read from its internal FIFO.
**************************************************************************************/
bool accInit       (void);
void AccDisable    (void);
void AccEnable     (void);
void ISR_accIrq    (void* param);
void acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size);

void Acc_task (uint32_t initial_data)
{
//	TIME_STRUCT time;
//	APPLICATION_MESSAGE *msg;
//	APPLICATION_MESSAGE *acc_msg;
//	_queue_id acc_qid        = _msgq_get_id (0, ACC_QUEUE       );
//	_queue_id usb_qid        = _msgq_get_id (0, USB_QUEUE       );
//	_queue_id powerMgmt_qid  = _msgq_get_id (0, POWER_MGMT_QUEUE);
	
	uint8_t buffer [100]  = {0};
	printf("\nACC Task: Start \n");

	// try to initialize accelerometer every 10 seconds
	while (accInit () == false) {
		_time_delay (10000);
	}

	while (1) {
#if 0
		msg = _msgq_receive(acc_qid, 10000);												// wait 10 second for message 
		if (msg == NULL) {																	// if message not recieved
			printf("\nACC Task: Info: Message not received in last 10 Seconds \n");
			continue;
		}

		switch (msg->header.SOURCE_QID) {
				case powerMgmt_qid:
					if (msg->DATA == ACC_ENABLE)					AccEnable  ();
					else											AccDisable ();
					break;
					
				case acc_qid:
					if ((acc_msg = _msg_alloc_system (sizeof(*acc_msg))) == NULL) {
						printf ("ACC Task: ERROR: message allocation failed\n");
						break;
					}
					
					acc_fifo_read (&(acc_msg->data), sizeof ((acc_msg->data)));
					_time_get(&time);
					acc_msg->timestamp0 = 0;
					acc_msg->timestamp1 = time.MILLISECONDS;
					acc_msg->header.SOURCE_QID = acc_qid;
					acc_msg->header.TARGET_QID = usb_qid;
					_msgq_send (acc_msg);
					break;

				default:
					printf ("ACC Task: Unexpected message - Source id %d\n", msg->HEADER.SOURCE_QID);
					break;
			}
		}
		_msg_free(msg);
#else
		acc_fifo_read (buffer, sizeof (buffer));
		_time_delay (1000);
#endif
	}
	
	// should never get here
	printf("\nACC Task: End \n");
	_task_block();
}


bool accInit (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};

	// read recognition device ID
	write_data[0] = ACC_REG_WHO_AM_I   ;
	if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;
	if (read_data == ACC_VALUE_ID)
		printf ("ACC Task: Device detected\n");
	else {
		printf ("ACC Task: Device NOT detected\n");
		goto _ACC_CONFIG_FAIL;
	}
	
	// reset device
	write_data[0] = ACC_REG_CTRL_REG2   ;
	write_data[1] = ACC_VALUE_RESET_CMD ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;
    OSA_TimeDelay(1);

	// set CTRL_REG1 to STANDBY Normal mode with 1.56Hz sample rates redas at SLEEP mode and 800Hz at ACTIVE mode
	write_data[0] = ACC_REG_CTRL_REG1   ;
	write_data[1] = 0x00 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_CTRL_REG1   ;
	write_data[1] = 0xC0 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	// configure device to Low power in Sleep mode and Normal power mode at Active
	write_data[0] = ACC_REG_CTRL_REG2   ;
	write_data[1] = 0x18 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	// configure interrupt source to FIFO interrupt on INT1 pin with watter mark of 10 samples
	// when FIFO sample count exceeding the watermark event does not stop the FIFO from accepting new data
	// FIFO always contains the most recent samples when overflowed (FMODE = 01)
	write_data[0] = ACC_REG_CTRL_REG5   ;
	write_data[1] = 0x40 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_CTRL_REG4   ;
	write_data[1] = 0x40 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	write_data[0] = ACC_REG_F_SETUP   ;
	write_data[1] = 0x4A ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;
	
	// configure output buffer data format using 8g scale range
	write_data[0] = ACC_REG_XYZ_DATA_CFG   ;
	write_data[1] = 0x02 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	// set ACC to ACTIVE mode
	write_data[0] = ACC_REG_CTRL_REG1   ;
	write_data[1] = 0xC1 ;
    if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_CONFIG_FAIL;

	printf ("ACC Task: Device Configured \n");
	return true;

_ACC_CONFIG_FAIL:
	printf ("ACC Task: ERROR: Device NOT Configured \n");
	return false;
} 

void AccEnable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};

	// read recognition register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_ENABLE_FAIL;

	write_data[1] = read_data |= 0x1;
	if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)			goto _ACC_ENABLE_FAIL;
	printf ("ACC Task: Accelerometer Enabled \n");
	return;

_ACC_ENABLE_FAIL:
	printf ("ACC Task: ERROR: Accelerometer NOT enabled \n");

}

void AccDisable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	
	// read recognition register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_DISABLE_FAIL;

	write_data[1] = read_data &= ~0x1;
	if (I2C_DRV_MasterSendDataBlocking    (ACC_I2C_PORT, &acc_device, NULL,  0, write_data, 2, TIME_OUT) != kStatus_I2C_Success)			goto _ACC_DISABLE_FAIL;
	printf ("ACC Task: Accelerometer Disabled \n");
	return;

_ACC_DISABLE_FAIL:
	printf ("ACC Task: ERROR: Accelerometer NOT disabled \n");
}

void acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size)
{
	uint8_t u8ByteCnt      =  0 ;
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	
	// read status register
	write_data[0] = ACC_REG_STATUS;
	if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT*10) != kStatus_I2C_Success)		goto _ACC_FIFO_READ_FAIL;

	u8ByteCnt  = (read_data & ACC_VALUE_STATUS_WATERMARK);				// get amount of samples in FIFO
	u8ByteCnt *= 6;														// read 2 Bytes per Sample (each sample is 12 bits)
	
	if (u8ByteCnt > max_buffer_size)
		u8ByteCnt = max_buffer_size;

	write_data[0] = ACC_REG_FIFO_SAMPLES;
	if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, buffer, u8ByteCnt, TIME_OUT) != kStatus_I2C_Success)		goto _ACC_FIFO_READ_FAIL;
	return;

_ACC_FIFO_READ_FAIL:
	printf ("ACC Task: ERROR: Accelerometer read failure \n");
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

