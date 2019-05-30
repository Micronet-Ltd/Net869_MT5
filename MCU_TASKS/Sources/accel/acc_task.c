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
int32_t acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size);

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
	uint64_t                    usb_access_time, prev_time;
	acc_data_messg              acc_data_buff;
	pcdc_mic_queue_element_t    pqMemElem;
	int res;
    static int wmrk_rx;

	//APPLICATION_MESSAGE_T *msg;
	//const _queue_id acc_qid        = _msgq_open ((_queue_number)ACC_QUEUE, 0);
	//const _queue_id power_mgmt_qid  = _msgq_open ((_queue_number)POWER_MGM_QUEUE, 0);

	//uint8_t acc_good_count = 0;
	//uint32_t acc_bad_count = 0;

	/* */
	_mqx_uint event_result;
//#if (DEBUG_LOG)
	static uint64_t current_time, start_time;
    //sttic int got = 0, sent = 0;

	start_time = current_time = ms_from_start();
	printf("%s: started %llu\n", __func__, current_time);
//#endif

	event_result = _event_create("event.AccInt");
	if(MQX_OK != event_result){
		printf("%s: Could not create acc. event \n", __func__);
	}

	event_result = _event_open("event.AccInt", &g_acc_event_h);
	if(MQX_OK != event_result){
		printf("%s: Could not open acc. event \n", __func__);
	}

	printf("\n%s: Start \n", __func__);

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
	prev_time = usb_access_time = ms_from_start();
	
	while (0 == g_flag_Exit)
	{
		res = -1;
        wmrk_rx = sizeof(acc_data_buff.buff);
		do {
			res = acc_fifo_read (&acc_data_buff.buff[sizeof(acc_data_buff.buff) - wmrk_rx], wmrk_rx);
            if (res < 0) {
                printf("%s: fifo error timeout\n", __func__);
                //delay after read error, samples will miss
                _time_delay(10);
            } else {
                wmrk_rx -= res;
                if (0 != wmrk_rx) {
                    printf("%s: not all samples read %d\n", __func__, wmrk_rx);
                    _time_delay(1);
                    continue;
                }
                // read FIFO zero length is unpredictable, see datasheet of acc
                // the FIFO will zero when gpio spourious interrupt (see errata of k20) has occured, samples shouldn't be sent
                // Vladimir
                if (res > 0) {
                #if 0
                    // debug sample rate and watermark interrupt
                    // Vladimir
                    got += sizeof(acc_data_buff.buff)/6;
                    if (got - 10*(got/10)) {
                        printf("%s: read less of wmrk %d[%r]\n", __func__, wmrk_rx);
                    }
                #endif
                    acc_data_buff.timestamp = ms_from_start();
                    usb_access_time = acc_data_buff.timestamp;
                #if 0
                    // debug sequantual reachment of samples rate to host
                    // Vladimir
                    acc_data_buff.timestamp |= (uint64_t)got << 32;
                    if (usb_access_time > current_time + 60000) {
                        printf("%s: samples stat [%llu->%llu] for %llu", __func__, got, sent, usb_access_time - start_time);
                        current_time = usb_access_time;
                    }
                #endif
                    
                    /* Add delay on back to back reads to avoid overwhelming the USB */
                    if (usb_access_time - prev_time == 0)
                        _time_delay (1);

                    prev_time = usb_access_time;
                    pqMemElem = GetUSBWriteBuffer (MIC_CDC_USB_2);
                    if (pqMemElem) {
                    #if 0
                        // debug sequantual reachment of samples rate to host
                        // Vladimir
                        sent += sizeof(acc_data_buff.buff)/6;
                    #endif
                        pqMemElem->send_size = frame_encode((uint8_t*)&acc_data_buff, (const uint8_t*)(pqMemElem->data_buff), sizeof(acc_data_buff) );

                        if (!SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_2)) {
                            printf("%s: Error send data to CDC1\n", __func__);
                        }
                    }
                } else {
                    printf("%s: zero fifo %d\n", __func__, res);
                    //break;
                }
			}
		} while (res < 0 || wmrk_rx);

        // enable interrupts from acc_irq gpio
        //
        PORT_HAL_SetPinIntMode (PORTA, GPIO_EXTRACT_PIN(ACC_INT), kPortIntLogicZero);

        // wait for interrupt
        //
        _event_wait_all(g_acc_event_h, 1, 0);
        _event_clear(g_acc_event_h, 1);
	}

	// should never get here
	printf("\n%s: End \n", __func__);

	_task_block();
}

bool acc_receive_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status ;
	_mqx_uint ret = MQX_OK;

	if (*cmd > ACC_REG_MAX_ADDR)
	{
		printf("%s: invalid command address, address %d \n", __func__, *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
//	if ((ret =_mutex_lock(&g_i2c0_mutex)) != MQX_OK)
//	{
//		printf("acc_receive_data: i2c mutex lock failed, ret %d \n", ret);
//		_task_block();
//	}

	if ((i2c_status = I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device_g, cmd,  cmd_size, data, data_size, ACC_TIME_OUT*(data_size+1))) != kStatus_I2C_Success)
	{
		printf ("%s: ERROR: Could not receive command 0x%X (I2C error code %d)\n", __func__, *cmd, i2c_status);
	}

//	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}

bool acc_send_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status ;
	_mqx_uint ret = MQX_OK;

	if (*cmd > ACC_REG_MAX_ADDR)
	{
		printf("%s: invalid command address, address %d \n", __func__, *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
//	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
//	{
//		printf("accel_send_data: i2c mutex lock failed, ret %d \n", ret);
//		_task_block();
//	}

	if ((i2c_status = I2C_DRV_MasterSendDataBlocking (ACC_I2C_PORT, &acc_device_g, cmd,  cmd_size, data, data_size, ACC_TIME_OUT)) != kStatus_I2C_Success)
		printf ("%s: ERROR: Could not send command 0x%X (I2C error code %d)\n", __func__, *cmd, i2c_status);

//	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}


bool accInit (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	_mqx_uint ret = MQX_OK;
	i2c_status_t ret_i2c;

	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
	I2C_Enable(ACC_I2C_PORT); /* Note:Both accelerometer and accel are on the same bus*/

	// read recognition device ID
	write_data[0] = ACC_REG_WHO_AM_I   ;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))
	{
		printf("%s: i2c read ACC_REG_WHO_AM_I failed\n", __func__);
		goto _ACC_CONFIG_FAIL;
	}
	if (read_data == ACC_VALUE_ID)
	{
		printf ("%s: Device detected\n", __func__);
	}
	else
	{
		printf ("%s: Device NOT detected\n", __func__);
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

	_mutex_unlock(&g_i2c0_mutex);
	printf ("%s: Device Configured \n", __func__);
	return true;

_ACC_CONFIG_FAIL:
	_mutex_unlock(&g_i2c0_mutex);
	printf ("%s: ERROR: Device NOT Configured \n", __func__);
	return false;
}

void AccEnable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	_mqx_uint ret = MQX_OK;

	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
	// read recognition register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_ENABLE_FAIL;

	write_data[1] = read_data |= 0x1;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_ENABLE_FAIL;
	printf ("%s: Accelerometer Enabled \n", __func__);
	acc_enabled_g = TRUE;
	_mutex_unlock(&g_i2c0_mutex);
	return;

_ACC_ENABLE_FAIL:
	_mutex_unlock(&g_i2c0_mutex);
	printf ("%s: ERROR: Accelerometer NOT enabled \n", __func__);

}

void AccDisable (void)
{
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	_mqx_uint ret = MQX_OK;

    if (!acc_enabled_g) {
        return;
    }
    if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
    acc_enabled_g = FALSE;
	// read control register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_DISABLE_FAIL;

	write_data[1] = read_data &= ~0x1;
	if (!acc_send_data(&write_data[0], 1, &write_data[1], 1))	goto _ACC_DISABLE_FAIL;

		// read control register
	write_data[0] = ACC_REG_CTRL_REG1;
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))	goto _ACC_DISABLE_FAIL;

	printf ("%s: Accelerometer Disabled \n", __func__);

	_mutex_unlock(&g_i2c0_mutex);

	return;

_ACC_DISABLE_FAIL:
	_mutex_unlock(&g_i2c0_mutex);
	printf ("%s: ERROR: Accelerometer NOT disabled \n", __func__);
}

/* AccReadRegister: Helper function to read 1 byte from a register */
void AccReadRegister(uint8_t address, uint8_t * read_data)
{
	_mqx_uint ret = MQX_OK;

    if (!acc_enabled_g) {
        return;
    }
	printf("%s: read address: %X\n", __func__, address);
	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
	if (!acc_receive_data(&address, 1, read_data, 1))
	{
		printf("%s: read address: %d failed\n", __func__, address);
	}
	_mutex_unlock(&g_i2c0_mutex);
}

/* AccWriteRegister: Helper function to read 1 byte from a register */
void AccWriteRegister(uint8_t address, uint8_t write_data)
{
	_mqx_uint ret = MQX_OK;

	printf("%s: address: %X\n", __func__, address);

    if (!acc_enabled_g) {
        return;
    }
	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
	if (!acc_send_data(&address, 1, &write_data, 1))
	{
		printf("%s: write address: %d with data %d failed\n", __func__, address, write_data);
	}
	_mutex_unlock(&g_i2c0_mutex);
}

int32_t acc_fifo_read (uint8_t *buffer, uint8_t max_buffer_size)
{
	uint8_t u8ByteCnt      =  0 ;
	uint8_t read_data      =  0 ;
	uint8_t write_data [2] = {0};
	_mqx_uint ret = MQX_OK;

    if (!acc_enabled_g) {
        return -1;
    }

	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK) {
		printf("%s: i2c mutex lock failed, ret %d \n", __func__, ret);
		_task_block();
	}
	// read status register
	write_data[0] = ACC_REG_STATUS;
	//if (I2C_DRV_MasterReceiveDataBlocking (ACC_I2C_PORT, &acc_device, write_data,  1, &read_data, 1, TIME_OUT*10) != kStatus_I2C_Success)		goto _ACC_FIFO_READ_FAIL;
	//TODO: Why was timout set to 100*10 milliseconds??? Abid
	if (!acc_receive_data(&write_data[0], 1, &read_data, 1))
        goto _ACC_FIFO_READ_FAIL;

	u8ByteCnt  = (read_data & ACC_VALUE_STATUS_WATERMARK);				// get amount of samples in FIFO
	u8ByteCnt *= 6;														// read 2 Bytes per Sample (each sample is 12 bits): Max 192 Samples

	if (u8ByteCnt > max_buffer_size)
		u8ByteCnt = max_buffer_size;

	write_data[0] = ACC_REG_FIFO_SAMPLES;
    if (u8ByteCnt > 0) {
        if (!acc_receive_data(&write_data[0], 1, buffer, u8ByteCnt))
            goto _ACC_FIFO_READ_FAIL;
    }

	_mutex_unlock(&g_i2c0_mutex);
	return u8ByteCnt;

_ACC_FIFO_READ_FAIL:
	_mutex_unlock(&g_i2c0_mutex);
	printf ("%s: ERROR: Accelerometer read failure \n", __func__);
	return -1;
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
