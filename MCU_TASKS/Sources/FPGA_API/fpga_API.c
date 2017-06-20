#include "tasks_list.h"
#include "fpga_api.h"

#include "i2c_configuration.h"
#include "fsl_i2c_master_driver.h"

#include "uart_configuration.h"
#include <mutex.h>

#define FPGA_UART_PORT						UART1_IDX
#define FPGA_UART_BAUDRATE					115200
#define FPGA_UART_TIMEOUT					100

#define FPGA_I2C_PORT						I2C1_IDX
#define FPGA_DEVICE_ADDRESS					0x5E
#define FPGA_I2C_BAUD_RATE					400
#define FPGA_I2C_TIMEOUT					100

#define FPGA_ONE_WIRE_MAX_RETRIES			100

#define FPGA_UART_RX_BUF_SIZE 		100

//#define J1708_DEBUG

const uart_user_config_t fpga_uart_config = {
	.bitCountPerChar = kUart8BitsPerChar,
	.parityMode      = kUartParityDisabled,
	.stopBitCount    = kUartOneStopBit,
	.baudRate        = FPGA_UART_BAUDRATE
};

static i2c_device_t fpga_device = {
	.address         = FPGA_DEVICE_ADDRESS,
	.baudRate_kbps   = FPGA_I2C_BAUD_RATE
};

uint8_t fpga_uart_rx_buf [FPGA_UART_RX_BUF_SIZE] = {0};
uint8_t fpga_uart_rx_buf_wr_idx = 0;
uint8_t fpga_uart_rx_buf_rd_idx = 0;

bool FPGA_GetData (uint8_t Register_Addr, uint32_t *Register_Data);
bool FPGA_SetData (uint8_t Register_Addr, uint32_t *Register_Data);
bool check_uart_rx_buffer_overflow (uint8_t length);

/*****************************************************************
*                   FPGA initialization                          *
*****************************************************************/
void FPGA_init (void)
{
	FPGA_port_enable ();
}

void FPGA_port_enable (void)
{
	UART_Enable  (FPGA_UART_PORT, &fpga_uart_config);
	I2C_Enable   (FPGA_I2C_PORT);
}
void FPGA_port_disable (void)
{
	UART_Disable  (FPGA_UART_PORT);
	I2C_Disable   (FPGA_I2C_PORT);
}

/*****************************************************************
*                   Version Register                             *
*****************************************************************/
bool FPGA_read_version (uint32_t *version)
{
	if (version == NULL)
		return false;

	return FPGA_GetData (FPGA_REG_ADDR_VERSION, version);
}

/*****************************************************************
*                      IRQ Registers                             *
*****************************************************************/
bool FPGA_read_irq_status (uint32_t *status)
{
	if (status == NULL)
		return false;

	return FPGA_GetData (FPGA_REG_ADDR_IRQ_STATUS, status);
}

bool FPGA_read_irq_mask (uint32_t *status)
{
	if (status == NULL)
		return false;

	return FPGA_GetData (FPGA_REG_ADDR_IRQ_MASK, status);
}

bool FPGA_set_irq (uint32_t irqMask)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_IRQ_MASK, &data))
		return false;

	data |= irqMask;
	return FPGA_SetData (FPGA_REG_ADDR_IRQ_MASK, &data);
}

bool FPGA_clear_irq (uint32_t irqMask)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_IRQ_MASK, &data))
		return false;

	data &= ~irqMask;

	return FPGA_SetData (FPGA_REG_ADDR_IRQ_MASK, &data);
}

/*****************************************************************
*                       LED Register                             *
*****************************************************************/
bool FPGA_read_led_status  (uint8_t ledNum, uint8_t *brightness, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	uint32_t data = 0;
	bool status = false;

	switch (ledNum) {
		case LED_RIGHT  : status = FPGA_GetData (FPGA_REG_ADDR_LED_RIGHT , &data); break;
		case LED_MIDDLE : status = FPGA_GetData (FPGA_REG_ADDR_LED_MIDDLE, &data); break;
		case LED_LEFT   : status = FPGA_GetData (FPGA_REG_ADDR_LED_LEFT, &data); break;
	}

	if (status == false)
		return false;

	if (brightness != NULL)			*brightness = (data >> FPGA_REG_LED_BRIGHTNESS_SHIFT) & 0xFF;
	if (red        != NULL)			*red        = (data >> FPGA_REG_LED_RED_SHIFT       ) & 0xFF;
	if (green      != NULL)			*green      = (data >> FPGA_REG_LED_GREEN_SHIFT     ) & 0xFF;
	if (blue       != NULL)			*blue       = (data >> FPGA_REG_LED_BLUE_SHIFT      ) & 0xFF;
	return true;
}

bool FPGA_write_led_status (uint8_t ledNum, uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue)
{
	uint32_t data = 0;

	data  = ((brightness) << FPGA_REG_LED_BRIGHTNESS_SHIFT);
	data |= ((red)        << FPGA_REG_LED_RED_SHIFT       );
	data |= ((green)      << FPGA_REG_LED_GREEN_SHIFT     );
	data |= ((blue)       << FPGA_REG_LED_BLUE_SHIFT      );

	switch (ledNum) {
		case LED_RIGHT  : return FPGA_SetData (FPGA_REG_ADDR_LED_RIGHT , &data);
		case LED_MIDDLE : return FPGA_SetData (FPGA_REG_ADDR_LED_MIDDLE, &data);
		case LED_LEFT   : return FPGA_SetData (FPGA_REG_ADDR_LED_LEFT, &data);
	}
	return false;
}

/*****************************************************************
*                J1708 CONTROL Registers                         *
*****************************************************************/
bool FPGA_J1708_enable (void)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_CONTROL , &data))
		return false;

	data |= FPGA_REG_J1708_CONTROL_ENABLE;
	return FPGA_SetData (FPGA_REG_ADDR_J1708_CONTROL , &data);
}

bool FPGA_J1708_disable (void)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_CONTROL , &data))
		return false;

	fpga_uart_rx_buf_wr_idx = 0;
	fpga_uart_rx_buf_wr_idx = 0;


	data &= (~FPGA_REG_J1708_CONTROL_ENABLE);
	return FPGA_SetData (FPGA_REG_ADDR_J1708_CONTROL , &data);
}

bool FPGA_read_J1708_status (bool *status)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_CONTROL , &data))
		return false;

	if (status == NULL)			*status = data & FPGA_REG_J1708_CONTROL_ENABLE;
	return true;
}

bool FPGA_read_J1708_priority (uint8_t *priority)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_CONTROL , &data))
		return false;

	if (priority != NULL)			*priority = (data >> FPGA_REG_J1708_CONTROL_PRIORITY_SHIFT) & 0x7;
	return true;
}

bool FPGA_write_J1708_priority (uint8_t *priority)
{
	uint32_t data = 0;

	if (priority == NULL)
		return false;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_CONTROL , &data))
		return false;

	data &= FPGA_REG_J1708_CONTROL_ENABLE;					// clear previous priority
	data |= FPGA_REG_J1708_CONTROL_PRIO(*priority);
	return FPGA_SetData (FPGA_REG_ADDR_J1708_CONTROL , &data);
}

/*****************************************************************
*                J1708 Rx Length Registers                       *
*****************************************************************/
bool FPGA_read_J1708_rx_register (bool *status, uint8_t *len)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_RX_LEN , &data))
		return false;

	if (status != NULL) 		*status = (bool) (data >> FPGA_REG_J1708_STATUS_SHIFT);
	if (len    != NULL) 		*len    = (data >> FPGA_REG_J1708_LEN_SHIFT) & 0xFF ;
	return true;
}

/*****************************************************************
*                J1708 Tx Length Registers                       *
*****************************************************************/
bool FPGA_read_J1708_tx_register (bool *status, uint8_t *len)
{
	uint32_t data = 0;

	if (!FPGA_GetData (FPGA_REG_ADDR_J1708_TX_LEN , &data))
		return false;

	if (status != NULL)			*status = (bool) (data >> FPGA_REG_J1708_STATUS_SHIFT);
	if (len    != NULL)			*len    = (data >> FPGA_REG_J1708_LEN_SHIFT) & 0xFF ;
	return true;
}

bool FPGA_write_J1708_tx_length (uint8_t *len)
{
	uint32_t data;

	if (len == NULL)
		return false;

	data = *len;
	return FPGA_SetData (FPGA_REG_ADDR_J1708_TX_LEN , &data);
}

/*****************************************************************
*                      FPGA 1-WIRE INTERFACE                     *
*****************************************************************/
bool FPGA_one_wire_enable (void)
{
	uint32_t command = FPGA_REG_1WIRE_CONTROL_ENABLE_BIT;
	return FPGA_SetData (FPGA_REG_1WIRE_CONTROL_REG, &command);
}

bool FPGA_one_wire_disable (void)
{
	uint32_t command = 0;
	return FPGA_SetData (FPGA_REG_1WIRE_CONTROL_REG, &command);
}

bool FPGA_one_wire_reset (void)
{
	uint32_t command = FPGA_REG_1WIRE_CONTROL_RESET_BIT | FPGA_REG_1WIRE_CONTROL_ENABLE_BIT;
        
	if (!FPGA_one_wire_ready_status())
		return false;
	
        // data will be written only if module is enabled and ready for new command
	return FPGA_SetData (FPGA_REG_1WIRE_CONTROL_REG, &command);
}

bool FPGA_one_wire_write_byte (uint8_t data)
{
	uint32_t command = FPGA_REG_1WIRE_CONTROL_WRITE_BIT | FPGA_REG_1WIRE_CONTROL_ENABLE_BIT;
	uint32_t local_data = data;
        	
	if (!FPGA_one_wire_ready_status())
		return false;
	
        // data will be written only if module is enabled and ready for new command
	if (!FPGA_SetData (FPGA_REG_1WIRE_TX_REG, &local_data))
		return false;
        
	return FPGA_SetData (FPGA_REG_1WIRE_CONTROL_REG, &command);
}

bool FPGA_one_wire_read_byte (uint8_t *data)
{
	uint32_t local_data = 0;
	uint32_t command = FPGA_REG_1WIRE_CONTROL_READ_BIT | FPGA_REG_1WIRE_CONTROL_ENABLE_BIT;
	
	if (data == NULL)
		return false;

	if (!FPGA_one_wire_ready_status())
		return false;
	
	// data will be written only if module is enabled and ready for new command        
	if (!FPGA_SetData (FPGA_REG_1WIRE_CONTROL_REG, &command))
		return false;

	// wait till data arrive or timeout
	if (!FPGA_one_wire_ready_status())
		return false;

	if (!FPGA_GetData (FPGA_REG_1WIRE_RX_REG, &local_data))
		return false;

	*data = (uint8_t) local_data;
	return true;
}

bool FPGA_one_wire_get_device_present (void)
{
	uint32_t status = 0;

	if (!FPGA_one_wire_ready_status ())
		return false;
	
	if (!FPGA_GetData (FPGA_REG_1WIRE_CONTROL_REG, &status))
		return false;
	
	return ((status & FPGA_REG_1WIRE_CONTROL_PRESENT_BIT) != 0);
}
  	

bool FPGA_one_wire_ready_status (void)
{
	uint32_t status = 0;
	uint32_t counter = 0;
        
	for (counter = 0; counter < FPGA_ONE_WIRE_MAX_RETRIES; counter++) {
		if (!FPGA_GetData (FPGA_REG_1WIRE_CONTROL_REG, &status))
			return false;

                // if module disabled
		if ((status & FPGA_REG_1WIRE_CONTROL_ENABLE_BIT) == 0)
			return false;
		
                // if module is ready
		if ((status & FPGA_REG_1WIRE_CONTROL_READY_BIT) != 0)
			break;
	}
	//printf("%s: counter = %d\n",__func__, counter);
	return (counter < FPGA_ONE_WIRE_MAX_RETRIES);
}



/*****************************************************************
*                      FPGA I2C INTERFACE                        *
*****************************************************************/
bool FPGA_GetData (uint8_t register_addr, uint32_t *register_data)
{
	i2c_status_t  i2c_status ;
	uint8_t       i2c_cmd  = register_addr;

	if ((i2c_status = I2C_DRV_MasterReceiveDataBlocking (FPGA_I2C_PORT, &fpga_device, &i2c_cmd,  1, (uint8_t *)register_data, 4, FPGA_I2C_TIMEOUT)) != kStatus_I2C_Success)
	{
		printf ("\nFPGA API GetData: ERROR: Could not read Address 0x%X (I2C error code %d)\n", register_addr, i2c_status);
	}
	return (i2c_status == kStatus_I2C_Success);
}

bool FPGA_SetData (uint8_t register_addr, uint32_t *register_data)
{
	i2c_status_t  i2c_status ;
	uint8_t       i2c_cmd  = register_addr;

	if ((i2c_status = I2C_DRV_MasterSendDataBlocking (FPGA_I2C_PORT, &fpga_device, &i2c_cmd,  1, (uint8_t *)register_data, 4, FPGA_I2C_TIMEOUT)) != kStatus_I2C_Success)
		printf ("\nFPGA API SetData: ERROR: Could not write Address 0x%X (I2C error code %d)\n", register_addr, i2c_status);

	return (i2c_status == kStatus_I2C_Success);
}

/*****************************************************************
*                      FPGA UART INTERFACE                       *
*****************************************************************/
bool FPGA_write_J1708_packet (uint8_t *buffer, uint8_t length)
{
	uart_status_t uart_status   ;

	if ((uart_status = UART_DRV_SendDataBlocking (FPGA_UART_PORT, buffer, length, FPGA_UART_TIMEOUT)) != kStatus_UART_Success)
		printf ("\nFPGA: ERROR: Message was not sent to FPGA (UART error code %d)\n", uart_status);

#ifdef J1708_DEBUG
	printf("%s: %x, %x .. %x, %x, len = %d \n", __func__, buffer[0], buffer[1], buffer[2], buffer[3], length);
#endif
	return (uart_status == kStatus_UART_Success);
}

bool FPGA_read_J1708_packet (uint8_t *buffer, uint8_t length)
{
	uint8_t *buf_ptr = buffer;
	uint8_t len  = length;
	uint8_t size = 0;

	if (buf_ptr == NULL)
		return false;

	if (check_uart_rx_buffer_overflow (len) == true){
		printf("%s: check_uart_rx_buffer_overflow \n");
		return false;
	}
	// Since the buffer is cyclic, it might be needed to generate the buffer by 2 stages
	// first stage - only if source filled up and returned to beginning
	if (fpga_uart_rx_buf_rd_idx + len >= FPGA_UART_RX_BUF_SIZE) {
		size  =  FPGA_UART_RX_BUF_SIZE - fpga_uart_rx_buf_rd_idx;
		len  -= (FPGA_UART_RX_BUF_SIZE - fpga_uart_rx_buf_rd_idx);
		memcpy (buf_ptr, &fpga_uart_rx_buf[fpga_uart_rx_buf_rd_idx], size);
		buf_ptr += size;
		fpga_uart_rx_buf_rd_idx = 0;
	}

	// second stage - copy the rest of the left buffer in case of loop around or in normal case copy the original buffer "AS IS"
	if (len != 0) {
		size += len;
		memcpy (buf_ptr, &fpga_uart_rx_buf[fpga_uart_rx_buf_rd_idx], size);
		fpga_uart_rx_buf_rd_idx += len;
	}
#ifdef J1708_DEBUG
	printf("%s: %x, %x .. %x, %x, len = %d \n", __func__, buffer[0], buffer[1], buffer[2], buffer[3], size);
#endif
	return true;
}


bool check_uart_rx_buffer_overflow (uint8_t length)
{
	if (fpga_uart_rx_buf_rd_idx < fpga_uart_rx_buf_wr_idx) {
		if (fpga_uart_rx_buf_rd_idx + length > fpga_uart_rx_buf_wr_idx) {
			printf ("\nFPGA_UART_Rx: ERROR: Internal Buffer Overflow (Rd = %d Wr = %d len = %d\n", fpga_uart_rx_buf_rd_idx, fpga_uart_rx_buf_wr_idx, length);
			return true;
		}
	} else if (fpga_uart_rx_buf_rd_idx + length > fpga_uart_rx_buf_wr_idx + FPGA_UART_RX_BUF_SIZE) {
		printf ("\nFPGA_UART_Rx: ERROR: Internal Buffer Overflow (Rd = %d Wr = %d len = %d\n", fpga_uart_rx_buf_rd_idx, fpga_uart_rx_buf_wr_idx, length);
		return true;
	}
	return false;
}

/*****************************************************************
*                        FPGA UART RX TASK                       *
*****************************************************************/
void FPGA_UART_Rx_task (uint32_t initial_data)
{
	uart_status_t uart_rx_status;

	fpga_uart_rx_buf_wr_idx = 0;
	printf ("\nFPGA_UART_Rx Task: Start \n");

	while (0 == g_flag_Exit) {
		// get message via UART channel
		uart_rx_status = UART_DRV_ReceiveDataBlocking (FPGA_UART_PORT, &fpga_uart_rx_buf [fpga_uart_rx_buf_wr_idx], 1, 1000);

		if (uart_rx_status == kStatus_UART_Success)
			if (++fpga_uart_rx_buf_wr_idx >= FPGA_UART_RX_BUF_SIZE)
				fpga_uart_rx_buf_wr_idx = 0;			// loop over

	}

	// should never get here
	printf ("\nFPGA_UART_Rx Task: End \n");
	_task_block();
}
