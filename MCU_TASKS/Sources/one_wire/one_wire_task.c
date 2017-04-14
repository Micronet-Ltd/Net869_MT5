#include "tasks_list.h"
#include "one_wire_task.h"
#include "fpga_api.h"
#include "gpio_pins.h"
#include "mic_typedef.h"
#include "protocol.h"
#include "control_task.h"

//#define ONE_WIRE_DEBUG 1
#define ONE_WIRE_DEFAULT_TIME_DELAY 500
#define ONE_WIRE_QUICK_TIME_DELAY	50

typedef struct
{
	uint32_t read_error;
	uint32_t crc_error;
	uint32_t invalid_family_code;
}one_wire_error_t;

bool crc_check (uint8_t *pdata);
void send_one_wire_data (uint8_t *one_wire_data, uint8_t size);

void one_wire_task (uint32_t initial_data)
{
	uint32_t i;
	uint32_t wait_time = ONE_WIRE_DEFAULT_TIME_DELAY;
	one_wire_error_t err = {0, 0, 0};
#ifdef ONE_WIRE_DEBUG
	uint32_t count = 0;
#endif        
    uint8_t data_rx [10] = {0};

	printf ("\n1-WIRE Task: Start \n");

	FPGA_one_wire_enable();

	while (1) 
	{
		_time_delay (wait_time);
		
		FPGA_one_wire_reset ();
		
		if (!FPGA_one_wire_get_device_present()) 
		{
			wait_time = ONE_WIRE_DEFAULT_TIME_DELAY;
			continue;
        }

		FPGA_one_wire_write_byte (DALLAS_BUTTON_READ);
			
		for (i = 0; i < 8; i++)
		{
			if (!(FPGA_one_wire_read_byte (&data_rx[i])))
			{
				err.read_error++;
				printf("%s: read error, count %d \n", __func__, err.read_error);
				break;	
			}
		}
		
		if (data_rx[0] == 0)
		{
			err.invalid_family_code++;
			printf("%s: invalid family code, count %d \n", __func__, err.invalid_family_code);
			wait_time = ONE_WIRE_QUICK_TIME_DELAY;
			continue;
		}

		if (crc_check (&data_rx[0]))
		{
			send_one_wire_data (data_rx, 8);
			wait_time = ONE_WIRE_DEFAULT_TIME_DELAY;
		}
		else
		{
			wait_time = ONE_WIRE_QUICK_TIME_DELAY;
			err.crc_error++;
			printf("%s: CRC error, count %d \n", __func__, err.crc_error);
		}
#ifdef ONE_WIRE_DEBUG
		count++;
		printf("%s: %d data_rx: %02x, %02x, %02x, %02x,", __func__, count, data_rx[7],  data_rx[6],  data_rx[5],  data_rx[4]);
		printf("%02x, %02x, %02x, %02x,\n",  data_rx[3],  data_rx[2],  data_rx[1],  data_rx[0]);
#endif		
	}

	// should never get here
	printf ("\n1-WIRE Task: End \n");
	_task_block();
}

void send_one_wire_data (uint8_t *one_wire_data, uint8_t size)
{
	packet_t one_wire_pkt;
	uint8_t i = 0;
	
	if (size > MAX_PACKET_SIZE)
	{
		size = 	MAX_PACKET_SIZE;
	}

	one_wire_pkt.pkt_type = ONE_WIRE_DATA;
	for (i=0; i < size; i++)
	{
		one_wire_pkt.data[i] = one_wire_data[i];
	}

	send_control_msg(&one_wire_pkt, size);
}


// for debug
bool crc_check (uint8_t *pdata)
{ 
	uint8_t local_data = 0;
	int crc_byte = 0; 
	int byte_counter; 
	int bit_counter;
        
	for(byte_counter=0; byte_counter < 8; byte_counter++, pdata++) 
	{
		local_data = *pdata;
		for(bit_counter=0; bit_counter < 8; bit_counter++) 
		{ 
        	if ( (local_data & 0x1) == (crc_byte & 0x1)) 
			{
				local_data >>= 1; 
				crc_byte    >>= 1; 
				continue; 
			} 
			
			local_data >>= 1; 
			crc_byte    >>= 1; 
			crc_byte = crc_byte ^ 0x8C; 
		}
	}

	return crc_byte == 0;
}
