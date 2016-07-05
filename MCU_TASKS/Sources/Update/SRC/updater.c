/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mqx_inc.h"
#include "tasks_list.h"
#include "fsl_uart_driver.h"

#include "updater.h"
#include "flash_funcs.h"
#include <event.h>
#include "version.h"
#include "nio_serial.h"
#include "spi_settings.h"
#include "W25X40CL.h"

extern int32_t	Set_IRQHandler_spec(void);

#ifdef DEB_PRINT_FIRST_IRQS
extern uint8_t s1[];
extern uint8_t rcfifo[];
extern uint8_t sfifo[];
extern uint8_t ch[];
extern TIME_STRUCT tt[];

void print_first_irqs(void)
{
	uint8_t i;
	printf("-------\n");
	for(i = 0; i < 16; i++)
	{
		printf("%x %x %x %x %d:%d\n", s1[i], rcfifo[i], sfifo[i], ch[i], tt[i].SECONDS, tt[i].MILLISECONDS);
	}
	printf("-------\n");
}
#else
	void print_first_irqs(void) {}
#endif

extern _task_id   g_TASK_ids[];

uint32_t transmit_polling(const uint8_t* pbuf, uint32_t length);
//
#define UPDATE_START		    0xFEFFFFFF
#define READY_RUN			    0xFCFFFFFF
#define COUNT_OFFSET			0x08
#define RESET_OFFSET			0x10
#define READY_OFFSET			0x0C

const 	char*	dev_name = "nser3:";

const 	char*	OK_str   = "OK";
const	char*	ERR_str  = "ERR";
const	char*	AA_str   = "AA";
const	char*	BB_str   = "BB";
const	char*	nRDY_str = "nRDY";
const	char*	nCRC_str = "nCRC";
//
typedef enum
{
  REV,
  STA,
  STB,
  ERA,
  ERB,
  RAB,
  PFD,
//------- FPGA commands --------
  FRE,
  STF,
  SFD,
  UNN	= -1
} cmd_id;

const char*	cmds[] =
{
  "REV",
  "STA",
  "STB",
  "ERA",
  "ERB",
  "RAB",
  "PFD",
//------- FPGA commands --------
  "FRE",
  "STF",
  "SFD",
  ""
};
const char*	cmds_srec[] =
{
  "S1",
  "S2",
  "S3",
  "S7",
  "S8",
  "S9",
  ""
};
uint32_t 	g_offset = NVFLASH_BASE;
uint8_t 	bbb[264] = {0};//[64] = {0};
uint8_t 	tmp_buf[32] = {0};
void*		g_event_ptr;
uint32_t	client_queue[sizeof(LWMSGQ_STRUCT)/sizeof(uint32_t) + NUM_MESSAGES * UPD_MSG_SIZE];
_mqx_uint  	msgf[UPD_MSG_SIZE];
_task_id	Upd_idTask = MQX_NULL_TASK_ID;//temp place
uint32_t	g_start_flag = 0;
uint8_t 	g_ok = 0;
int32_t		g_fd = -1;

int32_t cs_get(uint8_t* arr, uint32_t len)
{
  uint32_t cs = 0, i;
  for(i = 0; i < len; ++i)
	cs += arr[i];
  return cs;
}

int32_t find_cmd(const char* names[], char* buf, int32_t len)
{
	int32_t i = 0, ind = -1;
	while(0 != strcmp(names[i], ""))
	{
		if(0 == strncmp(names[i], buf, len))
		{
			ind = i;
			break;
		}
		i++;
	}

	return ind;
}
char char2hex(char *num)
{
	uint8_t digh, digl;
	digh = ('0' <= num[0] && num[0] <= '9') ? num[0] - '0':
		   ('A' <= num[0] && num[0] <= 'F') ? num[0] - 'A' + 10:
		   ('a' <= num[0] && num[0] <= 'f') ? num[0] - 'a' + 10:-1;
	if (digh < 0)
	{
		printf("updater[%s]: high not digit %c%c\n\n", __func__, num[0], num[1]);
		return digh;
	}

	digl = ('0' <= num[1] && num[1] <= '9') ? num[1] - '0':
		   ('A' <= num[1] && num[1] <= 'F') ? num[1] - 'A' + 10:
		   ('a' <= num[1] && num[1] <= 'f') ? num[1] - 'a' + 10:-1;
	if (digl < 0)
	{
		printf("updater[%s]: low not digit %c%c\n\n", __func__, num[0], num[1]);
		return digl;
	}

	return (digh << 4) | digl;
}

uint32_t chars2uint(char* num)
{
	uint32_t i = 0, n = 0;
	for(i = 0; i < 4; ++i)
	{
		n |= (char2hex(&num[i*2]) << (8 * (3 - i))) ;
	}

	return n;
}
void ClearUart_Reply(uint8_t* buf, char* pReply)
{
//	ResetRxCounter(UART_UPDATE_FW_IDX);
	if(pReply)
		transmit_polling((const uint8_t*)pReply, strlen(pReply));
}
uint32_t chars2hex_arr(uint8_t* arr, char *nums, uint32_t len)
{
	uint32_t i = 0;
	for(i = 0; i < len; ++i)
		arr[i] = char2hex(&nums[i*2]);
	return 0;
}

void tasks_kill(void)
{
	uint32_t i = 0;

	while(i < NUM_TASKS)
	{
		if(	0					!= g_TASK_ids[i] &&
			UPDATER_TASK 		!= i &&
			UPDATER_EXEC_TASK 	!= i )
		{
			if(MQX_INVALID_TASK_ID == _task_destroy(g_TASK_ids[i]) )
			{
				//printf("updater: cannot destroy task %d\n", i);
			}
		}
		i++;
	}
}
/*
	KERNEL_DATA_STRUCT_PTR   kernel_data;
	_GET_KERNEL_DATA(kernel_data);
	TD_STRUCT_PTR           td_ptr;
	_mqx_uint               size;
	_task_id				my_idTask = _task_get_id();

	/// SPR P171-0022-01 Use int disable, not a semaphore
//    _INT_DISABLE();
	// END SPR

	td_ptr = (TD_STRUCT_PTR)((unsigned char *)kernel_data->TD_LIST.NEXT - FIELD_OFFSET(TD_STRUCT,TD_LIST_INFO));

	size   = _QUEUE_GET_SIZE(&kernel_data->TD_LIST);
	while (size--)
	{
		if (td_ptr->TASK_ID != Upd_idTask && td_ptr->TASK_ID != my_idTask)
		{
			_task_destroy(td_ptr->TASK_ID);
		}
		td_ptr = (TD_STRUCT_PTR)((unsigned char *)td_ptr->TD_LIST_INFO.NEXT - FIELD_OFFSET(TD_STRUCT,TD_LIST_INFO));
	}

	// SPR P171-0022-01 Use int disable, not a semaphore
//    _int_enable();
	// END SPR
*/

int32_t start_update(uint32_t id)
{
	tasks_kill();

	Upd_idTask = _task_create(0, UPDATER_EXEC_TASK, 0);
	if(MQX_NULL_TASK_ID == Upd_idTask)
	{
		printf("updater: _task_create failed\n");
		return -1;
	}
//////////////////////////////

	if(FROM_P_FLASH == id)
	{
		g_offset = PFLASH_BASE;
	}
	else
	{
		Flash_Init();
		g_offset = NVFLASH_BASE;
	}
	return 0;
}
int32_t start_phys(uint32_t id)
{
	int32_t ret, upd = UPDATE_START;

	ret = EraseSectors(id);

	if(0 == ret)
		ret = Flash_Program(g_offset + READY_OFFSET, 4, (uint8_t*)&upd	);
	if(0 == ret)
	{
		upd = g_offset + RESET_OFFSET;
		ret = Flash_Program(g_offset, 4, (uint8_t*)&upd	);//address first
	}

	return ret;
}
int32_t end_update(void)
{
	int32_t ret = -1, upd = READY_RUN;
	uint32_t* ptr = (uint32_t*)(g_offset + READY_OFFSET);//*((uint32_t*)BOOT_REGISTERS_START_ADDR);

	//check start???
	if((UPDATE_START & 0xFF000000) != (*ptr & 0xFF000000))
	{
		printf("updater: update end without start\n");
		return ret;
	}

	ptr = (PFLASH_BASE == g_offset) ? (uint32_t*)(NVFLASH_BASE + COUNT_OFFSET) : (uint32_t*)(PFLASH_BASE + COUNT_OFFSET);
	upd = *ptr - 1;

	ret = Flash_Program(g_offset + COUNT_OFFSET, 4, (uint8_t*)&upd	);//counter

	if(0 == ret)//must be last!!!
	{
		upd = READY_RUN;
		ret = Flash_Program(g_offset + READY_OFFSET, 4, (uint8_t*)&upd	);
	}

	return ret;
}
uint32_t choose_flash(void)
{
	void* pfunc = (void*)choose_flash;

	uint32_t ret = FROM_P_FLASH;

	if((uint32_t)pfunc & 0x10000000)
	  ret = FROM_FLEX_NVM;

	return ret;
}
void version_srec(char* out_buf)
{
	sprintf(out_buf, "%02X%02X%02X%02X", FW_VER_BTLD_OR_APP, FW_VER_MAJOR, FW_VER_MINOR, FW_VER_BUILD);
}

int32_t exec_cmd(cmd_id id, char* out_buf)
{
	int32_t ret = 0;
	uint32_t val = 0;

	strcpy(out_buf, OK_str);
	printf("updater:%s  id %d\n", __func__, id);

	switch(id)
	{
		case REV:
		{
			version_srec(out_buf);
			printf("updater: REV %s\n", out_buf);
		}
		break;
		case STA:
		{
			ret = start_update(FROM_P_FLASH);
			printf("updater: STA ret %d\n", ret);
		}
		break;
		case STB:
		{
			ret = start_update(FROM_FLEX_NVM);
			printf("updater: STB ret %d\n", ret);
		}
		break;
		case ERA:
		{
			ret = start_phys(FROM_P_FLASH);
			if(0 == ret)
			  g_start_flag = 1;
			printf("updater: ERA ret %d\n", ret);
		}
		break;
		case ERB:
		{
			ret = start_phys(FROM_FLEX_NVM);
			if(0 == ret)
			  g_start_flag = 1;
			printf("updater: ERB ret %d\n", ret);
		}
		break;
		case RAB:
		{
			val = choose_flash();
			if(FROM_FLEX_NVM == val)
			{
			  strcpy(out_buf, BB_str);
			}
			else //			if(FROM_P_FLASH == val)
			{
			  strcpy(out_buf, AA_str);
			}

			printf("updater: RAB ret %s\n", out_buf);
		}
		break;
		case PFD:
		{
			print_first_irqs();
			printf("updater: PFD\n");

			g_start_flag = 0;
			if(g_ok)
				end_update();
			NVIC_SystemReset();
		}
		break;
		case FRE:
		{
			fpga_get_version(out_buf);
			printf("updater: FRE %s\n", out_buf);
		}
		break;
//		case STF:
//		{
//		}
//		break;
		case SFD:
		{
			printf("updater: SFD\n");
			fpga_deinit();
			g_start_flag = 0;
		}
		break;
		default:
		{
			ret = -1;
		}
		break;
	}
	return ret;
}

uint32_t transmit_polling(const uint8_t* pbuf, uint32_t length)//return received
{
	uint32_t st = kStatus_UART_Success;
	UART_Type * base = g_uartBase[UART_UPDATE_FW_IDX];

	UART_HAL_SendDataPolling(base, (const uint8_t*)pbuf, length);
	//int32_t err;
	//int32_t st;

	//st = _nio_write(g_fd, pbuf, length, &err);
	return st;
}
int32_t	make_msg_cs_check(uint32_t* mm, uint8_t* buf, uint8_t* tmp_buf)
{
	uint32_t tmp_len = 2, len = 0, data_len = 0, cs;

	len = char2hex((char*)&buf[2]);
	//check cs
	chars2hex_arr(tmp_buf, (char*)&buf[2], len + 1);
	cs = (cs_get(tmp_buf, len) & 0x000000FF)^0xFF;
	if(cs != tmp_buf[len])
	{
		printf("updater: cs error %d - %d\n", cs, tmp_buf[len]);
		return -1;
	}

	tmp_len = 2;//length of length
	if('2' == buf[1] || '8' == buf[1])
	{
		tmp_len = 3;//address bytes
	}
	else if('3' == buf[1] || '7' == buf[1])
	{
		tmp_len = 4;//address bytes
	}
	data_len = len - tmp_len - 1;
	memcpy((uint8_t*)&mm[3], &tmp_buf[tmp_len + 1], data_len);//copy data

	memset(tmp_buf, '0', 4*4);
	memcpy(&tmp_buf[(4 - tmp_len)*2], &buf[4], tmp_len*2);//chars of address

	mm[0] = 0;//type
	mm[1] = chars2uint((char*)tmp_buf);//+ g_offset + RESET_OFFSET
	mm[2] = data_len;

	//workaround for the last length
	if(16 > data_len)
	{
		if(data_len % 4)
		{
			mm[2] += (4 - data_len % 4);
		}
	}
	return 1;
}
void	set_result(uint32_t res)
{
	uint8_t* ptr = 0;
	_event_clear(g_event_ptr, 1);//for flash operations
	if(FTFx_OK == res)
	  ptr = (uint8_t*)OK_str;
	else
	{
	  printf("update: Flash_Program return error 0x%x\n", res);
	  ptr = (uint8_t*)ERR_str;
	}
	transmit_polling(ptr, strlen((char*)ptr));
}
uint32_t rstr_32_dig(uint8_t* buf)
{
	uint32_t  dig = 0;
	int32_t i;
	char b[2] = {0};
	for(i = 0; i < 8; i++)
	{
		b[0] = buf[i * 2 + 1];
		dig |= (strtol(b, 0, 16) & 0xFF) << (28 - i * 4);
	}
	return dig;
}
void fpga_data(uint8_t* buf, uint32_t len)
{
	int32_t		err = 0;//, count = 0;
	uint32_t 	tmp_len = 0, wr_len = 0, diff, crc;
	uint8_t* 	ptr = (uint8_t*)OK_str;
	uint32_t*	pTmp;
//	TIME_STRUCT tt;

	g_start_flag = 2;
	tasks_kill();

	len = rstr_32_dig(buf);
//	len = 135180;//temp!!!

	if(0 != fpga_init())
	{
		ptr = (uint8_t*)ERR_str;
		transmit_polling(ptr, strlen((char*)ptr));
		return;
	}
	transmit_polling((uint8_t*)OK_str, strlen((char*)OK_str));

	do
	{
		err = 0;
		ptr = (uint8_t*)OK_str;
		diff = (SPI_FLASH_SECTOR_SIZE < len - wr_len) ? SPI_FLASH_SECTOR_SIZE : len - wr_len;

		tmp_len = _nio_read(g_fd, &buf[4], diff + 4, &err);

		if(tmp_len < diff)
		{
			ptr = (uint8_t*)nCRC_str;// or ERR ???;
			err = 1;
		}

		if(0 == err)
		{
			pTmp = (uint32_t*)(&buf[diff + 4]);

			tmp_len = *pTmp;
			crc = crc_32(&buf[4], diff);
			//if(crc_32(buf, diff) != tmp_len)
			if(crc != tmp_len)
			{
				printf("updater: fpga crc error %x (%x)\n", crc, tmp_len);
				ptr = (uint8_t*)nCRC_str;
				err = 2;
			}
		}
		if(0 == err)
		{
			uint32_t deb = wr_len % ERASABLE_BLOCK_SIZE;
			if(0 == (wr_len % ERASABLE_BLOCK_SIZE))//sector offset
			{
//				_time_get(&tt);
//				printf("erase %d:%d\n", tt.SECONDS, tt.MILLISECONDS);
//			  	printf("erase 0x%x (%d)\n", wr_len, deb);
				if(0 != fpga_erase_sector(wr_len))
				{
					ptr = (uint8_t*)ERR_str;
					err = 4;
				}
//				_time_get(&tt);
//				printf("erase %d:%d\n", tt.SECONDS, tt.MILLISECONDS);
			}
		}
		if(0 == err)
		{
//			_time_get(&tt);
//			printf("fpga_write_data %d:%d\n", tt.SECONDS, tt.MILLISECONDS);
//		  	printf("fpga_write_data 0x%x + 0x%x (count %d)\n", wr_len, diff, ++count);
			if(0 != fpga_write_data(wr_len, buf, diff))
			{
				ptr = (uint8_t*)ERR_str;
				err = 5;
			}
//			_time_get(&tt);
//			printf("fpga_write_data %d:%d\n", tt.SECONDS, tt.MILLISECONDS);
		}

		transmit_polling(ptr, strlen((char*)ptr));

		if(0 == err)
			wr_len += diff;

	}while((len > wr_len) && (3 > err));
	buf[0] = 0;
}
void updater_task(uint32_t param)
{
/*
	const uart_user_config_t uart_config = {
		.bitCountPerChar = kUart9BitsPerChar, //kUart8BitsPerChar,
		.parityMode      = kUartParityOdd, //kUartParityDisabled,
		.stopBitCount    = kUartOneStopBit,
		.baudRate        = BAUD_115200 //38400
	};
*/
	const NIO_SERIAL_INIT_DATA_STRUCT nserial3_init =
	{
		.SERIAL_INSTANCE		= UART_UPDATE_FW_IDX,
		.BAUDRATE            	= 115228,
		.PARITY_MODE         	= kNioSerialParityOdd,
		.STOPBIT_COUNT       	= kNioSerialOneStopBit,
		.BITCOUNT_PERCHAR    	= 9,
		.MODULE					= kNioSerialUart,
		.RXTX_PRIOR		       	= 3,
	#if defined(BOARD_USE_UART) && defined(BOARD_UART_CLOCK_SOURCE)
		.CLK_SOURCE          = BOARD_UART_CLOCK_SOURCE,
	#else
		.CLK_SOURCE          = 1,
	#endif
		.RX_BUFF_SIZE        	= NIO_SERIAL_BUFF_SIZE,
		.TX_BUFF_SIZE        	= NIO_SERIAL_BUFF_SIZE,
	};
//	UART_Type* 	base = g_uartBase[UART_UPDATE_FW_IDX];
	_mqx_uint 	result;
	int32_t 	stat = -1;
	uint32_t 	len = 0;
	uint32_t 	tmp_len = 0;
	cmd_id 		id = UNN;
	int32_t 	error;
	NIO_DEV_STRUCT*	pNio = 0;

	printf("updater: task started\n");

	update_fw_uart_init();//muxing

	pNio = _nio_dev_install(dev_name, &nio_serial_dev_fn, (void*)&nserial3_init, NULL);
	if(0 == pNio)
	{
		printf("updater: _nio_dev_install failed\n");
		_mqx_exit(0);
	}
	Set_IRQHandler_spec();

	g_fd = _nio_open(dev_name, O_RDWR, 0);
	if(0 > g_fd)
	{
		printf("updater: cannot open nio dev\n");
		_mqx_exit(0);
	}

	result = _lwmsgq_init((void *)client_queue, NUM_MESSAGES, UPD_MSG_SIZE);
	if (result != MQX_OK)
	{
		printf("updater: lwmsgq_init client_queue failed\n");
		_mqx_exit(0);
	}
 //////////
	if (_event_create("globalll") != MQX_OK) {
		printf("updater: Make event failed\n");
		_mqx_exit(0);
		}
	if (_event_open("globalll", &g_event_ptr) != MQX_OK) {
		printf("updater: Open event failed\n");
	}
//////////////////
	while(true)
	{
		id = UNN;

		memset(bbb, 0, sizeof(bbb));
		// Wait to receive input data
		stat = _nio_read(g_fd, bbb, 3, &error);
		if(0 < stat)
		{
			tmp_len = 3;

			if(-1 != (id = (cmd_id)find_cmd(cmds_srec, (char*)bbb, 2)))
			{
				///S-records part started
				tmp_len += 1;
				stat = _nio_read(g_fd, &bbb[3], 1, &error);
				if(0 > stat)//== 1
				{
					printf("updater: error rcv len %d\n", tmp_len);
					ClearUart_Reply(bbb, (char*)nRDY_str);
					continue;
				}
				len = char2hex((char*)&bbb[2]);

				tmp_len += len * 2;
				stat = _nio_read(g_fd, &bbb[4], len * 2, &error);

				if(0 > stat)//== len * 2
				{
					printf("updater: error rcv len %d\n", len * 2);
					ClearUart_Reply(bbb, (char*)nRDY_str);
					continue;
				}

				if(0 > make_msg_cs_check(msgf, bbb, tmp_buf))
				{
					ClearUart_Reply(bbb, (char*)nRDY_str);
					continue;
				}
				if('7' <= bbb[1] && '9' >= bbb[1])
				{
					g_ok = 1;
					printf("ok %s\n", bbb);
					stat = transmit_polling((uint8_t*)OK_str, strlen(OK_str));
					continue;
				}
				_event_get_value(g_event_ptr, &tmp_len);
				if(1 == tmp_len)
				{
					stat = transmit_polling((uint8_t*)nRDY_str, strlen(nRDY_str));
					continue;
				}
				_event_set(g_event_ptr, 1);
				_lwmsgq_send((void *)client_queue, msgf, LWMSGQ_SEND_BLOCK_ON_FULL);
			}
			else //commands
			{
				uint8_t* ptr = 0;
				int32_t res = -1;
				int32_t len = 0;

				id = (cmd_id)find_cmd(cmds, (char*)bbb, 3);

				res = -1;

				if(UNN != id)
				{
						if(STF == id)
						{
							len = _nio_read(g_fd, bbb, 17, &error);
							printf("updater: read len %d (%d)\n", len, error);
							fpga_data(bbb, len);
							continue;
						}
					res = exec_cmd((cmd_id)id, (char*)tmp_buf);
					if(0 == res)
					{
						ptr = tmp_buf;
						len = strlen((char*)ptr);
						if(REV == id)
							len += 1;//for NULL
						if(SFD != id)
							stat = transmit_polling(ptr, len);
					}
					else
					{
						print_first_irqs();
						ClearUart_Reply(bbb, (char*)ERR_str);
					}
				}
				else//error
				{
					print_first_irqs();
					ClearUart_Reply(bbb, (g_start_flag ? ((char*)nRDY_str) : ((char*)ERR_str)));
				}
			}
		}
		else
		{
			printf("updater: r1 %d (0x%x)\n", stat, bbb[0]);
		}
	}

	return;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
