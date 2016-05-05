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
//#include <mqx_prv.h>

extern void UART_Enable   (uint8_t port, const uart_user_config_t *uartConfig);
extern _task_id   g_TASK_ids[];
//
#define UPDATE_START		    0xFEFFFFFF
#define READY_RUN			    0xFCFFFFFF
#define COUNT_OFFSET			0x08
#define RESET_OFFSET			0x10
#define READY_OFFSET			0x0C

const 	char*	OK_str   = "OK";
const	char*	ERR_str  = "ERR";
const	char*	AA_str   = "AA";
const	char*	BB_str   = "BB";
const	char*	nRDY_str = "nRDY";
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
uint32_t 	g_offset = FLEXNVM_BASE;//temp!!! place
uint32_t 	g_errors = 0;//temp!!! place
uint8_t 	bbb[516] = {0};
uint8_t 	tmp_buf[255] = {0};
void*		g_event_ptr;
uint32_t	client_queue[sizeof(LWMSGQ_STRUCT)/sizeof(uint32_t) + NUM_MESSAGES * UPD_MSG_SIZE];
_mqx_uint  	msgf[UPD_MSG_SIZE];
_task_id	Upd_idTask = MQX_NULL_TASK_ID;//temp place
uint32_t	g_start_flag = 0;

int32_t cs_get(uint8_t* arr, uint32_t len)
{
  uint32_t cs = 0, i;
  for(i = 0; i < len; ++i)
	cs += arr[i];
  return cs;
}
void uint_to_hex_string( uint32_t num, char  *string, uint32_t bytes_qty  )
{
	const char str_hex_array[] = "0123456789ABCDEF";
  	uint32_t i;
	
	for(i = 0; i < bytes_qty * 8; i++)
	{
	   string[i]  = str_hex_array[(num >> (bytes_qty * 8 - (i + 1)*4 ))&0xF];
	}
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
  	g_errors++;
	UART_DRV_ReceiveData(UART_UPDATE_FW_IDX, buf, 1000);//workaround: set busy for abort
	UART_DRV_AbortReceivingData(UART_UPDATE_FW_IDX);
	if(pReply)
		UART_DRV_SendDataBlocking(UART_UPDATE_FW_IDX, (uint8_t*)pReply, strlen(pReply), 1000u);
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
			  	printf("updater: cannot destroy task %d\n", i);
			}
		}
		i++;
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
}

int32_t start_update(uint32_t id)
{	
	tasks_kill();
	
/* 	KERNEL_DATA_STRUCT_PTR   kernel_data;
    _GET_KERNEL_DATA(kernel_data);

      TASK_TEMPLATE_STRUCT_PTR     task_template_ptr = kernel_data->INIT.TASK_TEMPLATE_LIST;

            while(task_template_ptr->TASK_TEMPLATE_INDEX)
			{
			  	if(	task_template_ptr->TASK_TEMPLATE_INDEX != MAIN_TASK && 
				   	task_template_ptr->TASK_TEMPLATE_INDEX != POWER_MGM_TASK &&
					task_template_ptr->TASK_TEMPLATE_INDEX != UPDATER_TASK && 
				   	task_template_ptr->TASK_TEMPLATE_INDEX != UPDATER_EXEC_TASK )
				{
	            	_task_destroy(task_template_ptr->TASK_TEMPLATE_INDEX);
				  	
				}
                ++task_template_ptr;
            }
*/
/*               TASK_TEMPLATE_STRUCT    task_template = {0};
               task_template.TASK_NAME          = "Clock_child";
               task_template.TASK_PRIORITY      = SHELL_CLOCK_CHILD_PRIO;
               task_template.TASK_STACKSIZE     = SHELL_CLOCK_CHILD_STACK;
               task_template.TASK_ADDRESS       = Clock_child_task;
               task_template.CREATION_PARAMETER = (uint32_t) &server_cxt;
               if ((Upd_idTask = _task_create(0, 0, (uint32_t)&task_template)) == MQX_NULL_TASK_ID) 
			   {
                  printf("updater: failed to spawn child task\n");
               } 
*/
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
	  	g_offset = FLEXNVM_BASE;
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

	ptr = (uint32_t*)(g_offset + COUNT_OFFSET);
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
	uint32_t val = *((uint32_t*)(FLEXNVM_BASE + READY_OFFSET));
  	
	uint32_t ret = FROM_P_FLASH;

	if((uint32_t)pfunc & 0x10000000)
	  ret = FROM_FLEX_NVM;
	
	return ret;
}
void version_srec(char* out_buf)
{
//	char tmp_str[4] = {0};
//	uint32_t i; 

	memcpy(out_buf, "4E455438", 8);//temp!!!
	out_buf[8] = 0;
//	memset(out_buf, '0', 10); 
//4E455438
//	tmp_str[0] = ;//FW_VER_BTLD_OR_APP;
//	tmp_str[1] = 2;//FW_VER_MAJOR;
//	tmp_str[2] = 3;//FW_VER_MINOR;
//	tmp_str[3] = 4;//FW_VER_BUILD;
	//ver = (tmp_str[0] << 3*4) + (tmp_str[1] << 2*4) + (tmp_str[2] << 1*4) + tmp_str[3];  
	//uint_to_hex_string(ver, &out_buf[8+i], 4);
//	for(i = 0; i < 4; ++i)
//	{
//		uint_to_hex_string(tmp_str[i], &out_buf[8+i], 1);
//	}
		  
//	out_buf[8] = 0;
}

int32_t exec_cmd(cmd_id id, char* out_buf)
{
	int32_t ret = 0;
	uint32_t val = 0;

	strcpy(out_buf, OK_str);

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
			printf("updater: PFD\n");			
			g_start_flag = 0;
			end_update();
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

uint32_t transmit_with_timeout(const uint8_t* pbuf, uint32_t length)//return received
{
  	uint32_t st = kStatus_UART_Success;
	uint32_t count = 60000; //33err - 50000 * length;
	do
	{
        if(kStatus_UART_TxBusy != (st = UART_DRV_GetTransmitStatus(UART_UPDATE_FW_IDX, NULL)))
		{
			break;
		}
	}while(count--);
	if(kStatus_UART_Success != st)
	{
	  	printf("updater: transmit failed, stat 0x%x\n", st);
	  	return st;
	} 
	st = UART_DRV_SendDataBlocking(UART_UPDATE_FW_IDX, (const uint8_t*)pbuf, length, 500 * length);
//	st = UART_DRV_SendData(UART_UPDATE_FW_IDX, (const uint8_t*)pbuf, length);

//	while(kStatus_UART_TxBusy == (st = UART_DRV_GetTransmitStatus(UART_UPDATE_FW_IDX, NULL)))///temp!!! debug
//	{
//	  	printf("updater: transmit failed, stat 0x%x\n", st);
//	}
	
	return st;
}
//buffer MUST be 0
uint32_t recieve_with_timeout(uint8_t* pbuf, uint32_t length)//return received
{
  	uint32_t r_length = 0;
	uint32_t count = 60000 * length; //33err - 60000 * length;
  	uint64_t to = length * 2000;//temp!!!
	
//  	TIME_STRUCT	start_tick, end_tick;
//	bool overflow;
	
	UART_DRV_ReceiveData(UART_UPDATE_FW_IDX, pbuf, length);

//	_time_get(&start_tick);
	
	do
	{
        if(kStatus_UART_RxBusy != UART_DRV_GetReceiveStatus(UART_UPDATE_FW_IDX, NULL))
		{
			break;
		}

//		_time_get(&end_tick);
	   	//if(to > _time_diff_milliseconds(&end_tick, &start_tick, &overflow))
//		if(to > (end_tick.SECONDS * 1000 + end_tick.MILLISECONDS - start_tick.SECONDS * 1000 - start_tick.MILLISECONDS))
//		{
//		  	printf("timeout\n");
//		 	break;
//		}
	}
    while(count--);//(to > _time_diff_milliseconds(&end_tick, &start_tick, &overflow));
	  
	r_length = strlen((char*)pbuf);  
	if(r_length < length)
	{
	  	printf("updater[%s]: timeout\n", __func__);
	}
	return r_length;
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
	transmit_with_timeout(ptr, strlen((char*)ptr));
}
void updater_task(uint32_t param)
{
	const uart_user_config_t uart_config = {
		.bitCountPerChar = kUart8BitsPerChar,
		.parityMode      = kUartParityDisabled,
		.stopBitCount    = kUartOneStopBit,
		.baudRate        = BAUD_115200
	};
	
  	_mqx_uint  	result;
  	//uart_status_t stat = kStatus_UART_Success;
	uint32_t stat = kStatus_UART_Success;
	uint32_t len = 0;
//	uint32_t abort_res_flag = 0;
  	uint32_t tmp_len = 0;
	cmd_id id = UNN;
	
	
/*  	Upd_idTask = _task_create(0, UPDATER_EXEC_TASK, 0);//temp!!! place
	if(MQX_NULL_TASK_ID == Upd_idTask)
	{
      	printf("updater: _task_create failed\n");
		_mqx_exit(0);
	}
*/
	update_fw_uart_init();//muxing
	UART_Enable(UART_UPDATE_FW_IDX, &uart_config);
    
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
		stat = UART_DRV_ReceiveDataBlocking(UART_UPDATE_FW_IDX, bbb, 1u, 2000u);//2 sec
		if(kStatus_UART_Success == stat && 0 != bbb[0])		  
		{
			tmp_len = recieve_with_timeout(&bbb[1], 2);
//			if (kStatus_UART_Success == UART_DRV_ReceiveDataBlocking(UART_UPDATE_FW_IDX, &bbb[1], 2u, 2000u))//2 sec
			{
				if(2 != tmp_len)//3 != strlen((char*)bbb))
				{
					printf("updater: error rcv len %d\n", tmp_len);
					ClearUart_Reply(bbb, (g_start_flag ? ((char*)nRDY_str) : ((char*)ERR_str)));
					continue;
				}
				 //UART_DRV_SendDataBlocking(UART_UPDATE_FW_IDX, &rxChar, 1u, 1000u);
				if(-1 != (id = (cmd_id)find_cmd(cmds_srec, (char*)bbb, 2)))
				{
				  	//maybe check g_start_flag???
				  
					///S-records part started				  
					tmp_len = recieve_with_timeout(&bbb[3], 1);
					if(1 != tmp_len)
					{
						printf("updater: error rcv len %d\n", tmp_len);
						ClearUart_Reply(bbb, (char*)nRDY_str);
						continue;
					}
					
					len = char2hex((char*)&bbb[2]);

					tmp_len = recieve_with_timeout(&bbb[4], len * 2);
					if(tmp_len < len * 2)
					{
						ClearUart_Reply(bbb, (char*)nRDY_str);
						continue;
					}
					
					if(0 > make_msg_cs_check(msgf, bbb, tmp_buf))
					{
						ClearUart_Reply(bbb, (char*)nRDY_str);
						continue;
					}

					//stat = Flash_Program(msgf[1], msgf[2], (uint8_t*)&msgf[3]);///debug - 1 task
					//if(FTFx_OK != stat)
					//{
					//	printf("Flash_Program failed 0x%x\n", stat);
					//	stat = transmit_with_timeout((uint8_t*)ERR_str, strlen(ERR_str));
					//	//UART_DRV_SendDataBlocking(UART_UPDATE_FW_IDX, (uint8_t*)ERR_str, strlen(ERR_str), 1000u);
							//error exit
					//}
					_event_get_value(g_event_ptr, &tmp_len);
					if(1 == tmp_len)
					{
					  stat = transmit_with_timeout((uint8_t*)nRDY_str, strlen(nRDY_str));
					  continue;
					}
					_event_set(g_event_ptr, 1);
					_lwmsgq_send((void *)client_queue, msgf, LWMSGQ_SEND_BLOCK_ON_FULL);
				}
				else
				{
				  	uint8_t* ptr = 0;
				  	int32_t res = -1;

					id = (cmd_id)find_cmd(cmds, (char*)bbb, 3);
					
					res = -1;

					if(UNN != id)
					{
					  	res = exec_cmd((int32_t)id, (char*)bbb);
					  	if(0 > res)
						{
							ptr = (uint8_t*)ERR_str;
						}
						else
						  ptr = bbb;
					}
					else//error
					{
					  	printf("updater: cmd is not found (%s)\n", bbb);
					  	//if(error > max or burn is not started -(char*)ERR_str
						ClearUart_Reply(bbb, (g_start_flag ? ((char*)nRDY_str) : ((char*)ERR_str)));
						continue;
					}

					stat = transmit_with_timeout(ptr, strlen((char*)ptr));
				}
			}
		}
		else if(kStatus_UART_RxBusy == stat)
		{
		  	ClearUart_Reply(bbb, 0);//try normalize - sent nothing
		}
    }
	
	return;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
