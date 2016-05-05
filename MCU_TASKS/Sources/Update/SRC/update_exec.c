#include "updater.h"
#include "flash_funcs.h"


void upd_exec_task(uint32_t index)
{
   	_mqx_uint   			msg[UPD_MSG_SIZE];
	uint32_t	res = FTFx_OK; 
   	while (TRUE) 
   	{
     	/* wait for a return message */
    	if(MQX_OK == _lwmsgq_receive((void *)client_queue, msg, LWMSGQ_RECEIVE_BLOCK_ON_EMPTY, 0, 0))
		{
		  	//maybe msg[0] - msg type
		  	res = Flash_Program(msg[1], msg[2], (uint8_t*)&msg[3]);
			set_result(res);
		}
   	}
}
