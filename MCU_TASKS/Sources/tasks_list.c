#include "tasks_list.h"


TASK_TEMPLATE_STRUCT MQX_template_list[] = 
{ 
//  Task number,	Entry point,	Stack, Pri,		String, 		    Auto start ? 
   {MAIN_TASK         ,	Main_task          ,	1500,	8 ,		"MAIN_TASK"         ,	MQX_AUTO_START_TASK},
   {J1708_RX_TASK     ,	J1708_Rx_task      ,	1500,	9 ,		"J1708_RX_TASK"     ,	0                  },
   {J1708_TX_TASK     ,	J1708_Tx_task      ,	1500,	9 ,		"J1708_TX_TASK"     ,	0                  },
   {FPGA_UART_RX_TASK,	FPGA_UART_Rx_task  ,	1500,	9 ,		"FPGA_UART_RX_TASK",	0                  },

/*
   {USB_TASK   ,	Usb_task   ,	1500,	4 ,		"USB_TASK"     ,	0                  },
   {CAN_TASK   ,	Can_task   ,	1500,	9 ,		"CAN_TASK"  ,	0                  },
   {ACC_TASK   ,	Acc_task   ,	1500,	9 ,		"ACC_TASK"  ,	0                  },
*/
   {0          ,	0          ,	0   ,	0 ,		0           ,	0                  }
};
