#include "tasks_list.h"


TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
//  Task number,		Entry point,			Stack,     Pri,	    Task name       	,	    Auto start        Creation Param      Time Slice
	{ MAIN_TASK,			Main_task,			1500,       8,		"MAIN_TASK",			MQX_AUTO_START_TASK,		0,            	0 },
	{ ACC_TASK,				Acc_task,			4000,      	9,		"ACC_TASK",				0,        					0,              0 },
	{ USB_TASK,				Usb_task,			1500,      	4,		"USB_TASK",				0,					        0,              0 },
	{ CAN_TASK_RX_0,		FLEXCAN_Rx_Task,	4000,   	10,		"CAN_TASK_RX",			0,							0,              0 },
	{ J1708_RX_TASK,		J1708_Rx_task,		1500,		11,		"J1708_RX_TASK",		0,							0,				0 },
	{ J1708_TX_TASK,		J1708_Tx_task,		1500,		11,		"J1708_TX_TASK",		0,							0,				0 },
	{ FPGA_UART_RX_TASK,	FPGA_UART_Rx_task,	1500,		12,		"FPGA_UART_RX_TASK",	0,							0,				0 },
	{ POWER_MGM_TASK,		Power_MGM_task,		1500,		5,		"POWER_MGM_TASK",		0,							0,				0 },

	{ 0	}
};
