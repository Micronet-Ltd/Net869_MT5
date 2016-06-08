#include "tasks_list.h"


TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
//  Task number,		Entry point,			Stack,     Pri,	    					Task name       	,	    Auto start        Creation Param      Time Slice
	{ MAIN_TASK,			Main_task,			1024,       MAIN_TASK_PRIORITY,			"MAIN_TASK",			MQX_AUTO_START_TASK,		0,            	0 },
	{ ACC_TASK,				Acc_task,			1024,      	ACC_TASK_PRIORITY,			"ACC_TASK",				0,        					0,              0 },
	{ USB_TASK,				Usb_task,			1024,      	USB_TASK_PRIORITY,			"USB_TASK",				0,					        0,              0 },
	{ CAN_TASK_RX_0,		FLEXCAN_Rx_Task,	1024,   	CAN_TASK_RX_PRIORITY,		"CAN_TASK_RX0",			0,							0,              0 },
	{ CAN_TASK_RX_1,		FLEXCAN_Rx_Task,	1024,   	CAN_TASK_RX_PRIORITY,		"CAN_TASK_RX1",			0,							0,              0 },
	{ CAN_TASK_TX_0,		FLEXCAN_Tx_Task,	1024,   	CAN_TASK_TX_PRIORITY,		"CAN_TASK_TX0",			0,							0,              0 },
	{ CAN_TASK_TX_1,		FLEXCAN_Tx_Task,	1024,   	CAN_TASK_TX_PRIORITY,		"CAN_TASK_TX1",			0,							0,              0 },
	{ J1708_RX_TASK,		J1708_Rx_task,		1024,		J1708_RX_TASK_PRIORITY,		"J1708_RX_TASK",		0,							0,				0 },
	{ J1708_TX_TASK,		J1708_Tx_task,		1024,		J1708_TX_TASK_PRIORITY,		"J1708_TX_TASK",		0,							0,				0 },
	{ FPGA_UART_RX_TASK,	FPGA_UART_Rx_task,	1024,		FPGA_UART_RX_TASK_PRIORITY,	"FPGA_UART_RX_TASK",	0,							0,				0 },
	{ POWER_MGM_TASK,		Power_MGM_task,		1024,		POWER_MGM_TASK_PRIORITY,	"POWER_MGM_TASK",		0,							0,				0 },
	{ CONTROL_TASK,			control_task,		1024,		CONTROL_TASK_PRIORITY,		"CONTROL_TASK",			0,							0,				0 },
	{ 0	}
};
