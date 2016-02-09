#ifndef __tasks_list_h_
#define __tasks_list_h_

#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <stdio.h>

#define MAX_MSG_DATA_LEN	 0x40
#define NUM_CLIENTS			 40

#define START_APPLICATION_PRIORITY (16)

typedef enum {
	//Regular priority tasks
	
	USB_TASK_PRIORITY = (START_APPLICATION_PRIORITY + 1),
	POWER_MGM_TASK_PRIORITY,
	CAN_TASK_RX_PRIORITY,
	CAN_TASK_TX_PRIORITY,
	J1708_TX_TASK_PRIORITY,
	J1708_RX_TASK_PRIORITY,
	FPGA_UART_RX_TASK_PRIORITY,
	ACC_TASK_PRIORITY,
	VIB_SENSOR_TASK_PRIORITY  ,
	REG_TASK_PRIORITY,
	MAIN_TASK_PRIORITY = (REG_TASK_PRIORITY + 16),
}PRIORITY_TASK_INDEX_T;

typedef enum {
	MAIN_TASK   =   5,
	USB_TASK         ,
	POWER_MGM_TASK	 ,
	CAN_TASK_RX_0    ,
	CAN_TASK_TX_0    ,
	CAN_TASK_RX_1    ,
	CAN_TASK_TX_1    ,
	J1708_TX_TASK    ,
	J1708_RX_TASK    ,
	FPGA_UART_RX_TASK,
	ACC_TASK         ,
	REG_TASK		 ,
	NUM_TASKS		 ,
} TASK_TEMPLATE_INDEX_T;

typedef enum {
	MAIN_QUEUE       ,
	USB_QUEUE        ,
	CAN1_QUEUE       ,
	CAN2_QUEUE       ,
	J1708_TX_QUEUE   ,
	J1708_RX_QUEUE   ,
	FPGA_UART_RX_QUEUE,
	ACC_QUEUE        ,
	POWER_MGM_QUEUE  ,
	REG_QUEUE   	 ,
	USB_TEST_QUEUE	,
} APPLICATION_QUEUE_T;

typedef struct {
	MESSAGE_HEADER_STRUCT 		header;
	uint64_t					timestamp; /* in milliseconds */
	uint8_t						data[MAX_MSG_DATA_LEN];
	uint8_t						portNum;
} APPLICATION_MESSAGE_T, *APPLICATION_MESSAGE_PTR_T;

extern void Main_task        (uint32_t);
extern void Power_MGM_task   (uint32_t);

extern void Acc_task         (uint32_t);
extern void Usb_task         (uint32_t);

extern void J1708_Tx_task     (uint32_t);
extern void J1708_Rx_task     (uint32_t);
extern void FPGA_UART_Rx_task (uint32_t );

extern void FLEXCAN_Tx_Task( uint32_t param );
extern void FLEXCAN_Rx_Task( uint32_t param ); 

#endif /* __tasks_list_h_ */

