
#ifndef __tasks_list_h_
#define __tasks_list_h_

#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <stdio.h>

#define MAX_MSG_DATA_LEN	 0x40
#define NUM_CLIENTS			 (16)


//#define USB_CAN_MAX_USABLE      (NUM_CLIENTS-10)

#define START_APPLICATION_PRIORITY (16)

typedef enum {
	//Regular priority tasks
	USB_TASK_PRIORITY = (START_APPLICATION_PRIORITY + 1),
	POWER_MGM_TASK_PRIORITY,
	CAN_TASK_RX_PRIORITY,
	UPDATER_EXEC_TASK_PRIORITY,
	CAN_TASK_TX_PRIORITY,
	CONTROL_TASK_PRIORITY,
	J1708_TX_TASK_PRIORITY,
	J1708_RX_TASK_PRIORITY,
	FPGA_UART_RX_TASK_PRIORITY,
	ACC_TASK_PRIORITY,
	ONE_WIRE_TASK_PRIORITY,
//	VIB_SENSOR_TASK_PRIORITY  ,
	UPDATER_TASK_PRIORITY,
	MAIN_TASK_PRIORITY = (UPDATER_TASK_PRIORITY + START_APPLICATION_PRIORITY),
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
	ONE_WIRE_TASK    ,
	CONTROL_TASK	 ,
	UPDATER_TASK	 ,
	UPDATER_EXEC_TASK	 ,
	NUM_TASKS		 ,
} TASK_TEMPLATE_INDEX_T;

typedef enum {
	MAIN_QUEUE       ,
	USB_QUEUE        ,
	CAN1_TX_QUEUE    ,
	CAN2_TX_QUEUE    ,
	CAN1_RX_QUEUE	 ,
	CAN2_RX_QUEUE	 ,
	J1708_TX_QUEUE   ,
	J1708_RX_QUEUE   ,
	FPGA_UART_RX_QUEUE,
	ACC_QUEUE        ,
	ONE_WIRE_QUEUE   ,
	POWER_MGM_QUEUE  ,
	CONTROL_RX_QUEUE ,
	CONTROL_TX_QUEUE ,
	USB_TEST_QUEUE	,
} APPLICATION_QUEUE_T;

typedef struct {
	MESSAGE_HEADER_STRUCT 		header;
	uint64_t					timestamp;
	uint8_t						portNum;
	uint8_t						data[MAX_MSG_DATA_LEN];
} APPLICATION_MESSAGE_T, *APPLICATION_MESSAGE_PTR_T;

#define APP_MESSAGE_NO_ARRAY_SIZE (sizeof(MESSAGE_HEADER_STRUCT)+sizeof(uint64_t)+sizeof(uint8_t))

extern uint8_t g_flag_Exit; 

extern void Main_task        (uint32_t);
extern void Power_MGM_task   (uint32_t);

extern void Acc_task         (uint32_t);
extern void Usb_task         (uint32_t);

extern void J1708_Tx_task     (uint32_t);
extern void J1708_Rx_task     (uint32_t);
extern void FPGA_UART_Rx_task (uint32_t );

extern void control_task (uint32_t );

extern void FLEXCAN_Tx_Task( uint32_t param );
extern void FLEXCAN_Rx_Task( uint32_t param );

extern void updater_task (uint32_t );
extern void upd_exec_task (uint32_t );

extern void one_wire_task (uint32_t);

#endif /* __tasks_list_h_ */
