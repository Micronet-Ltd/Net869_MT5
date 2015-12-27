#ifndef __tasks_list_h_
#define __tasks_list_h_

#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <stdio.h>

#define MAX_MSG_DATA_LEN	 0xFF
#define NUM_CLIENTS			 10

typedef enum {
	//Regular priority tasks
	MAIN_TASK_PRIOR = 5,
	USB_TASK_PRIOR = 8

}PRIORITY_TASK_INDEX_T;

typedef enum {
	MAIN_TASK   =   5,
	USB_TASK         ,
	CAN_TASK_RX_0    ,
	CAN_TASK_TX_0    ,
	CAN_TASK_RX_1    ,
	CAN_TASK_TX_1    ,
	J1708_TX_TASK    ,
	J1708_RX_TASK    ,
	FPGA_UART_RX_TASK,
	ACC_TASK         ,
	VIB_SENSOR_TASK  ,
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
	VIB_SENSOR_QUEUE ,
	REG_QUEUE   	 ,
	POWER_MGMT_QUEUE ,
} APPLICATION_QUEUE_T;

typedef struct {
	MESSAGE_HEADER_STRUCT 		header;
	uint8_t						data[MAX_MSG_DATA_LEN];
	TIME_STRUCT					timestamp;
} APPLICATION_MESSAGE_T, *APPLICATION_MESSAGE_PTR_T;

extern void Main_task        (uint32_t);

extern void Acc_task         (uint32_t);
extern void Usb_task         (uint32_t);

extern void J1708_Tx_task     (uint32_t);
extern void J1708_Rx_task     (uint32_t);
extern void FPGA_UART_Rx_task (uint32_t );

extern void FLEXCAN_Tx_Task( uint32_t param );
extern void FLEXCAN_Rx_Task( uint32_t param ); 

/*
extern void Can_task         (uint32_t);
extern void J1708_task       (uint32_t);
extern void Acc_task         (uint32_t);
extern void Vib_sensor_task  (uint32_t);
extern void Reg_task         (uint32_t);
*/

extern _pool_id message_pool;

#endif /* __tasks_list_h_ */
