#ifndef __tasks_list_h_
#define __tasks_list_h_

#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <stdio.h>

#define MAX_MSG_DATA_LEN	256
#define NUM_CLIENTS			 10

typedef enum {
	MAIN_TASK   =   5,
	USB_TASK         ,
	CAN_TASK         ,
	J1708_TASK       ,
	ACC_TASK         ,
	VIB_SENSOR_TASK  ,
	REG_TASK        
} TASK_TEMPLATE_INDEX_T;

typedef enum {
	MAIN_QUEUE       ,
	USB_QUEUE        ,
	CAN1_QUEUE       ,
	CAN2_QUEUE       ,
	J1708_QUEUE      ,
	ACC_QUEUE        ,
	VIB_SENSOR_QUEUE ,
	REG_QUEUE   
} APPLICATION_QUEUE_T;

typedef struct {
	MESSAGE_HEADER_STRUCT 		HEADER;
	uint8_t						DATA[MAX_MSG_DATA_LEN];
} APPLICATION_MESSAGE;

extern void Main_task        (uint32_t);

extern void Acc_task         (uint32_t);
extern void Usb_task         (uint32_t);

/*
extern void Usb_task         (uint32_t);
extern void Can_task         (uint32_t);
extern void J1708_task       (uint32_t);
extern void Acc_task         (uint32_t);
extern void Vib_sensor_task  (uint32_t);
extern void Reg_task         (uint32_t);
*/
#endif /* __tasks_list_h_ */
