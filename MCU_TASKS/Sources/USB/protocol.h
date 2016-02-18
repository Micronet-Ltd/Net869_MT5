#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <stdio.h>

#define CONTEXT_CONTROL_EP 	100
#define CONTEXT_J1708_EP 	101
#define CONTECT_ACCEL_EP 	102
#define CONTEXT_SLCAN_0_EP 	110
#define CONTEXT_SLCAN_1_EP 	111

typedef enum
{
	SYNC_INFO = 0x0, //0
	COMM_WRITE_REQ, //1
	COMM_READ_REQ,  //2
	COMM_READ_RESP, //3
	PING_REQ, //4
	PING_RESP, //5
	GPIO_INT_STATUS, //6
}packet_type_enum;

int8_t protocol_process_receive_data(uint8_t context, uint8_t * data, uint32_t size);

#endif /* _PROTOCOL_H */
