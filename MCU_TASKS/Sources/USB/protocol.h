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
	SYNC_INFO 		= 0x0,
	COMM_WRITE_REQ 	= 0x1,
	COMM_READ_REQ 	= 0x2,
	COMM_READ_RESP 	= 0x3,
	PING_REQ 		= 0x4,
	PING_RESP 		= 0x5,
	GPIO_INT_STATUS = 0x6,
}packet_type_enum;

typedef struct
{
	uint8_t seq;
	packet_type_enum pkt_type;
	uint8_t *data;
}packet_t;

void protocol_init_data();
int8_t protocol_process_receive_data(uint8_t context, uint8_t * data, uint32_t size);

#endif /* _PROTOCOL_H */
