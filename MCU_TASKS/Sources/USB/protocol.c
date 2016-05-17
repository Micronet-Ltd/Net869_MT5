#include "protocol.h"
#include "frame.h"
#include "command.h"
#include "control_task.h"
#include "EXT_GPIOS.h"

frame_t g_control_frame;
frame_t g_j1708_frame;

uint8_t control_data_buffer[32];
uint8_t j1708_data_buffer[128]; // Maybe overkill 21+checksum is max only while engine is running otherwise it can be bigger.


// TODO: required stub functions
void j1708_xmit(uint8_t * data, uint32_t size)
{
}


uint8_t j1708_frame_buffer[ (sizeof(j1708_data_buffer)*2) + 2]; // 128*2+2=258
void j1708_recv(uint8_t * data, uint32_t size)
{
	int len;

	len = frame_encode(data, j1708_frame_buffer, size);

	//TODO: Implement
	// TODO: which thread context will this be called from
	// TODO: what locking, waiting is needed, if any?
	// USB_CDC_J1708_EP_TRANSMIT(j1708_frame_buffer, len);

}

int process_receive_slcan(int contex, uint8_t * data, uint32_t size)
{
	// CONTEXT_SLCAN_0_EP
	// CONTEXT_SLCAN_1_EP
	// TODO: Send data to SLCAN parser
	return 0;

}

/// END stub functions




/// BEGIN 

void protocol_init_data()
{
	frame_setbuffer(&g_control_frame, control_data_buffer, sizeof(control_data_buffer));
	frame_setbuffer(&g_j1708_frame, j1708_data_buffer, sizeof(j1708_data_buffer));
}

static int packet_receive(int context, uint8_t * data, uint32_t size)
{
	int8_t result = 0;
	static int8_t last_seq;
	packet_t req;
	packet_t resp;
	uint8_t resp_size = 0;

	req.seq = data[0];
	req.pkt_type = (packet_type_enum)data[1];
	memcpy(req.data, &data[2], size - 2);
	// NOTE: maybe these should be separate functions, maybe called by caller instead.
	if(CONTEXT_CONTROL_EP == context)
	{
		last_seq = -1;

		if(size < 2)
			return -1; // too small

		// TODO: implement sequence logic
		//TODO: Commenting out seq. logic for now, Abid
//		if(-1 == last_seq)
//		{
//			// TODO: wait for seq
//			if(0 != req->seq)
//				return -1; // wait for sync
//		}
//		else
//		{
//			if( ((last_seq+1)&0xff) != req->seq )
//			{
//				// TODO: set error state
//				last_seq = -1;
//				return -1;
//			}
//			last_seq = req->seq; // SEQ_OFFSET
//		}


		// Handle message
		switch(req.pkt_type)//data[0])
		{
			case SYNC_INFO: // Sync/Info packet
				last_seq = req.seq;
				// TODO: set sync (normal) state
				break;
			case COMM_WRITE_REQ:
				result = command_set(&req.data[0], size-2);
				break;
			case COMM_READ_REQ:
				result = command_get(&req.data[0], size-2, &resp, &resp_size);
				resp.seq = req.seq;
				resp.pkt_type = COMM_READ_RESP;
				send_control_msg(&resp, resp_size);
				break; // TODO: Register read request
			case COMM_READ_RESP: return -1; // BUG: Should never receive this

			case PING_REQ:
				resp.seq = req.seq;
				resp.pkt_type = PING_RESP;
				send_control_msg(&resp, resp_size);
				break;
			case PING_RESP: break;
			case GPIO_INT_STATUS:
				/* process a GPIO output message */
				gpio_set_multiple_outputs(&req.data[0], &req.data[1]);
				break; // BUG: Should never receive this

			// 0x80-0xff reserved for future PDU format changes with backwards compatibility
			default: return -1; // Unknown Ignore
		}
	}
	else if(CONTEXT_J1708_EP == context)
	{
		// TODO: add J1708 code here
		j1708_xmit(req.data, size);
	}
	else
	{
		return -1;
	}

	// added by eyal
	//memcpy (&data[2], &req.data, MAX_PACKET_SIZE);

	return result;
}

int8_t protocol_process_receive_data(uint8_t context, uint8_t * data, uint32_t size)
{
	uint32_t offset = 0;
	// TODO: use type for MCU monotonic clock
	//static uint32_t lastrx = 0;
	// TODO: get current monotonic clock value
	//uint32_t now = 0; // Set to current monotonic clock time

	frame_t * frame;

	switch(context)
	{
		case CONTEXT_CONTROL_EP:
			frame = &g_control_frame;
			break;

		case CONTEXT_J1708_EP:
			frame = &g_j1708_frame;
			break;

		case CONTEXT_SLCAN_0_EP:
		case CONTEXT_SLCAN_1_EP:
			//FIXME: may need to have caller call this function instead to save stack space
			return process_receive_slcan(context, data, size);

		default:
			return -1;
	}

	// If there is a gap of 10ms then reset and data already received.
	// This might need to be something other then 10ms, maybe more or less.
	// Otherwise if the process crashes in middle of a frame, we will receive
	// a partial frame. So this timeout should be greater than min delay between
	// transmissions, this will need to be determined through testing.
	// TODO: implement or add conversion for clock ticks to ms here
	//if( now - lastrx > MS(10) ) // MS(N) returns 10ms in clock ticks.
		frame_reset(frame);


	while(size - offset > 0)
	{
		offset += frame_process_buffer(frame, data + offset, size - offset);

		if(frame_data_ready(frame))
		{
			// TODO: check results
			packet_receive(context, frame->data, frame->data_len);
			frame_reset(frame);
		}
	}

	// TODO: status return here
	return 0;
}
