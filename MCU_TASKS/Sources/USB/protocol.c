#include "protocol.h"
#include "frame.h"

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

int process_receive_slcan(uint8_t * data, uint32_t size)
{
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

static int packet_recieve(int context, uint8_t * data, uint32_t size)
{
	// NOTE: maybe these should be separate functions, maybe called by caller instead.
	if(CONTEXT_CONTROL_EP == context)
	{
		static int last_seq = -1;

		if(size < 2)
			return -1; // too small

		// TODO: implement sequence logic
		if(-1 == last_seq)
		{
			// TODO: wait for seq
			if(0 != data[0])
				return -1; // wait for sync
		}
		else
		{
			if( (last_seq+1)&0xff != data[1] )
			{
				// TODO: set error state
				last_seq = -1;
				return -1;
			}
			last_seq = data[1]; // SEQ_OFFSET
		}


		// Handle message
		switch(data[0])
		{
			case 0: // Sync/Info packet
				last_seq = data[1];
				// TODO: set sync (normal) state
				break;
			case 0x01: break; // TODO: Write register
			case 0x02: break; // TODO: Register read request
			case 0x03: return -1; // BUG: Register read response should never receive this
			case 0x04: break; // TODO: RTC Write
			case 0x05: break; // TODO: RTC Read request
			case 0x06: return -1; // BUG: RTC Read response
			case 0x07: break; // TODO: Ping request
			case 0x08: break; // TODO: Ping response
			case 0x09: return -1; // BUG: GPIO Interrup Status

			// 0x80-0xff reserved for future PDU format changes with backwards compatability
			default: return -1; // Unknown Ignore
		}
	}
	else if(CONTEXT_J1708_EP == context)
	{
		// TODO: add J1708 code here
		j1708_xmit(data, size);
	}
	else
	{
		return -1;
	}

	return 0;
}

int protocol_process_receive_data(int context, uint8_t * data, uint32_t size)
{
	uint32_t offset = 0;
	// TODO: use type for MCU monotonic clock
	static uint32_t lastrx = 0;
	// TODO: get current monotonic clock value
	uint32_t now = 0; // Set to current monotonic clock time

	frame_t * frame;

	switch(context)
	{
		case CONTEXT_CONTROL_EP:
			frame = &g_control_frame;
			break;

		case CONTEXT_J1708_EP:
			frame = &g_j1708_frame;
			break;

		case CONTEXT_SLCAN_EP:
			//FIXME: may need to have caller call this function instead to save stack space
			return process_receive_slcan(data, size);

		default:
			return -1;
	}

	// If there is a gap of 10ms then reset and data already received.
	// This migh need to be something other then 10ms, maybe more or less.
	// Otherwise if the process crashes in middle of a frame, we will receive
	// a partial frame. So this timeout should be greather then min delay between
	// transmissions, this will need to be determined through testing.
	// TODO: implement or add conversion for clock ticks to ms here
	if( now - lastrx > MS(10) ) // MS(N) returns 10ms in clock ticks.
		frame_reset(frame);


	while(size - offset > 0)
	{
		offset = frame_process_buffer(frame, data + offset, size - offset);

		if(frame_data_ready(frame))
		{
			// TODO: check results
			packet_recieve(context, frame->data, frame->data_len);
			frame_reset(frame);
		}
	}

	// TODO: status return here
	return 0;
}
