#ifndef _FRAME_H
#define _FRAME_H

#include <stdint.h>
#include <stdbool.h>
typedef struct
{
	uint8_t * data;
	uint32_t data_alloc;
	uint32_t data_len;

	bool escape_flag;
	bool data_ready;
} frame_t;

void frame_reset(frame_t * frame);
void frame_setbuffer(frame_t * frame, uint8_t * buffer, uint32_t len);
int frame_process_buffer(frame_t * frame, uint8_t * buffer, uint32_t len);

int frame_encode(uint8_t * s, const uint8_t * d, int len);
bool frame_data_ready(frame_t * frame);

#endif /* _FRAME_H */

