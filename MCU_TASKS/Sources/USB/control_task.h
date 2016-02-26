/*
 * control_task.h
 *
 *  Created on: Feb 16, 2016
 *      Author: abid.esmail
 */

#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "protocol.h"

void send_control_msg(packet_t * msg, uint8_t msg_size);

#endif /* _CONTROL_TASK_H_ */
