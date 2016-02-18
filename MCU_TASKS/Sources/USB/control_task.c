/*
 * control_task.c
 *
 *  Created on: Feb 16, 2016
 *      Author: abid.esmail
 */

#include "tasks_list.h"
#include "protocol.h"

void control_task (uint32_t initial_data)
{
	APPLICATION_MESSAGE_T *msg;
	const _queue_id     control_qid = _msgq_open (CONTROL_QUEUE, 0);

	uint8_t       cnt;
	uint8_t       uart_tx_length;

	printf ("\n control_task: Start \n");

	protocol_init_data();

	while (1)
	{
		// wait 10 second for interrupt message
		msg = _msgq_receive(control_qid, 10000);
		if (msg == NULL)
		{
			printf ("\n control_task: WARNING: Message not received in last 10 Seconds \n");
			continue;
		}
		else
		{
			printf("\n data : %x,%x, \t ,%x,%x, size %d \n", msg->data[0], msg->data[1],msg->data[msg->header.SIZE -2], msg->data[msg->header.SIZE -1], msg->header.SIZE);
			//printf("\n control_task: yay got the the message \n");
		}

		protocol_process_receive_data(CONTEXT_CONTROL_EP, msg->data, msg->header.SIZE);
		/* process message */

		/* send host response if there is a response needed */
		_msg_free   (msg);
	}

	/* should never get here */
	printf ("\n control_task: End \n");
	_task_block();
}


