/*
 * control_task.c
 *
 *  Created on: Feb 16, 2016
 *      Author: abid.esmail
 */

#include "tasks_list.h"
#include "protocol.h"
#include "mic_typedef.h"

extern _pool_id   g_out_message_pool;

void send_control_msg(packet_t * msg, uint8_t msg_size)
{
	APPLICATION_MESSAGE_T *ctl_tx_msg;
	_mqx_uint err_task;
	if ((ctl_tx_msg = (APPLICATION_MESSAGE_PTR_T) _msg_alloc (g_out_message_pool)) == NULL)
	{
		if (MQX_OK != (err_task = _task_get_error()))
		{
			_task_set_error(MQX_OK);
		}
		printf("send_control_msg: ERROR: message allocation failed %x\n", err_task);
	}

	if(ctl_tx_msg)
	{
		/* copy seq and packet type */
		memcpy(ctl_tx_msg->data,(uint8_t *) msg, 2);
		memcpy(ctl_tx_msg->data+2, msg->data, msg_size);
		free(msg->data);
		ctl_tx_msg->header.SOURCE_QID = _msgq_get_id(0, CONTROL_TX_QUEUE);
		ctl_tx_msg->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
		ctl_tx_msg->header.SIZE = (msg_size) + 2;//add seq and packet type
		ctl_tx_msg->portNum = MIC_CDC_USB_1;
		_msgq_send (ctl_tx_msg);

		if (MQX_OK != (err_task = _task_get_error()))
		{
			printf("send_control_msg: ERROR: message send failed %x\n", err_task);
			_task_set_error(MQX_OK);
		}
	}
}

void control_task (uint32_t initial_data)
{
	APPLICATION_MESSAGE_T *ctl_rx_msg;
	const _queue_id     control_rx_qid = _msgq_open (CONTROL_RX_QUEUE, 0);

	int8_t ret;
	packet_t resp;
	uint8_t resp_size = 0;

	printf ("\n control_task: Start \n");

	protocol_init_data();

	while (1)
	{
		// wait 10 second for interrupt message
		ctl_rx_msg = _msgq_receive(control_rx_qid, 10000);
		if (ctl_rx_msg == NULL)
		{
			printf ("\n control_task: WARNING: Message not received in last 10 Seconds \n");
			continue;
		}
		else
		{
			//TODO: For debug only
			printf("\n data : %x,%x, \t ,%x,%x, size %d \n", ctl_rx_msg->data[0],
					ctl_rx_msg->data[1],ctl_rx_msg->data[ctl_rx_msg->header.SIZE -2],
					ctl_rx_msg->data[ctl_rx_msg->header.SIZE -1], ctl_rx_msg->header.SIZE);
		}

		/* process message */
		ret = protocol_process_receive_data(CONTEXT_CONTROL_EP, ctl_rx_msg->data, ctl_rx_msg->header.SIZE,
				&resp, &resp_size);
		_msg_free   (ctl_rx_msg);

		/* send host response if there is a response needed */
		if (resp_size != 0)
		{
			send_control_msg(&resp, resp_size);
		}

	}

	/* should never get here */
	printf ("\n control_task: End \n");
	_task_block();
}


