/*
 * control_task.c
 *
 *  Created on: Feb 16, 2016
 *      Author: abid.esmail
 */

#include "tasks_list.h"
#include "protocol.h"
#include "mic_typedef.h"
#include "event.h"

extern _pool_id   g_out_message_pool;
extern void *g_GPIO_event_h;

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
		//TODO: for now hard coding the sequence
		msg->seq = 0x99;
		/* copy seq and packet type */
		memcpy(ctl_tx_msg->data,(uint8_t *) msg, 2);
		/* copy the data */
		memcpy(ctl_tx_msg->data+2, msg->data, msg_size);

		ctl_tx_msg->header.SOURCE_QID = _msgq_get_id(0, CONTROL_TX_QUEUE);
		ctl_tx_msg->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
		ctl_tx_msg->header.SIZE = (msg_size + 2);//add seq and packet type
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

	printf ("\n control_task: Start \n");

	protocol_init_data();

	while (1)
	{
		/* wait forever for interrupt message */
		ctl_rx_msg = _msgq_receive(control_rx_qid, 0);
		if (ctl_rx_msg != NULL)
		{
			//TODO: For debug only
			printf("\n data : %x,%x, \t ,%x,%x, size %d \n", ctl_rx_msg->data[0],
					ctl_rx_msg->data[1],ctl_rx_msg->data[ctl_rx_msg->header.SIZE -2],
					ctl_rx_msg->data[ctl_rx_msg->header.SIZE -1], ctl_rx_msg->header.SIZE);
			/* process message */
			protocol_process_receive_data(CONTEXT_CONTROL_EP, ctl_rx_msg->data, ctl_rx_msg->header.SIZE);
			_msg_free   (ctl_rx_msg);
		}

		//_event_wait_all(g_GPIO_event_h, 0xff, 1);

		//_event_clear(g_GPIO_event_h, 0xff);
	}

	/* should never get here */
	printf ("\n control_task: End \n");
	_task_block();
}


