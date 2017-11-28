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


#include "frame.h"
#include "Device_control_GPIO.h"

//extern _pool_id   g_out_message_pool;
extern void *g_GPIO_event_h;
extern DEVICE_STATE_t device_state_g;

void send_control_msg(packet_t * msg, uint8_t msg_size)
{
#if 1	
#if (DEBUG_LOG == 1)
  	uint64_t current_time;
	
	current_time = ms_from_start();
	printf("%s: %llu ms, %x\n", __func__, current_time, msg->pkt_type);
#endif
	
	/* Do not try to send a control message if the A8 board is not ON */
	if ((device_state_g != DEVICE_STATE_ON) && (device_state_g != DEVICE_STATE_BACKUP_RECOVERY))
	{
		return;	
	}

	pcdc_mic_queue_element_t pqMemElem;

	pqMemElem = GetUSBWriteBuffer (MIC_CDC_USB_1);
	if (NULL == pqMemElem) {
		printf("%s: Error get mem for USB drop\n", __func__);
		return;
	}

	//TODO: for now hard coding the sequence
	msg->seq = 0x99;

	pqMemElem->send_size = frame_encode((uint8_t*)msg, (const uint8_t*)(pqMemElem->data_buff), (msg_size + 2) );

	if (!SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_1)) {
		printf("%s: Error send data to CDC0\n", __func__);
	}
#else
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
		/* copy seq(1byte) and packet type(1byte)*/
		memcpy(ctl_tx_msg->data,(uint8_t *) msg, sizeof(msg->seq) + sizeof(msg->pkt_type));
		/* copy the data */
		memcpy(ctl_tx_msg->data + sizeof(msg->seq) + sizeof(msg->pkt_type), msg->data, msg_size);

		ctl_tx_msg->header.SOURCE_QID = _msgq_get_id(0, CONTROL_TX_QUEUE);
		ctl_tx_msg->header.TARGET_QID = _msgq_get_id(0, USB_QUEUE);
		ctl_tx_msg->header.SIZE = (msg_size + sizeof(msg->seq) + sizeof(msg->pkt_type));//add seq and packet type
		ctl_tx_msg->portNum = MIC_CDC_USB_1;
		_msgq_send (ctl_tx_msg);

		if (MQX_OK != (err_task = _task_get_error()))
		{
			printf("send_control_msg: ERROR: message send failed %x\n", err_task);
			_task_set_error(MQX_OK);
		}
	}
#endif
}

void control_task (uint32_t initial_data)
{
	APPLICATION_MESSAGE_T *ctl_rx_msg;
	const _queue_id     control_rx_qid = _msgq_open (CONTROL_RX_QUEUE, 0);
	uint64_t current_time = 0;

	printf ("\n %s: Start \n", __func__);

	protocol_init_data();

	while (0 == g_flag_Exit)
	{
		/* wait forever for interrupt message */
		ctl_rx_msg = _msgq_receive(control_rx_qid, 0);
		 
		do {

		  	if (ctl_rx_msg != NULL && ctl_rx_msg->header.SIZE >= 2)
			{
#if (DEBUG_LOG == 1)
				//TODO: For debug only
				current_time = ms_from_start();
				printf("%s: time: %llu ms, data : %x,%x, \t ,%x,%x, size %d \n", __func__, current_time,
					   ctl_rx_msg->data[0], ctl_rx_msg->data[1], ctl_rx_msg->data[ctl_rx_msg->header.SIZE -2],
						ctl_rx_msg->data[ctl_rx_msg->header.SIZE -1], ctl_rx_msg->header.SIZE);
#endif
				/* process message */
				protocol_process_receive_data(CONTEXT_CONTROL_EP, ctl_rx_msg->data, ctl_rx_msg->header.SIZE);
				break;
			}
		} while (0);

		if (NULL != ctl_rx_msg) {
#if (DEBUG_LOG == 1)
			current_time = ms_from_start();
			printf("%s: time: %llu ms, data free msg\n", __func__, current_time);
#endif
			_msg_free   (ctl_rx_msg);
			ctl_rx_msg = NULL;
		}
	}

	/* should never get here */
	printf ("\n control_task: End \n");
	_task_block();
}
