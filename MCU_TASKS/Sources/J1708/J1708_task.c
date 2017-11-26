/**************************************************************************************
* The J1708 is implemented by the on board FPGA. There are 2 J1708 independent tasks  *
* Rx task and Tx task. The FPGA - MCU data protocol interface is implemented via UART *
* channel, while the command protocol interface is implemente via I2C channel. Both   *
* Tx and Rx tasks use the same UART and I2C channels.                                 *
*                                                                                     *
* J1708 Tx task:                                                                      *
* --------------                                                                      *
* Tx task receives data from CPU and trnasfer it "AS IS" to the on board FPGA. When   *
* transfer is completed, the MCU sends an I2C command, indicating the amount of byte  *
* in the message. By that, the MCU also indicates that package is ready to be sent.   *
*                                                                                     *
* J1708 Rx task:                                                                      *
* --------------                                                                      *
* Rx task receives data from on board FPGA and trnasfer it "AS IS" to the MCU. When   *
* FPGA has a complete packet that needs to be sent to CPU, it generates an interrupt  *
* to the MCU. if The MCU interrupt rutine detects that J1708 message is ready it      *
* notifys to Rx task that a message is ready to be read from the FPGA.                *
* The Rx task reads the amount of bytes to be read (message length) via I2C channel   *
* and reads the message via UART channel. When message is read it is being sent to    *
* USB task.                                                                           *
**************************************************************************************/

#include "tasks_list.h"
#include "J1708_task.h"
#include "frame.h"

#include "fpga_api.h"

#include "gpio_pins.h"

#include "mic_typedef.h"

#define J1708_DISABLED							0
#define J1708_ENABLED							1

#define J1708_MAX_MESSAGE_SIZE                  21

#ifdef __cplusplus
extern "C"
{
#endif

static uint8_t J1708_state    = J1708_DISABLED;
static uint8_t J1708_priority = 7;			// lowest priority by default

void * g_J1708_event_h;

void J1708_reset   (void);

void J1708_Tx_task (uint32_t initial_data)
{
	APPLICATION_MESSAGE_T *msg;
	const _queue_id     J1708_tx_qid = _msgq_open (J1708_TX_QUEUE, 0);

	uint8_t       cnt;
	bool          J1708TxFull   ;
	uint8_t       uart_tx_length;

//	printf ("\nJ1708_Tx Task: Start \n");
	uint64_t current_time;
	TIME_STRUCT time;

	_time_get(&time);
	current_time = (uint64_t)1000*time.SECONDS + time.MILLISECONDS;
	printf("%s: started %llu\n", __func__, current_time);

	while (0 == g_flag_Exit) {
		while (J1708_state == J1708_DISABLED)
			_time_delay (10000);

		// wait 10 second for interrupt message
		msg = _msgq_receive(J1708_tx_qid, 10000);
		if (msg == NULL) {
			//printf ("\nJ1708_Tx: WARNING: Message not received in last 10 Seconds \n");
			continue;
		}

		// read J1708 package length register via I2C channel and verify there is enough space for new message
		for (cnt = 0; cnt < 10; cnt++) {
			if (FPGA_read_J1708_tx_register (&J1708TxFull, NULL))
				if (!J1708TxFull)
					break;
		}

		if (cnt >= 10) {
			printf ("\nJ1708_Tx: ERROR: Previous transfer not completed !!!\n");
			J1708_reset ();
			_msg_free   (msg);
			continue;
		}

		// send message via UART channel
		uart_tx_length = msg->header.SIZE;
		if (!FPGA_write_J1708_packet  (msg->data, uart_tx_length)) {
			_msg_free   (msg);
			J1708_reset ();
			continue;
		}

		_msg_free   (msg);

		// send command via I2C channel
		if (!FPGA_write_J1708_tx_length (&uart_tx_length)) {
			J1708_reset ();
			continue;
		}
	}

	// should never get here
	printf ("\nJ1708_Tx Task: End \n");
	_task_block();
}

//#define MIC_LED_TEST

void J1708_Rx_task (uint32_t initial_data)
{
	pcdc_mic_queue_element_t    pqMemElem;
	_mqx_uint                   err_task;

    uint8_t                     readbuff[J1708_MAX_MESSAGE_SIZE];    
	uint32_t                    J1708_rx_event_bit;
	bool                        J1708_rx_status;
	uint8_t                     J1708_rx_len;

	uint64_t current_time;
	TIME_STRUCT time;

	_time_get(&time);
	current_time = (uint64_t)1000*time.SECONDS + time.MILLISECONDS;
	printf("%s: started %llu\n", __func__, current_time);

	_event_create ("event.J1708_RX");
	_event_open   ("event.J1708_RX", &g_J1708_event_h);
//	printf ("\nJ1708_Rx Task: Start \n");

	while (0 == g_flag_Exit) {
		while (J1708_state == J1708_DISABLED)
			_time_delay (10000);


		// wait 10 second for interrupt message
		_event_wait_all (g_J1708_event_h, EVENT_J1708_RX, 3000);
		_event_get_value (g_J1708_event_h, &J1708_rx_event_bit);
		if (J1708_rx_event_bit == EVENT_J1708_RX)
			_event_clear    (g_J1708_event_h, EVENT_J1708_RX);
		else {
			//printf ("\nJ1708_Rx: WARNING: No interrupt in last 10 Seconds \n");
			continue;
		}

		// read J1708 package length
		if (!FPGA_read_J1708_rx_register (&J1708_rx_status, &J1708_rx_len)) {
			J1708_reset ();
			continue;
		}

		// check if this is a real new message
		if (J1708_rx_status == false) {
			printf ("\nJ1708_Rx: ERROR: Received interrupt without register bit indication\n");
			continue;
		}

        if (J1708_MAX_MESSAGE_SIZE < J1708_rx_len ) {
            printf("%s: Wrong max msg size %d", __func__, J1708_rx_len);
            continue;
        }
        // send buffer - Since the buffer is cyclic, it might be needed to split buffer to 2 buffers
		pqMemElem = GetUSBWriteBuffer (MIC_CDC_USB_5);
		if (NULL == pqMemElem) {
			printf("%s: Error get mem for USB drop\n", __func__);
			continue;
		}
        pqMemElem->send_size = 0;

		// add header size to message length
		//pqMemElem->send_size = J1708_rx_len;

		// calculate actual buffer size
		if (!FPGA_read_J1708_packet (readbuff, J1708_rx_len)) {
			printf ("\nJ1708_Rx: ERROR: Could not read UART message buffer\n");
			J1708_reset ();
            SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_5);
            continue;
		}

        pqMemElem->send_size = frame_encode((uint8_t*)readbuff, (const uint8_t*)(pqMemElem->data_buff), J1708_rx_len );

		if (!SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_5)) {
			printf("%s: Error send data to CDC5\n", __func__);
		}
	}

	// should never get here
	printf ("\nJ1708_Rx Task: End \n");
	_task_block();
}


void J1708_reset (void)
{
	J1708_disable ();
	J1708_enable (J1708_priority);
}

void J1708_enable (uint8_t priority)
{
	uint8_t prio = priority;

	if (FPGA_set_irq (FPGA_REG_J1708_RX_IRQ_BIT))
		printf ("\nJ1708: Set FPGA J1708 Rx IRQ\n");
	else
		printf ("\nJ1708: ERROR: FPGA J1708 Rx IRQ NOT Set !!!\n");

	// set enable bit
	if (!FPGA_write_J1708_priority (&prio))
		printf ("J1708: ERROR: J1708 Priority NOT SET\n");

	if (FPGA_J1708_enable ())
		printf ("\nJ1708: J1708 Enabled\n");
	else
		printf ("\nJ1708: ERROR: J1708 NOT Enabled !!!\n");

	GPIO_DRV_ClearPinOutput (J1708_ENABLE);

	J1708_state = J1708_ENABLED;
	J1708_priority = priority;
}

void J1708_disable (void)
{
	GPIO_DRV_SetPinOutput   (J1708_ENABLE);

	if (FPGA_clear_irq (FPGA_REG_J1708_RX_IRQ_BIT))
		printf ("\nJ1708: Clear FPGA J1708 Rx IRQ\n");
	else
		printf ("\nJ1708: ERROR: FPGA J1708 Rx IRQ NOT Cleared !!!\n");

	if (FPGA_J1708_disable ())
		printf ("\nJ1708: J1708 Disabled\n");
	else
		printf ("\nJ1708: ERROR: J1708 NOT Disabled !!!\n");

	J1708_state = J1708_DISABLED;
}

#ifdef __cplusplus
}
#endif
