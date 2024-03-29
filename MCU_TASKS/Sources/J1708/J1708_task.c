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

	current_time = ms_from_start();
	printf("%s: started %llu\n", __func__, current_time);

	while (0 == g_flag_Exit) {
		while (J1708_state == J1708_DISABLED)
			_time_delay (1000);

		// wait 1 second for interrupt message
		msg = _msgq_receive(J1708_tx_qid, 1000);
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

bool verify_checksum(uint8_t * readbuff, uint8_t J1708_rx_len){
	bool valid_frame = false;
	uint16_t sum = 0;
	uint8_t i = 0;

	for (i = 0; i < J1708_rx_len; i++){
		sum = sum + *(readbuff + i);
	}

	if ((sum & 0xff) == 0){
		valid_frame = true;
	}
	return valid_frame;
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
	uint32_t session_frames_rx = 0;
	uint32_t session_bad_checksum_cnt = 0;
	uint32_t total_bad_checksum_cnt = 0;

	uint64_t current_time;

	current_time = ms_from_start();
	printf("%s: started %llu\n", __func__, current_time);

	_event_create ("event.J1708_RX");
	_event_open   ("event.J1708_RX", &g_J1708_event_h);
//	printf ("\nJ1708_Rx Task: Start \n");

	while (0 == g_flag_Exit) {
		// wait 1 second for interrupt message
		_event_wait_any (g_J1708_event_h, EVENT_J1708_RX | EVENT_J1708_DISABLE | EVENT_J1708_ENABLE, 0);
		_event_get_value (g_J1708_event_h, &J1708_rx_event_bit);
		
		if (J1708_state == J1708_DISABLED && J1708_rx_event_bit == 0){
			//printf ("\nJ1708_Rx: J1708_DISABLED 0x%x\n", J1708_rx_event_bit);
			continue;
		}

		if (J1708_rx_event_bit&EVENT_J1708_RX){
			_event_clear    (g_J1708_event_h, EVENT_J1708_RX);
		}

		if (J1708_rx_event_bit&EVENT_J1708_DISABLE){
			printf ("\nJ1708_Rx: J1708_rx_event_bit&EVENT_J1708_DISABLE 0x%x\n", J1708_rx_event_bit);
			J1708_disable();
			_event_clear    (g_J1708_event_h, EVENT_J1708_DISABLE);
			continue;
		}
		if (J1708_rx_event_bit&EVENT_J1708_ENABLE){
			printf ("\nJ1708_Rx: J1708_rx_event_bit&EVENT_J1708_ENABLE 0x%x\n", J1708_rx_event_bit);
			J1708_enable(7);
			_event_clear    (g_J1708_event_h, EVENT_J1708_ENABLE);
			session_bad_checksum_cnt = 0;
			session_frames_rx = 0;
			continue;
		}

		// read J1708 package length
		if (!FPGA_read_J1708_rx_register (&J1708_rx_status, &J1708_rx_len)) {
			J1708_reset ();
			continue;
		}

		// check if this is a real new message
		if (J1708_rx_status == false) {
			//printf ("\nJ1708_Rx: ERROR: Received interrupt without register bit indication\n");
			continue;
		}

        if (J1708_MAX_MESSAGE_SIZE < J1708_rx_len ) {
            printf("%s: Wrong max msg size %d", __func__, J1708_rx_len);
            continue;
        }

		// add header size to message length
		//pqMemElem->send_size = J1708_rx_len;

		// calculate actual buffer size
		if (!FPGA_read_J1708_packet (readbuff, J1708_rx_len)) {
			printf ("\nJ1708_Rx: ERROR: Could not read UART message buffer\n");
			J1708_reset ();
            //SetUSBWriteBuffer(pqMemElem, MIC_CDC_USB_5);
            continue;
		}

		session_frames_rx++;
		if (!verify_checksum(readbuff, J1708_rx_len)){
			session_bad_checksum_cnt++;
			total_bad_checksum_cnt++;
			printf("\n%s: session_frames_rx=%d, session_bad_checksum_cnt=%d, total_bad_checksum_cnt=%d\n",__func__, session_frames_rx, session_bad_checksum_cnt, total_bad_checksum_cnt);
			if (session_frames_rx == 1){
				printf("\n%s: FIRST PACKET BAD", __func__);
				continue; /* if first packet is bad drop the packet  */
			}
		}
		
		// send buffer - Since the buffer is cyclic, it might be needed to split buffer to 2 buffers
		pqMemElem = GetUSBWriteBuffer (MIC_CDC_USB_5);
		if (NULL == pqMemElem) {
			printf("%s: Error get mem for USB drop\n", __func__);
			_time_delay (10); /* give the USB some time to send packets */
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
	
	GPIO_DRV_SetPinOutput(CAN1_J1708_PWR_ENABLE); /* below V6 board */
	GPIO_DRV_SetPinOutput(J1708_PWR_EN); /*V6 board */

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
	int ret = 0;
	bool J1708_rx_status;
	uint8_t J1708_rx_len;
	uint8_t readbuff[J1708_MAX_MESSAGE_SIZE]; 		
	uint8_t count = 0;
	GPIO_DRV_SetPinOutput   (J1708_ENABLE);
	
	do{
		count++;
		ret = FPGA_read_J1708_rx_register (&J1708_rx_status, &J1708_rx_len);
		printf("%s: FPGA_read_J1708_rx_register rx_status=%x, rx_len = %x, ret=%x, count=%d\n", __func__,  J1708_rx_status, J1708_rx_len, ret, count);
		if (J1708_rx_status == true){
			ret = FPGA_read_J1708_packet (readbuff, J1708_rx_len);	 /* clearing J1708 data register */
			printf("%s: FPGA_read_J1708_packet ret=%x\n", __func__, ret);
		}
	}while (J1708_rx_status);
	
	if (FPGA_clear_irq (FPGA_REG_J1708_RX_IRQ_BIT))
		printf ("\nJ1708: Clear FPGA J1708 Rx IRQ\n");
	else
		printf ("\nJ1708: ERROR: FPGA J1708 Rx IRQ NOT Cleared !!!\n");

	if (FPGA_J1708_disable ())
		printf ("\nJ1708: J1708 Disabled\n");
	else
		printf ("\nJ1708: ERROR: J1708 NOT Disabled !!!\n");

	GPIO_DRV_ClearPinOutput(CAN1_J1708_PWR_ENABLE); /* below V6 board */
	GPIO_DRV_ClearPinOutput(J1708_PWR_EN); /*V6 board */
	
	J1708_state = J1708_DISABLED;
}

#ifdef __cplusplus
}
#endif
