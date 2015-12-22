/***************************************************************************************************
* J1708_TX_SM module implements the arbitration of J1708 data transmission.                        *
*                                                                                                  *
* Arbitration steps:                                                                               *
* 	1. Wait for the bus to become IDLE.                                                            *
* 	2. Wait the required priority delay after the IDLE period has started.                         *
* 	3. Make sure the bus is still Idle. If the bus is not IDLE, go back to step 1.                 *
* 	4. Transmit the device MID on the bus.                                                         *
* 	5. Receive the transmitted MID and determine that the sent MID matches the received MID.       *
* 	6. If the match was successful - Send the packet.                                              *
* 	7. If the match failed, Wait for a pseudo-random number of bit times (between 0-7).            *
* 	8. Go to step 1.                                                                               *
*                                                                                                  *
* MESSAGE PRIORITY                                                                                 *
* All messages are assigned a message priority by the defining application document. The message   *
* priority is converted into a message priority delay by the following equation:                   *
* 			Priority Delay (bits) = 2 * Message Priority                                           *
*                                                                                                  *
* 	Priority 1 and 2 - Messages that require immediate access to the bus                           *
* 	Priority 3 and 4 - Messages that prevent Mechanical Damage                                     *
* 	Priority 5 and 6 - Messages that control economy or efficiency                                 *
* 	Priority 7 and 8 - All other messages                                                          *
*                                                                                                  *
* PSEUDO-RANDOM NUMBERS                                                                            *
* Pseudo-random numbers are used by J1708 to prevent a deadlock condition where two MIDs become    *
* synchronized in their attempt to gain access to the data bus. This is most likely to occur if    *
* two identical systems are sharing the bus. The Pseudo-random value is based on the 2 LSB of      *
* running timer.                                                                                   *
*                                                                                                  *
* STATE MACHINE OPERATION                                                                          *
* state machine waits for new message to be ready to sent. When there is a new message, the state  *
* machine waits till the bus is in IDLE and then waits for another priority period. At that time   *
* if the bus is still in IDLE, the first message byte (MID) is sent through UART TX block. The     *
* state machine waits to get Rx and Tx valid indication and checks if collision occur.             *
* In case of collision, a random counter is initialized counting down to 0 and return to IDLE      *
* state. If there is no collision, next byte is read, and being sent to the UART Tx block.         *
* Once again, the Rx and Tx byte valid signal causes the next byte to be read, till all data bytes *
* are being sent.                                                                                  *
***************************************************************************************************/
`timescale 1 ns / 1 ps

module J1708_TX_SM (
	input        [7:0] message_length        ,			// number of bytes to be sent including MID and Checksum
	input        [2:0] message_priority      ,			// message priority for arbitration delay 
	input              message_new           ,			// new message is ready to be sent
	output             message_length_read   ,			// read message length 
	input              fifo_data_empty       ,			// data fifo status
	output reg         message_done      = 0 ,			// all bytes were sent
	output             message_next_data     ,			// read next byte to be sent	
	output reg         message_process   = 0 ,			// message in process indication - avoiding writing the data being sent to Rx buffer
	output             J1708_tx_enable       ,			// enable message transfer
	input              J1708_line_idle       ,			// J1708 IDLE line indication
	input              J1708_data_collision  ,			// J1708 MID collision indication
	input              J1708_tx_ready        ,   		// J1708 next byte can be sent
	input              J1708_rx_ready        ,   		// J1708 next byte can be sent
	input              enable                ,
	input              rst                   ,
	input              clk
);

parameter	CLK_FRQ_MHZ          =   24 ;

localparam	BIT_PERIOD           = (105 * CLK_FRQ_MHZ) ;		// 105 usec

localparam	
			STATE_RESET          = 0 ,
			STATE_IDLE           = 1 ,
			STATE_PRIORITY_DELAY = 2 ,
			STATE_GET_MID        = 3 ,
			STATE_LOAD_MID       = 4 ,
			STATE_READ_MID       = 5 ,
			STATE_CHECK_MID      = 6 ,
			STATE_RANDOM_DELAY   = 7 ,
			STATE_READ_DATA      = 8 ,
			STATE_LOAD_DATA      = 9 ,
			STATE_SEND_DATA      = 10;

reg         mid_loaded       = 0;
reg  [1 :0] delay_value      = 0;
reg  [31:0] cnt_delay_time   = 0;
reg  [7 :0] cnt_msg_length   = 0;
reg         delay_time_over  = 0;
reg         data_cnt_done    = 0;
reg  [3:0]  sm               = STATE_RESET;
wire        cnt_msg_enable      ;

assign J1708_tx_enable       =  (sm == STATE_LOAD_MID) || (sm == STATE_LOAD_DATA);

assign cnt_msg_enable        =  ((sm == STATE_GET_MID  ) &~ mid_loaded)     ||
								 (sm == STATE_LOAD_DATA)                     ;

assign message_length_read   =  ((sm == STATE_GET_MID  ) &~ mid_loaded)     ;

assign message_next_data     =  ((sm == STATE_GET_MID  ) &~ mid_loaded    &~ fifo_data_empty) || 
								((sm == STATE_READ_DATA) &~ data_cnt_done &~ fifo_data_empty) ;

always @(posedge clk or posedge rst)
begin
	if        (rst)				sm <= STATE_RESET;
	else if   (!enable)			sm <= STATE_RESET;
	else case (sm)
		STATE_RESET          :	sm <= J1708_line_idle                               ? STATE_IDLE           : STATE_RESET ;
		STATE_IDLE           :	sm <= (J1708_line_idle & 
									  (message_new |  mid_loaded) ) 			? STATE_PRIORITY_DELAY : STATE_IDLE ;

		STATE_PRIORITY_DELAY :	sm <= (J1708_line_idle & delay_time_over) 			? STATE_GET_MID        : 
								delay_time_over                             		? STATE_IDLE           :		// bus is busy
																					  STATE_PRIORITY_DELAY ;

		STATE_GET_MID        :	sm <= ~fifo_data_empty 								? STATE_LOAD_MID :
																					  STATE_GET_MID  ;
		
		STATE_LOAD_MID       :	sm <= STATE_READ_MID      ;

		STATE_READ_MID       :	sm <=  J1708_tx_ready & J1708_rx_ready             	 ? STATE_CHECK_MID     : STATE_READ_MID   ;

		STATE_CHECK_MID      :	sm <= J1708_data_collision                         	? STATE_RANDOM_DELAY   : STATE_READ_DATA   ;

		STATE_RANDOM_DELAY   :	sm <= delay_time_over                               ? STATE_IDLE           : STATE_RANDOM_DELAY;

		STATE_READ_DATA      :	sm <= data_cnt_done                                 ? STATE_IDLE           : 
								~fifo_data_empty                               		? STATE_LOAD_DATA      :
																					  STATE_READ_DATA      ;

		STATE_LOAD_DATA      :	sm <= STATE_SEND_DATA ;

		STATE_SEND_DATA      :	sm <= J1708_tx_ready  & J1708_rx_ready             	? STATE_READ_DATA      : STATE_SEND_DATA ;

		default              :	sm <= STATE_RESET;
	endcase 
end

// wait random time based on free running counter
always @(posedge clk or posedge rst)
begin
	if (rst)
		{delay_time_over, cnt_delay_time, cnt_delay_time} <= 0;
	else if (!enable)
		{delay_time_over, cnt_delay_time, cnt_delay_time} <= 0;
	else begin
		delay_value <= delay_value + 1;						// free running counter
		
		if ( sm == STATE_IDLE    )													cnt_delay_time <= (message_priority + 1) * (2 * BIT_PERIOD);	// message priority delay in bits: 2 - 16 
		if ((sm == STATE_CHECK_MID) & J1708_data_collision)							cnt_delay_time <= (delay_value      + 1) * (2 * BIT_PERIOD);	// random time delay in bits: 2 - 8
		else if ((sm == STATE_RANDOM_DELAY) || (sm == STATE_PRIORITY_DELAY))		cnt_delay_time <= cnt_delay_time - 1;

		delay_time_over <= (cnt_delay_time == 2);
	end
end

always @(posedge clk or posedge rst)
begin
	if (rst)
		{message_process, mid_loaded, cnt_msg_length, data_cnt_done, message_done}  <= 0;
	else if (!enable)
		{message_process, mid_loaded, cnt_msg_length, data_cnt_done, message_done}  <= 0;
	else begin
		if (sm == STATE_LOAD_MID)												message_process <= 	1;
		else if ((sm == STATE_IDLE) || (sm == STATE_RANDOM_DELAY))				message_process <= 	0;

		if (sm == STATE_LOAD_MID)												mid_loaded      <= 	1;
		else if (message_done)													mid_loaded      <= 	0;
		
		if (sm == STATE_IDLE)													cnt_msg_length  <= 0;
		else if (cnt_msg_enable) 												cnt_msg_length  <= cnt_msg_length + 1;
		
		data_cnt_done      <=    cnt_msg_length == message_length;
		message_done       <=    (sm == STATE_READ_DATA) &&  data_cnt_done ;
	end
end

endmodule

