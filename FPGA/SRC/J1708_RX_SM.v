/**************************************************************************************************
* J1708_RX_SM module implements the J1708 receiver.                                               *
*                                                                                                 *
* The state machine (SM) always waits for bus to become active by waiting in IDLE state to bus    *
* become non-idle. If the J1708 module is in transmit mode, the SM ignores all received bytes and *
* waits for the bus to become IDLE again. If the J1708 module is not in transmit mode, then       *
* another node sends a message and the SM reads the MID byte. If there is a MID collision, the SM *
* can't know that automatically. However, in such case the bus becomes IDLE. Since a message with *
* single byte is illegal, the byte is ignored and is not written to FIFO.                         *
* If data continues, then a message is in process, and all data including MID are written to FIFO.*
* Notice, that there is 1 byte propagation delay between data reception and till it is written.   *
* after legal message received and bus becomes IDLE, a DONE pulse is generated storing data       *
* length at the J1708 register and generates interrupt status.                                    *
**************************************************************************************************/
`timescale 1 ns / 1 ps

module J1708_RX_SM (
	output reg   [2:0] test, 
	
	output reg   [7:0] message_length     = 0 ,
	output reg         message_done       = 0 ,
	output reg   [7:0] message_byte       = 0 ,
	output reg         message_byte_valid = 0 ,

	input              J1708_line_idle        ,
	input    	   [7:0] J1708_rx_byte        ,
	input              J1708_rx_byte_valid    ,
	input              tx_message_process     ,
	input              rst                    ,
	input              enable                 ,
	input              clk
);

localparam	
			STATE_RESET             = 0 ,
			STATE_IDLE              = 1 ,
			STATE_GET_MID           = 2 ,
			STATE_GET_BYTE          = 3 ,
			STATE_WAIT_FOR_TX_IDLE  = 4 ,
			STATE_DONE              = 5 ;

reg  [2:0]  sm                          = STATE_RESET;
reg  [2:0]  J1708_rx_byte_valid_sampled	= 0;

always @(posedge clk or posedge rst)
begin
	if (rst)					  sm <= STATE_RESET;
	else if   (!enable)			  sm <= STATE_RESET;
	else case (sm)
		STATE_RESET    			: sm <= J1708_line_idle    ? STATE_IDLE : STATE_RESET ;   // at reset, wait to line become idle- prevent wakeup at unknown bus state
		STATE_IDLE     			: sm <= J1708_line_idle    ? STATE_IDLE : 
										tx_message_process ? STATE_WAIT_FOR_TX_IDLE :
															 STATE_GET_MID ;
			
		STATE_GET_MID  			: sm <=  J1708_rx_byte_valid ? STATE_GET_BYTE     : STATE_GET_MID  ;
			
		STATE_GET_BYTE 			: sm <=  J1708_line_idle & (message_length == 1)  ? STATE_IDLE     :
								         J1708_line_idle                          ? STATE_DONE     : 
																				    STATE_GET_BYTE ;

		STATE_WAIT_FOR_TX_IDLE : sm <= J1708_line_idle ? STATE_IDLE : STATE_WAIT_FOR_TX_IDLE ;

		STATE_DONE             : sm <= STATE_IDLE;

		default                : sm <= STATE_RESET;
	endcase 
end


always @(posedge clk or posedge rst)
begin
	if (rst)
		{message_length, message_byte, message_byte_valid, message_done, J1708_rx_byte_valid_sampled}  <= 0;
	else if (!enable)		
		{message_length, message_byte, message_byte_valid, message_done, J1708_rx_byte_valid_sampled}  <= 0;
	else begin	
		if (sm == STATE_IDLE)					message_length <= 0;			
		else if (J1708_rx_byte_valid)				message_length <= message_length + 1;

		message_byte_valid <=    (sm == STATE_DONE) || ((sm == STATE_GET_BYTE) & J1708_rx_byte_valid) ;
		
		message_done       <=     sm == STATE_DONE ;
		
		J1708_rx_byte_valid_sampled <= {J1708_rx_byte_valid_sampled, J1708_rx_byte_valid};
		
		if (J1708_rx_byte_valid_sampled [2])
  		  message_byte <= J1708_rx_byte ;
								
	end
end

always @(posedge clk)
begin
	test [0] <= message_byte_valid      ?  ~test [0] : test [0] ;
	test [1] <= sm == STATE_GET_BYTE    ;
	test [2] <= J1708_rx_byte_valid     ?  ~test [2] : test [2] ;
end	
endmodule
