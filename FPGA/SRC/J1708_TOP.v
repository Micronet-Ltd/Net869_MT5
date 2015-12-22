/***************************************************************************************************
* SAE J1708 is a standard that implements a bidirectional, serial communication link among modules *
* containing microcomputers. SAE J1708 defines the data link, and physical layer having the        *
* electrical specifications as RS-485. The physical media is a two-wire bus using 18-gauge twisted *
* pair with  maximum length of 40m and up to 20 nodes.                                             *
*                                                                                                  *
* SAE J1708 defines the BAUD rate as 9600, where the duration of one bit time is 104 uSec. The bit *
* value is defined as the voltage differences between the wire pairs level greater than or equal   *
* to 0.2 V.                                                                                        *
*    '1' when point A is at least 0.2 V more positive than point B.                                *
*    '0' when point B is at least 0.2 V more positive than point A.                                *
* Each character consists of 10 bits. Starting with one low logic level bit (START BIT), followed  *
* by 8 bits of data and ending the character with a high logic level bit (STOP BIT). The data bits *
* are sent with MSB first.                                                                         *
*                                                                                                  *
* A message consist of 3 - 21 characters in length.                                                *
* The first message byte is Message IDentifier (MID).                                              *
* MID is followed by 1 - 19 characters of data                                                     *
* the last message byte is a checksum.                                                             *
* The total length of a message is limited to 21 characters when vehicle running, and undefined    *
* when vehicle is not running.                                                                     *
*                                                                                                  *
* The maximum bus nodes is 20 nodes. Regardless of this number, the bit transitions must remain as *
* defined in the specification.                                                                    *
* 	a low-to-high transition must least up to 10  uSec.                                            *
* 	a high-to-low transition must least up to 2.3 uSec.                                            *
*                                                                                                  *
* Each node may access the bus randomly once the bus is idle for a predetermined access time.      *
* Therefore, prior sending of a new message the communication link has remained at IDLE (high      *
* logic level) for a duration of at least 12 * bit time. If two or more nodes attempt to access    *
* the bus at the same time, the contending nodes must arbitrate for the bus.                       *
* This is based on among other things, the specified priority (1 - top to 8 - low) for that        *
* message, resulting in a value called bus access time. This bus access time tells how long the    *
* bus should have been in an idle state before a specific message is allowed accessing the         *
* communication channel. The delay between characters inside a message, must not exceed 2 bit time.*
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
* MESSAGE CHECKSUM                                                                                 *
* The checksum is calculated by adding each packet and subtracting the total from zero. By adding  *
* up the data in the receiver, the total should reach zero. If the total is not zero, an error     *
* occurred in the data transmission. J1708 does not specify any error correction or data           *
* retransmission when a mismatch occurs. The data is simply assumed to be invalid and should be    *
* discarded.                                                                                       *
*                                                                                                  *
* To compute a checksum (255 - (sigma of all bytes % 256)) + 1:                                    *
* 		1. add all the bytes                                                                       *
* 		2. use only 8 LSBs of sum                                                                  *
* 		3. invert the sum                                                                          *
* 		4. add 1                                                                                   *
*                                                                                                  *
* The receiver has 2 choices after discarding the bad packet, if the data is a regularly scheduled *
* broadcast, the receiver can simply wait for the next broadcast while using the stale data, or    *
* the receiver can send a request for the data to be resent.                                       *
*                                                                                                  *
* IMPLEMENTATION                                                                                   *
* --------------                                                                                   *
* Eeach path - Tx and Rx - contains 3 sub modules in opossite order : FIFO, State machine and UART.*
* The FIFO units interface the processor unit which can send the data in any interface as long as  *
* some upper unit converts the data from byte level to processor unit level.                       *
* The UART units interface the J1708 bus and therefore operates at baudrate of 9600 with MSB first.*
* The state machine (SM) implementation is different between the TX and RX paths.                  *
* The Tx SM reads the data from FIFO according to the length value, implements the timing          *
* arbitration including priority and random delay as described above.                              *
* The Rx SM recieves all data and writes the data to FIFO if the module is not in a transmision    *
* cycle.                                                                                           *
* The module also detects a MID contention if the transmited MID is different than the recieved    *
* MID. The module DOES NOT calculate the check sum at any stage and transfer all data "AS IS" in   *
* raw data mdoe.                                                                                   *
***************************************************************************************************/
`timescale 1 ns / 1 ps

module J1708_TOP (
	input        [7:0] tx_message_byte          ,			// message bytes to be sent including MID and Checksum
	input              tx_message_byte_valid    ,  			// message bytes to be sent valid indication
	input        [7:0] tx_message_length        ,			// number of message bytes to be sent including MID and Checksum
	input        [2:0] tx_message_priority      ,			// message priority for arbitration delay 
	input              tx_message_new           ,			// new message is ready to be sent
	output             tx_done                  ,			// All message transaction completed

	input              rx_message_byte_read     ,   		// read next byte
	output       [7:0] rx_message_byte          ,			// received bit converted to bytes to be sent to destivation 
	output reg         rx_message_byte_valid = 0,			// message byte is valid after read
	output             rx_message_new_byte      ,			// valid output byte exists
	
	input              rx_message_length_read   ,			// read next message length
	output       [7:0] rx_message_length        ,			// number of received bytes including MID and Checksum
	output             rx_message_length_valid  ,			// number of received bytes valid signal 
	output             rx_message_length_exist  ,			// a message has been recieved completely and its length is stioed in FIFO

	output             J1708_tx                 ,			// data that is sent to J1708 bus 
	input              J1708_rx                 ,			// data that is received from J1708 bus 

	input              enable                   ,	
	input              rst                      ,
	input              clk
);

parameter	CLK_FRQ_MHZ   =   24 ;

localparam	BAUD_RATE     = 9600 ,
			LSB_FIRST     =    1 ,
			TIMER_1100_US = (1110 * CLK_FRQ_MHZ) ;		// 1100 usec with some spare

wire        J1708_tx_int			;			// J1708 TX before pin FF
wire        J1708_rx_int			;			// J1708 RX after  pin FF

wire [7:0]  tx_process_length       ;			// Tx message length that is being in process (output from FIFO)
wire        tx_fifo_len_empty       ;
wire        tx_fifo_len_full        ;
wire        tx_message_length_read  ;
wire [7:0]  tx_message_byte_out     ;
wire        tx_message_byte_read    ;
wire        tx_message_process      ;
wire        tx_fifo_data_empty      ;
wire        J1708_tx_enable         ;
wire        J1708_tx_busy           ;

wire        J1708_rx_busy           ;
wire [7:0]  J1708_rx_byte           ;
wire        J1708_rx_byte_valid     ;
wire [7:0]  rx_internal_byte        ;
wire        rx_internal_byte_valid  ;
wire        rx_data_fifo_empty      ;

wire [7:0]  rx_message_len_to_fifo  ;
wire        rx_fifo_len_empty       ;
wire        rx_message_done         ;

reg  [31:0] cnt_idle	         = 0;
reg         cnt_idle_clear       = 1;
reg         J1708_line_idle      = 0;
reg         J1708_data_collision = 0;

/******************************************************************************
*                    J1708 TX - DATA FROM CPU TO J1708 BUS                    *
* Get data from CPU through UART at baud rate of 115200.                      *
* Store data in FIFO till SEND command is asserted.                           *
* Implement J1708 TX protocal using state machine.                            *
* Send the data through UART at baud rate of 9600.                            * 
******************************************************************************/
FIFO_512x8 J1708_TX_DATA_FIFO (
	.data_in               (tx_message_byte         ),
	.write                 (tx_message_byte_valid   ),
	.read                  (tx_message_byte_read    ),
	.data_out              (tx_message_byte_out     ),
	.full                  (                        ),
	.empty                 (tx_fifo_data_empty      ),
	.enable                (enable                  ),
	.rst                   (rst                     ),
	.clk                   (clk                     )
);

SMALL_FIFO #(
	.DATA_WIDTH (8),
	.FIFO_DEPTH (10)
) J1708_TX_LEN_FIFO (
	.data_in               (tx_message_length      ),
	.write                 (tx_message_new         ),
	.read                  (tx_message_length_read ),
	.data_out              (tx_process_length      ),
	.data_out_valid        (                       ),
	.full                  (tx_fifo_len_full       ),
	.empty                 (tx_fifo_len_empty      ),
	.enable                (enable                 ),
	.rst                   (rst                    ),
	.clk                   (clk                    )
);

J1708_TX_SM #(
	.CLK_FRQ_MHZ           (CLK_FRQ_MHZ             )
) J1708_TX_SM (                                     
	.message_length        (tx_process_length       ),
	.message_priority      (tx_message_priority     ),
	.message_new           (~tx_fifo_len_empty      ),
	.message_length_read   (tx_message_length_read  ),
	.fifo_data_empty       (tx_fifo_data_empty      ),
	.message_done          (                        ),
	.message_next_data     (tx_message_byte_read    ),
	.message_process       (tx_message_process      ),
	                                                
	.J1708_tx_enable       (J1708_tx_enable         ),
	.J1708_line_idle       (J1708_line_idle         ),
	.J1708_data_collision  (J1708_data_collision    ),
	.J1708_tx_ready        (~J1708_tx_busy          ),
	.J1708_rx_ready        (~J1708_rx_busy          ),
	.enable                (enable                  ),
	.rst                   (rst                     ),
	.clk                   (clk                     )
);

UART_TX #(
	.CLK_FRQ_MHZ           (CLK_FRQ_MHZ             ),
	.BAUD_RATE             (BAUD_RATE               ),
	.LSB_FIRST             (LSB_FIRST               )
) J1708_TX_UART (        
	.data_in               (tx_message_byte_out     ),
	.data_in_valid         (J1708_tx_enable         ),
	.uart_tx               (J1708_tx_int            ),
	.uart_tx_busy          (J1708_tx_busy           ),
	.enable                (enable                  ),
	.rst                   (rst                     ),
	.clk                   (clk                     )
);

IO_OUTPUT  IO_J1708_TX (
	.IO_pin                  (J1708_tx              ),
	.data                    (~J1708_tx_int         ),
	.clk                     (clk                   )
);

/******************************************************************************
*                    J1708 RX - DATA FROM J1708 BUS TO CPU                    *
* Read data from J1708 bus through UART at baud rate of 9600.                 *
* Store data in FIFO if no bus colision or no J1708 TX.                       *
* Send data to CPU through UART at baud rate of 115200                        * 
******************************************************************************/
IO_INPUT  IO_J1708_RX (
	.IO_pin                  (J1708_rx             ),
	.data                    (J1708_rx_int         ),
	.clk                     (clk                  )
);

 UART_RX #(
	.CLK_FRQ_MHZ           (CLK_FRQ_MHZ            ),
	.BAUD_RATE             (BAUD_RATE              ),
	.LSB_FIRST             (LSB_FIRST              )
) J1708_RX_UART (
	.uart_rx               (J1708_rx_int           ),
	.data_out              (J1708_rx_byte          ),
	.data_out_valid        (J1708_rx_byte_valid    ),
	.uart_rx_busy          (J1708_rx_busy          ),
	.enable                (enable                 ),
	.rst                   (rst                    ),
	.clk                   (clk                    )
);

J1708_RX_SM J1708_RX_SM (
	.message_length        (rx_message_len_to_fifo ),
	.message_done          (rx_message_done        ),
	.message_byte          (rx_internal_byte       ),
	.message_byte_valid    (rx_internal_byte_valid ),

	.J1708_line_idle       (J1708_line_idle        ),
	.J1708_rx_byte         (J1708_rx_byte          ),
	.J1708_rx_byte_valid   (J1708_rx_byte_valid    ),
	.tx_message_process    (tx_message_process     ),
	.enable                (enable                 ),
	.rst                   (rst                    ),
	.clk                   (clk                    )
);

FIFO_512x8 J1708_RX_DATA_FIFO (
	.data_in               (rx_internal_byte       ),
	.write                 (rx_internal_byte_valid ),
	.read                  (rx_message_byte_read   ),
	.data_out              (rx_message_byte        ),
	.full                  (                       ),
	.empty                 (rx_data_fifo_empty     ),
	.enable                (enable                 ),
	.rst                   (rst                    ),
	.clk                   (clk                    )
);

SMALL_FIFO #(
	.DATA_WIDTH (8),
	.FIFO_DEPTH (10)
) J1708_RX_LEN_FIFO (
	.data_in               (rx_message_len_to_fifo ),
	.write                 (rx_message_done        ),
	.read                  (rx_message_length_read ),
	.data_out              (rx_message_length      ),
	.data_out_valid        (rx_message_length_valid),
	.full                  (                       ),
	.empty                 (rx_fifo_len_empty      ),
	.enable                (enable                 ),
	.rst                   (rst                    ),
	.clk                   (clk                    )
);
assign rx_message_new_byte     = !rx_data_fifo_empty;
assign rx_message_length_exist = !rx_fifo_len_empty ;
assign tx_done                 =  tx_fifo_len_full  ;

// idle counter counts 150 usec from uart_rx is high. 
// if uart_rx falls, counter is restarted
// if state machine is not in IDLE state, counter is begin restart
// line is idle if counter reaches 150 usec
always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{J1708_line_idle, cnt_idle, J1708_data_collision} <= 0;
		cnt_idle_clear <= 1;
	end else if (!enable) begin
		{J1708_line_idle, cnt_idle, J1708_data_collision} <= 0;
		cnt_idle_clear <= 1;
	end else begin
		cnt_idle_clear  <= tx_message_process | ~J1708_rx;
		J1708_line_idle <= (cnt_idle == TIMER_1100_US - 1) || (cnt_idle == TIMER_1100_US);
		
		if      (cnt_idle_clear)								cnt_idle <= 0;
		else if (J1708_line_idle) 								cnt_idle <= TIMER_1100_US;
		else 													cnt_idle <= cnt_idle + 1;

		if (J1708_line_idle)									J1708_data_collision <= 0;
		else if (J1708_rx_byte_valid)							J1708_data_collision <= (J1708_rx_byte != tx_message_byte_out);
	end
end


always @(posedge clk or posedge rst)
begin
	if (rst)
		rx_message_byte_valid <= 0;
	else
		rx_message_byte_valid <= rx_message_byte_read;
end

endmodule
