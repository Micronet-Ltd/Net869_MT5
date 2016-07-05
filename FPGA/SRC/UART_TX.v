/************************************************************************************************
* UART_TX module receives 1byte (8 bit) data at a time, indicated by data_in_valid. Next data   *
* byte will be received and process only at the end of the current byte process, when UART Tx   *
* block is READY.                                                                               *
*                                                                                               *
* The shift register contains 1 start bit, 8 data bits, 1 stop bit, and 1 additional stop bit   *
* for 1 UART bit delay between 2 following bytes.                                               *
*                                                                                               *
* The bit rate counter divides the FPGA clock rate into UART clock rate, generate an internal   *
* pulse of baud_rate that shifts the data output shift register one bit to the left.            *
************************************************************************************************/
`timescale 1 ns / 1 ps

module UART_TX (
	input [7:0] data_in          ,			// input data to be sent
	input       data_in_valid    ,			// input data valid signal
	output      uart_tx          ,			// UART Tx data output
	output reg  uart_tx_busy  = 0,			// UART sending data bytes including START and STOP BITs 
	input       enable           ,
	input       rst              ,
	input       clk
);

parameter	CLK_FRQ_MHZ   =   24 ,
			BAUD_RATE     = 9600 ,
			LSB_FIRST     =    1 ;

localparam	integer BAUD_RATE_CLK_RATIO = (CLK_FRQ_MHZ * 1e6 / BAUD_RATE) ;
localparam	     			UART_DATA_WIDTH	    =   11 ,		// START_BIT, DATA, STOP_BIT, STOP_BIT
                			START_BIT           = 1'b0 ,
                			STOP_BIT            = 1'b1 ;

reg  [UART_DATA_WIDTH - 1 : 0]	uart_tx_data = {UART_DATA_WIDTH {STOP_BIT}};
reg  [3:0]  uart_tx_cnt     = 0;
reg  [31:0]	baud_rate_cnt   = 0;
reg   		baud_rate_pulse = 0;

assign uart_tx = LSB_FIRST ? uart_tx_data [UART_DATA_WIDTH - 1] : uart_tx_data [0];

always @(posedge clk or posedge rst)
begin
	if (rst)
		{baud_rate_pulse, baud_rate_cnt} <= 0;
	else if (!enable)
		{baud_rate_pulse, baud_rate_cnt} <= 0;
	else  begin
		baud_rate_pulse <=  baud_rate_cnt == (BAUD_RATE_CLK_RATIO - 1);
		baud_rate_cnt   <= (baud_rate_pulse |~ uart_tx_busy) ? 0 : baud_rate_cnt + 1;
	end
end

// when new data is loaded it must be padded with START BIT and STOP BIT, after that it is shifted every baud rate clock
always @(posedge clk or posedge rst)
begin
	if (rst)									uart_tx_data <= {UART_DATA_WIDTH {STOP_BIT}};
	else if (!enable)							uart_tx_data <= {UART_DATA_WIDTH {STOP_BIT}};
	else if (data_in_valid &~ uart_tx_busy) 	uart_tx_data <= LSB_FIRST ? {START_BIT, data_in, STOP_BIT, STOP_BIT} : {STOP_BIT , STOP_BIT, data_in, START_BIT} ;
	else if (baud_rate_pulse)					uart_tx_data <= LSB_FIRST ? {uart_tx_data, STOP_BIT}                  : {STOP_BIT, uart_tx_data [UART_DATA_WIDTH - 1 : 1]};
	
end

always @(posedge clk or posedge rst)
begin
	if (rst)
	   {uart_tx_busy, uart_tx_cnt}  <= 0;
	else if (!enable)
	   {uart_tx_busy, uart_tx_cnt}  <= 0;
	else begin
		if (~uart_tx_busy)								uart_tx_cnt <= 0;								// reset counter 
		else if (baud_rate_pulse)						uart_tx_cnt <= uart_tx_cnt + 1;					// increment counter every new bit

		if (uart_tx_cnt == UART_DATA_WIDTH)				uart_tx_busy <= 0;								// process is done after 2 STOP BIT clocks are sent
		else if (data_in_valid &~ uart_tx_busy)			uart_tx_busy <= 1;								// clear flag while data is being sent
	end
end

endmodule
