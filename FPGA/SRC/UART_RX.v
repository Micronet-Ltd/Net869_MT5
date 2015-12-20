/************************************************************************************************
* UART_RX module receives 1 bit at a time. When low edge is detected, a START_BIT detection     *
* begins. If Rx signal level is constantly low for half BAUD RATE, then a START_BIT is detected *
* and Rx process begins. The input is sampled every BAUD RATE at the middle of the BAUD RATE,   *
* loaded into shift register.                                                                   *
*                                                                                               *
* when all data bits are loaded in the shift register, a single pulse of data out valid is      *
* being generated, indicating that the data byte is ready and valid.                            *
************************************************************************************************/
`timescale 1 ns / 1 ps

module UART_RX (
	input            uart_rx            ,
	output reg [7:0] data_out        = 0,
	output reg       data_out_valid  = 0,
	output reg   	 uart_rx_busy    = 0,
	input            enable             ,
	input            rst                ,
	input            clk
);
parameter	CLK_FRQ_MHZ   =   24 ,
			BAUD_RATE     = 9600 ,
			LSB_FIRST     =    1 ;

localparam	integer BAUD_RATE_CLK_RATIO = (CLK_FRQ_MHZ * 1e6 / BAUD_RATE);
localparam				UART_DATA_WIDTH	    =   10 ,		// START_BIT, DATA, STOP_BIT
          			START_BIT           = 1'b0 ,
          			STOP_BIT            = 1'b1 ;

reg  [3:0]  uart_rx_cnt     = 0;
reg  [31:0]	baud_rate_cnt   = 0;
reg   		sample_rate     = 0;
reg   		baud_rate_pulse = 0;

always @(posedge clk or posedge rst)
begin
	if (rst)
		{sample_rate, baud_rate_pulse, uart_rx_busy, baud_rate_cnt} <= 0;
	else if (!enable)
		{sample_rate, baud_rate_pulse, uart_rx_busy, baud_rate_cnt} <= 0;
	else  begin
		sample_rate     <=  baud_rate_cnt == (BAUD_RATE_CLK_RATIO / 2);
		baud_rate_pulse <=  baud_rate_cnt == (BAUD_RATE_CLK_RATIO - 1);
		
		if      (sample_rate & ~uart_rx_busy)		uart_rx_busy  <= 1;							// UART starts when START BIT is detected (continues low Rx for half BAUD RATE)
		else if (data_out_valid) 					uart_rx_busy  <= 0;							// UART process ends when data is valid
		
		if      (baud_rate_pulse)					baud_rate_cnt <= 0;							// reset counter according to baud rate
		else if (uart_rx_busy)						baud_rate_cnt <= baud_rate_cnt + 1;			// regular operation, count baud rate
		else if (!uart_rx)							baud_rate_cnt <= baud_rate_cnt + 1;			// detect START BIT when UART is not in process but Rx is low
		else 										baud_rate_cnt <= 0;							// if not START BIT and not in process - no baud rate
		
		if (!uart_rx_busy)							uart_rx_cnt <= 0;							// UART data bits counter
		else if (sample_rate)						uart_rx_cnt <= uart_rx_cnt + 1;
	end
end

always @(posedge clk or posedge rst)
begin
	if (rst)
		{data_out, data_out_valid}  <= 0;
	else if (!enable)
		{data_out, data_out_valid}  <= 0;
	else begin
		data_out_valid <= baud_rate_pulse & (uart_rx_cnt == UART_DATA_WIDTH - 2);				// ignore STOP_BIT
		
		if (sample_rate)
			data_out <= LSB_FIRST ? {data_out, uart_rx} : {uart_rx, data_out[7:1]};				// shift register
	end
end

endmodule
