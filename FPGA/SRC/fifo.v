`timescale 1 ns / 1 ps

module FIFO_512x8 (
	input  [7 : 0]              data_in     ,
	input                       write       ,
	input                       read        ,
	output [7 : 0]              data_out    ,
	output reg                  full     = 0,
	output reg                  empty    = 1,
	input                       enable      ,
	input                       rst         ,
	input                       clk
);

localparam	DATA_WIDTH = 8,   
			ADDR_WIDTH = 8,
			DEPTH      = (1 << ADDR_WIDTH);


reg [ADDR_WIDTH - 1 : 0] wr_address = 0; 
reg [ADDR_WIDTH - 1 : 0] rd_address = 0;
reg [ADDR_WIDTH - 1 : 0] data_cnt   = 0;

wire rd = read  &~ empty ;
wire wr = write &~ full  ;

always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{wr_address, rd_address, data_cnt, full} <= 0;
		empty <= 1;
	end else begin
		if      (!enable)	      wr_address <= 0;
		else if (wr     )	      wr_address <= wr_address + 1;
		
		if      (!enable)	      rd_address <= 0;
		else if (rd     )	      rd_address <= rd_address + 1;

		// increment data counter if write cycle, decrease if read cycle and don't change if write and read are simultaniously
		if      (!enable )        data_cnt   <= 0           ;
		else if (wr &~ rd)        data_cnt   <= data_cnt + 1;
		else if (rd &~ wr)        data_cnt   <= data_cnt - 1;

		// FIFO becomes full if there is write cycle (without read) and the data is 'almost full'. FULL is cleared on read cycle
		if      (!enable    )                               full  <= 0 ;
		else if (read  &~ wr)                               full  <= 0;
		else if (write &~ read  & (data_cnt == DEPTH - 2))  full  <= 1;

		// FIFO becomes empty if there is read cycle (without write) and the data is 'almost empty'. EMPTY is cleared on write cycle
		if      (!enable    )                               empty  <= 1;
		else if (write &~ rd)                               empty  <= 0;
		else if (read  &~ write & (data_cnt == 1))          empty  <= 1;
	end
end


SB_RAM512x8 ram512X8_inst0 (
	.RDATA	(data_out		),
	.RADDR	(rd_address		),
	.RE		(rd				),
	.RCLKE	(1'b1			),
	.RCLK	(clk			),

	.WDATA	(data_in		),
	.WADDR	(wr_address		),
	.WE		(wr				),
	.WCLKE	(1'b1			),
	.WCLK	(clk			)
);
endmodule

