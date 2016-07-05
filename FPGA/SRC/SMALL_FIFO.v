/***************************************************************************************************
* SMALL_FIFO module implements a small size FIFO based on FFs instead of memory blocks.            *
*                                                                                                  *
* The assumption is that write and read are not in burst modes but only at a signle cycles.        *
* There cannot be 2 following write operations or 2 following read operations. However, the module *
* supports simultaneously read and write requests or write after read. Read after write is         *
* supported if FIFO was not empty (see next paragraph).                                            *
*                                                                                                  *
* The small fifo implementation is implemented by using as less as possible FFs. The penalty for   *
* that is latency. A word is always being written to the last cell and read from the first cell.   *
* Every clock, the data word is being shifted to the next empty cell. Therefore, the latency from  *
* writing the first word to an empty FIFO till empty flag is being cleared is the FIFO DEPTH -     *
* number of cells in FIFO. However, the empty signals sets immediately when the last word is read  *
* from the FIFO.                                                                                   *
*                                                                                                  *
* The full signal is set immediately  when the data word is written to the last cell and there is  *
* no free cell to be shifted to and cleared immediately  when there is a free cell                 *
***************************************************************************************************/
`timescale 1 ns / 1 ps

module SMALL_FIFO #(parameter 
	DATA_WIDTH = 8,
	FIFO_DEPTH = 20
) (
	input        [DATA_WIDTH - 1 : 0]  data_in            ,
	input                              write              ,
	input                              read               ,
	output reg   [DATA_WIDTH - 1 : 0]  data_out       = 0 ,
	output reg                         data_out_valid = 0 ,
	
	output                             empty              ,
	output                             full               ,

	input                              enable             ,
	input                              rst,
	input                              clk
);

integer i;
reg [DATA_WIDTH - 1 : 0] mem_data  [0 : FIFO_DEPTH];
reg                      mem_wr_en [0 : FIFO_DEPTH];

wire   wr = write &~ full ;
wire   rd = read &~ empty;

assign empty = ~mem_wr_en [0];
assign full  =  mem_wr_en [FIFO_DEPTH - 1] &  mem_wr_en [FIFO_DEPTH - 2] ;

always @(posedge clk or posedge rst)
begin
	if (rst)
		for (i = 0; i <= FIFO_DEPTH; i = i + 1)
			{mem_wr_en [i], mem_data [i]} <= 0;
	else if (!enable)
		for (i = 0; i <= FIFO_DEPTH; i = i + 1)
			mem_wr_en [i] <= 0;	
	else begin			
		// always copy next cell except when current cells is marked with data
		if (rd || (mem_wr_en [0] == 0)) begin
			mem_data  [0] <= mem_data  [1];
			mem_wr_en [0] <= mem_wr_en [1];
		end
		
		// always copy next cell except when both previous and current cells are marked with data
		for (i = 1; i < FIFO_DEPTH; i = i + 1) begin
			if (rd || (mem_wr_en  [i-1] == 0) || (mem_wr_en [i] == 0)) begin
				mem_data  [i] <= mem_data  [i + 1];
				mem_wr_en [i] <= mem_wr_en [i + 1];
			end	
		end
		
		// last cell
		mem_data  [FIFO_DEPTH] <= data_in;
		mem_wr_en [FIFO_DEPTH] <= wr;
	end
end

always @(posedge clk or posedge rst)
begin
	if (rst)
		{data_out_valid, data_out} <= 0;
	else begin
		data_out_valid <= enable & rd;
		
		if (enable & rd)
			data_out <= mem_data[0];
	end
end

endmodule
