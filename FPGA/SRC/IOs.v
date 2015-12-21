`timescale 1 ns / 1 ps

module IO_INPUT (output reg data, input IO_pin, input clk);

reg [2:0] data_sampled = 0;

always @(clk)		{data, data_sampled} <= {data_sampled, IO_pin};

endmodule


module IO_OUTPUT (output reg IO_pin, input data, input clk);

always @(clk)		IO_pin <= data;

endmodule



module IO_INOUT (inout IO_pin, output reg data_in, input data_out, input data_oe, input clk);

reg       data_out_sampled = 0;
reg       data_oe_sampled  = 0;
reg [2:0] data_sampled     = 0;

assign IO_pin = data_oe_sampled ?  1'bz : data_out_sampled;

always @(clk)		
begin
	data_out_sampled        <= data_out ;
	data_oe_sampled         <= data_oe  ;	
	{data_in, data_sampled} <= {data_sampled, IO_pin};
end

endmodule


