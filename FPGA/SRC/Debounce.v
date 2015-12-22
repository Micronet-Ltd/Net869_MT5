/************************************************************************************************
* Debounce module samples the input pin and modify the pin value only if all the Debounce FFs   *
* are equal to the same value. Otherwise, the pin value remains unchanged.                      *
************************************************************************************************/
`timescale 1 ns / 1 ps

module Debounce (
	input      in  ,
	output reg out ,
	input      rst ,
	input      clk
);

parameter DEBOUNCE_LENGTH = 3;

reg [DEBOUNCE_LENGTH - 1 : 0] input_pipe = 0;

always @(posedge clk or posedge rst)
begin
	if (rst)
		input_pipe <= 0;
	else begin
		input_pipe <= {input_pipe, in};
		
		if      (input_pipe == {DEBOUNCE_LENGTH {1'b1}})		out <= 1   ;
		else if (input_pipe == {DEBOUNCE_LENGTH {1'b0}})		out <= 0   ;
		else                                                    out <= out ;
	end
end

endmodule
