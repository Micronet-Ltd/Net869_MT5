/************************************************************************************************
* I2C module controls the I2C interface between MCU and FPGA.                                   *
* The MCU is the master while the FPGA is the slave.                                            *
************************************************************************************************/
`timescale 1 ns / 1 ps

module I2C_SLAVE (
	inout             I2C_sda                    ,
	input             I2C_scl                    ,
	output reg [ 5:0] register_address       = 0 ,
	input      [31:0] data_from_register         ,
	output reg        data_from_register_rd  = 0 ,
	output reg [31:0] data_to_register       = 0 ,
	output reg        data_to_register_wr    = 0 ,
	input             rst                        ,
	input             clk
);

parameter	FPGA_I2C_ADDR =   7'h5E ;

localparam	SDA_OUT_PIPE     = 10 ;

reg                           sda_sampled  = 0 ;
reg                           scl_sampled  = 0 ;

reg                           i2c_start        = 0 ;
reg                           i2c_stop         = 1 ;
reg                           i2c_clk          = 0 ;
reg                           i2c_process      = 0 ;
reg                           i2c_change_data  = 0 ;

reg [35:0]                    data_out         = 0 ;
reg [8:0]                     data_in          = 0 ;
reg [SDA_OUT_PIPE - 1 : 0]    sda_out          = 0 ;
reg                           ack_cycle        = 0 ;
reg                           ack_valid        = 0 ;
reg [8:0]                     bit_cnt          = 0 ;
reg [2:0]                     byte_cnt         = 0 ;
reg                           byte_valid       = 0 ;

reg                           i2c_write_cycle  = 0 ;
reg                           i2c_read_cycle   = 0 ;

Debounce SDA_DEBOUNCE (.in (I2C_sda),	.out (sda_int), 	.rst (rst), 	.clk (clk));
Debounce SCL_DEBOUNCE (.in (I2C_scl),	.out (scl_int), 	.rst (rst), 	.clk (clk));

assign   I2C_sda = sda_out [SDA_OUT_PIPE - 1] ? 1'bz : 1'b0;

always @(posedge clk or posedge rst)
begin
	if (rst)				{sda_sampled, scl_sampled } <= 0;
	else 				  	{sda_sampled, scl_sampled } <= {sda_int, scl_int} ;
end

// input detection
always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{i2c_start, i2c_clk, i2c_change_data, i2c_process, data_in} <= 0;
		i2c_stop <= 1;
	end else begin
		i2c_start       <=  scl_int &  sda_sampled &~ sda_int ;    // falling edge of SDA while SCL is high
		i2c_stop        <=  scl_int & ~sda_sampled &  sda_int ;    // rising  edge of SDA while SCL is high
		i2c_clk         <=  scl_int & ~scl_sampled            ;    // rising  edge of SCL
		i2c_change_data <= ~scl_int &  scl_sampled            ;    // falling edge of SCL

		if (i2c_clk)			data_in     <= {data_in, sda_int};	
		  
		if      (i2c_stop)		i2c_process <=  0;
		else if (i2c_start)		i2c_process <=  1;	  
  end
end

// bit and Byte counters - reset counters when bus is not active or if repeated start
always @(posedge clk or posedge rst)
begin
	if (rst)
		{bit_cnt, byte_cnt} <= 0;
	else begin
		if      ((i2c_change_data & (bit_cnt == 9)) | ~i2c_process | i2c_start)		bit_cnt     <= 0;
		else if (i2c_clk)															bit_cnt     <= bit_cnt  + 1;      

		// reset byte counter when bus is not active or if repeated start
		if      (~i2c_process | i2c_start)											byte_cnt    <= 0;
		else if (byte_valid  )														byte_cnt    <= byte_cnt + 1;      
  end
end    
    
// acknowledge cycle conditions
always @(posedge clk or posedge rst)
begin
	if (rst)
		{ack_cycle, ack_valid, i2c_write_cycle, i2c_read_cycle} <= 0;
	else begin
		// ack cycles is defined after falling edge of 8th bit clock and till falling edge of 9th bit clock
		if      (i2c_change_data & (bit_cnt == 8))												ack_cycle   <=  1;
		else if (i2c_change_data & (bit_cnt == 9))												ack_cycle   <=  0;
      
		// in write cycles - ack may be generated for all bytes
		// in read  cycles - ack may be generated only for device address
		if      (i2c_write_cycle)																ack_valid   <= 1;
		else if (i2c_read_cycle & i2c_change_data & (byte_cnt == 0)) 							ack_valid   <= 1;
		else if (i2c_change_data)																ack_valid   <= 0;
	  
		// write and read cycles are defined by the 8th bit in the 1st byte (device address byte) 
		// only when device address matches the FPGA address
		if (~i2c_process)                                           	                        {i2c_write_cycle, i2c_read_cycle} <= 0;
		else if ((byte_cnt == 0) & (bit_cnt == 8) & (data_in[7:1] == FPGA_I2C_ADDR))			{i2c_write_cycle, i2c_read_cycle} <= {~data_in[0], data_in[0]};
	end
end

always @(posedge clk or posedge rst)
begin
	if (rst)  
		{data_out, sda_out} <= ~0;	  
	else begin
		if (data_from_register_rd)
			data_out <= {data_from_register [ 7 :  0], 1'b0,            // data_out contains also "spare" bits for data acknowledge
						 data_from_register [15 :  8], 1'b0,
						 data_from_register [23 : 16], 1'b0,
						 data_from_register [31 : 24], 1'b1};
		else if (i2c_change_data)                                               
			data_out <= {data_out, 1'b1};
      
		// SDA output forced to be High-Z while bus is :
		//    1. not active 
		//    2. not in read cycles
		//    3. even if in read cycle, while device address (byte 0) is active
		// SDA output forced to be LOW during acknowledge cycle according to ack_valid condtion 
		// during read cycle, SDA output is set to data_out MSB value
		if      (i2c_change_data)										sda_out[0]  <= data_out [35];
		else if (ack_cycle & ack_valid)									sda_out[0]  <= 1'b0;
		else if (~i2c_process | ~i2c_read_cycle | (byte_cnt == 0) )		sda_out[0]  <= 1'b1;

		sda_out [SDA_OUT_PIPE - 1 : 1] <= sda_out [SDA_OUT_PIPE - 2 : 0];
	end
end
      
      
always @(posedge clk or posedge rst)
begin
	if (rst)
		{byte_valid, register_address, data_to_register, data_to_register_wr} <= 0;
	else begin
		byte_valid             <= i2c_clk & (bit_cnt == 8) ;

		data_to_register_wr    <= i2c_write_cycle & i2c_stop   & (byte_cnt != 2);
		data_from_register_rd  <= i2c_read_cycle  & byte_valid & (byte_cnt == 0);

		if (byte_valid) 		data_to_register    <= {data_in[8:1], data_to_register [31:8]};

		if (byte_valid & i2c_write_cycle & (byte_cnt == 1))
			register_address    <= data_in [8:1];
	end
end

endmodule
