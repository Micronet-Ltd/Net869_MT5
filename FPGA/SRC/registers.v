/*****************************************************************************************************
* Register module contains all register information configured externally or internally.             *
* Data can be written by processor unit or internally reflecting FPGA status.                        *
* All registers are up to 32 bits width.                                                             *
* The module also generates and interrupt to the processor unit based on interrupt mask register     *
*                                                                                                    *
* The supported registers are:                                                                       *
*   FPGA VERSION     - Read only register. Contains the version of FPGA                              *
*                                                                                                    *
*   INTERRUPT STATUS - Read only register. Contains the State of all interrupt sources, regardless   *
*                      of generating interrupt or not. Each bit flag is independent rises when       *
*                      interrupt request starts and remain at that state till register is read.      *
*                                                                                                    *
*   INTERRUPT MASK   - Read\Write register. All bit flags order is the same as the interrupt status  *
*                      register. '1' enables interrupt when occurs, while '0' masks interrupt.       *
*                                                                                                    *
*   LED(x) RGB       - 3 Read\Write RGB registers for 3 LEDs. Each byte in the Register represents   *
*                      the RGB code value to generate a color.                                       *
*                      MSB contains the DC of the LED to blink. 0 - LED is off, 255 - LED is on.     *
*                                                                                                    *
*   J1708 TX         - Read\Write register. Contains the data message length and priority.           *
*                      The MSB is internal read only bit indicates whether the J1708 module is ready *
*                      to send new message (1) or not (0),i.e., previous message is still in process *
*                                                                                                    *
*   J1708 RX         - Read only register. Contains the data message length.                         *
*                      The MSB is internal read only bit indicates whether a new message has received*
*                      since last time register was read (1) or no new data exists (0).              *
*****************************************************************************************************/
`timescale 1 ns / 1 ps

module REGISTERS (
	input        [31:0] data_in            ,
	input               data_in_valid      ,
	input        [ 5:0] address            ,
	output reg   [31:0] data_out       = 0 ,
	output reg          interrupt      = 0 ,
	input               register_read      ,

	output reg   [ 7:0] led2_dutycycle = 0 ,
	output reg   [ 7:0] led2_red       = 0 ,
	output reg   [ 7:0] led2_green     = 0 ,
	output reg   [ 7:0] led2_blue      = 0 ,

	output reg   [ 7:0] led3_dutycycle = 0 ,
	output reg   [ 7:0] led3_red       = 0 ,
	output reg   [ 7:0] led3_green     = 0 ,
	output reg   [ 7:0] led3_blue      = 0 ,

	
	output reg          J1708_enable   = 0 ,
	output reg   [ 7:0] J1708_TX_len   = 0 ,
	output reg   [ 2:0] J1708_TX_prio  = 0 ,
	output reg          J1708_TX_new   = 0 ,
	input               J1708_TX_done      ,

	input        [ 7:0] J1708_RX_len       ,			// number of received bytes including MID and Checksum
	input               J1708_RX_len_valid ,			// number of received bytes 
	input               J1708_RX_len_exist ,			// new message length exists
	output              J1708_RX_len_read  ,			// read new message length from FIFO

	input               rst,
	input               clk
);

parameter  FPGA_VERSION_TYPE  = "A" ,
           FPGA_VERSION_MAJOR =  0  ,
           FPGA_VERSION_MINOR =  0  ,
           FPGA_VERSION_DEBUG =  0  ;

localparam ADDR_VERSION     = 6'h00,
           ADDR_IRQ_STATUS  = 6'h01,
           ADDR_IRQ_MASK    = 6'h02,

           ADDR_LED2_RGB    = 6'h10,
           ADDR_LED3_RGB    = 6'h11,
           
           ADDR_J1708_CNTRL = 6'h20,
           ADDR_J1708_TX    = 6'h21,
           ADDR_J1708_RX    = 6'h22;
		   
wire [ 7:0] fpga_version_type  = FPGA_VERSION_TYPE  ;
wire [ 7:0] fpga_version_major = FPGA_VERSION_MAJOR ;
wire [ 7:0] fpga_version_minor = FPGA_VERSION_MINOR ;
wire [ 7:0] fpga_version_debug = FPGA_VERSION_DEBUG ;

reg  [ 7:0] J1708_RX_len_register = 0 ;
reg         J1708_RX_new          = 0 ;
reg  [31:0] irq_status            = 0 ;
reg  [31:0] irq_mask              = 0 ;	
wire [31:0] irq_trigger               ;	

// set register value
always @(posedge clk or posedge rst)
begin
	if (rst) begin
		{led2_dutycycle, led2_red, led2_green, led2_blue } <= 0;
		{led3_dutycycle, led3_red, led3_green, led3_blue } <= 0;
		{J1708_enable,   J1708_TX_len, J1708_TX_prio     } <= 0;
	end else if (data_in_valid)
		case (address)
			ADDR_IRQ_MASK     :  irq_mask                                          <= data_in ;
			ADDR_LED2_RGB     : {led2_dutycycle, led2_red, led2_green, led2_blue } <= data_in ;
			ADDR_LED3_RGB     : {led3_dutycycle, led3_red, led3_green, led3_blue } <= data_in ;
			ADDR_J1708_CNTRL  : begin
								 J1708_TX_prio                                     <= data_in [7:5];
								 J1708_enable                                      <= data_in [0];
								end
			ADDR_J1708_TX     : J1708_TX_len                                       <= data_in [7:0] ;
		endcase
end

// read register value
always @(posedge clk or posedge rst)
begin
	if (rst)
	  data_out <= 0;
	else
	 case (address)
	     ADDR_VERSION    : data_out <= {fpga_version_type, fpga_version_major, fpga_version_minor, fpga_version_debug};
	     ADDR_IRQ_STATUS : data_out <= irq_status;
 	     ADDR_IRQ_MASK   : data_out <= irq_mask;
	     ADDR_LED2_RGB   : data_out <= {led2_dutycycle, led2_red, led2_green, led2_blue } ;
	     ADDR_LED3_RGB   : data_out <= {led3_dutycycle, led3_red, led3_green, led3_blue } ;
		 
		 ADDR_J1708_CNTRL: begin
	                       data_out [31 :  8] <= 0              ;
	                       data_out [7  :  5] <= J1708_TX_prio  ;
	                       data_out [4  :  1] <= 0              ;
	                       data_out [0]       <= J1708_enable   ;
	                      end
						  
	     ADDR_J1708_TX   : begin
	                       data_out [31]      <= J1708_TX_done  ;
	                       data_out [30 :  8] <= 0              ;
	                       data_out [ 7 :  0] <= J1708_TX_len   ;
	                      end
	                      
	     ADDR_J1708_RX   : begin
	                       data_out [31]      <= J1708_RX_new   ;
	                       data_out [30 :  8] <= 0              ;
	                       data_out [ 7 :  0] <= J1708_RX_len_register   ;
	                      end

      default	       :  data_out <= 0 ;
	  endcase
end

// J1708 new message status
always @(posedge clk or posedge rst)
begin
	if (rst)
		{J1708_TX_new, J1708_RX_new, J1708_RX_len_register} <= 0 ;
	else begin
		if (J1708_RX_len_valid)                          			J1708_RX_len_register <= J1708_RX_len ;

		J1708_TX_new <= data_in_valid & (address == ADDR_J1708_TX);

		if (register_read && (address == ADDR_J1708_RX))            J1708_RX_new  <= 0 ;
		else if (J1708_RX_len_valid)                     			J1708_RX_new  <= 1 ;

	end
end

assign J1708_RX_len_read = J1708_RX_len_exist & ~J1708_RX_new &~ J1708_RX_len_valid  ;	// sigle cycle read pulse

assign irq_trigger [0]    = J1708_RX_len_valid;
assign irq_trigger [31:1] = 0;

// interrupt 
always @(posedge clk or posedge rst)
begin
	if (rst) 
		{interrupt, irq_status} <= 0;
	else begin
		interrupt <= |(irq_trigger & irq_mask);

		if   (register_read && (address == ADDR_IRQ_STATUS))		irq_status    <= 0 ;
		else if (irq_trigger[0])									irq_status[0] <= 1 ;
	end
end
endmodule
