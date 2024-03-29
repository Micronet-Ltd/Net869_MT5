`timescale 1 ns / 1 ps

module NBOARD869V1_TOP (
	input  CPU_UART_rx   ,				// FPGA UART Rx - MCU UART Tx
	output CPU_UART_tx   ,				// FPGA UART Tx - MCU UART Rx

	input  J1708_UART_rx ,				// FPGA J1708 Rx - J1708 transiver Tx
	output J1708_UART_tx ,				// FPGA J1708 Tx - J1708 transiver Rx
		 
	input  CPU_I2C0_scl   ,				// I2C0 clock
	inout  CPU_I2C0_sda   ,				// I2C0 data

	input  CPU_I2C1_scl   ,				// I2C1 clock
	inout  CPU_I2C1_sda   ,				// I2C1 data

	output led1_red       ,
	output led1_green     ,
	output led1_blue      ,

	output led2_red       ,
	output led2_green     ,
	output led2_blue      ,

	output led3_red       ,
	output led3_green     ,
	output led3_blue      ,

	input  OneWire_Rx     ,				// 1-Wire Rx
	output OneWire_Tx     ,				// 1-Wire Tx

	input  [7:1] gpio_in  ,
	output       gpio_test,
		 
	output irq            ,				// CPU interrupt
	//  input  rst        ,
	input  clk
);

localparam	CLK_FRQ_MHZ          = 26     ,
			CPU_UART_BAUD_RATE   = 115200 ,
			J1708_UART_BAUD_RATE = 9600   ,
			FPGA_I2C_ADDR        = 7'h5E  ;
          

localparam	FPGA_VERSION_TYPE    = "A"    ,
			FPGA_VERSION_MAJOR   =  0     ,
			FPGA_VERSION_MINOR   =  0     ,
			FPGA_VERSION_DEBUG   =  3     ;
        
wire irq_int      ;

wire J1708_enable ;

// J1708 Tx Signals
wire [ 7:0] J1708_tx_message_byte          ;
wire        J1708_tx_message_byte_valid    ;
wire [ 7:0] J1708_tx_message_length        ;
wire [ 2:0] J1708_tx_message_priority      ;
wire        J1708_tx_message_new           ;
wire        J1708_tx_done                  ;

// J1708 Rx Signals
wire        CPU_UART_tx_busy               ;
wire [ 7:0] J1708_rx_message_byte          ;
wire        J1708_rx_message_byte_valid    ;
wire        J1708_rx_message_new_byte      ;
wire        J1708_rx_message_byte_read     ;

wire        J1708_rx_message_length_read   ;
wire [ 7:0] J1708_rx_message_length        ;
wire        J1708_rx_message_length_valid  ;
wire        J1708_rx_message_length_exist  ;

// Register Signals
wire [ 5:0] register_address               ;
wire [31:0] register_data_out              ;
wire [31:0] register_data_in               ;
wire        register_data_in_valid         ;
wire        data_from_register_rd          ;

wire [ 7:0] led1_dutycycle_value ,  led2_dutycycle_value ,  led3_dutycycle_value ;
wire [ 7:0] led1_red_value       ,  led2_red_value       ,  led3_red_value       ;
wire [ 7:0] led1_green_value     ,  led2_green_value     ,  led3_green_value     ;
wire [ 7:0] led1_blue_value      ,  led2_blue_value      ,  led3_blue_value      ;

// 1-Wire Signals
wire        OneWire_dataToSend; 
wire        OneWire_dataRecieved;
wire        OneWire_enable;
wire        OneWire_startResetPulse;
wire        OneWire_startDataWrite;
wire        OneWire_startDataRead;
wire        OneWire_presentStatus;
wire        OneWire_shift;
wire        OneWire_ready;
wire        OneWire_done;

IO_OUTPUT IO_IRQ         (.IO_pin (irq), .data (irq_int), .clk (clk) );

reg  [3:0]  rst_int = 0;
reg         rst = 0;

assign gpio_test    = (&gpio_in) & CPU_I2C0_scl;		// for now, ignore GPIO inputs and I2C0_scl but they must be defined as inputs
assign CPU_I2C0_sda = 1'bz;

always @(posedge clk)
begin
	rst  <= rst_int == 7;

	if (rst_int == 10)		rst_int <= 10;
	else					rst_int <= rst_int + 1;
end

MCU_INTERFACE #(
	.CLK_FRQ_MHZ             (CLK_FRQ_MHZ                  ),
	.BAUD_RATE               (CPU_UART_BAUD_RATE           ),
	.FPGA_I2C_ADDR           (FPGA_I2C_ADDR                )	
) MCU_INTERFACE (
	.uart_rx                 (CPU_UART_rx                  ),
	.uart_tx                 (CPU_UART_tx                  ),			
	.I2C_sda                 (CPU_I2C1_sda                 ),	
	.I2C_scl                 (CPU_I2C1_scl                 ),
	
	.uart_rx_data_out        (J1708_tx_message_byte        ),
	.uart_rx_data_out_valid  (J1708_tx_message_byte_valid  ),
	.uart_tx_data_in         (J1708_rx_message_byte        ),
	.uart_tx_data_wr         (J1708_rx_message_byte_valid  ),
	.uart_tx_busy            (CPU_UART_tx_busy             ),
	.uart_enable             (J1708_enable                 ),

	.register_address        (register_address             ),
	.data_from_register      (register_data_out            ),
	.data_from_register_rd   (data_from_register_rd        ),
	.data_to_register        (register_data_in             ),
	.data_to_register_wr     (register_data_in_valid       ),

	.rst                     (rst                          ),
	.clk                     (clk                          )
);



J1708_TOP #(
	.CLK_FRQ_MHZ             (CLK_FRQ_MHZ                  )
) J1708_TOP (
	.tx_message_byte         (J1708_tx_message_byte        ),
	.tx_message_byte_valid   (J1708_tx_message_byte_valid  ),
                                                           
	.tx_message_length       (J1708_tx_message_length      ),
	.tx_message_priority     (J1708_tx_message_priority    ),
	.tx_message_new          (J1708_tx_message_new         ),
	.tx_done                 (J1708_tx_done                ),
                                                           
	.rx_message_byte_read    (J1708_rx_message_byte_read   ),
	.rx_message_byte         (J1708_rx_message_byte        ),
	.rx_message_byte_valid   (J1708_rx_message_byte_valid  ),
	.rx_message_new_byte     (J1708_rx_message_new_byte    ),

	.rx_message_length_read  (J1708_rx_message_length_read ),
	.rx_message_length       (J1708_rx_message_length      ),
	.rx_message_length_valid (J1708_rx_message_length_valid),
	.rx_message_length_exist (J1708_rx_message_length_exist),
	
	.J1708_tx                (J1708_UART_tx                ),
	.J1708_rx                (J1708_UART_rx                ),
                                                         
	.enable                  (J1708_enable                 ),
	.rst                     (rst                          ),
	.clk                     (clk                          )
);

REGISTERS #(
	.FPGA_VERSION_TYPE       (FPGA_VERSION_TYPE            ),
	.FPGA_VERSION_MAJOR      (FPGA_VERSION_MAJOR           ),
	.FPGA_VERSION_MINOR      (FPGA_VERSION_MINOR           ),
	.FPGA_VERSION_DEBUG      (FPGA_VERSION_DEBUG           ) 
) REGISTERS (                                            
	.data_in                 (register_data_in             ),
	.data_in_valid           (register_data_in_valid       ),
	.address                 (register_address             ),
	.data_out                (register_data_out            ),
	.interrupt               (irq_int                      ),
	.register_read           (data_from_register_rd        ),

	.led1_dutycycle          (led1_dutycycle_value         ),
	.led1_red                (led1_red_value               ),
	.led1_green              (led1_green_value             ),
	.led1_blue               (led1_blue_value              ),
                                                         
	.led2_dutycycle          (led2_dutycycle_value         ),
	.led2_red                (led2_red_value               ),
	.led2_green              (led2_green_value             ),
	.led2_blue               (led2_blue_value              ),
                                                         
	.led3_dutycycle          (led3_dutycycle_value         ),
	.led3_red                (led3_red_value               ),
	.led3_green              (led3_green_value             ),
	.led3_blue               (led3_blue_value              ),
                                                         
	.J1708_enable            (J1708_enable                 ),
	.J1708_TX_len            (J1708_tx_message_length      ),
	.J1708_TX_prio           (J1708_tx_message_priority    ),
	.J1708_TX_new            (J1708_tx_message_new         ),
	.J1708_TX_done           (J1708_tx_done                ),
                        

	.J1708_RX_len_read       (J1708_rx_message_length_read ),
	.J1708_RX_len            (J1708_rx_message_length      ),
	.J1708_RX_len_valid      (J1708_rx_message_length_valid),
	.J1708_RX_len_exist      (J1708_rx_message_length_exist),
	
	.OneWire_dataToSend      (OneWire_dataToSend           ),
	.OneWire_dataRecieved    (OneWire_dataRecieved         ),  
	.OneWire_enable          (OneWire_enable               ),       
	.OneWire_startResetPulse (OneWire_startResetPulse      ),        
	.OneWire_startDataWrite  (OneWire_startDataWrite       ),      
	.OneWire_startDataRead   (OneWire_startDataRead        ),
	.OneWire_presentStatus   (OneWire_presentStatus        ),
	.OneWire_shift           (OneWire_shift                ),
	.OneWire_ready           (OneWire_ready                ),
	.OneWire_done            (OneWire_done                 ),

	.rst                     (rst                          ),
	.clk                     (clk                          )
);

LEDS LEDS (                            

	// LED1 control         
	.led1_DC_value           (led1_dutycycle_value         ),
	.led1_red_value          (led1_red_value               ),
	.led1_green_value        (led1_green_value             ),
	.led1_blue_value         (led1_blue_value              ),
                                                          
	// LED1 output                                        
	.led1_red                (led1_red                     ),
	.led1_green              (led1_green                   ),
	.led1_blue               (led1_blue                    ),

	// LED2 control         
	.led2_DC_value           (led2_dutycycle_value         ),
	.led2_red_value          (led2_red_value               ),
	.led2_green_value        (led2_green_value             ),
	.led2_blue_value         (led2_blue_value              ),
                                                          
	// LED2 output                                        
	.led2_red                (led2_red                     ),
	.led2_green              (led2_green                   ),
	.led2_blue               (led2_blue                    ),
                                                          
	// LED3 control                                       
	.led3_DC_value           (led3_dutycycle_value         ),
	.led3_red_value          (led3_red_value               ),
	.led3_green_value        (led3_green_value             ),
	.led3_blue_value         (led3_blue_value              ),
                                                          
	// LED3 output                                        
	.led3_red                (led3_red                     ),
	.led3_green              (led3_green                   ),
	.led3_blue               (led3_blue                    ),
                                         
	.rst                     (rst                          ),
	.clk                     (clk                          )
);

OneWireMaster #(
	.CLK_FRQ_MHZ             (CLK_FRQ_MHZ                  )
) OneWireMaster (
	.dataToSend              (OneWire_dataToSend           ),
	.dataRecieved            (OneWire_dataRecieved         ),
	.enable                  (OneWire_enable               ),
	.startResetPulse         (OneWire_startResetPulse      ),
	.startDataWrite          (OneWire_startDataWrite       ),
	.startDataRead           (OneWire_startDataRead        ),
	.presentStatus           (OneWire_presentStatus        ),
	.shift                   (OneWire_shift                ),
	.ready                   (OneWire_ready                ),
	.done                    (OneWire_done                 ),
	.oneWireRx               (OneWire_Rx                   ),
	.oneWireTx               (OneWire_Tx                   ),
	.rst                     (rst                          ),
	.clk                     (clk                          )
);

assign J1708_rx_message_byte_read = J1708_rx_message_new_byte & ~CPU_UART_tx_busy;

endmodule
	