/***************************************************************************************************
* MCU interface module implements the FPGA - MCU interface containing:                             *
*    1. UART Rx   - receives  data from MCU Tx pin for J1708 interface                             *
*    2. UART Tx   - Transmits data to   MCU Rx pin for J1708 interface                             *
*    3. I2C Salve - for commands and status control                                                *
***************************************************************************************************/
`timescale 1 ns / 1 ps

module MCU_INTERFACE (
	// UART external siganls
	input              uart_rx               ,			// UART Rx - FPGA input
	output             uart_tx               ,			// UART Tx - FPGA output
	
	// UART internal siganls
	output       [7:0] uart_rx_data_out      ,			// recieved Uart byte
	output             uart_rx_data_out_valid,			// recieved Uart byte valid
	input        [7:0] uart_tx_data_in       ,			// data to be transmited
	input              uart_tx_data_wr       ,			// write data to be transmited
	input              uart_tx_busy          ,			// UART TX is busy
	input              uart_enable           ,          // UART Rx and Tx enable

	// I2C external siganls 
	inout             I2C_sda                ,			// I2C data
	input             I2C_scl                ,			// I2C clock
	
	// I2C internal siganls 
	output       [5:0] register_address      ,			// I2C register address
	output      [31:0] data_to_register      ,			// I2C register data input
	output             data_to_register_wr   ,			// I2C register data write
	input       [31:0] data_from_register    ,			// I2C register data output
	output             data_from_register_rd ,			// I2C register data read

	// CONTROL
	input              rst                   ,
	input              clk
);

parameter	CLK_FRQ_MHZ		=     24 ,
			BAUD_RATE		= 115200 ,
			FPGA_I2C_ADDR	=  7'h5E ;
			
localparam  LSB_FIRST		=      1 ;

wire        uart_tx_int			;			// UART TX before pin FF
wire        uart_rx_int			;			// UART RX after  pin FF

IO_INPUT  IO_CPU_UART_TX (
	.IO_pin                  (uart_rx                ),
	.data                    (uart_rx_int            ),
	.clk                     (clk                    )
);
		
UART_RX #(
	.CLK_FRQ_MHZ             (CLK_FRQ_MHZ            ),
	.BAUD_RATE               (BAUD_RATE              ),
	.LSB_FIRST               (LSB_FIRST              )
) CPU_UART_RX_J1708 (                                      
	.uart_rx                 (uart_rx_int            ),
	.data_out                (uart_rx_data_out       ),
	.data_out_valid          (uart_rx_data_out_valid ),
	.uart_rx_busy            (                       ),
	.enable                  (1            ),
	.rst                     (rst                    ),
	.clk                     (clk                    )
);                          

UART_TX #(                  
	.CLK_FRQ_MHZ             (CLK_FRQ_MHZ            ),
	.BAUD_RATE               (BAUD_RATE              ),
	.LSB_FIRST               (LSB_FIRST              )
) CPU_UART_TX_J1708 (                                     
	.data_in                 (uart_tx_data_in        ),
	.data_in_valid           (uart_tx_data_wr        ),
	.uart_tx                 (uart_tx_int            ),
	.uart_tx_busy            (uart_tx_busy           ),
	.enable                  (1            ),	
	.rst                     (rst                    ),
	.clk                     (clk                    )
);

IO_OUTPUT IO_CPU_UART_RX (
	.IO_pin                  (uart_tx                ),
	.data                    (uart_tx_int            ),
	.clk                     (clk                    )
);


I2C_SLAVE #(
	.FPGA_I2C_ADDR           (FPGA_I2C_ADDR          )
) CPU_I2C_SLAVE (                                    
	.I2C_sda                 (I2C_sda                ),	
	.I2C_scl                 (I2C_scl                ),
	.register_address        (register_address       ),
	.data_from_register      (data_from_register     ),
	.data_from_register_rd   (data_from_register_rd  ),
	.data_to_register        (data_to_register       ),
	.data_to_register_wr     (data_to_register_wr    ),
	.rst                     (rst                    ),
	.clk                     (clk                    )
);

endmodule
