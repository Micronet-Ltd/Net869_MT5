
#ifndef __gpio_pins_H
#define __gpio_pins_H

#include "fsl_gpio_driver.h"

void GPIO_Config( void );

enum _gpio_pins_pinNames {
    // SWD signals
    //SWD_CLK								= GPIO_MAKE_PIN(GPIOA_IDX,  0),
    //SWD_DIO								= GPIO_MAKE_PIN(GPIOA_IDX,  3),
    //SWD_RST								(RESET_B),

    // POWRE RAIL CONTROL
    POWER_3V3_ENABLE					= GPIO_MAKE_PIN(GPIOA_IDX, 14),
    POWER_5V0_ENABLE					= GPIO_MAKE_PIN(GPIOA_IDX, 11),
    POWER_CHARGE_ENABLE					= GPIO_MAKE_PIN(GPIOA_IDX, 15),
    POWER_DISCHARGE_ENABLE				= GPIO_MAKE_PIN(GPIOB_IDX,  8),

    // TELEMETRY OUTPUTS CONTROL
    GPIO_OUT1							= GPIO_MAKE_PIN(GPIOB_IDX, 11),
    GPIO_OUT2							= GPIO_MAKE_PIN(GPIOB_IDX, 16),
    GPIO_OUT3							= GPIO_MAKE_PIN(GPIOB_IDX, 17),
    GPIO_OUT4							= GPIO_MAKE_PIN(GPIOB_IDX,  5),

    // BUTTONS SWITCHES and LEDS
    BUTTON1								= GPIO_MAKE_PIN(GPIOA_IDX, 25),
    BUTTON2								= GPIO_MAKE_PIN(GPIOA_IDX, 26),
    BUTTON3								= GPIO_MAKE_PIN(GPIOC_IDX,  3),

    OTG_ID								= GPIO_MAKE_PIN(GPIOE_IDX, 11),
	CPU_OTG_ID								= GPIO_MAKE_PIN(GPIOE_IDX, 12),

    LED_RED								= GPIO_MAKE_PIN(GPIOA_IDX,  4),
    LED_GREEN							= GPIO_MAKE_PIN(GPIOA_IDX,  5),
    LED_BLUE							= GPIO_MAKE_PIN(GPIOA_IDX,  7),

    // FPGA INTERFACE
    FPGA_PWR_ENABLE						= GPIO_MAKE_PIN(GPIOA_IDX, 13),
    FPGA_RESET							= GPIO_MAKE_PIN(GPIOE_IDX, 10),
    FPGA_RSTB							= GPIO_MAKE_PIN(GPIOA_IDX, 12),
    FPGA_DONE							= GPIO_MAKE_PIN(GPIOE_IDX,  9),
    FPGA_GPIO0							= GPIO_MAKE_PIN(GPIOC_IDX,  8),
    FPGA_GPIO1							= GPIO_MAKE_PIN(GPIOC_IDX,  9),
    FPGA_GPIO2							= GPIO_MAKE_PIN(GPIOE_IDX,  2),
    FPGA_GPIO3							= GPIO_MAKE_PIN(GPIOE_IDX,  3),

    // FPGA SPI MEMORY INTERFACE CHANNEL 2
    SPI_MEM_CS							= GPIO_MAKE_PIN(GPIOB_IDX, 20),
    SPI_MEM_CLK							= GPIO_MAKE_PIN(GPIOB_IDX, 21),
    SPI_MEM_DI							= GPIO_MAKE_PIN(GPIOB_IDX, 22),
    SPI_MEM_DO							= GPIO_MAKE_PIN(GPIOB_IDX, 23),

    // CAN BUS INTERFACE
	CAN1_J1708_PWR_ENABLE				= GPIO_MAKE_PIN(GPIOC_IDX,  0),
	CAN2_SWC_PWR_ENABLE					= GPIO_MAKE_PIN(GPIOC_IDX,  4),
    CAN1_TERM_ENABLE					= GPIO_MAKE_PIN(GPIOE_IDX, 24),
    CAN2_TERM_ENABLE					= GPIO_MAKE_PIN(GPIOA_IDX, 10),
    CAN2_SWC_SELECT						= GPIO_MAKE_PIN(GPIOC_IDX,  6),
    J1708_ENABLE						= GPIO_MAKE_PIN(GPIOA_IDX,  9),
    SWC_ENABLE							= GPIO_MAKE_PIN(GPIOA_IDX,  8),
    SWC_MODE0							= GPIO_MAKE_PIN(GPIOC_IDX, 12),
    SWC_MODE1							= GPIO_MAKE_PIN(GPIOC_IDX, 13),

    CAN1_RX_MCU							= GPIO_MAKE_PIN(GPIOB_IDX, 19),
    CAN1_TX_MCU							= GPIO_MAKE_PIN(GPIOB_IDX, 18),
    CAN2_RX_MCU							= GPIO_MAKE_PIN(GPIOC_IDX, 16),
    CAN2_TX_MCU							= GPIO_MAKE_PIN(GPIOC_IDX, 17),

    // USB INTERFACE
    USB_ENABLE							= GPIO_MAKE_PIN(GPIOE_IDX,  8),
    USB_HUB_RSTN						= GPIO_MAKE_PIN(GPIOC_IDX, 18),
    USB_OTG_OE							= GPIO_MAKE_PIN(GPIOE_IDX,  6),
    USB_OTG_SEL							= GPIO_MAKE_PIN(GPIOE_IDX,  7),
	USB_OTG_PWR_REQ						= GPIO_MAKE_PIN(GPIOE_IDX, 28),
    //	USB_DN							(USB0_DM),;
    //	USB_DP							(USB0_DP),;

    // UART INTERFACE
    UART_ENABLE							= GPIO_MAKE_PIN(GPIOB_IDX,  1),
    //RS485_ENABLE						= GPIO_MAKE_PIN(GPIOA_IDX,  2),
    FTDI_RSTN							= GPIO_MAKE_PIN(GPIOC_IDX, 19),

    // UART CHANNEL 4
    //UART_RX								= GPIO_MAKE_PIN(GPIOC_IDX, 14),
    //UART_TX								= GPIO_MAKE_PIN(GPIOC_IDX, 15),

    // UART CHANNEL 3
    UART_MCU2CPU_RX						= GPIO_MAKE_PIN(GPIOE_IDX,  5),
    UART_MCU2CPU_TX						= GPIO_MAKE_PIN(GPIOE_IDX,  4),

    // UART CHANNEL 1
//    UART_MCU_FPGA_CTS					= GPIO_MAKE_PIN(GPIOE_IDX,  2),
//    UART_MCU_FPGA_RTS					= GPIO_MAKE_PIN(GPIOE_IDX,  3),
    UART_MCU_FPGA_RX					= GPIO_MAKE_PIN(GPIOE_IDX,  1),
    UART_MCU_FPGA_TX					= GPIO_MAKE_PIN(GPIOE_IDX,  0),

    // AUDIO CONTROL
	SPKR_RIGHT_EN						= GPIO_MAKE_PIN(GPIOA_IDX,  6),
	SPKR_LEFT_EN						= GPIO_MAKE_PIN(GPIOA_IDX, 28),
    SPKR_EXT_EN							= GPIO_MAKE_PIN(GPIOA_IDX, 27),
    MIC_SEL								= GPIO_MAKE_PIN(GPIOC_IDX,  7),

    // ACCELOREMTER & GYRO INTERFACE
    ACC_ENABLE							= GPIO_MAKE_PIN(GPIOA_IDX, 19),
    ACC_INT								= GPIO_MAKE_PIN(GPIOA_IDX, 24),
    VIB_SENS							= GPIO_MAKE_PIN(GPIOA_IDX, 29),
    GYRO_INT							= GPIO_MAKE_PIN(GPIOA_IDX, 16),

    // CPU INTERFACE
    CPU_ON_OFF							= GPIO_MAKE_PIN(GPIOB_IDX,  7),
    CPU_SPKR_EN 						= GPIO_MAKE_PIN(GPIOB_IDX, 9),  //swapped CPU_POWER_LOSS with CPU_SPKR_EN in schematic
    FORCE_USB_BOOT						= GPIO_MAKE_PIN(GPIOC_IDX,  1),
    CPU_RF_KILL							= GPIO_MAKE_PIN(GPIOC_IDX,  5),
    CPU_WATCHDOG						= GPIO_MAKE_PIN(GPIOB_IDX, 10),
    CPU_STATUS							= GPIO_MAKE_PIN(GPIOC_IDX,  2),
	CPU_POWER_LOSS					    = GPIO_MAKE_PIN(GPIOE_IDX, 25), //swapped CPU_SPKR_EN with CPU_POWER_LOSS in schematic
	CPU_MIC_EN						    = GPIO_MAKE_PIN(GPIOE_IDX, 26),



    PORT_D_SPARE1						= GPIO_MAKE_PIN(GPIOD_IDX,  0),
    PORT_D_SPARE2						= GPIO_MAKE_PIN(GPIOD_IDX,  1),
    PORT_D_SPARE3						= GPIO_MAKE_PIN(GPIOD_IDX,  2),
    PORT_D_SPARE4						= GPIO_MAKE_PIN(GPIOD_IDX,  3),
    PORT_D_SPARE5						= GPIO_MAKE_PIN(GPIOD_IDX,  4),
    PORT_D_SPARE6						= GPIO_MAKE_PIN(GPIOD_IDX,  5),
    PORT_D_SPARE7						= GPIO_MAKE_PIN(GPIOD_IDX,  6),
    PORT_D_SPARE8						= GPIO_MAKE_PIN(GPIOD_IDX,  7),
    PORT_D_SPARE9						= GPIO_MAKE_PIN(GPIOD_IDX,  8),
    PORT_D_SPARE10						= GPIO_MAKE_PIN(GPIOD_IDX,  9),
    PORT_D_SPARE11						= GPIO_MAKE_PIN(GPIOD_IDX, 10),
    PORT_D_SPARE12						= GPIO_MAKE_PIN(GPIOD_IDX, 11),
    PORT_D_SPARE13						= GPIO_MAKE_PIN(GPIOD_IDX, 12),
    PORT_D_SPARE14						= GPIO_MAKE_PIN(GPIOD_IDX, 13),
    PORT_D_SPARE15						= GPIO_MAKE_PIN(GPIOD_IDX, 14),
    PORT_D_SPARE16						= GPIO_MAKE_PIN(GPIOD_IDX, 15),

//    PORT_E_SPARE1						= GPIO_MAKE_PIN(GPIOE_IDX, 24),
    PORT_E_SPARE4						= GPIO_MAKE_PIN(GPIOE_IDX, 27),


//	I2C_SDA = GPIO_MAKE_PIN(GPIOB_IDX, 2),
//	I2C_SCL = GPIO_MAKE_PIN(GPIOB_IDX, 3),
};

// I2C INTERFACE - CHANNEL 2
#define I2C0_SDA_GPIO_PORT			PORTB
#define I2C0_SDA_GPIO_PIN			3

#define I2C0_SCL_GPIO_PORT			PORTB
#define I2C0_SCL_GPIO_PIN			2

#define I2C1_SDA_GPIO_PORT			PORTC
#define I2C1_SDA_GPIO_PIN			11

#define I2C1_SCL_GPIO_PORT			PORTC
#define I2C1_SCL_GPIO_PIN			10

//	UART MCU FPGA INTERFACE  - CHANNEL 1
#define UART_MCU_FPGA_TX_PORT			PORTE
#define UART_MCU_FPGA_TX_PIN			0

#define UART_MCU_FPGA_RX_PORT			PORTE
#define UART_MCU_FPGA_RX_PIN			1

#define EXTAL0_PORT   				PORTA
#define EXTAL0_PIN    				18


// ANALOG INPUTS
#define ADC_POWER_IN					kAdc16Chn17
#define ADC_POWER_VCAP					kAdc16Chn8
#define ADC_TEMPERATURE					kAdc16Chn16
#define ADC_CABLE_TYPE					kAdc16Chn10

// TELEMETRY INPUTS (ANALOG INPUTS),
#define ADC_ANALOG_IN1					kAdc16Chn12
#define ADC_GPIO_IN1					kAdc16Chn20			// dm1
#define ADC_GPIO_IN2					kAdc16Chn1			// dp1
#define ADC_GPIO_IN3					kAdc16Chn3			// dp3 and dm3
#define ADC_GPIO_IN4					kAdc16Chn3			// dp3
#define ADC_GPIO_IN5					kAdc16Chn19			// dm0
#define ADC_GPIO_IN6					kAdc16Chn0			// dp0
#define ADC_GPIO_IN7					kAdc16Chn18

#ifdef __cplusplus
extern "C"
{
#endif

#if 0
//_mqx_int _bsp_flexcan_io_init (uint8_t);
#endif

#ifdef __cplusplus
}
#endif


#endif //#ifndef __gpio_pins_H


