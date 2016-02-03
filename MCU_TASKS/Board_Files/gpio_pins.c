
#if 1
#include "gpio_pins.h"
#include <stdbool.h>

const gpio_output_pin_user_config_t outputPins[] = {
	// Power Rail Control
	{ .pinName = POWER_3V3_ENABLE, 		    .config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_5V0_ENABLE, 		    .config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_CHARGE_ENABLE, 		.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_DISCHARGE_ENABLE, 	.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_HEATER_ENABLE,	 	.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

	//Telemetry Output Control
	{ .pinName = GPIO_OUT1, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT2, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT3, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT4, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

	// LED
	{ .pinName = LED_RED, 					.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = LED_GREEN, 				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = LED_BLUE, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{ .pinName = ACC_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//  {.pinName = FPGA_PWR_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// CAN BUS INTERFACE
	{ .pinName = CAN_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN1_TERM_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN2_TERM_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN2_SWC_SELECT,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//  { .pinName = SWC_ENABLE,			    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//  { .pinName = SWC_MODE0,			        .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//  { .pinName = SWC_MODE1,			        .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },


	// USB INTERFACE
	{ .pinName = USB_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_HUB_RSTN,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_OTG_OE,				.config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_OTG_SEL,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// UART INTERFACE
	{ .pinName = UART_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//    { .pinName = RS485_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = FTDI_RSTN,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{.pinName = J1708_ENABLE,				.config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	
//  {.pinName = I2C_SCL, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// CPU control

	{.pinName = CPU_ON_OFF, 				    .config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_POWER_LOSS, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = FORCE_USB_BOOT, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_RF_KILL, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_SPKR_EN, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_MIC_EN, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = BUTTON1,     				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{ .pinName = GPIO_PINS_OUT_OF_RANGE                                                                                                                                                               }
};


const gpio_input_pin_user_config_t inputPins[] = {
	{.pinName = ACC_INT,			.config.isPullEnable = true,	.config.pullSelect = kPortPullUp,	.config.isPassiveFilterEnabled = false,	.config.interrupt = kPortIntFallingEdge },
	//{.pinName = VIB_SENS,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntRisingEdge },
	{.pinName = FPGA_GPIO0,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntRisingEdge },
	{.pinName = SWITCH1   ,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = SWITCH2   ,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = USB_OTG_PWR_REQ,	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },

	{.pinName = UART_MCU2CPU_RX,	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = UART_MCU2CPU_TX,	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = CPU_WATCHDOG,   	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = CPU_STATUS,      	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },


	{ .pinName = GPIO_PINS_OUT_OF_RANGE	}
};


void GPIO_Config( void ) {
	/*
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	CLOCK_SYS_EnablePortClock(PORTC_IDX);
	CLOCK_SYS_EnablePortClock(PORTD_IDX);
*/

	GPIO_DRV_Init(inputPins, outputPins);

	// I2C0 configuration
	PORT_HAL_SetMuxMode(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, true);

	PORT_HAL_SetMuxMode(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, true);

	// UART1 configuration
	PORT_HAL_SetMuxMode(UART_MCU_FPGA_TX_PORT, UART_MCU_FPGA_TX_PIN, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(UART_MCU_FPGA_RX_PORT, UART_MCU_FPGA_RX_PIN, kPortMuxAlt3);

	PORT_HAL_SetMuxMode(EXTAL0_PORT,       EXTAL0_PIN,       kPortPinDisabled);
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_flexcan_io_init
* Returned Value   : 0 or 1 for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to FLEXCAN
*
*END*----------------------------------------------------------------------*/
#if 0
_mqx_int _bsp_flexcan_io_init
(
 uint8_t dev_num
 ){
	volatile CAN_MemMapPtr  can_reg_ptr;
	SIM_MemMapPtr   sim = SIM_BASE_PTR;
	PORT_MemMapPtr  pctl;

	OSC_CR |= OSC_CR_ERCLKEN_MASK;


	switch (dev_num){
	case 0:
		/* Configure GPIO for FlexCAN 0 peripheral function */
		pctl = (PORT_MemMapPtr)PORTA_BASE_PTR;
		pctl->PCR[12] = (PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK);    /* CAN0_TX.A12  */
		pctl->PCR[13] = (PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK);    /* CAN0_RX.A13  */

		/* Enable clock gate to FlexCAN 0 module */
		sim->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

		/* Select the bus clock as CAN engine clock source */
		can_reg_ptr = _bsp_get_flexcan_base_address (0);
		can_reg_ptr->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

		break;
	case 1:
		/* Configure GPIO for FlexCAN 1 peripheral function */
		pctl = (PORT_MemMapPtr)PORTE_BASE_PTR;
		pctl->PCR[24] = (PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK);    /* CAN1_TX.E24  */
		pctl->PCR[25] = (PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK);    /* CAN1_RX.E25  */

		/* Enable clock gate to FlexCAN 1 module */
		sim->SCGC3 |= SIM_SCGC3_FLEXCAN1_MASK;

		/* Select the bus clock as CAN engine clock source */
		can_reg_ptr = _bsp_get_flexcan_base_address (1);
		can_reg_ptr->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

		break;

	default:
		return -1;
	}
	return 0;

}
#endif //#if 0

#endif
