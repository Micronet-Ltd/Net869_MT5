
#if 1
#include "gpio_pins.h"
#include <stdbool.h>
#include <mqx.h>

const gpio_output_pin_user_config_t outputPins[] = {
	// Power Rail Control
	{ .pinName = POWER_3V3_ENABLE, 		    .config.outputLogic = 1,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_5V0_ENABLE, 		    .config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_CHARGE_ENABLE, 		.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = POWER_DISCHARGE_ENABLE, 	.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

	//Telemetry Output Control
	{ .pinName = GPIO_OUT1, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT2, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT3, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
	{ .pinName = GPIO_OUT4, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

	// LED
	{ .pinName = LED_RED, 					.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = LED_GREEN, 				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = LED_BLUE, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{ .pinName = ACC_VIB_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = FPGA_PWR_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// CAN BUS INTERFACE
	{ .pinName = CAN1_J1708_PWR_ENABLE,		.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN2_SWC_PWR_ENABLE,		.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{ .pinName = CAN1_TERM_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN2_TERM_ENABLE,			.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CAN2_SWC_SELECT,			.config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = SWC_ENABLE,			    .config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = SWC_MODE0,			        .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = SWC_MODE1,			        .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },


	// USB INTERFACE
	{ .pinName = USB_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_HUB_RSTN,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_OTG_OE,				.config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = USB_OTG_SEL,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = CPU_OTG_ID,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// UART INTERFACE
	{ .pinName = UART_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
    { .pinName = RS485_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{ .pinName = FTDI_RSTN,				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{.pinName = FPGA_RSTB,					.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = J1708_ENABLE,				.config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

//  {.pinName = I2C_SCL, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	// CPU control

	{.pinName = CPU_ON_OFF, 				    .config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_POWER_LOSS, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = FORCE_USB_BOOT, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
// EYAL_0523		{.pinName = CPU_RF_KILL, 				    .config.outputLogic = 1,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = CPU_MIC_EN, 				    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },


	// AUDIO
	{.pinName = SPKR_LEFT_EN,                   .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = SPKR_RIGHT_EN,                  .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
	{.pinName = SPKR_EXT_EN,                    .config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

	{ .pinName = GPIO_PINS_OUT_OF_RANGE                                                                                                                                                               }
};


const gpio_input_pin_user_config_t inputPins[] = {
	{.pinName = ACC_INT,			.config.isPullEnable = true,	.config.pullSelect = kPortPullUp,	.config.isPassiveFilterEnabled = false,	.config.interrupt = kPortIntDisabled },//kPortIntLogicZero },
	{.pinName = VIB_SENS,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntRisingEdge },
	{.pinName = FPGA_GPIO0,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled },
	{.pinName = FPGA_DONE,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = OTG_ID   ,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = true,  .config.interrupt = kPortIntDisabled  },
	{.pinName = CPU_INT,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntEitherEdge  }, //interrupt needs to disabled by default coz MSM is not ON yet
	{.pinName = UART_MCU2CPU_RX,	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = UART_MCU2CPU_TX,	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = CPU_WATCHDOG,   	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  }, //interrupt needs to disabled by default coz MSM is not ON yet
	{.pinName = CPU_STATUS,      	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  }, //interrupt needs to disabled by default coz MSM is not ON yet
	{.pinName = BUTTON1,     		.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  },
	{.pinName = CPU_SPKR_EN,        .config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled  }, //interrupt needs to disabled by default coz MSM is not ON yet

// EYAL_0523
	{.pinName = CPU_RF_KILL,        .config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntDisabled    },

	{ .pinName = GPIO_PINS_OUT_OF_RANGE	}
};

void configure_msm_gpio_input_pins(bool interrupt_disable)
{
	gpio_input_pin_user_config_t input_pins_msm[] = {
		{.pinName = CPU_WATCHDOG,   	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntRisingEdge  },
		{.pinName = CPU_STATUS,      	.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntEitherEdge  },
		{.pinName = CPU_SPKR_EN,        .config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntEitherEdge  },
		{.pinName = CPU_INT,			.config.isPullEnable = false,	.config.pullSelect = kPortPullUp,  .config.isPassiveFilterEnabled = false,  .config.interrupt = kPortIntEitherEdge  },
		{ .pinName = GPIO_PINS_OUT_OF_RANGE	}
	};

	gpio_input_pin_user_config_t * inputPins = input_pins_msm;
	/* Initialize input pins.*/
	while (inputPins->pinName != GPIO_PINS_OUT_OF_RANGE)
	{
		if (interrupt_disable)
		{
			inputPins->config.interrupt = kPortIntDisabled;
		}
		GPIO_DRV_InputPinInit(inputPins++);
	}
}

/* Enables or disables the MSM/A8 power rail */
/* IO pin interrupts from the MSM have to be disabled when the MSM power rail is turned off */
void enable_msm_power(bool enable)
{
	if (enable)
	{
		GPIO_DRV_SetPinOutput(POWER_5V0_ENABLE);
		configure_msm_gpio_input_pins(false);
	}
	else
	{
		GPIO_DRV_ClearPinOutput(POWER_5V0_ENABLE);
		configure_msm_gpio_input_pins(true);
	}
}

/* I2C_reset_bus: If the SDA line is stuck in the low state, we try to send some 
additional clocks and generate a stop condition

http://www.analog.com/media/en/technical-documentation/application-notes/54305147357414AN686_0.pdf

NOTE: Since the i2c port is changed to a GPIO for bitbanging, 
You need to manually set the port back to i2c mode and reinit the i2c port
*/
bool I2C_reset_bus(uint32_t sda_pin_name, uint32_t scl_pin_name)
{
	uint8_t i;
	const gpio_input_pin_user_config_t input_pin_SDA =
	{
		  .pinName = sda_pin_name,
		  .config.isPullEnable = true,
		  .config.pullSelect = kPortPullUp,
		  .config.isPassiveFilterEnabled = false,
		  .config.interrupt = kPortIntDisabled,
	};
	const gpio_input_pin_user_config_t input_pin_SCL =
	{
		  .pinName = scl_pin_name,
		  .config.isPullEnable = true,
		  .config.pullSelect = kPortPullUp,
		  .config.isPassiveFilterEnabled = false,
		  .config.interrupt = kPortIntDisabled,
	};
	
	const gpio_output_pin_user_config_t output_pin_SDA =
	{
		.pinName = sda_pin_name,	    
		.config.outputLogic = 1,
		.config.slewRate = kPortFastSlewRate,
		.config.isOpenDrainEnabled = false,  
		.config.driveStrength = kPortHighDriveStrength,
	};
	const gpio_output_pin_user_config_t output_pin_SCL =
	{
		.pinName = scl_pin_name,
		.config.outputLogic = 1,
		.config.slewRate = kPortFastSlewRate,
		.config.isOpenDrainEnabled = false,  
		.config.driveStrength = kPortHighDriveStrength,
	};

	/* configure i2c lines as GPInputs */
	GPIO_DRV_InputPinInit(&input_pin_SDA);
	GPIO_DRV_InputPinInit(&input_pin_SCL);
  	
	/* Check if they are already both high */ 
	if(GPIO_DRV_ReadPinInput(sda_pin_name) && GPIO_DRV_ReadPinInput(scl_pin_name)) 
	{
		return true;
	}

	_time_delay (10);
	if(!GPIO_DRV_ReadPinInput(scl_pin_name)) 
	{
		return false; /* SCL held low externally, nothing we can do */
	}
	
	for(i = 0; i<9; i++) /* up to 9 clocks until SDA goes high */
	{
		GPIO_DRV_OutputPinInit(&output_pin_SCL);
		GPIO_DRV_WritePinOutput(scl_pin_name, 0);
		_time_delay (10);
		GPIO_DRV_InputPinInit(&input_pin_SCL);
		_time_delay (10);
		if(GPIO_DRV_ReadPinInput(sda_pin_name)) 
		{
			break; /* finally SDA high so we can generate a STOP */
		}
	}
		
	if(!GPIO_DRV_ReadPinInput(sda_pin_name)) 
	{
		return false; /* after 9 clocks still nothing */
	}
	
	return (GPIO_DRV_ReadPinInput(sda_pin_name) && GPIO_DRV_ReadPinInput(scl_pin_name)); /* both high then we succeeded */
}


void GPIO_Config( void ) {
	/*
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	CLOCK_SYS_EnablePortClock(PORTC_IDX);
	CLOCK_SYS_EnablePortClock(PORTD_IDX);
*/

	bool ret;
	GPIO_DRV_Init(inputPins, outputPins);
	I2C_reset_bus(I2C0_SDA, I2C0_SCL);
	I2C_reset_bus(I2C1_SDA, I2C1_SCL);

	// I2C0 configuration
	PORT_HAL_SetMuxMode     (I2C0_SDA_GPIO_PORT, I2C0_SDA_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C0_SDA_GPIO_PORT, I2C0_SDA_GPIO_PIN, true);

	PORT_HAL_SetMuxMode     (I2C0_SCL_GPIO_PORT, I2C0_SCL_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C0_SCL_GPIO_PORT, I2C0_SCL_GPIO_PIN, true);

	// I2C1 configuration
	PORT_HAL_SetMuxMode     (I2C1_SDA_GPIO_PORT, I2C1_SDA_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C1_SDA_GPIO_PORT, I2C1_SDA_GPIO_PIN, true);

	PORT_HAL_SetMuxMode     (I2C1_SCL_GPIO_PORT, I2C1_SCL_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd(I2C1_SCL_GPIO_PORT, I2C1_SCL_GPIO_PIN, true);

	// UART1 configuration
	PORT_HAL_SetMuxMode(UART_MCU_FPGA_TX_PORT, UART_MCU_FPGA_TX_PIN, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(UART_MCU_FPGA_RX_PORT, UART_MCU_FPGA_RX_PIN, kPortMuxAlt3);

	// config EXTAL0 to be input clock
	PORT_HAL_SetMuxMode(EXTAL0_PORT,       EXTAL0_PIN,       kPortPinDisabled);
	
	// Setup PWM for CPU_CRADLE_DET (using Flex timer module)
	PORT_HAL_SetDriveStrengthMode(PWM_CPU_CRADLE_DET_PORT, PWM_CPU_CRADLE_DET_PIN,kPortLowDriveStrength);
	PORT_HAL_SetMuxMode(PWM_CPU_CRADLE_DET_PORT, PWM_CPU_CRADLE_DET_PIN, kPortMuxAlt4);
	PORT_HAL_SetSlewRateMode(PWM_CPU_CRADLE_DET_PORT, PWM_CPU_CRADLE_DET_PIN,kPortSlowSlewRate);
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
