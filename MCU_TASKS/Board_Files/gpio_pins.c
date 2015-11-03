
#if 1
#include "gpio_pins.h"
#include <stdbool.h>

const gpio_output_pin_user_config_t outputPins[] = {
	// Power Rail Control
  {.pinName = POWER_3V3_ENABLE, 		.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = POWER_5V0_ENABLE, 		.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = POWER_CHARGE_ENABLE, 		.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = POWER_DISCHARGE_ENABLE, 	.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = POWER_HEATER_ENABLE,	 	.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

	//Telemetry Output Control
  {.pinName = GPIO_OUT1, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = GPIO_OUT2, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = GPIO_OUT3, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },
  {.pinName = GPIO_OUT4, 				.config.outputLogic = 0,    .config.slewRate = kPortSlowSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortLowDriveStrength  },

  // LED
  {.pinName = LED_RED, 					.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
  {.pinName = LED_GREEN, 				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
  {.pinName = LED_BLUE, 				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

  {.pinName = ACC_ENABLE,				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },
//  {.pinName = I2C_SCL, 				.config.outputLogic = 0,    .config.slewRate = kPortFastSlewRate,    .config.isOpenDrainEnabled = false,    .config.driveStrength = kPortHighDriveStrength  },

  {.pinName = GPIO_PINS_OUT_OF_RANGE                                                                                                                                                               }
};


const gpio_input_pin_user_config_t inputPins[] = {
#if 0
  {.pinName = POWER_3V3_ENABLE,    .config.isPullEnable = false,    .config.pullSelect = kPortPullUp,    .config.isPassiveFilterEnabled = false,    .config.interrupt = kPortIntDisabled  },
  {
    .pinName = kGpioSW2,
    .config.isPullEnable = true,
    .config.pullSelect = kPortPullUp,
    .config.isPassiveFilterEnabled = false,
    .config.interrupt = kPortIntDisabled
  },
#endif
  {.pinName = GPIO_PINS_OUT_OF_RANGE                                                                                                                                                               }
};


void GPIO_Config (void)
{
	/*
	CLOCK_SYS_EnablePortClock(PORTA_IDX);
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	CLOCK_SYS_EnablePortClock(PORTC_IDX);
	CLOCK_SYS_EnablePortClock(PORTD_IDX);
*/

	GPIO_DRV_Init (inputPins, outputPins);

	// I2C0 configuration
	PORT_HAL_SetMuxMode      (I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd (I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, true);

	PORT_HAL_SetMuxMode      (I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, kPortMuxAlt2);
	PORT_HAL_SetOpenDrainCmd (I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, true);

}

#endif
