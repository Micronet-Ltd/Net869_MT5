#ifndef I2C_CONFIGURATION_H
#define I2C_CONFIGURATION_H

#include "fsl_i2c_master_driver.h"
#include "i2c_configuration.h"


void I2C_Enable          (uint8_t port);
void I2C_Disable         (uint8_t port);
void I2C_Reset           (uint8_t port);
	
void MQX_I2C0_IRQHandler (void);
void MQX_I2C1_IRQHandler (void);

#endif /* I2C_CONFIGURATION_H */

