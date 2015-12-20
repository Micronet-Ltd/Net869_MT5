#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "i2c_configuration.h"

static i2c_master_state_t i2c_master [I2C_INSTANCE_COUNT]; 

void I2C_Enable  (uint8_t port) 
{
	switch (port) {
		case I2C0_IDX:	NVIC_SetPriority      (I2C0_IRQn, 6U);
						OSA_InstallIntHandler (I2C0_IRQn, MQX_I2C0_IRQHandler);
						break;

		case I2C1_IDX:	NVIC_SetPriority      (I2C1_IRQn, 6U);
						OSA_InstallIntHandler (I2C1_IRQn, MQX_I2C1_IRQHandler);
						break;
						
		default:		printf("\nI2C Enable - illeagal port\n");
						return;
	}
	
	I2C_DRV_MasterInit (port,  &i2c_master[port]);
	printf("\nI2C %d Enabled\n", port);
}

void I2C_Disable (uint8_t port) 
{
	I2C_DRV_MasterDeinit (port);
	printf("\nI2C %d Disabled\n", port);
}

void I2C_Reset  (uint8_t port)
{
	I2C_Disable (port) ;
	I2C_Enable  (port) ;
}

void MQX_I2C0_IRQHandler        (void)		{ I2C_DRV_MasterIRQHandler (0); }
void MQX_I2C1_IRQHandler        (void)		{ I2C_DRV_MasterIRQHandler (1); }
