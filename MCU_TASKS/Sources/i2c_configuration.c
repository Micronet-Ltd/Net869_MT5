#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <message.h>
#include <lwmsgq.h>
#include "fsl_device_registers.h"
#include "fsl_port_hal.h"
#include "fsl_sim_hal.h"
#include "i2c_configuration.h"
#include "mic_typedef.h"

typedef struct i2c_master_state_with_enable_s
{
	i2c_master_state_t i2c_master_state;
	bool enabled;
}i2c_master_state_with_enable_t;

static i2c_master_state_with_enable_t i2c_master_g [I2C_INSTANCE_COUNT] =
		{[I2C0_IDX].enabled = FALSE,
		[I2C1_IDX].enabled =  FALSE};

void I2C_Enable  (uint8_t port)
{
	i2c_status_t ret = kStatus_I2C_Success;
	if (port >= I2C_INSTANCE_COUNT)
	{
		printf("\nI2C Enable - illegal port\n");
		return;
	}

	/* only enable the port if it hasn't been enabled before */
	if (i2c_master_g[port].enabled == FALSE)
	{
		switch (port)
		{
			case I2C0_IDX:
				NVIC_SetPriority      (I2C0_IRQn, I2C_NVIC_IRQ_Priority);
				OSA_InstallIntHandler (I2C0_IRQn, MQX_I2C0_IRQHandler);
				break;

			case I2C1_IDX:
				NVIC_SetPriority      (I2C1_IRQn, I2C_NVIC_IRQ_Priority);
				OSA_InstallIntHandler (I2C1_IRQn, MQX_I2C1_IRQHandler);
				break;

			default:
				printf("\nI2C Enable - illegal port\n");
				return;
		}

		ret = I2C_DRV_MasterInit (port,  &i2c_master_g[port].i2c_master_state);
		if (ret == kStatus_I2C_Success)
		{
			i2c_master_g[port].enabled = TRUE;
			printf("\nI2C %d Enabled\n", port);
		}
		else
		{
			printf("\nI2C Enable - I2C_DRV_MasterInit failed, error code %d \n", ret);
		}
	}
}

void I2C_Disable (uint8_t port)
{
	i2c_status_t ret = kStatus_I2C_Success;
	if (port >= I2C_INSTANCE_COUNT)
	{
		printf("\nI2C Disable - illegal port %d\n", port);
		return;
	}

	if (ret == kStatus_I2C_Success)
	{
		ret = I2C_DRV_MasterDeinit (port);
		i2c_master_g[port].enabled = FALSE;
		printf("\nI2C %d Disabled\n", port);
	}
	else
	{
		printf("\nI2C Disable - I2C_DRV_MasterDeinit failed, error code %d \n", ret);
	}
}

void I2C_Reset  (uint8_t port)
{
	I2C_Disable (port) ;
	I2C_Enable  (port) ;
}

void MQX_I2C0_IRQHandler        (void)		{ I2C_DRV_MasterIRQHandler (0); }
void MQX_I2C1_IRQHandler        (void)		{ I2C_DRV_MasterIRQHandler (1); }
