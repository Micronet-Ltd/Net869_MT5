/*
 * command.c
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#include <mqx.h>
#include "command.h"
#include "protocol.h"
#include "gpio_pins.h"
#include "EXT_GPIOS.h"
#include "version.h"
#include "fpga_API.h"
#include "Device_control_GPIO.h"

#define GET_COMMAND 0
#define SET_COMMAND 1

static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver);
static void get_fpga_ver(uint8_t * data, uint16_t data_size, uint8_t * pfpga_ver);
static void get_gp_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpi_volt);
static void get_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status);
static void set_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status);
static void get_turn_on_reason(uint8_t * data, uint16_t data_size, uint8_t * pturn_on_reason);

static comm_t comm_g[COMM_ENUM_SIZE] =
{
	[COMM_GET_FW_VERSION ] = {get_fw_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_FPGA_VERSION ] = {get_fpga_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_GPI_INPUT_VOLTAGE] = {get_gp_input_voltage,
							GET_COMMAND,
							sizeof(uint32_t)},
	[COMM_GET_LED_STATUS] = {get_led_status,
							GET_COMMAND,
							4},
	[COMM_SET_LED_STATUS] = {set_led_status,
							SET_COMMAND,
							0},
	[COMM_GET_TURN_ON_REASON] = {get_turn_on_reason,
							GET_COMMAND,
							sizeof(uint8_t)},
};

int8_t command_set(uint8_t * data, uint16_t data_size)
{
	uint8_t address = data[0];
	if (address > COMM_ENUM_SIZE || address < 0)
	{
		return INVALID_COMM;
	}

	if (comm_g[address].get_set != SET_COMMAND)
	{
		return INVALID_TYPE;
	}

	comm_g[address].fx(&data[1], data_size, NULL);

	return SUCCESS;
}

// always send response with COMM_READ_RESP
int8_t command_get(uint8_t * data, uint16_t data_size,
		packet_t * resp, uint8_t * resp_size)
{
	uint8_t address = data[0];
	if (address > COMM_ENUM_SIZE || address < 0)
	{
		return INVALID_COMM;
	}

	if (comm_g[address].get_set != GET_COMMAND)
	{
		return INVALID_TYPE;
	}

	if (comm_g[address].response_size > MAX_PACKET_SIZE)
	{
		return INVALID_SIZE;
	}

	resp->data[0] = address;
	*resp_size = comm_g[address].response_size + 1;
	if (comm_g[address].fx != NULL )
	{
		comm_g[address].fx(&data[1], data_size, &resp->data[1]);
	}

	return SUCCESS;
}

/* returns 4 bytes with fw version */
static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver)
{
	pfw_ver[0] = FW_VER_BTLD_OR_APP;
	pfw_ver[1] = FW_VER_MAJOR;
	pfw_ver[2] = FW_VER_MINOR;
	pfw_ver[3] = FW_VER_BUILD;
}

/* returns 4 bytes with fpga version */
static void get_fpga_ver(uint8_t * data, uint16_t data_size, uint8_t * pfpga_ver)
{
	FPGA_read_version((uint32_t *)pfpga_ver);
}

/* returns voltage of the requested GPInput */
static void get_gp_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpi_volt)
{
	uint32_t gpi_voltage = 0;
	gpi_voltage = GPIO_INPUT_get_voltage_level(data[0]);
	memcpy(pgpi_volt, (uint8_t *)&gpi_voltage , sizeof(uint32_t));
}

static void get_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status)
{
	FPGA_read_led_status(data[0], &pled_status[0], &pled_status[1], &pled_status[2], &pled_status[3]);
}

static void set_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status)
{
	FPGA_write_led_status (data[0], &data[1], &data[2], &data[3], &data[4]);
}

static void get_turn_on_reason(uint8_t * data, uint16_t data_size, uint8_t * pturn_on_reason)
{
	Device_get_turn_on_reason(pturn_on_reason);
}
