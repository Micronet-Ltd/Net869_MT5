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

#define GET_COMMAND 0
#define SET_COMMAND 1

typedef union gpio_config_type
{
	uint8_t val;
	struct{
		uint8_t in_out_b1 : 1;
		uint8_t ris_fall_both_edge_b2_b3 : 2; /*0: rising, 1: falling, both: 2 */
		uint8_t pd_pu_f_b4_b5: 2;/* 0: pull down, 1: pull up, 2: floating */
		uint8_t unused_b6_b8: 3;
	};
}gpio_config_t;

static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver);
static void get_fpga_ver(uint8_t * data, uint16_t data_size, uint8_t * pfpga_ver);
static void get_gpio_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_gpio_output_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void get_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);

static comm_t comm_g[COMM_ENUM_SIZE] =
{
	[COMM_GET_FW_VERSION ] = {get_fw_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_FPGA_VERSION ] = {get_fpga_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
//	[COMM_GET_GPIO_IN_CNFG] = {NULL,
//						  GET_COMMAND,
//						  sizeof(uint8_t)},
//	[COMM_SET_GPIO_IN_CNFG] = {NULL,
//						SET_COMMAND,
//						sizeof(uint8_t)},
	[COMM_GET_GPIO] = {get_gpio_input_voltage,
						GET_COMMAND,
						sizeof(uint8_t)},
	[COMM_SET_GPIO] = {set_gpio_output_val,
						SET_COMMAND,
						0},
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

/* data[0]: GPIO value */
static void get_gpio_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{
	//TODO: Need to use EXT_GPIO.c Abid
	//pgpio_val[0] = (uint8_t)GPIO_DRV_ReadPinInput(gpio_out_mapping[data[0]]);
}

static void set_gpio_output_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{
	gpio_set_output(data[0], data[1]);
}
