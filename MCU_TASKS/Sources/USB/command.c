/*
 * command.c
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#include <mqx.h>
#include "command.h"
#include "protocol.h"

#define FW_VER_BTLD_OR_APP 0x0A /* 0xA : Application, 0xB: Bootloader */
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 1
#define FW_VER_BUILD 1

#define GET_COMMAND 0
#define SET_COMMAND 1

static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver);
static void get_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_gpio_val(uint8_t * data, uint16_t data_size, void * val);
static void get_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);

static const uint32_t fw_ver_g = (((FW_VER_BTLD_OR_APP & 0xFF)<<24) |
								((FW_VER_MAJOR & 0xFF) << 16) |
								((FW_VER_MINOR & 0xFF) << 8) |
								(FW_VER_BUILD & 0xFF));

static comm_t comm_g[COMM_ENUM_SIZE] =
{
	[COMM_FW_VERSION ] = {get_fw_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GPIO_CNFG] = {NULL,
						GET_COMMAND,
						sizeof(uint8_t)},
	[COMM_GPIO_READ] = {get_gpio_val,
						GET_COMMAND,
						sizeof(uint8_t)},
	[COMM_GPIO_WRITE] = {set_gpio_val,
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

	comm_g[address].fx(data, data_size, NULL);

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

	resp->data = (uint8_t *) malloc(sizeof(comm_g[address].response_size)+1);
	resp->data[0] = address;
	*resp_size = comm_g[address].response_size + 1;
	comm_g[address].fx(data, data_size, &resp->data[1]);

	//TODO: Send response back
	//memcpy(data, (void *) &comm_g[address].val, sizeof(uint32_t));
	return SUCCESS;
}

/* returns 4 bytes with fw version */
static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver)
{
	//uint8_t * pfw_ver = (uint8_t *) malloc(sizeof(fw_ver));
	memcpy(pfw_ver, (uint8_t *)&fw_ver_g, sizeof(fw_ver_g));
}

/* data[0]: GPIO value */
static void get_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{
	printf("get GPIO toggle \n");
}

static void set_gpio_val(uint8_t * data, uint16_t data_size, void * val)
{
	printf("set GPIO num: %d to val %d \n", data[0], data[1]);
}

/* data[0]: GPIO value */
static void get_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{

}

static void set_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{

}
