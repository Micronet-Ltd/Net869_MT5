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

#define FW_VER_BTLD_OR_APP 0x0A /* 0xA : Application, 0xB: Bootloader */
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 1
#define FW_VER_BUILD 1

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
static void get_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void get_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);
static void set_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val);

static const uint32_t fw_ver_g = (((FW_VER_BTLD_OR_APP & 0xFF)<<24) |
								((FW_VER_MAJOR & 0xFF) << 16) |
								((FW_VER_MINOR & 0xFF) << 8) |
								(FW_VER_BUILD & 0xFF));

static comm_t comm_g[COMM_ENUM_SIZE] =
{
	[COMM_GET_FW_VERSION ] = {get_fw_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_GPIO_IN_CNFG] = {NULL,
							GET_COMMAND,
							sizeof(uint8_t)},
	[COMM_SET_GPIO_IN_CNFG] = {NULL,
						SET_COMMAND,
						sizeof(uint8_t)},
	[COMM_GET_GPIO] = {get_gpio_val,
						GET_COMMAND,
						sizeof(uint8_t)},
	[COMM_SET_GPIO] = {set_gpio_val,
						SET_COMMAND,
						0},
};

static uint16_t gpio_out_mapping[] = {
		[GPIO0] = GPIO_OUT1,
		[GPIO1] = GPIO_OUT2,
		[GPIO2] = GPIO_OUT3,
		[GPIO3] = GPIO_OUT4,
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

	resp->data = (uint8_t *) malloc(sizeof(comm_g[address].response_size)+1);
	resp->data[0] = address;
	*resp_size = comm_g[address].response_size + 1;
	comm_g[address].fx(&data[1], data_size, &resp->data[1]);

	return SUCCESS;
}

/* returns 4 bytes with fw version */
static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver)
{
	memcpy(pfw_ver, (uint8_t *)&fw_ver_g, sizeof(fw_ver_g));
}

/* data[0]: GPIO value */
static void get_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{
	pgpio_val[0] = (uint8_t)GPIO_DRV_ReadPinInput(gpio_out_mapping[data[0]]);
}

static void set_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{
	GPIO_DRV_WritePinOutput(gpio_out_mapping[data[0]], data[1] );
	//printf("set GPIO num: %d to val %d \n", data[0], data[1]);
}

/* data[0]: GPIO value */
static void get_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{

}

static void set_all_gpio_val(uint8_t * data, uint16_t data_size, uint8_t * pgpio_val)
{

}
