/*
 * register.c
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#include <mqx.h>
#include "register.h"

#define FW_VER_CUSTOMER 'X'
#define FW_VER_MAJOR 0
#define FW_VER_MINOR 1
#define FW_VER_PATCH 1

#define RD_ONLY 0
#define RD_WR   1

static reg_t reg_g[REGISTERS_ENUM_SIZE] =
{
	[FW_VERSION ] = {(((FW_VER_CUSTOMER & 0xFF)<<24) |
					((FW_VER_MAJOR & 0xFF) << 16) |
					((FW_VER_MINOR & 0xFF) << 8) |
					(FW_VER_PATCH & 0xFF)),
					RD_ONLY},
};

void register_init()
{

}

//returns 0 for success, -1 for failure
int8_t register_set(uint8_t address, uint32_t val)
{
	if (address > REGISTERS_ENUM_SIZE || address < 0)
	{
		return INVALID_REG;
	}

	if (reg_g[address].read_write == RD_WR)
	{
		return READ_ONLY_REG;
	}

	reg_g[address].val = val;
	return SUCCESS;
}

int8_t register_get(uint8_t address, uint32_t * data)
{
	if (address > REGISTERS_ENUM_SIZE || address < 0)
	{
		return INVALID_REG;
	}

	memcpy(data, (void *) &reg_g[address].val, sizeof(uint32_t));
	return SUCCESS;
}


