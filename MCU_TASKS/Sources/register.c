/*
 * register.c
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#include <mqx.h>
#include "register.h"

#define FW_VER_MAJOR 0x1
#define FW_VER_MINOR 0x1

static uint32_t reg_g[REGISTERS_ENUM_SIZE] =
	{
		[FW_VERSION ] = ((FW_VER_MAJOR << 16) | (FW_VER_MINOR & 0x00FF)),
	};

void register_init()
{

}

//returns 0 for success, -1 for failure
int8_t register_set(uint8_t address, uint32_t val)
{
	if (address > REGISTERS_ENUM_SIZE)
		return -1;

	reg_g[address] = val;
	return 0;
}

int8_t register_get(uint8_t address, uint32_t * data)
{
	memcpy(data, (void *) &reg_g[address], sizeof(uint32_t));
	//*data =  reg_g[address];
	return 0;
}


