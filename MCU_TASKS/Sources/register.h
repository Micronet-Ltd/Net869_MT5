/*
 * register.h
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#ifndef _REGISTER_H_
#define _REGISTER_H_

typedef enum
{
	FW_VERSION, /* 0 */

	REGISTERS_ENUM_SIZE
}registers_enum;

typedef enum reg_err
{
	INVALID_REG = -2,
	READ_ONLY_REG = -1,
	SUCCESS = 0,

	ERR_ENUM_SIZE
}reg_err_t;

typedef struct reg_type
{
	uint32_t val;
	bool  read_write; /*read only = 0, read/write = 1 */
}reg_t;

int8_t register_set(uint8_t address, uint32_t val);
int8_t register_get(uint8_t address, uint32_t * data);

#endif /* _REGISTER_H_ */
