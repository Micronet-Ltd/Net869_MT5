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
	FW_VERSION,
	REGISTERS_ENUM_SIZE
}registers_enum;

int8_t register_set(uint8_t address, uint32_t val);
int8_t register_get(uint8_t address, uint32_t * data);

#endif /* _REGISTER_H_ */
