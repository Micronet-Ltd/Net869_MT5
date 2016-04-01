/*
 * command.h
 *
 *  Created on: Jan 25, 2016
 *      Author: abid.esmail
 */

#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "protocol.h"

typedef enum
{
	COMM_GET_FW_VERSION = 0x0,
	COMM_GET_FPGA_VERSION = 0x1,
	COMM_GET_GPI_THRESHOLD = 0x2,
	COMM_SET_GPI_THRESHOLD = 0x3,
	COMM_GET_GPI_OR_ADC_INPUT_VOLTAGE = 0x4,
	COMM_GET_LED_STATUS = 0x5,
	COMM_SET_LED_STATUS = 0x6,
	COMM_GET_POWER_ON_THRESHOLD = 0x7,
	COMM_SET_POWER_ON_THRESHOLD = 0x8,
	COMM_GET_TURN_ON_REASON = 0x9,
	COMM_SET_DEVICE_OFF = 0xA,
	COMM_GET_RTC_DATE_TIME = 0xB,
	COMM_SET_RTC_DATE_TIME = 0xC,

	COMM_ENUM_SIZE
}command_enum;

typedef enum comm_err
{
	INVALID_SIZE = -3,
	INVALID_COMM = -2,
	INVALID_TYPE = -1, /* case when command a setCommand is issued for a get or
						vice versa */
	SUCCESS = 0,

	ERR_ENUM_SIZE
}comm_err_t;

typedef struct comm_type
{
	void (*fx)(uint8_t * req, uint16_t size, uint8_t * resp);
	bool  get_set; /*get = 0, set = 1 */
	uint8_t response_size; /* (in bytes) */
}comm_t;

int8_t command_set(uint8_t * val, uint16_t size);
int8_t command_get(uint8_t * data, uint16_t data_size,
		packet_t * resp, uint8_t * resp_size);

#endif /* _COMMAND_H_ */
