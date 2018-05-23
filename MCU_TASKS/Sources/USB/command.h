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
	COMM_GET_FW_VERSION = 0x00,
	COMM_GET_FPGA_VERSION = 0x01,
	COMM_GET_GPI_THRESHOLD = 0x02,
	COMM_SET_GPI_THRESHOLD = 0x03,
	COMM_GET_GPI_OR_ADC_INPUT_VOLTAGE = 0x04,
	COMM_GET_LED_STATUS = 0x05,
	COMM_SET_LED_STATUS = 0x06,
	COMM_GET_POWER_ON_THRESHOLD = 0x07,
	COMM_SET_POWER_ON_THRESHOLD = 0x08,
	COMM_GET_TURN_ON_REASON = 0x09,
	COMM_SET_DEVICE_OFF = 0x0A,
	COMM_GET_RTC_DATE_TIME = 0x0B,
	COMM_SET_RTC_DATE_TIME = 0x0C,
	COMM_GET_RTC_CAL_REGISTERS = 0x0D,
	COMM_SET_RTC_CAL_REGISTERS = 0x0E,
	COMM_SET_GPI_UPDATE_ALL_VALUES = 0x0F,
	COMM_GET_RTC_REG_DBG = 0x10,
	COMM_SET_RTC_REG_DBG = 0x11,
	COMM_GET_MCU_GPIO_STATE_DBG = 0x12,
	COMM_SET_MCU_GPIO_STATE_DBG = 0x13,
	COMM_SET_APP_WATCHDOG_REQ = 0x14,
	COMM_SET_WIGGLE_EN_REQ_DBG = 0x15,
	COMM_GET_WIGGLE_COUNT_REQ_DBG = 0x16,
	COMM_SET_ACCEL_STANDBY_ACTIVE_DBG = 0x17,
	COMM_GET_ACCEL_REGISTER_DBG = 0x18,
	COMM_SET_ACCEL_REGISTER_DBG = 0x19,
	COMM_GET_BRD_INFO = 0x1A,
	COMM_GET_BRD_INFO_ADC_DBG = 0x1B,

	COMM_ENUM_SIZE
}command_enum;

typedef enum comm_err
{
	INVALID_SIZE = -3,
	INVALID_COMM = -2,
	INVALID_TYPE = -1, /* case when a setCommand is issued for a get or
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
