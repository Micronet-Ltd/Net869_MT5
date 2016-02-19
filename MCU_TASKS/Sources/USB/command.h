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
	COMM_FW_VERSION, /* 0 */
	COMM_GPIO_CNFG, /* 1 */
	COMM_GPIO_READ, /* 2 */
	COMM_GPIO_WRITE, /* 3 */

	COMM_ENUM_SIZE
}command_enum;

typedef enum comm_err
{
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

/* GPIO related stuff, might want to move this to a separate file */
typedef enum
{
	GPIO0,
	GPIO1,
	GPIO2,
	GPIO3,
	GPIO4,
	GPIO5,
	GPIO6,
	GPIO7,
	GPIO8,
	GPIO9,

	GPIO_NUM
}virtual_gpio_names_enum;

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

int8_t command_set(uint8_t * val, uint16_t size);
int8_t command_get(uint8_t * data, uint16_t data_size,
		packet_t * resp, uint8_t * resp_size);

#endif /* _COMMAND_H_ */
