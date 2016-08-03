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
#include "Device_control_GPIO.h"
#include "Wiggle_sensor.h"
#include "power_mgm.h"
#include "rtc.h"

#define GET_COMMAND 0
#define SET_COMMAND 1

static void get_fw_ver(uint8_t * data, uint16_t data_size, uint8_t * pfw_ver);
static void get_fpga_ver(uint8_t * data, uint16_t data_size, uint8_t * pfpga_ver);
static void get_gp_or_adc_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpi_volt);
static void get_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status);
static void set_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status);
static void get_power_on_threshold(uint8_t * data, uint16_t data_size, uint8_t * ppower_on_threshold);
static void set_power_on_threshold(uint8_t * data, uint16_t data_size, uint8_t * ppower_on_threshold);
static void get_turn_on_reason(uint8_t * data, uint16_t data_size, uint8_t * pturn_on_reason);
static void set_device_off(uint8_t * data, uint16_t data_size, uint8_t * pdevice_off);
static void set_device_off(uint8_t * data, uint16_t data_size, uint8_t * pdevice_off);
static void get_rtc_date_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time);
static void set_rtc_date_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time);
static void get_rtc_date_time(uint8_t * data, uint16_t data_size, uint8_t * pbatt_ignition_voltage);
static void set_gpi_update_all_values(uint8_t * data, uint16_t data_size, uint8_t * pgpi_values);
static void get_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg);
static void set_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg);
static void get_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void set_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void get_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio);
static void set_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio);

static comm_t comm_g[COMM_ENUM_SIZE] =
{
	[COMM_GET_FW_VERSION ] = {get_fw_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_FPGA_VERSION ] = {get_fpga_ver,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_GPI_OR_ADC_INPUT_VOLTAGE] = {get_gp_or_adc_input_voltage,
							GET_COMMAND,
							sizeof(uint32_t)},
	[COMM_GET_LED_STATUS] = {get_led_status,
							GET_COMMAND,
							4},
	[COMM_SET_LED_STATUS] = {set_led_status,
							SET_COMMAND,
							0},
	[COMM_GET_POWER_ON_THRESHOLD] = {get_power_on_threshold,
							GET_COMMAND,
							(sizeof(uint16_t)*3)},
	[COMM_SET_POWER_ON_THRESHOLD] = {set_power_on_threshold,
							SET_COMMAND,
							0},
	[COMM_GET_TURN_ON_REASON] = {get_turn_on_reason,
								GET_COMMAND,
								sizeof(uint8_t)},
	[COMM_SET_DEVICE_OFF] = {set_device_off,
							 SET_COMMAND,
							 0},
	[COMM_GET_RTC_DATE_TIME] = {get_rtc_date_time,
								GET_COMMAND,
								RTC_BCD_SIZE},
	[COMM_SET_RTC_DATE_TIME] = {set_rtc_date_time,
								SET_COMMAND,
								0},
	[COMM_GET_RTC_CAL_REGISTERS] = {get_rtc_cal_register,
									GET_COMMAND,
									(sizeof(uint8_t)*2)},
	[COMM_SET_RTC_CAL_REGISTERS] = {set_rtc_cal_register,
									SET_COMMAND,
									0},
	[COMM_SET_GPI_UPDATE_ALL_VALUES] = {set_gpi_update_all_values,
										SET_COMMAND,
										0},
	[COMM_GET_RTC_REG_DBG] = {get_rtc_data_dbg,
							   GET_COMMAND,
							   sizeof(uint8_t)},
	[COMM_SET_RTC_REG_DBG] = {set_rtc_data_dbg,
							   SET_COMMAND,
							   0},
	[COMM_GET_MCU_GPIO_STATE_DBG] = {get_mcu_gpio_state_dbg,
							   GET_COMMAND,
							   sizeof(uint8_t)},
	[COMM_SET_MCU_GPIO_STATE_DBG] = {set_mcu_gpio_state_dbg,
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

/* returns voltage of the requested GPInput or ADC */
static void get_gp_or_adc_input_voltage(uint8_t * data, uint16_t data_size, uint8_t * pgpi_volt)
{
	uint32_t gpi_or_adc_voltage = 0;
	if (data[0] < kADC_CHANNELS)
	{
		gpi_or_adc_voltage = ADC_get_value((KADC_CHANNELS_t)data[0]);
	}
	memcpy(pgpi_volt, (uint8_t *)&gpi_or_adc_voltage , sizeof(uint32_t));
}

/* sends the GPI values to the OS since some of the updates could have been sent
 * during bootup */
static void set_gpi_update_all_values(uint8_t * data, uint16_t data_size, uint8_t * pgpi_values)
{
	uint8_t gpio_mask = 0xff;
	send_gpi_change(&gpio_mask);
}

static void get_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status)
{
	FPGA_read_led_status(data[0], &pled_status[0], &pled_status[1], &pled_status[2], &pled_status[3]);
}

static void set_led_status(uint8_t * data, uint16_t data_size, uint8_t * pled_status)
{
	FPGA_write_led_status (data[0], data[1], data[2], data[3], data[4]);
}

static void  get_power_on_threshold(uint8_t * data, uint16_t data_size, uint8_t * ppower_on_threshold)
{
	uint32_t vibration_threshold = 0;
	uint32_t duration_threshold = 0;
	uint32_t ignition_threshold = 0;
	Wiggle_sensor_get_vibration_TH  (&vibration_threshold, &duration_threshold);
	get_ignition_threshold(&ignition_threshold);
	/*litte Endian */
	ppower_on_threshold[0] = (uint8_t) (vibration_threshold&0xFF);
	ppower_on_threshold[1] = (uint8_t) ((vibration_threshold>>8)&0xFF);
	ppower_on_threshold[2] = (uint8_t) (duration_threshold&0xFF);
	ppower_on_threshold[3] = (uint8_t) ((duration_threshold>>8)&0xFF);
	ppower_on_threshold[4] = (uint8_t) (ignition_threshold&0xFF);
	ppower_on_threshold[5] = (uint8_t) ((ignition_threshold>>8)&0xFF);
}

static void  set_power_on_threshold(uint8_t * data, uint16_t data_size, uint8_t * ppower_on_threshold)
{
	/* Data is in little Endian format*/
	uint32_t vibration_threshold = (uint32_t)((data[1]<<8)|data[0]);
	uint32_t duration_threshold = (uint32_t)((data[3]<<8)|data[2]);
	uint32_t ignition_threshold = (uint32_t)((data[5]<<8)|data[4]);
	Wiggle_sensor_set_vibration_TH  (vibration_threshold, duration_threshold);
	set_ignition_threshold(ignition_threshold);
}

static void get_turn_on_reason(uint8_t * data, uint16_t data_size, uint8_t * pturn_on_reason)
{
	Device_get_turn_on_reason(pturn_on_reason);
}

static void set_device_off(uint8_t * data, uint16_t data_size, uint8_t * pdevice_off)
{
	Device_off_req(FALSE, data[0]);
}

static void get_rtc_date_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time)
{
	rtc_get_time(pdate_time);
}

static void set_rtc_date_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time)
{
	rtc_set_time(&data[0]);
}

static void get_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg)
{
	rtc_get_cal_register(&pcal_reg[0], &pcal_reg[1]);
}

static void set_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg)
{
	rtc_set_cal_register(&data[0], &data[1]);
}

static void get_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg)
{
	rtc_receive_data (&data[0], 1, &p_reg[0], 1);
}

static void set_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg)
{
	rtc_send_data (&data[0], 1, &data[1], 1);
}

static void get_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio)
{
	uint32_t gpio_pin_name = (uint32_t)((data[0]<<8) | data[1]);
	uint8_t gpio_port = data[0];

	if (gpio_port < GPIO_INSTANCE_COUNT)
	{
		p_gpio[0] = (uint8_t)GPIO_DRV_ReadPinInput(gpio_pin_name);
	}
	else
	{
		p_gpio[0] = 0xFF; /*invalid port number request */
	}
}

static void set_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio)
{
	uint32_t gpio_pin_name = (uint32_t) ((data[0]<<8) | data[1]);
	uint32_t gpio_val = (uint32_t)data[2];
	uint8_t gpio_port = data[0];
	if (gpio_port < GPIO_INSTANCE_COUNT)
	{
		GPIO_DRV_WritePinOutput(gpio_pin_name, gpio_val);
	}
}