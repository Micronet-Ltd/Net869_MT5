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
#include "watchdog_mgmt.h"
#include "acc_task.h"
#include "board_type.h"
#include "J1708_task.h"
#include "board_type.h"

#define GET_COMMAND 0
#define SET_COMMAND 1

static void get_board_info(uint8_t * data, uint16_t data_size, uint8_t * pbrd_info);
static void get_board_adc_value_dbg(uint8_t * data, uint16_t data_size, uint8_t * pbrd_adc);
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
static void set_rtc_alarm1_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time);
static void get_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg);
static void set_rtc_cal_register(uint8_t * data, uint16_t data_size, uint8_t * pcal_reg);
static void get_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void set_rtc_data_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void get_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio);
static void set_mcu_gpio_state_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_gpio);
static void set_app_watchdog_req(uint8_t * data, uint16_t data_size, uint8_t * p_watchdog);
static void set_wiggle_en_req(uint8_t * data, uint16_t data_size, uint8_t * p_wig_en);
static void get_wiggle_sensor_count_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_wig_cnt);
static void set_accel_standby_active_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_accel_active);
static void get_accel_register_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void set_accel_register_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg);
static void get_rtc_flags(uint8_t * data, uint16_t data_size, uint8_t * flags);

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
	[COMM_SET_APP_WATCHDOG_REQ] = {set_app_watchdog_req,
							   SET_COMMAND,
							   0},
	[COMM_SET_WIGGLE_EN_REQ_DBG] = {set_wiggle_en_req,
							   SET_COMMAND,
							   0},
	[COMM_GET_WIGGLE_COUNT_REQ_DBG] = {get_wiggle_sensor_count_dbg,
							   GET_COMMAND,
							   (sizeof(uint8_t)*4)},
	[COMM_SET_ACCEL_STANDBY_ACTIVE_DBG] = {set_accel_standby_active_dbg,
							   SET_COMMAND,
							   0},
	[COMM_GET_ACCEL_REGISTER_DBG] = {get_accel_register_dbg,
							   GET_COMMAND,
							   sizeof(uint8_t)},
	[COMM_SET_ACCEL_REGISTER_DBG] = {set_accel_register_dbg,
							   SET_COMMAND,
							   0},
	[COMM_GET_BRD_INFO] = {get_board_info,
						  GET_COMMAND,
						  sizeof(uint32_t)},
	[COMM_GET_BRD_INFO_ADC_DBG] = {get_board_adc_value_dbg,
						  GET_COMMAND,
						  sizeof(uint32_t)},
    [COMM_SET_RTC_ALARM1_TIME] = {set_rtc_alarm1_time,
								SET_COMMAND,
							    0},
	[COMM_GET_RTC_ALARM1_INIT_FLAGS] = {get_rtc_flags,
								GET_COMMAND,
								(sizeof(uint8_t)*4)},


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

/* returns 4 bytes Byte1 = Board Version, Byte2 = Board Config, Byte3 = 0, Byte4 = 0 */
static void get_board_info(uint8_t * data, uint16_t data_size, uint8_t * pbrd_info)
{
	pbrd_info[0] = get_saved_board_revision();
	pbrd_info[1] = get_saved_board_configuration();
	pbrd_info[2] = 0;
	pbrd_info[3] = 0;
}

/* get_board_adc_value_dbg returns the raw adc voltage of the board info
* Caution: the ADC can take a while to read the ADC (over 5 seconds)
* args:
*			data[0] = 0 returns the board version ADC value in mV
*			data[0] = 1 returns the board config ADC value in mV
*/
static void get_board_adc_value_dbg(uint8_t * data, uint16_t data_size, uint8_t * pbrd_adc)
{
	uint32_t board_adc_voltage = 0;
	if (data[0] == 0)
	{
		board_adc_voltage = get_board_adc_value(ADC_BOARD_VER, 300);
	}
	else if (data[0] == 1)
	{
		board_adc_voltage = get_board_adc_value(ADC_BOARD_CONFIG, 80);
	}
	memcpy(pbrd_adc, (uint8_t *)&board_adc_voltage , sizeof(uint32_t));
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

static void get_rtc_flags(uint8_t * data, uint16_t data_size, uint8_t * flags)
{
	rtc_get_init_flags((uint32_t *)flags);
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

static void set_rtc_alarm1_time(uint8_t * data, uint16_t data_size, uint8_t * pdate_time)
{
	rtc_set_alarm1(&data[0]);
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
	
	/* To avoid putting J1708 in a bad state, we disable J1708 if the J1708 Power is disabled */
	if (gpio_pin_name == J1708_PWR_EN){
		if (gpio_val == 1){
			_event_set(g_J1708_event_h, EVENT_J1708_ENABLE);
		}else{
			GPIO_DRV_SetPinOutput   (J1708_ENABLE); //disable any more messages from coming through
			_event_set(g_J1708_event_h, EVENT_J1708_DISABLE);
		}
	}
	else{
		if (gpio_port < GPIO_INSTANCE_COUNT)
			GPIO_DRV_WritePinOutput(gpio_pin_name, gpio_val);
	}
}

static void set_app_watchdog_req(uint8_t * data, uint16_t data_size, uint8_t * p_watchdog)
{
	uint8_t color = LED_RED_GPIO_NUM;
	printf("\r\n App Watchdog Expired, resetting MCU! \r\n");
	handle_watchdog_expiry(&color);
}

static void set_wiggle_en_req(uint8_t * data, uint16_t data_size, uint8_t * p_wig_en)
{
	uint8_t wiggle_en = data[0];
	if (wiggle_en)
	{
		Wiggle_sensor_start();
		Wiggle_sensor_restart();
	}
	else
	{
		Wiggle_sensor_stop();
	}
}

static void get_wiggle_sensor_count_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_wig_cnt)
{
	uint32_t wiggle_count = 0;
	wiggle_count = get_wiggle_sensor_count();
	/*little endian */
	p_wig_cnt[0] = (uint8_t) (wiggle_count&0xFF);
	p_wig_cnt[1] = (uint8_t) ((wiggle_count>>8)&0xFF);
	p_wig_cnt[2] = (uint8_t) ((wiggle_count>>16)&0xFF);
	p_wig_cnt[3] = (uint8_t) ((wiggle_count>>24)&0xFF);
}

/* 0: Accel standby, 1: Accel Active, fifo enabled */
static void set_accel_standby_active_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_accel_active)
{
	uint8_t accel_active = data[0];
	if (accel_active)
	{
		AccDisable();
		_time_delay(10);
		accInit(); /* Reenable Fifo */
		_time_delay(10);
		AccEnable(); /* Go into accelorometer active mode */
	}
	else
	{
		AccDisable();
	}
}

static void get_accel_register_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg)
{
	AccReadRegister (data[0], &p_reg[0]);
}

static void set_accel_register_dbg(uint8_t * data, uint16_t data_size, uint8_t * p_reg)
{
	AccWriteRegister (data[0], data[1]);
}
