/*
 * rtc.c
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 */
#include <stdio.h>
#include <mqx.h>
#include <bsp.h>

#include "fsl_i2c_master_driver.h"

#define RTC_DEVICE_ADDRESS 			0x68
#define	RTC_I2C_BAUD_RATE			400
#define RTC_TIME_OUT				100		//  in milliseconds

#define	RTC_I2C_PORT				I2C0_IDX

#define RTC_DECI_SEC_ADDR				0x0
#define RTC_SECOND_ADDR					0x1
#define RTC_MINUTES_ADDR				0x2
#define RTC_HOURS_ADDR					0x3
#define RTC_DAY_OF_WEEK_ADDR			0x4
#define RTC_DAY_OF_MONTH_ADDR			0x5
#define RTC_MONTH_ADDR					0x6
#define RTC_YEAR_ADDR					0x7

#define RTC_ALRM1_HOUR_ADDR				0xC
#define RTC_FLAGS_ADDR					0xF

static i2c_device_t rtc_device = {.address = RTC_DEVICE_ADDRESS,    .baudRate_kbps = RTC_I2C_BAUD_RATE};

/* returns TRUE if the oscillator is running fine, also clears oscil. flag if set */
bool rtc_check_oscillator(void)
{
	uint8_t flags = 0 ;
	uint8_t cmd_buff = RTC_FLAGS_ADDR;
	if (I2C_DRV_MasterReceiveDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &flags, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
	{
		printf("rtc_check_oscillator: failed i2c read \n");
		return FALSE;
	}
	/* Oscillator fail flag is high when oscil. has stopped or was stopped */
	if ((flags & 0x4))
	{
		/* clear oscillator flag */
		flags &= ~0x4;
		if (I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &flags, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
		{
			printf("rtc_check_oscillator: clearing oscillator bit failed \n");
			return FALSE;
		}
		return FALSE;
	}
	return TRUE;
}

/* returns TRUE if the halt bit is clear, also clears halt bit if it set */
bool rtc_check_halt_bit(void)
{
	uint8_t alarm1_hr = 0 ;
	uint8_t cmd_buff = RTC_ALRM1_HOUR_ADDR;
	if (I2C_DRV_MasterReceiveDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &alarm1_hr, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
	{
		printf("rtc_check_halt_bit: failed i2c read \n");
		return FALSE;
	}
	/* Oscillator fail flag is high when oscil. has stopped or was stopped */
	if ((alarm1_hr & 0x40))
	{
		/* clear halt bit */
		alarm1_hr &= ~0x40;
		if (I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &alarm1_hr, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
		{
			printf("rtc_check_halt_bit: clearing halt bit failed \n");
			return FALSE;
		}
		return FALSE;
	}
	return TRUE;
}


void rtc_init(void)
{
	uint8_t cmd_buff = RTC_SECOND_ADDR;
	uint8_t tx_buff = 0;

	/* setting bit D7 of register 01h to cause the oscillator to stop */
//	tx_buff |= 0x80;
//	if (I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &tx_buff, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
//	{
//		printf("rtc_init: setting oscillator bit failed \n");
//		return;
//	}
//	/* clearing bit D7 of register 01h to cause the oscillator to start */
//	tx_buff &= 0x7f;
//	if (I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, &tx_buff, 1, RTC_TIME_OUT) != kStatus_I2C_Success)
//	{
//		printf("rtc_init: clearing oscillator bit failed \n");
//		return;
//	}

	if (!rtc_check_oscillator())
	{
		printf("rtc_init: oscillator not good \n");
	}

	if (!rtc_check_halt_bit())
	{
		printf("rtc_init: halt bit set \n");
	}
}

void rtc_print_time(uint8_t * dt)
{
	float hundreth_seconds = ((float)(dt[0]>>4) * 0.1) + ((float)(dt[0]&0x0F) * 0.01);
	uint8_t hundreth_sec_int = (uint8_t) (hundreth_seconds * 100);
	uint8_t seconds = (((dt[1]>>4)&0x7) * 10) + (dt[1]&0x0F);
	uint8_t minutes = (((dt[2]>>4)&0x7) * 10) + (dt[2]&0x0F);
	uint8_t hours = (((dt[3]>>4)&0x3) * 10) + (dt[3]&0x0F);
	uint8_t century = (dt[3]>>6);
	//uint8_t day_of_week = dt[4]&0x7;
	uint8_t day_of_month = (((dt[5]>>4)&0x3) * 10) + (dt[5]&0x0F);
	uint8_t month = (((dt[6]>>4)&0x1) * 10) + (dt[6]&0x0F);
	uint16_t year = ((dt[7]>>4) * 10) + (dt[7]&0x0F);
	//seconds = seconds + hundreth_seconds;
	year = 2000 + (century * 100) + year;

	printf("rtc date: %02d/%02d/%04d time: %02d:%02d:%02d.%02d \n",
			month, day_of_month, year, hours, minutes, seconds, hundreth_sec_int);
}

void rtc_get(char * time_val, bool print_time)
{
	uint8_t rx_buff[8] =  {0} ;
	uint8_t cmd_buff = 0;

	if (!rtc_check_oscillator())
	{
		printf("rtc_init: oscillator not good \n");
	}

	cmd_buff = RTC_DECI_SEC_ADDR;
	if (I2C_DRV_MasterReceiveDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, rx_buff, 8, RTC_TIME_OUT) != kStatus_I2C_Success)
	{
		printf("rtc_get: read time failed \n");
		return;
	}

	if (print_time)
	{
		rtc_print_time(rx_buff);
	}
	return;
}

void rtc_set(void)
{
	/* Mar 23 2016, 23:59:00.00 */
	uint8_t dt[8];
	uint8_t cmd_buff = RTC_DECI_SEC_ADDR;

	dt[0] = 0x00; /* DeciSecond */
	dt[1] = 0x00; /* Seconds */
	dt[2] = 0x59; /* Minutes */
	dt[3] = 0x23;   /* century/hours */
	dt[4] = 0x04 ; /* Day of week */
	dt[5] = 0x23 ; /* Day of month */
	dt[6] = 0x03; /* Month */
	dt[7] = 0x16; /* Year */

	if (I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device, &cmd_buff,  1, dt, 8, RTC_TIME_OUT) != kStatus_I2C_Success)
	{
		printf("rtc_init: setting oscillator bit failed \n");
		return;
	}
	printf("rtc_set: ");
	rtc_print_time(dt);
	printf("\n");
	printf("rtc_get: ");
	rtc_get(dt,1);
	printf("\n");

}

void rtc_test(void)
{
	char time_val[8];
	uint8_t i = 0;
	rtc_get(time_val, 1);
	printf("rtc_test: ");
	for (i = 0; i < sizeof(time_val); i++)
	{
		printf("%d, ", time_val[i]);
	}
	printf("\n");
}



