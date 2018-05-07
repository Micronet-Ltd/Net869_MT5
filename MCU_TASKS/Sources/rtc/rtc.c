/*
 * rtc.c
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 * 
 *  Updated on: May 1, 2018
 *      Author: barak.avigdory
 */
#include <stdio.h>
#include <mqx.h>
#include <bsp.h>
#include <mutex.h>
#include <event.h>

#include "rtc.h"

#include "fsl_i2c_master_driver.h"
#include "i2c_configuration.h"

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
#define RTC_DIGITAL_CAL					0x8

#define RTC_ALRM1_MONTH_ADDR			0xA
#define RTC_ALRM1_DAY_ADDR				0xB
#define RTC_ALRM1_HOUR_ADDR				0xC
#define RTC_ALRM1_MIN_ADDR				0xD
#define RTC_ALRM1_SEC_ADDR				0xE
#define RTC_ALRM1_FLAG_OFFSET			0x40
#define RTC_ALRM2_FLAG_OFFSET			0x20

#define RTC_FLAGS_ADDR					0xF
#define RTC_ANALOG_CAL					0x12
#define RTC_MIN_ADDR					0x0
#define RTC_MAX_ADDR					0x1f

#define RTC_STRING_SIZE 				23

//#define RTC_DEBUG						1

#define ALARM1_TRIGGERED_BIT           0x1

extern MUTEX_STRUCT g_i2c0_mutex;

static i2c_device_t rtc_device_g = {.address = RTC_DEVICE_ADDRESS,    .baudRate_kbps = RTC_I2C_BAUD_RATE};

bool rtc_oscillator_kick_start(void);


bool rtc_receive_data(uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status;
	_mqx_uint ret = MQX_OK;

	if (*cmd > RTC_MAX_ADDR)
	{
		printf("rtc_receive_data: invalid command address, address %d \n", *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
	if ((ret =_mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("rtc_receive_data: i2c mutex lock failed, ret %d \n", ret);
		_task_block();
	}

	if ((i2c_status = I2C_DRV_MasterReceiveDataBlocking (RTC_I2C_PORT, &rtc_device_g, cmd,  cmd_size, data, data_size, RTC_TIME_OUT)) != kStatus_I2C_Success)
	{
		printf ("rtc_receive_data: ERROR: Could not receive command 0x%X (I2C error code %d)\n", *cmd, i2c_status);
	}

	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}

bool rtc_send_data(uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size)
{
	i2c_status_t  i2c_status ;
	_mqx_uint ret = MQX_OK;

	if (*cmd > RTC_MAX_ADDR)
	{
		printf("rtc_send_data: invalid command address, address %d \n", *cmd);
		return FALSE;
	}

	/* Get i2c0 mutex */
	if ((ret = _mutex_lock(&g_i2c0_mutex)) != MQX_OK)
	{
		printf("rtc_send_data: i2c mutex lock failed, ret %d \n", ret);
		_task_block();
	}

	if ((i2c_status = I2C_DRV_MasterSendDataBlocking (RTC_I2C_PORT, &rtc_device_g, cmd,  cmd_size, data, data_size, RTC_TIME_OUT)) != kStatus_I2C_Success)
		printf ("rtc_send_data: ERROR: Could not send command 0x%X (I2C error code %d)\n", *cmd, i2c_status);

	_mutex_unlock(&g_i2c0_mutex);
	return (i2c_status == kStatus_I2C_Success);
}

/*Reading the flags in the rtc is unsafe as it nullifies the alarm bits(P 36 in the CD00127116 manual section 3.5).
  Here is an implementation of a safe flag reading function that signals to the program if the alarm flag
  was set by updating the event bit at alarm_event_g;
*/
bool rtc_check_flags_in_low_pow_mode(uint8_t *flags_to_return)
{
	uint8_t flag_cmd_buff = RTC_FLAGS_ADDR;

	if(!rtc_receive_data(&flag_cmd_buff,1 ,flags_to_return, 1))
	{
		printf("rtc_check_flags: ERROR: couldn't read the flagS\n");
		return FALSE;
	}

	if((*flags_to_return)&RTC_ALRM1_FLAG_OFFSET)
	{
		_event_set(alarm_event_g, ALARM1_TRIGGERED_BIT);
	}
 
	return TRUE;
}

/* sets the alarm to activate once at the given date in dt_bcd
 dt_bcd should be given as a binary date. Note that there is no need
 to call rtc_activate_or_deactivate_alarm_polling after calling this function*/
bool rtc_set_alarm1_once(uint8_t *dt_bcd)
{
	uint8_t alarm_data_buff[RTC_NUM_OF_ALARM_BYTES_BCD] = {0};
	uint8_t cmd_buff; 

	uint8_t flag_data_buff;

	const uint8_t RTC_10_DIGIT_LIMITS[RTC_NUM_OF_ALARM_BYTES_BCD] 	= {0x1 /*10 months*/ ,0x11/*30 days*/ ,0x10/*20 hours*/ ,0x5/*50 minutes*/,0x5/*50 seconds*/};
	const uint8_t RTC_TIME_RANGE_LIMITS[RTC_NUM_OF_ALARM_BYTES_BCD] = {12 /*months*/     ,31/*days*/      ,24/*hours*/      ,59/*minutes*/    ,59/*seconds*/};

	uint8_t  i;

    //nullify the alarm event bit from previous usage
	_event_clear(alarm_event_g, ALARM1_TRIGGERED_BIT);

    //check that the given date is legal
	for(i = 0; i < RTC_NUM_OF_ALARM_BYTES_BCD; ++i)
	{
		if(dt_bcd[i]&0xF0 <= RTC_10_DIGIT_LIMITS[i] && //checks wheter the 10 digits in the BCD representation of dt_bcd is legal
			dt_bcd[i]&0x0F <= 9 && //checks wheter the first digit in the BCD  representation of dt_bcd is legal
			((dt_bcd[i]&0xF0>>4)*10 + dt_bcd[i]&0x0F) <= RTC_TIME_RANGE_LIMITS[i]//checks wheter the whole BCD date value is legal: If there are more than 12 months, more than 24 hours etc...
		  )
		{
			alarm_data_buff[i] |= dt_bcd[i];
		}
		else
		{
			printf("rtc_set_alarm1_once: ERROR: the provided date had an illegal bcd format\n");
			return  FALSE;
		}
	}

	/*
	    Before setting the timer address, it seems like a good idea to
		nullify the AF flag on the RTC by reading it with the other flags
	*/
	cmd_buff = RTC_FLAGS_ADDR; 

	if(!rtc_receive_data(&cmd_buff,1 ,&flag_data_buff, 1))
	{
		printf("rtc_set_alarm1_once: ERROR: could not read and nuliffy the bytes related to alarm1\n");
		return FALSE;
	}
	/*	if(!rtc_receive_data(&cmd_buff,1 &,flag_data_buff, 1) && 0 != (flag_data_buff&RTC_ALRM1_FLAG_OFFSET) )//check if this read nullified the bit
	{
		printf("rtc_set_alarm1_once: ERROR: alarm flag hasn't been nullified automatically after reading it once \n")//as it should ,at least by what is written in the manual...
		return FALSE;
	}*/

	cmd_buff = RTC_ALRM1_MONTH_ADDR; 

	if(!rtc_send_data(&cmd_buff,1 ,alarm_data_buff, RTC_NUM_OF_ALARM_BYTES_BCD))
	{
		printf("rtc_set_alarm1_once: ERROR: could not update the bytes related to alarm1\n");
		return  FALSE;
	}

    //Activate the polling for the alarm 
    _event_set(alarm_event_g, ALARM1_ACTIVATE_BIT);  

	return TRUE;
}

void rtc_periodically_check_alarm1(uint32_t *time_diff)
{
    static uint32_t time_since_last_alarm_poll;
    uint32_t alarm_bits_s = 0;

    uint8_t flag_cmd_buff = RTC_FLAGS_ADDR;
    uint8_t flag_data_buff;

    if(MQX_OK != _event_get_value(alarm_event_g, &alarm_bits_s))
    {
        printf("rtc_is_alarm1_bit_set: ERROR: alarm event doesn't exist\n");
    }


    if (0 != alarm_bits_s&ALARM1_ACTIVATE_BIT && WAIT_BETWEEN_ALARM_POLL_TIME < (time_since_last_alarm_poll+=(*time_diff))) 
    {
       
        time_since_last_alarm_poll = 0;

        if(!rtc_receive_data(&flag_cmd_buff,1 ,&flag_data_buff, 1))
        {
            printf("rtc_check_flags: ERROR: couldn't read the flagS\n");
            return;
        }
        if(0 != flag_data_buff&RTC_ALRM1_FLAG_OFFSET)
        {
            _event_clear(alarm_event_g, ALARM1_ACTIVATE_BIT);
            _event_set(alarm_event_g,ALARM1_TRIGGERED_BIT);
        }
    }

    return;
}

bool rtc_check_if_alarm1_has_been_triggered()
{
    uint32_t alarm_bits_s = 0;

    if(MQX_OK != _event_get_value(alarm_event_g, &alarm_bits_s))
    {
        printf("rtc_is_alarm1_bit_set: ERROR: alarm event doesn't exist\n");
        return false;
    }

    return (0 != alarm_bits_s&ALARM1_ACTIVATE_BIT);
}

/*returns the time set of the current registered alarm*/
/*
void rtc_get_alarm1_time(uint8_t *dt_bcd)
{
    uint8_t rtc_alarm1_addr = RTC_ALRM1_MONTH_ADDR;

    if(!rtc_receive_data(&rtc_alarm1_addr, 1, dt_bcd, RTC_NUM_OF_ALARM_BYTES_BCD))
    {
        printf("fetching alarm time failed \n");
    }

    dt_bcd[0] &= 0x1F;//month 
    dt_bcd[1] &= 0x3F;//day
    dt_bcd[2] &= 0x3F;//hour
    dt_bcd[3] &= 0x7F;//minutes
	dt_bcd[4] &= 0x7F;//seconds
}*/

/* returns TRUE if the oscillator is running fine, also clears oscil. flag if set */
bool rtc_check_oscillator(void)
{
	uint8_t flags = 0 ;
	uint8_t cmd_buff = RTC_FLAGS_ADDR;
	if (!rtc_receive_data (&cmd_buff,  1, &flags, 1))
	{
		printf("rtc_check_oscillator: failed i2c read \n");
		return FALSE;
	}
	/* Oscillator fail flag is high when oscil. has stopped or was stopped */
	if ((flags & 0x4))
	{
		rtc_oscillator_kick_start();
		/* The oscillator must start and have run for at least 4 seconds before
		attempting to reset the OF bit to 0 pg 44 of RTC data sheet */
		printf("rtc_check_oscillator: Sleep 4 seconds before clearing oscillator flag \n");
		_time_delay(4100);

		/* clear oscillator flag */
		flags &= ~0x4;
		if (!rtc_send_data (&cmd_buff,  1, &flags, 1))
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
	if (!rtc_receive_data (&cmd_buff,  1, &alarm1_hr, 1))
	{
		printf("rtc_check_halt_bit: failed i2c read \n");
		return FALSE;
	}
	/* Oscillator fail flag is high when oscil. has stopped or was stopped */
	if ((alarm1_hr & 0x40))
	{
		/* clear halt bit */
		alarm1_hr &= ~0x40;
		if (!rtc_send_data(&cmd_buff,  1, &alarm1_hr, 1))
		{
			printf("rtc_check_halt_bit: clearing halt bit failed \n");
			return FALSE;
		}
		return FALSE;
	}
	return TRUE;
}

/* returns TRUE if the battery is good, FALSE if no battery or low battery */
bool rtc_check_battery_good(void)
{
	uint8_t flags = 0 ;
	uint8_t cmd_buff = RTC_FLAGS_ADDR;
	if (!rtc_receive_data(&cmd_buff,  1, &flags, 1))
	{
		printf("rtc_check_battery_good: failed i2c read \n");
		return FALSE;
	}
	/* Oscillator fail flag is high when oscil. has stopped or was stopped */
	if ((flags & 0x10))
	{
		return FALSE;
	}
	return TRUE;
}

/*returns TRUE if the alarm1 AF1 bit is working correctly by sending a time that already passed and checking if the bit 
  is being set*/
bool rtc_check_alarm1_working(void)
{	
	uint8_t flag_cmd_buff = RTC_FLAGS_ADDR;
    uint8_t flag_data_buff = 0;
	uint8_t current_dt_bcd[RTC_BCD_SIZE];
	uint8_t i;

     _mqx_uint event_result;

	rtc_get_time(current_dt_bcd);

	/*try to set current_dt_bcd to a time that have already passed and then check the alarm bit
	   the fastest way to get time that has already passed is to nullify the first of the seconds/minutes etc...
	    that is not zero in our currect time for example : if  the hour is 17:20:35 then 17:20:00 has passed already
		just need to take note that miliseconds aren't included so it should start in 1 and that*/
	for(i = 1; i < RTC_BCD_SIZE; ++i)
	{
		if(current_dt_bcd[i] > 0)
		{
			current_dt_bcd[i] = 0;
			break;
		}
	}

	if(RTC_BCD_SIZE == i)
	{
		printf("rtc_check_alarm1_working: FAILED to retreive the correct time, retreived an array of zeros instead \n");
		return FALSE;
	}

	if(FALSE == rtc_set_alarm1_once(current_dt_bcd))
	{
		printf("rtc_check_alarm1_working: FAILED to set ALARM to past time \n");
		return FALSE;
	}

	if(!rtc_receive_data(&flag_cmd_buff,1 ,&flag_data_buff, 1) && 1 != (flag_data_buff&RTC_ALRM1_FLAG_OFFSET))
	{
		printf("rtc_check_alarm1_working: FAILED to measure alarm bit after setting the alarm to past time");
		return FALSE;
	}
   
   /*if timer works correctly should create an event handler for the AF1 flag and alarm activation
     before the routine alarm flag check will be activated*/
    event_result = _event_create("event.TimerEvent");

	if(MQX_OK != event_result){
		printf("rtc_check_alarm_working: Could not create Timer event \n");
	}

    event_result = _event_open("event.TimerEvent", &alarm_event_g);
	if(MQX_OK != event_result){
		printf("rtc_check_alarm_working: Could not open Timer event \n");
	}

	return TRUE;
}

//true is to activate false is to deactivate
void rtc_activate_or_deactivate_alarm_polling(bool should_poll)
{
	if(should_poll)
	{
		_event_set(alarm_event_g, ALARM1_ACTIVATE_BIT);
	}
	else
	{
		_event_clear(alarm_event_g, ALARM1_ACTIVATE_BIT);
	}
}

/* returns TRUE if successful */
bool rtc_oscillator_kick_start(void)
{
	uint8_t cmd_buff = RTC_SECOND_ADDR;
	uint8_t tx_buff = 0;

	if (!rtc_receive_data (&cmd_buff,  1, &tx_buff, 1))
	{
		printf("rtc_oscillator_kick_start: reading second address failed \n");
	}
	/* setting bit D7 of register 01h to cause the oscillator to stop */
	tx_buff |= 0x80;
	if (!rtc_send_data (&cmd_buff,  1, &tx_buff, 1))
	{
		printf("rtc_oscillator_kick_start: setting stop bit failed \n");
		return FALSE;
	}
	/* clearing bit D7 of register 01h to cause the oscillator to start */
	tx_buff &= 0x7f;
	if (!rtc_send_data (&cmd_buff,  1, &tx_buff, 1))
	{
		printf("rtc_oscillator_kick_start: clearing stop bit failed \n");
		return FALSE;
	}
	return TRUE;
}


void rtc_init(void)
{
	I2C_Enable(RTC_I2C_PORT); /* Note:Both accelerometer and RTC are on the same bus*/
	if (!rtc_check_oscillator())
	{
		printf("rtc_init: oscillator not good \n");
	}

	if (!rtc_check_halt_bit())
	{
		printf("rtc_init: halt bit set \n");
	}
	if (!rtc_check_battery_good())
	{
		printf("rtc_init: battery low bit set \n");
	}
	if(!rtc_check_alarm1_working())
	{
		printf("alarm bit doesn't work correctly \n");
	}

#ifdef RTC_DEBUG	
	rtc_test();
#endif
}



/* dt_str format : year-month-day hour:minute:seconds.decisecond (Always GMT)
 * 			  Ex : 2016-03-29 19:09:06.58  (22 chars + 1(for \0))
 */
void rtc_convert_bcd_to_string(uint8_t * dt_bcd, char * dt_str, bool print_time)
{
	uint8_t hundreth_sec_int = ((dt_bcd[0]>>4)* 10) + (dt_bcd[0]&0x0F);
	uint8_t seconds = (((dt_bcd[1]>>4)&0x7) * 10) + (dt_bcd[1]&0x0F);
	uint8_t minutes = (((dt_bcd[2]>>4)&0x7) * 10) + (dt_bcd[2]&0x0F);
	uint8_t hours = (((dt_bcd[3]>>4)&0x3) * 10) + (dt_bcd[3]&0x0F);
	uint8_t century = dt_bcd[3]>>6;
	//uint8_t day_of_week = dt[4]&0x7;
	uint8_t day_of_month = (((dt_bcd[5]>>4)&0x3) * 10) + (dt_bcd[5]&0x0F);
	uint8_t month = (((dt_bcd[6]>>4)&0x1) * 10) + (dt_bcd[6]&0x0F);
	uint16_t year = ((dt_bcd[7]>>4) * 10) + (dt_bcd[7]&0x0F);

	year = 2000 + (century * 100) + year;

	snprintf(dt_str, RTC_STRING_SIZE , "%04d-%02d-%02d %02d:%02d:%02d.%02d\0",
			year, month, day_of_month, hours, minutes, seconds, hundreth_sec_int);
	if (print_time)
	{
		printf("rtc date_time: %04d-%02d-%02d %02d:%02d:%02d.%02d\n",
				year, month, day_of_month, hours, minutes, seconds, hundreth_sec_int);
	}
}

/* rtc_get : Gets time value from RTC (8 bytes BCD format).
 * dt_bcd is in BCD format as described in the datasheet CD00127116.pdf (M41T82)
 *
 * Note: For debugging you can uncomment rtc_convert_bcd_to_string() to print time
 */
void rtc_get_time(uint8_t * dt_bcd)
{
	uint8_t cmd_buff = RTC_DECI_SEC_ADDR;
	uint8_t i = 0;
#ifdef RTC_DEBUG
	char dt_str[RTC_STRING_SIZE] = {0};
#endif
	if (!rtc_check_oscillator())
	{
		printf("rtc_get: oscillator not good \n");
	}

	if (!rtc_check_halt_bit())
	{
		printf("rtc_init: halt bit set \n");
	}

	cmd_buff = RTC_DECI_SEC_ADDR;
	if (!rtc_receive_data (&cmd_buff,  1, dt_bcd, 8))
	{
		printf("rtc_get: read time failed \n");
		/* Fill RTC return with zeros on failure */
		for(i=0; i<8; i++)
		{
			dt_bcd=0x0;
		}
		return;
	}
#ifdef RTC_DEBUG
	rtc_convert_bcd_to_string(dt_bcd, dt_str, TRUE);
#endif
	return;
}

/* rtc_set : Sets time value from RTC (8 bytes BCD format).
 * dt_bcd is in BCD format as described in the datasheet CD00127116.pdf (M41T82)
 *
 * Note: rtc_convert_bcd_to_string() describes the conversion from bcd to string
 */
void rtc_set_time(uint8_t * dt_bcd)
{
	uint8_t cmd_buff = RTC_DECI_SEC_ADDR;

	if (!rtc_send_data(&cmd_buff,  1, dt_bcd, 8))
	{
		printf("rtc_set: set time failed \n");
	}
}

/* rtc_get_cal_register : Gets the analog and digital calibration register */
void rtc_get_cal_register(uint8_t * digital_cal, uint8_t * analog_cal)
{
	uint8_t cmd_buff = RTC_DIGITAL_CAL;

	if (!rtc_receive_data (&cmd_buff,  1, digital_cal, 1))
	{
		printf("rtc_get_cal_register: get digital cal failed \n");
	}

	cmd_buff = RTC_ANALOG_CAL;
	if (!rtc_receive_data (&cmd_buff,  1, analog_cal, 1))
	{
		printf("rtc_get_cal_register: get analog cal failed \n");
	}
}

/* rtc_set_cal_register : Sets the analog and digital calibration register */
void rtc_set_cal_register(uint8_t *digital_cal, uint8_t *analog_cal)
{
	uint8_t cmd_buff = RTC_DIGITAL_CAL;

	if (!rtc_send_data(&cmd_buff,  1, digital_cal, 1))
	{
		printf("rtc_set_cal_register: set digital cal failed \n");
	}

	cmd_buff = RTC_ANALOG_CAL;
	if (!rtc_send_data(&cmd_buff,  1, analog_cal, 1))
	{
		printf("rtc_set_cal_register: set analog cal failed \n");
	}
}


#ifdef RTC_DEBUG
/*RTC init needs to run before running RTC test */
void rtc_test(void)
{
	uint8_t dt_bcd[RTC_BCD_SIZE] = {0};
	char dt_str[RTC_STRING_SIZE] = {0};
	// dt_str : 2016-03-29 19:09:06.58
	dt_bcd[0] = 0x58; /* DeciSecond */
	dt_bcd[1] = 0x06; /* Seconds */
	dt_bcd[2] = 0x09; /* Minutes */
	dt_bcd[3] = 0x19;   /* century/hours */
	dt_bcd[4] = 0x4 ; /* Day of week */
	dt_bcd[5] = 0x29 ; /* Day of month */
	dt_bcd[6] = 0x03; /* Month */
	dt_bcd[7] = 0x16; /* Year */
	//rtc_set_time(dt_bcd);
	_time_delay(1000);
	rtc_get_time(dt_bcd);
	rtc_convert_bcd_to_string(dt_bcd, dt_str, TRUE);
}
#endif
