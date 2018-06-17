/*
 * rtc.h
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 */

#ifndef _RTC_RTC_H_
#define _RTC_RTC_H_

#define RTC_BCD_SIZE				      	8
#define RTC_NUM_OF_ALARM_BYTES_BCD			5
#define RTC_DEFAULT_MILISEC_WAIT_POLL       100000 

#define WATCH_DOG_BIT                       (1 << 7)
#define ALARM1_ACTIVATE_BIT                 (1 << 6)     
#define BATTERY_LOW_BIT                     (1 << 4)
#define OCCILATOR_BIT                       (1 << 2)
#define ALARM1_EXIST_BIT                    1  
  
extern void *rtc_flags_g;
extern uint64_t poll_timeout_g;

void rtc_init(void);
void rtc_set_time(uint8_t * dt_bcd);
void rtc_get_time(uint8_t * dt_bcd);
void rtc_get_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
void rtc_set_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
bool rtc_receive_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
bool rtc_send_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
void rtc_test(void);
//bool rtc_check_if_alarm_date_passed();
bool rtc_check_if_alarm_date_iz_zero();

void rtc_get_init_flags(uint32_t *flags);

bool rtc_set_alarm1(uint8_t *dt_bcd);

bool rtc_get_flags();

bool rtc_alarm1_is_trigered();

#endif /* _RTC_RTC_H_ */
