/*
 * rtc.h
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 */

#ifndef _RTC_RTC_H_
#define _RTC_RTC_H_

#define RTC_BCD_SIZE					8

void rtc_init(void);
void rtc_get_time(uint8_t * dt_bcd);
void rtc_set_time(uint8_t * dt_bcd);
void rtc_get_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
void rtc_set_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
bool rtc_receive_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
bool rtc_send_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
void rtc_test(void);



#endif /* _RTC_RTC_H_ */
