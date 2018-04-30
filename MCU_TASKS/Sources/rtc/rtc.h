/*
 * rtc.h
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 */

#ifndef _RTC_RTC_H_
#define _RTC_RTC_H_

#define RTC_BCD_SIZE					8
#define RTC_NUM_OF_ALARM_BYTES			5

void rtc_init(void);
void rtc_get_time(uint8_t * dt_bcd);
void rtc_set_time(uint8_t * dt_bcd);
void rtc_get_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
void rtc_set_cal_register(uint8_t *digital_cal, uint8_t *analog_cal);
bool rtc_receive_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
bool rtc_send_data (uint8_t * cmd, uint8_t cmd_size, uint8_t * data, uint8_t data_size);
void rtc_test(void);

/*Reading the flags in the rtc is unsafe as it nullifies the alarm bits(P 36 in the CD00127116 manual section 3.5).
  Here is an implementation of a safe flag reading function.
*/
bool rtc_check_flags(uint8_t *flags_to_return)

/*  dt_bcd should be given as binary date ,this function will resolve into polling the date directly 
    if alarm1 doesn't work properly*/
void rtc_set_time_for_alarm1(uint8_t *dt_bcd);

/*returns TRUE if the time set in rtc_set_time_for_alarm1, passed, FALSE otherwise.
In case the time has passed, it is now safe to configure new alarm and to reset the alarm*/
bool rtc_check_time_for_alarm1();


#endif /* _RTC_RTC_H_ */
