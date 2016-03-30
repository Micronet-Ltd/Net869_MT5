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
void rtc_get(uint8_t * dt_bcd);
void rtc_set(uint8_t * dt_bcd);
void rtc_test(void);



#endif /* _RTC_RTC_H_ */
