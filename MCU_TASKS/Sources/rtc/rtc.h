/*
 * rtc.h
 *
 *  Created on: Mar 21, 2016
 *      Author: abid.esmail
 */

#ifndef _RTC_RTC_H_
#define _RTC_RTC_H_

void rtc_init(void);
void rtc_get(char * time_val, bool print_time);
void rtc_set(void);



#endif /* _RTC_RTC_H_ */
