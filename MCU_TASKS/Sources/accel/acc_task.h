/*
 * acc_task.h
 *
 *  Created on: Apr 25, 2016
 *      Author: abid.esmail
 */

#ifndef _ACC_TASK_H_
#define _ACC_TASK_H_

void AccEnable (void);
void AccDisable (void);
bool accInit (void);
void AccReadRegister(uint8_t address, uint8_t * read_data);
void AccWriteRegister(uint8_t address, uint8_t write_data);

#endif /* _ACC_TASK_H_ */
