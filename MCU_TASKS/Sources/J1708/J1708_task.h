
#ifndef J1708_TASK_H_
#define J1708_TASK_H_

#include <event.h>

#define EVENT_J1708_RX	1

void * g_J1708_event_h;

void J1708_enable  (uint8_t priority);
void J1708_disable (void);

#endif /* J1708_TASK_H_ */
