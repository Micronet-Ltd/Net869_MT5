
#ifndef J1708_TASK_H_
#define J1708_TASK_H_

#include <event.h>

#define EVENT_J1708_RX	1

#ifdef __cplusplus
extern "C"
{
#endif

extern void * g_J1708_event_h;

extern void J1708_enable  (uint8_t priority);
extern void J1708_disable (void);

#ifdef __cplusplus
}
#endif

#endif /* J1708_TASK_H_ */

