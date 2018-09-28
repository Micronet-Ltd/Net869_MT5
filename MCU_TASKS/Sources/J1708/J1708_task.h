
#ifndef J1708_TASK_H_
#define J1708_TASK_H_

#include <event.h>

#define EVENT_J1708_RX	1<<0
#define EVENT_J1708_DISABLE 1<<1
#define EVENT_J1708_ENABLE 1<<2

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
