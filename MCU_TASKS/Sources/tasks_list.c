#include "tasks_list.h"


TASK_TEMPLATE_STRUCT MQX_template_list[] = 
{ 
//  Task number,	Entry point,	Stack, Pri,		String, 		    Auto start ? 
   {MAIN_TASK  ,	Main_task  ,	1500,	8 ,		"MAIN_TASK" ,	MQX_AUTO_START_TASK},
   {ACC_TASK   ,	Acc_task   ,	1500,	9 ,		"ACC_TASK"  ,	0                  },
/*   
   {USB_TASK   ,	Usb_task   ,	1500,	9 ,		"USB_TASK"  ,	0                  },
   {CAN_TASK   ,	Can_task   ,	1500,	9 ,		"CAN_TASK"  ,	0                  },
   {J1708_TASK ,	J1708_task ,	1500,	9 ,		"J1708_TASK",	0                  },
   {ACC_TASK   ,	Acc_task   ,	1500,	9 ,		"ACC_TASK"  ,	0                  },
*/
   {0          ,	0          ,	0   ,	0 ,		0           ,	0                  }
};
