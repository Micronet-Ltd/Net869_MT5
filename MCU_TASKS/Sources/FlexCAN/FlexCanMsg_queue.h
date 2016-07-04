
/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 2004-2008 Embedded Access Inc.
* Copyright 1989-2008 ARC International
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the queue functions.
*
*
*END************************************************************************/

#ifndef __FlexCanMsg_queue_h__
#define __FlexCanMsg_queue_h__ 1

/*--------------------------------------------------------------------------*/
/*                     DATA STRUCTURE DEFINITIONS                           */


/*--------------------------------------------------------------------------*/
/*                       EXTERNAL DECLARATIONS                              */

#ifdef __cplusplus
extern "C" {
#endif

extern QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_dequeue  (QUEUE_STRUCT_PTR);
extern bool                     FlexCanMsg_queue_enqueue  (QUEUE_STRUCT_PTR, QUEUE_ELEMENT_STRUCT_PTR);
extern _mqx_uint                FlexCanMsg_queue_get_size (QUEUE_STRUCT_PTR);
extern void                     FlexCanMsg_queue_init     (QUEUE_STRUCT_PTR, uint16_t);
extern bool                     FlexCanMsg_queue_insert   (QUEUE_STRUCT_PTR, QUEUE_ELEMENT_STRUCT_PTR, QUEUE_ELEMENT_STRUCT_PTR);
extern bool                     FlexCanMsg_queue_is_empty (QUEUE_STRUCT_PTR);
extern QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_head     (QUEUE_STRUCT_PTR);
extern QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_next     (QUEUE_STRUCT_PTR, QUEUE_ELEMENT_STRUCT_PTR);
extern void                     FlexCanMsg_queue_unlink   (QUEUE_STRUCT_PTR, QUEUE_ELEMENT_STRUCT_PTR);
extern _mqx_uint                FlexCanMsg_queue_test     (QUEUE_STRUCT_PTR, void   **);

#ifdef __cplusplus
}
#endif

#endif  /* __FlexCanMsg_queue_h__ */
/* EOF */
