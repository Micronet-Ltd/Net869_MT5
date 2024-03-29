
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
*   This include file is used to define constants and data types for
*   the light weight semaphore component.
*
*
*END************************************************************************/

#ifndef __lwsem_h__
#define __lwsem_h__ 1

#include "mqx.h"
//#include "mqx_cnfg.h"
//#include "psptypes.h"

/*                         LWSEM STRUCTURE */
/*!
 * \brief Lightweight semaphore.
 *
 * This structure defines a lightweight semaphore.
 * \n These sempahores implement a simple counting semaphore.
 * \n Tasks wait on these semaphores in a FIFO manner.
 * \n Priority inheritance is NOT implemented for these semaphores.
 * \n The semaphores can be embedded into data structures similarly to mutexes.
 *
 * \see _lwsem_create
 * \see _lwsem_create_hidden
 * \see _lwsem_destroy
 * \see _lwsem_poll
 * \see _lwsem_post
 * \see _lwsem_usr_check
 * \see _lwsem_wait
 * \see _lwsem_wait_for
 * \see _lwsem_wait_ticks
 * \see _lwsem_wait_until
 * \see _usr_lwsem_create
 * \see _usr_lwsem_destroy
 * \see _usr_lwsem_poll
 * \see _usr_lwsem_post
 * \see _usr_lwsem_wait
 * \see _usr_lwsem_wait_for
 * \see _usr_lwsem_wait_ticks
 * \see _usr_lwsem_wait_until
 */
typedef struct lwsem_struct
{

	/* The next two fields are used to maintain a list of all LWSEMS */

	/*! \brief Pointer to the next lightweight semaphore in the list of lightweight
	 *  semaphores. */
	struct lwsem_struct       *NEXT;

	/*! \brief Pointer to the previous lightweight semaphore in the list of
	 *  lightweight semaphores. */
	struct lwsem_struct       *PREV;

	/*! \brief  Manages the queue of tasks that are waiting for the lightweight
	 *  semaphore. The NEXT and PREV fields in the task descriptors link the tasks. */
	QUEUE_STRUCT               TD_QUEUE;

	/*! \brief When MQX creates the lightweight semaphore, it initializes the field.
	 *  When MQX destroys the lightweight semaphore, it clears the field. */
	_mqx_uint                  VALID;

	/*! \brief Count of the semaphore. MQX decrements the field when a task waits
	 *  for the semaphore. If the field is not 0, the task gets the semaphore. If
	 *  the field is 0, MQX puts the task in the lightweight semaphore queue until
	 *  the count is a non-zero value.the semaphore value. */
	_mqx_int                   VALUE;

} LWSEM_STRUCT, * LWSEM_STRUCT_PTR;




/*--------------------------------------------------------------------------*/
/*                           EXTERNAL DECLARATIONS                          */

#ifdef __cplusplus
extern "C" {
#endif

	extern _mqx_uint        _lwsem_create(LWSEM_STRUCT_PTR, _mqx_int);
	extern _mqx_uint        _lwsem_create_hidden(LWSEM_STRUCT_PTR, _mqx_int);
	extern _mqx_uint        _lwsem_destroy(LWSEM_STRUCT_PTR);
	extern   bool        _lwsem_poll(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _lwsem_post(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _lwsem_test(void **, void **);
	extern _mqx_uint        _lwsem_wait(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _lwsem_wait_ticks(LWSEM_STRUCT_PTR, _mqx_uint);
	extern _mqx_uint        _lwsem_wait_for(LWSEM_STRUCT_PTR, MQX_TICK_STRUCT_PTR);
	extern _mqx_uint        _lwsem_wait_until(LWSEM_STRUCT_PTR, MQX_TICK_STRUCT_PTR);

#if MQX_ENABLE_USER_MODE
	extern _mqx_uint        _usr_lwsem_create(LWSEM_STRUCT_PTR, _mqx_int);
	extern _mqx_uint        _usr_lwsem_destroy(LWSEM_STRUCT_PTR);
	extern   bool        _usr_lwsem_poll(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _usr_lwsem_post(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _usr_lwsem_wait(LWSEM_STRUCT_PTR);
	extern _mqx_uint        _usr_lwsem_wait_ticks(LWSEM_STRUCT_PTR, _mqx_uint);
	extern _mqx_uint        _usr_lwsem_wait_for(LWSEM_STRUCT_PTR, MQX_TICK_STRUCT_PTR);
	extern _mqx_uint        _usr_lwsem_wait_until(LWSEM_STRUCT_PTR, MQX_TICK_STRUCT_PTR);
	extern _mqx_uint        _lwsem_usr_check(LWSEM_STRUCT_PTR);
#endif /* MQX_ENABLE_USER_MODE */

#ifdef __cplusplus
}
#endif

#endif /* __lwsem_h__ */
/* EOF */
