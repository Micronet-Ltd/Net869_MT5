
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

#define MQX_FORCE_USE_INLINE_MACROS     1

#include "mqx_inc.h"
#include "FlexCanMsg_queue.h"
#include "message.h"
#include "mic_typedef.h"

static uint32_t g_BasePri_prev = 0;

void FlexCanMsg_Lock() {
	g_BasePri_prev = __get_BASEPRI ();
	__set_BASEPRI((CAN_NVIC_IRQ_Priority << 4) & 0xFF);
}

void FlexCanMsg_UnLock () {
	__set_BASEPRI(g_BasePri_prev);
}

#define QUEUE_ATOMIC_SET(x, set)

//#define QUEUE_ATOMIC_SET(x, set) \
//    do  { \
//    }while( set == __LDREXW (x) ); \
//    __STREXW( set, x );
//

/*!
 * \brief Initializes the provided queue.
 *
 * \param[in] q_ptr Pointer to the queue to initialize.
 * \param[in] size  One of the following:
 * \li Maximum number of elements that the queue can hold.
 * \li 0 (Unlimited number.)
 *
 * \warning If q_ptr is not a pointer to QUEUE_STRUCT, the function might behave
 * unpredictably.
 *
 * \see _queue_enqueue
 * \see _queue_dequeue
 * \see QUEUE_STRUCT
 */
void FlexCanMsg_queue_init (QUEUE_STRUCT_PTR q_ptr, uint16_t size )
{ /* Body */

	_QUEUE_INIT(q_ptr, size);

} /* Endbody */

/*!
 * \brief Adds the element to the end of the queue.
 *
 * \param[in] q_ptr Pointer to the queue to which to add the element; initialized
 * with _queue_init().
 * \param[in] e_ptr Pointer to the element to add.
 *
 * \return TRUE (success) or FALSE (Failure: the queue is full.)
 *
 * \warning The function might behave unpredictably if either:
 * \li Q_ptr is not a pointer to QUEUE_STRUCT.
 * \li E_ptr is not a pointer to QUEUE_ELEMENT_STRUCT.
 *
 * \see _queue_init
 * \see _queue_dequeue
 * \see _queue_init
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
bool FlexCanMsg_queue_enqueue ( QUEUE_STRUCT_PTR q_ptr, QUEUE_ELEMENT_STRUCT_PTR e_ptr )
{ /* Body */

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);
	if ((q_ptr->MAX != 0) && (q_ptr->SIZE >= q_ptr->MAX))
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		return (FALSE);
	} /* Endif */

	_QUEUE_ENQUEUE(q_ptr, e_ptr);
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();
	return (TRUE);

} /* Endbody */

/*!
 * \brief Removes the first element from the queue.
 *
 * \param[in] q_ptr Pointer to the queue from which to remove the first element;
 * initialized with _queue_init().
 *
 * \return Pointer to the removed first queue element.
 * \return NULL (Queue is empty.)
 *
 * \warning If q_ptr is not a pointer to QUEUE_STRUCT, the function might behave
 * unpredictably.
 *
 * \see _queue_enqueue
 * \see _queue_init
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_dequeue ( QUEUE_STRUCT_PTR q_ptr)
{ /* Body */
	QUEUE_ELEMENT_STRUCT_PTR    e_ptr;

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);
	if (q_ptr->SIZE == 0)
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		return (NULL);
	} /* Endif */

	_QUEUE_DEQUEUE(q_ptr, e_ptr);
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();
	return (e_ptr);

} /* Endbody */

/*!
 * \brief Gets a pointer to the element at the start of the queue, but do not
 * remove the element.
 *
 * \param[in] q_ptr Pointer to the queue to use; initialized with _queue_init().
 *
 * \return Pointer to the element that is at the start of the queue.
 * \return NULL (Queue is empty.)
 *
 * \warning If q_ptr is not a pointer to QUEUE_STRUCT, the function might behave
 * unpredictably.
 *
 * \see _queue_dequeue
 * \see _queue_init
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_head ( QUEUE_STRUCT_PTR q_ptr )
{ /* Body */
	QUEUE_ELEMENT_STRUCT_PTR e_ptr;

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);
	if (q_ptr->SIZE == 0)
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		return (NULL);
	} /* Endif */

	e_ptr = q_ptr->NEXT;
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();
	return (e_ptr);

} /* Endbody */

/*!
 * \brief Inserts the element in the queue.
 *
 * \param[in] q_ptr  Pointer to the queue to insert into; initialized with
 * _queue_init().
 * \param[in] qe_ptr One of the following:
 * \li Pointer to the element after which to insert the new element.
 * \li NULL (Insert the element at the start of the queue.)
 * \param[in] e_ptr  Pointer to the element to insert.
 *
 * \return TRUE (success) or FALSE (Failure: queue is full.)
 *
 * \warning The function might behave unpredictably if either:
 * \li Q_ptr is not a pointer to QUEUE_STRUCT.
 * \li E_ptr is not a pointer to QUEUE_ELEMENT_STRUCT.
 *
 * \see _queue_init
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
bool FlexCanMsg_queue_insert ( QUEUE_STRUCT_PTR q_ptr, QUEUE_ELEMENT_STRUCT_PTR qe_ptr, QUEUE_ELEMENT_STRUCT_PTR e_ptr )
{ /* Body */

	if (qe_ptr == NULL)
	{
		/* Insert at the front */
		qe_ptr = (QUEUE_ELEMENT_STRUCT_PTR) ((void *) q_ptr);
	} /* Endif */

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);
	if ((q_ptr->MAX != 0) && (q_ptr->SIZE >= q_ptr->MAX))
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		return (FALSE);
	} /* Endif */

	_QUEUE_INSERT(q_ptr, qe_ptr, e_ptr);
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();
	return (TRUE);

} /* Endbody */

/*!
 * \brief Gets a pointer to the element after the provided one in the queue, but
 * do not remove the element.
 *
 * This function returns NULL if either:
 * \li E_ptr is NULL.
 * \li E_ptr is a pointer to the last element.
 *
 * \param[in] q_ptr Pointer to the queue for which to get info; initialized with
 * _queue_init().
 * \param[in] e_ptr Get the element after this one.
 *
 * \return Pointer to the next queue element (success).
 * \return NULL (Failure: see Description.)
 *
 * \warning The function might behave unpredictably if either:
 * \li Q_ptr is not a pointer to QUEUE_STRUCT.
 * \li E_ptr is not a pointer to QUEUE_ELEMENT_STRUCT.
 *
 * \see _queue_init
 * \see _queue_dequeue
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
QUEUE_ELEMENT_STRUCT_PTR FlexCanMsg_queue_next ( QUEUE_STRUCT_PTR q_ptr, QUEUE_ELEMENT_STRUCT_PTR e_ptr )
{ /* Body */
	QUEUE_ELEMENT_STRUCT_PTR    next_ptr;

	if (e_ptr == NULL)
	{
		return NULL;
	} /* Endif */
	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);

	next_ptr = e_ptr->NEXT;

	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();

	if (next_ptr == (QUEUE_ELEMENT_STRUCT_PTR) ((void *) q_ptr))
	{
		/* At end of queue */
		next_ptr = NULL;
	} /* Endif */
	return (next_ptr);

} /* Endbody */

/*!
 * \brief Tests the queue for consitstency and validity.
 *
 * This function checks the queue pointers to ensure that they form a circular,
 * double linked list, with the same number of elements that the queue header
 * specifies.
 *
 * \param[in]  q_ptr                Pointer to the queue to test; initialized
 * with _queue_init().
 * \param[out] element_in_error_ptr Pointer to the first element with an error
 * (initialized only if an error is found).
 *
 * \return MQX_OK (No errors were found.)
 * \return MQX_CORRUPT_QUEUE (An error was found.)
 *
 * \see _queue_init
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
_mqx_uint FlexCanMsg_queue_test ( QUEUE_STRUCT_PTR q_ptr, void **element_in_error_ptr )
{ /* Body */
	QUEUE_ELEMENT_STRUCT_PTR element_ptr;
	QUEUE_ELEMENT_STRUCT_PTR prev_ptr;
	_mqx_uint size;

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);

	size = _QUEUE_GET_SIZE(q_ptr) + 1;
	element_ptr = q_ptr->NEXT;
	prev_ptr = (QUEUE_ELEMENT_STRUCT_PTR)((void *)q_ptr);
	while (--size)
	{
		if (element_ptr == (void *)q_ptr)
		{
			QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
			FlexCanMsg_UnLock();
			/* Size too big for # elements on queue */
			*element_in_error_ptr = element_ptr;
			return(MQX_CORRUPT_QUEUE);
		} /* Endif */
		if (element_ptr->PREV != prev_ptr)
		{
			QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
			FlexCanMsg_UnLock();
			*element_in_error_ptr = element_ptr;
			return(MQX_CORRUPT_QUEUE);
		} /* Endif */
		prev_ptr = element_ptr;
		element_ptr = element_ptr->NEXT;
	} /* Endwhile */

	/* Does the last element in the ring point back to the queue head */
	if ((void *)element_ptr != (void *)q_ptr)
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		*element_in_error_ptr = element_ptr;
		return(MQX_CORRUPT_QUEUE);
	} /* Endif */

	/* Is the last element in ring pointed to by queues PREV field */
	if (q_ptr->PREV != prev_ptr)
	{
		QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
		FlexCanMsg_UnLock();
		*element_in_error_ptr = element_ptr;
		return(MQX_CORRUPT_QUEUE);
	} /* Endif */

	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();
	return(MQX_OK);

} /* Endbody */

/*!
 * \brief Removes a specified element from the queue.
 *
 * \param[in] q_ptr Pointer to the queue from which to remove the element;
 * initialized with _queue_init().
 * \param[in] e_ptr Pointer to the element to remove.
 *
 * \warning The function might behave unpredictably if either:
 * \li Q_ptr is not a pointer to QUEUE_STRUCT.
 * \li E_ptr is not a pointer to QUEUE_ELEMENT_STRUCT.
 *
 * \see _queue_init
 * \see _queue_dequeue
 * \see QUEUE_STRUCT
 * \see QUEUE_ELEMENT_STRUCT
 */
void FlexCanMsg_queue_unlink ( QUEUE_STRUCT_PTR q_ptr, QUEUE_ELEMENT_STRUCT_PTR e_ptr )
{ /* Body */

	FlexCanMsg_Lock();
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 1);
	_QUEUE_REMOVE(q_ptr, e_ptr);
	QUEUE_ATOMIC_SET((volatile unsigned long*)&(q_ptr->ATOMIC), 0);
	FlexCanMsg_UnLock();

} /* Endbody */

/*!
 * \brief Gets the number of elements in the queue.
 *
 * \param[in] q_ptr Pointer to the queue for which to get info; initialized with
 * _queue_init().
 *
 * \return Number of elements in the queue.
 *
 * \warning If q_ptr is not a pointer to QUEUE_STRUCT, the function might behave
 * unpredictably.
 *
 * \see _queue_enqueue
 * \see _queue_init
 * \see QUEUE_STRUCT
 */
_mqx_uint FlexCanMsg_queue_get_size ( QUEUE_STRUCT_PTR q_ptr )
{ /* Body */

	return _QUEUE_GET_SIZE(q_ptr);

} /* Endbody */

/*!
 * \brief Determines whether the queue is empty.
 *
 * \param[in] q_ptr Pointer to the queue for which to get info; initialized with
 * _queue_init().
 *
 * \return TRUE (Queue is empty.) or FALSE (Queue is not empty.)
 *
 * \warning If q_ptr is not a pointer to QUEUE_STRUCT, the function might behave
 * unpredictably.
 *
 * \see _queue_init
 * \see QUEUE_STRUCT
 */
bool FlexCanMsg_queue_is_empty ( QUEUE_STRUCT_PTR q_ptr )
{ /* Body */

	return _QUEUE_IS_EMPTY(q_ptr);

} /* Endbody */

/* EOF */
