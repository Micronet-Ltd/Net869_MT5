
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
*  This file contains the functions for lwevent object.
*
*
*END************************************************************************/

#include "mqx_inc.h"

#if MQX_USE_LWEVENTS
#include "lwevent.h"
#include "lwevent_prv.h"

/*!
 * \cond DOXYGEN_PRIVATE
 * \private
 *
 * \brief Used by a task to create an instance of a lightweight event.
 *
 * \param[in] event_ptr Pointer representing location of the event.
 * \param[in] flags     Flags for the light weight event.
 * \param[in] user      User mode
 *
 * \return MQX_OK
 * \return MQX_EINVAL (lwevent is already initialized.)
 * \return MQX_LWEVENT_INVALID (In case of user mode, MQX tries to access
 * a lwevent with inappropriate access rights.)
 *
 * \see _lwevent_create
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_create_internal
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           flags,
	bool             user
)
{
	KERNEL_DATA_STRUCT_PTR  kernel_data;
	LWEVENT_STRUCT_PTR      event_chk_ptr;

#if MQX_ENABLE_USER_MODE
	if (user && !_psp_mem_check_access_mask((uint32_t)event_ptr,
											 sizeof(LWEVENT_STRUCT),
											 MPU_UM_R, MPU_UM_RW) )
	{
		return MQX_LWEVENT_INVALID;
	}
#endif

	_GET_KERNEL_DATA(kernel_data);

	_KLOGE2(KLOG_lwevent_create, event_ptr);

	_QUEUE_INIT(&event_ptr->WAITING_TASKS, 0);
	event_ptr->VALUE = 0;
	event_ptr->FLAGS = flags;

	if (flags & LWEVENT_AUTO_CLEAR)
		event_ptr->AUTO = ~0;
	else
		event_ptr->AUTO = 0;

	_int_disable();

#if MQX_ENABLE_USER_MODE
	if (user)
	{
		if (kernel_data->USR_LWEVENTS.NEXT == NULL)
		{
			/* Initialize the light weight event queue */
			_QUEUE_INIT(&kernel_data->USR_LWEVENTS, 0);
		}
	}
	else
#endif
	{
		if (kernel_data->LWEVENTS.NEXT == NULL)
		{
			/* Initialize the light weight event queue */
			_QUEUE_INIT(&kernel_data->LWEVENTS, 0);
		}
	}

	event_ptr->VALID = LWEVENT_VALID;

#if MQX_CHECK_ERRORS
	/* Check if lwevent is already initialized */
#if MQX_ENABLE_USER_MODE
	if (user)
	{
		event_chk_ptr = (LWEVENT_STRUCT_PTR)((void *)kernel_data->USR_LWEVENTS.NEXT);
		while (event_chk_ptr != (LWEVENT_STRUCT_PTR)((void *)&kernel_data->USR_LWEVENTS))
		{
			if (event_chk_ptr == event_ptr)
			{
				_int_enable();
				_KLOGX2(KLOG_lwevent_create, MQX_EINVAL);
				return(MQX_EINVAL);
			}
			event_chk_ptr = (LWEVENT_STRUCT_PTR)((void *)event_chk_ptr->LINK.NEXT);
		}
	}
	else
#endif
	{
		event_chk_ptr = (LWEVENT_STRUCT_PTR) ((void *) kernel_data->LWEVENTS.NEXT);
		while (event_chk_ptr != (LWEVENT_STRUCT_PTR) ((void *) &kernel_data->LWEVENTS))
		{
			if (event_chk_ptr == event_ptr)
			{
				_int_enable();
				_KLOGX2(KLOG_lwevent_create, MQX_EINVAL);
				return (MQX_EINVAL);
			}
			event_chk_ptr = (LWEVENT_STRUCT_PTR) ((void *) event_chk_ptr->LINK.NEXT);
		}
	}
#endif

#if MQX_ENABLE_USER_MODE
	if (user)
	{
		_QUEUE_ENQUEUE(&kernel_data->USR_LWEVENTS, &event_ptr->LINK);
	}
	else
#endif
	{
		_QUEUE_ENQUEUE(&kernel_data->LWEVENTS, &event_ptr->LINK);
	}

	_int_enable();

	_KLOGX2(KLOG_lwevent_create, MQX_OK);
	return (MQX_OK);
}
/*! \endcond */


/*!
 * \brief Used by a task to create an instance of a lightweight event.
 *
 * \param[in] event_ptr Pointer to the lightweight event to initialize.
 * \param[in] flags     Creation flag; one of the following:
 * \n - LWEVENT_AUTO_CLEAR - all bits in the lightweight event are made autoclearing.
 * \n - 0 - lightweight event bits are not set as autoclearing by default.
 * \n Note: The autoclearing bits can be changed any time later by calling
 * _lwevent_set_auto_clear.
 *
 * \return MQX_OK
 * \return MQX_EINVAL (lwevent is already initialized.)
 * \return MQX_LWEVENT_INVALID (In case of user mode, MQX tries to access
 * a lwevent with inappropriate access rights.)
 *
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_test
 * \see _lwevent_wait_for
 * \see _lwevent_wait_ticks
 * \see _lwevent_wait_until
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_create
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           flags
)
{
#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_create(event_ptr, flags);
	}
#endif

	return _lwevent_create_internal(event_ptr, flags, FALSE);
}


#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to create an instance of a lightweight event.
 *
 * This function is an equivalent to the _lwevent_create() API call but it can be
 * executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer required to meet the memory
 * protection requirements as described in the parameter list below
 *
 * \param[in] event_ptr Read only. Pointer to the lightweight event to initialize.
 * \param[in] flags     Creation flag; one of the following:
 * \n - LWEVENT_AUTO_CLEAR - all bits in the lightweight event are made autoclearing.
 * \n - 0 - lightweight event bits are not set as autoclearing by default.
 * \n Note: The autoclearing bits can be changed any time later by calling
 * _usr_lwevent_set_auto_clear.
 *
 * \return MQX_OK
 * \return MQX_EINVAL (lwevent is already initialized.)
 * \return MQX_LWEVENT_INVALID (In case of user mode, MQX tries to access
 * a lwevent with inappropriate access rights.)
 *
 * \see _lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_create
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           flags
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr,
									(uint32_t)flags, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_CREATE, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \cond DOXYGEN_PRIVATE
 * \private
 *
 * \brief Used by a task to destroy an instance of a lightweight event.
 *
 * \param[in] event_ptr Pointer to the lightweight event to be deinitialized.
 * \param[in] user      User mode
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was not valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_destroy
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_destroy_internal
(
	LWEVENT_STRUCT_PTR  event_ptr,
	bool             user
)
{
	KERNEL_DATA_STRUCT_PTR kernel_data;
#if MQX_COMPONENT_DESTRUCTION
	TD_STRUCT_PTR td_ptr;
#endif

#if MQX_ENABLE_USER_MODE
	if (user && !_psp_mem_check_access_mask((uint32_t)event_ptr,
											sizeof(LWEVENT_STRUCT),
											MPU_UM_R, MPU_UM_RW))
	{
		return MQX_LWEVENT_INVALID;
	}
#endif
	_GET_KERNEL_DATA(kernel_data);

   /* Avoid warning for kernel_data in release targets combinated with lite configuration */
	(void)kernel_data;
	_KLOGE2(KLOG_lwevent_destroy, event_ptr);

#if MQX_COMPONENT_DESTRUCTION

#if MQX_CHECK_ERRORS
	if (kernel_data->IN_ISR)
	{
		_KLOGX2(KLOG_lwevent_destroy, MQX_CANNOT_CALL_FUNCTION_FROM_ISR);
		return (MQX_CANNOT_CALL_FUNCTION_FROM_ISR);
	} /* Endif */
#endif

	_int_disable();
#if MQX_CHECK_VALIDITY
	if (event_ptr->VALID != LWEVENT_VALID)
	{
		_int_enable();
		_KLOGX2(KLOG_lwevent_destroy, MQX_LWEVENT_INVALID);
		return (MQX_LWEVENT_INVALID);
	} /* Endif */
#endif

	/* Effectively stop all access to the event */
	event_ptr->VALID = 0;
	while (_QUEUE_GET_SIZE(&event_ptr->WAITING_TASKS))
	{
		_QUEUE_DEQUEUE(&event_ptr->WAITING_TASKS, td_ptr);
		_BACKUP_POINTER(td_ptr, TD_STRUCT, AUX_QUEUE);
		_TIME_DEQUEUE(td_ptr, kernel_data);
		_TASK_READY(td_ptr, kernel_data);
	} /* Endwhile */

	/* remove event from kernel LWEVENTS queue */
#if MQX_ENABLE_USER_MODE
	if (user)
	{
		_QUEUE_REMOVE(&kernel_data->USR_LWEVENTS, event_ptr);
	}
	else
#endif
	{
		_QUEUE_REMOVE(&kernel_data->LWEVENTS, event_ptr);
	}

	_int_enable();

	/* May need to let higher priority task run */
	_CHECK_RUN_SCHEDULER();
#endif

	_KLOGX2(KLOG_lwevent_destroy, MQX_OK);
	return (MQX_OK);
}
/*! \endcond */

/*!
 * \brief Used by a task to destroy an instance of a lightweight event.
 *
 * To reuse the lightweight event, a task must reinitialize it by calling _lwevent_create()
 *
 * \param[in] event_ptr Pointer to the event to be deinitialized.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was not valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \warning Cannot be called from an ISR.
 *
 * \see _lwevent_create
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_test
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_destroy
(
	LWEVENT_STRUCT_PTR  event_ptr
)
{
#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_destroy(event_ptr);
	}
#endif

	return _lwevent_destroy_internal(event_ptr, FALSE);
}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to destroy an instance of a lightweight event.
 *
 * This function is an equivalent to the _lwevent_destroy() API call but it can be
 * executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer are required to meet the memory
 * protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the event to be deinitialized.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was not valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \warning Cannot be called from an ISR.
 *
 * \see _lwevent_destroy
 * \see _usr_lwevent_create
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_destroy
(
	LWEVENT_STRUCT_PTR  event_ptr
)
{
	MQX_API_CALL_PARAMS params = {(uint32_t)event_ptr, 0, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_DESTROY, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Used by a task to clear the specified event bits in the ligtweight event.
 *
 * \param[in] event_ptr Pointer to the event.
 * \param[in] bit_mask  Bit mask. Each bit represents an event bit to clear.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event is not valid.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_test
 * \see _lwevent_wait_for
 * \see _lwevent_wait_ticks
 * \see _lwevent_wait_until
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_clear
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask
)
{
	_KLOGM(KERNEL_DATA_STRUCT_PTR kernel_data);

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_clear(event_ptr, bit_mask);
	}
#endif

	_KLOGM(_GET_KERNEL_DATA(kernel_data));
	_KLOGE3(KLOG_lwevent_clear, event_ptr, bit_mask);

	_INT_DISABLE();
#if MQX_CHECK_VALIDITY
	if (event_ptr->VALID != LWEVENT_VALID)
	{
		_int_enable();
		_KLOGX2(KLOG_lwevent_clear, MQX_LWEVENT_INVALID);
		return (MQX_LWEVENT_INVALID);
	} /* Endif */
#endif

	event_ptr->VALUE &= ~bit_mask;
	_INT_ENABLE();

	_KLOGX2(KLOG_lwevent_clear, MQX_OK);
	return (MQX_OK);
}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to clear the specified event bits in the ligtweight event.
 *
 * This function is an equivalent to the _lwevent_clear() API call but it can be
 * executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer are required to meet the memory
 * protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the event.
 * \param[in] bit_mask   Bit mask. Each bit represents an event bit to clear.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event is not valid.)
 *
 * \see _lwevent_clear
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_clear
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr,
									(uint32_t)bit_mask, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_CLEAR, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Gets mask of event bits in the lwevent which unblocked the most recent
 *  wait call executed by the current task.
 *
 * If _lwevent_wait_xxx(...) was recently called in a task, the subsequent call of
 * _lwevent_get_signalled returns the mask of bit(s) that unblocked the wait call.
 * User can expect valid data only when the recent _lwevent_wait_xxx(...) operation
 * did not return LWEVENT_WAIT_TIMEOUT or an error value. This is useful primarily
 * for events that are cleared automatically and thus the corresponding value in the
 * LWEVENT_STRUCT was automatically reset and holds new value.
 *
 * \return bit_mask Bit mask of the last task's lwevent_wait_xxx call that
 * unblocked the task.
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_wait_for
 * \see _lwevent_wait_ticks
 * \see _lwevent_wait_until
 */
_mqx_uint _lwevent_get_signalled(void)
{
	KERNEL_DATA_STRUCT_PTR kernel_data;
	_mqx_uint value;

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_get_signalled();
	}
#endif

	_GET_KERNEL_DATA(kernel_data);

	_KLOGE1(KLOG_lwevent_get_signalled);

	value = kernel_data->ACTIVE_PTR->LWEVENT_BITS;

	_KLOGX2(KLOG_lwevent_get_signalled, value);
	return (value);
}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Gets mask of event bits in the lwevent which unblocked the most recent call
 * executed by the current task.
 *
 * This function is an equivalent to the _lwevent_get_signalled() API call but it
 * can be executed from within the User task or other code running in the CPU User
 * mode. Parameters passed to this function by pointer are required to meet the
 * memory protection requirements as described in the parameter list below.
 *
 * \return bit_mask Bit mask of the last task's lwevent_wait_xxx call that
 * unblocked the task.
 *
 * \see _lwevent_get_signalled
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 */
_mqx_uint _usr_lwevent_get_signalled(void)
{
	MQX_API_CALL_PARAMS params = {0, 0, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_GET_SIGNALLED, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Sets autoclearing behavior of event bits in the lightweight event.
 *
 * Used by a task to set functionality of the specified bits in an event to
 * automatic or manual (1 = automatic, 0 = manual).
 *
 * \param[in] event_ptr Pointer to the lightweight event to set bits in.
 * \param[in] auto_mask Mask of events, that is set auto-clear (if corresponding
 * bit of mask is set) or manual-clear (if corresponding bit of mask is clear).
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was invalid.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_clear
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_set_auto_clear
(
	LWEVENT_STRUCT_PTR  event_ptr,

	_mqx_uint           auto_mask
)
{
	_KLOGM(KERNEL_DATA_STRUCT_PTR kernel_data);

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_set_auto_clear(event_ptr, auto_mask);
	}
#endif

	_KLOGM(_GET_KERNEL_DATA(kernel_data));

	_KLOGE3(KLOG_lwevent_set_auto_clear, event_ptr, auto_mask);

	_INT_DISABLE();
#if MQX_CHECK_VALIDITY
	if (event_ptr->VALID != LWEVENT_VALID)
	{
		_int_enable();
		_KLOGX2(KLOG_lwevent_set, MQX_LWEVENT_INVALID);
		return (MQX_LWEVENT_INVALID);
	} /* Endif */
#endif

	event_ptr->AUTO = auto_mask;

	_INT_ENABLE();

	_KLOGX2(KLOG_lwevent_set_auto_clear, MQX_OK);
	return (MQX_OK);

}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Sets autoclearing behavior of event bits in the lightweight event.
 *
 * This function is an equivalent to the _lwevent_set_auto_clear() API call but it
 * can be executed from within the User task or other code running in the CPU User
 * mode. Parameters passed to this function by pointer are required to meet the
 * memory protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the lightweight event to set bits in.
 * \param[in] auto_mask Mask of events, that is set auto-clear (if corresponding
 * bit of mask is set) or manual-clear (if corresponding bit of mask is clear).
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was invalid.)
 *
 * \see _lwevent_set_auto_clear
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_clear
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_set_auto_clear
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           auto_mask
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr,
									(uint32_t)auto_mask, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_SET_AUTO_CLEAR, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Used by a task to set the specified event bits in an event.
 *
 * \param[in] event_ptr Pointer to the lightweight event to set bits in.
 * \param[in] bit_mask  Bit mask. Each bit represents an event bit to be set.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was invalid.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_test
 * \see _lwevent_wait_for
 * \see _lwevent_wait_ticks
 * \see _lwevent_wait_until
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_set
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask
)
{
	KERNEL_DATA_STRUCT_PTR kernel_data;
	QUEUE_ELEMENT_STRUCT_PTR q_ptr;
	QUEUE_ELEMENT_STRUCT_PTR next_q_ptr;
	TD_STRUCT_PTR td_ptr;
	_mqx_uint set_bits;

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_set(event_ptr, bit_mask);
	}
#endif

	_GET_KERNEL_DATA(kernel_data);

	_KLOGE3(KLOG_lwevent_set, event_ptr, bit_mask);

	_INT_DISABLE();
#if MQX_CHECK_VALIDITY
	if (event_ptr->VALID != LWEVENT_VALID)
	{
		_int_enable();
		_KLOGX2(KLOG_lwevent_set, MQX_LWEVENT_INVALID);
		return (MQX_LWEVENT_INVALID);
	} /* Endif */
#endif

	set_bits = event_ptr->VALUE | bit_mask;

	if (_QUEUE_GET_SIZE(&event_ptr->WAITING_TASKS))
	{
		/* Schedule waiting task(s) to run if bits ok */

		q_ptr = event_ptr->WAITING_TASKS.NEXT;
		while (q_ptr != (QUEUE_ELEMENT_STRUCT_PTR) ((void *) &event_ptr->WAITING_TASKS))
		{
			td_ptr = (void *) q_ptr;
			_BACKUP_POINTER(td_ptr, TD_STRUCT, AUX_QUEUE);
			next_q_ptr = q_ptr->NEXT;
			if (((td_ptr->FLAGS & TASK_LWEVENT_ALL_BITS_WANTED) && ((td_ptr->LWEVENT_BITS & set_bits)
							== td_ptr->LWEVENT_BITS)) || ((!(td_ptr->FLAGS & TASK_LWEVENT_ALL_BITS_WANTED))
							&& (td_ptr->LWEVENT_BITS & set_bits)))
			{
				_QUEUE_REMOVE(&event_ptr->WAITING_TASKS, q_ptr);
				_TIME_DEQUEUE(td_ptr, kernel_data);
				td_ptr->INFO = 0;
				_TASK_READY(td_ptr, kernel_data);

				/* store information about which bits caused task to be unblocked */
				td_ptr->LWEVENT_BITS &= set_bits;
				set_bits &= ~(event_ptr->AUTO & td_ptr->LWEVENT_BITS);

			} /* Endif */
			q_ptr = next_q_ptr;
		} /* Endwhile */
	} /* Endif */

	event_ptr->VALUE = set_bits;
	_INT_ENABLE();

	/* May need to let higher priority task run */
	_CHECK_RUN_SCHEDULER();

	_KLOGX2(KLOG_lwevent_set, MQX_OK);
	return (MQX_OK);

}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to set the specified event bits in an event.
 *
 * This function is an equivalent to the _lwevent_set() API call but it can be
 * executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer are required to meet the memory
 * protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the lightweight event to set bits in.
 * \param[in] bit_mask  Bit mask. Each bit represents an event bit to be set.
 *
 * \return MQX_OK
 * \return MQX_LWEVENT_INVALID (Lightweight event was invalid.)
 *
 * \see _lwevent_set
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_set
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr,
									(uint32_t)bit_mask, 0, 0, 0};

	return _mqx_api_call(MQX_API_LWEVENT_SET, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Tests the event component for validity and consistency.
 *
 * \param[out] event_error_ptr Pointer to the lightweight event that has an
 * error if MQX found an error in the lightweight event component (NULL if no error
 * is found).
 * \param[out] td_error_ptr    TD on the lightweight event in error (NULL if no
 * error is found).
 *
 * \return MQX_OK
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 * \return MQX_LWEVENT_INVALID (A lightweight event was invalid.)
 * \return code from _queue_test() (Waiting queue for a lightweight event has an error.)
 *
 * \warning Cannot be called from an ISR.
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 */
_mqx_uint _lwevent_test
(
	void      **event_error_ptr,
	void      **td_error_ptr
)
{
	KERNEL_DATA_STRUCT_PTR  kernel_data;
	LWEVENT_STRUCT_PTR      event_ptr;
	_mqx_uint               result;
	_mqx_uint               queue_size;

	_GET_KERNEL_DATA(kernel_data);

	_KLOGE2(KLOG_lwevent_test, event_error_ptr);

	*td_error_ptr = NULL;
	*event_error_ptr = NULL;

#if MQX_CHECK_ERRORS
	if (kernel_data->IN_ISR)
	{
		_KLOGX2(KLOG_lwevent_test, MQX_CANNOT_CALL_FUNCTION_FROM_ISR);
		return (MQX_CANNOT_CALL_FUNCTION_FROM_ISR);
	}/* Endif */
#endif

	/*
	 * It is not considered an error if the lwevent component has not been
	 * created yet
	 */
	if (kernel_data->LWEVENTS.NEXT == NULL)
	{
		return (MQX_OK);
	} /* Endif */

	result = _queue_test((QUEUE_STRUCT_PTR) &kernel_data->LWEVENTS, event_error_ptr);
	if (result != MQX_OK)
	{
		_KLOGX3(KLOG_lwevent_test, result, *event_error_ptr);
		return (result);
	} /* Endif */

	event_ptr = (LWEVENT_STRUCT_PTR) ((void *) kernel_data->LWEVENTS.NEXT);
	queue_size = _QUEUE_GET_SIZE(&kernel_data->LWEVENTS);
	while (queue_size--)
	{
		if (event_ptr->VALID != LWEVENT_VALID)
		{
			result = MQX_LWEVENT_INVALID;
			break;
		} /* Endif */
		result = _queue_test(&event_ptr->WAITING_TASKS, td_error_ptr);
		if (result != MQX_OK)
		{
			break;
		} /* Endif */
		event_ptr = (LWEVENT_STRUCT_PTR) (void *) event_ptr->LINK.NEXT;
	} /* Endwhile */

	_int_enable();

	if (result != MQX_OK)
	{
		*event_error_ptr = (void *) event_ptr;
	} /* Endif */
	_KLOGX4(KLOG_lwevent_test, result, *event_error_ptr, *td_error_ptr);
	return (result);

}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Checks whether the specified pointer corresponds with a valid lwevent.
 *
 * \param[in] tst_event_ptr Pointer to the lwevent.
 *
 * \return MQX_OK (Valid lwevent.)
 * \return MQX_LWEVENT_INVALID (Specified lwevent is not valid.)
 *
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_usr_check
(
	LWEVENT_STRUCT_PTR  tst_event_ptr
)
{
	KERNEL_DATA_STRUCT_PTR  kernel_data;
	LWEVENT_STRUCT_PTR      event_ptr;
	_mqx_uint               result = MQX_LWEVENT_INVALID;
	_mqx_uint               queue_size;

	_GET_KERNEL_DATA(kernel_data);

	event_ptr = (LWEVENT_STRUCT_PTR)((void *)kernel_data->USR_LWEVENTS.NEXT);
	queue_size = _QUEUE_GET_SIZE(&kernel_data->USR_LWEVENTS);

	while (queue_size--)
	{
		if (event_ptr->VALID != LWEVENT_VALID)
		{
			break;
		}

		if (tst_event_ptr == event_ptr)
		{
			result = MQX_OK;
			break;
		}

		event_ptr = (LWEVENT_STRUCT_PTR)(void *)event_ptr->LINK.NEXT;
	}

	return result;
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Used by a task to wait for the event for the number of ticks (in tick time).
 *
 * \param[in] event_ptr Pointer to the lightweight event.
 * \param[in] bit_mask  Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all       TRUE (wait for all bits in bit_mask to be set), FALSE
 * (wait for any bit in bit_mask to be set).
 * \param[in] tick_ptr  Pointer to the maximum number of ticks to wait for the
 * events to be set. If the value is NULL, then the timeout will be infinite.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_wait_ticks
 * \see _lwevent_wait_until
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 * \see MQX_TICK_STRUCT
 */
_mqx_uint _lwevent_wait_for
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	MQX_TICK_STRUCT_PTR tick_ptr
)
{
	_KLOGM(KERNEL_DATA_STRUCT_PTR kernel_data);
	_mqx_uint result;

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_wait_for(event_ptr, bit_mask, all, tick_ptr);
	}
#endif

	_KLOGM(_GET_KERNEL_DATA(kernel_data));

	_KLOGE5(KLOG_lwevent_wait_for, event_ptr, bit_mask, all, tick_ptr);

	result = _lwevent_wait_internal(event_ptr, bit_mask, all, tick_ptr, FALSE);

	_KLOGX2(KLOG_lwevent_wait_for, result);
	return (result);

}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to wait for the event for the number of ticks (in tick time).
 *
 * This function is an equivalent to the _lwevent_wait_for() API call but it can
 * be executed from within the User task or other code running in the CPU User
 * mode. Parameters passed to this function by pointer are required to meet the
 * memory protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the lightweight event.
 * \param[in] bit_mask  Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all       TRUE (wait for all bits in bit_mask to be set), FALSE
 * (wait for any bit in bit_mask to be set).
 * \param[in] tick_ptr  Pointer to the maximum number of ticks to wait for the
 * events to be set. If the value is NULL, then the timeout will be infinite.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \warning Blocks until the event combination is set or until the timeout expires.
 * Cannot be called from an ISR.
 *
 * \see _lwevent_wait_for
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 * \see MQX_TICK_STRUCT
 */
_mqx_uint _usr_lwevent_wait_for
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	MQX_TICK_STRUCT_PTR tick_ptr
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr, (uint32_t)bit_mask,
									(uint32_t)all,       (uint32_t)tick_ptr, 0};

	return _mqx_api_call(MQX_API_LWEVENT_WAIT_FOR, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \cond DOXYGEN_PRIVATE
 * \private
 *
 * \brief This is internal function used by a task to wait for a specified event.
 *
 * \param[in] event_ptr          Read only. Pointer to the lightweight event.
 * \param[in] bit_mask           Bit mask. Each set bit represents an event bit
 * to wait for.
 * \param[in] all                TRUE (wait for all bits in bit_mask to be set),
 * FALSE (wait for any bit in bit_mask to be set).
 * \param[in] tick_ptr           Pointer to the maximum number of ticks to wait
 * for the events to be set. If the value is NULL, then the timeout will be infinite.
 * \param[in] ticks_are_absolute TRUE (ticks represents absolute time), FALSE
 * (ticks represents relative time).
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_wait_for
 * \see _usr_lwevent_wait_for
 * \see _lwevent_wait_until
 * \see _usr_lwevent_wait_until
 * \see _lwevent_wait_ticks
 * \see _usr_lwevent_wait_ticks
 * \see LWEVENT_STRUCT
 * \see MQX_TICK_STRUCT
 */
_mqx_uint _lwevent_wait_internal
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	MQX_TICK_STRUCT_PTR tick_ptr,
	bool             ticks_are_absolute
)
{
	KERNEL_DATA_STRUCT_PTR  kernel_data;
	TD_STRUCT_PTR           td_ptr;
	_mqx_uint               result;

	_GET_KERNEL_DATA(kernel_data);

#if MQX_CHECK_ERRORS
	if (kernel_data->IN_ISR)
	{
		return (MQX_CANNOT_CALL_FUNCTION_FROM_ISR);
	} /* Endif */
#endif

	result = MQX_OK;
	td_ptr = kernel_data->ACTIVE_PTR;
	_INT_DISABLE();

#if MQX_CHECK_VALIDITY
	if (event_ptr->VALID != LWEVENT_VALID)
	{
		_int_enable();
		return (MQX_LWEVENT_INVALID);
	} /* Endif */
#endif

	if (    (all &&  (event_ptr->VALUE & bit_mask) == bit_mask)
		 || (!all && (event_ptr->VALUE & bit_mask)))
	{
		/* store information about which bits caused task to be unblocked */
		td_ptr->LWEVENT_BITS = event_ptr->VALUE & bit_mask;
		/* clear used automatic events */
		event_ptr->VALUE &= ~(event_ptr->AUTO & bit_mask);

		_INT_ENABLE();
		return (result);
	} /* Endif */

	/* Must wait for a event to become available */

	td_ptr->LWEVENT_BITS = bit_mask;
	if (all)
	{
		td_ptr->FLAGS |= TASK_LWEVENT_ALL_BITS_WANTED;
	}
	else
	{
		td_ptr->FLAGS &= ~TASK_LWEVENT_ALL_BITS_WANTED;
	} /* Endif */

	/* Enqueue at end */
	_QUEUE_ENQUEUE(&event_ptr->WAITING_TASKS, &td_ptr->AUX_QUEUE);

	/* Now put the task to sleep */
	td_ptr->STATE = LWEVENT_BLOCKED;
	td_ptr->INFO = (_mqx_uint) &event_ptr->WAITING_TASKS;
	if (tick_ptr)
	{
		if (ticks_are_absolute)
		{
			_time_delay_until(tick_ptr);
		}
		else
		{
			_time_delay_for(tick_ptr);
		} /* Endif */
		if (td_ptr->INFO)
		{
			/* Must have timed out */
			/*_QUEUE_REMOVE(&event_ptr->WAITING_TASKS, &td_ptr->AUX_QUEUE);*/
			result = LWEVENT_WAIT_TIMEOUT;
		} /* Endif */
	}
	else
	{
		_task_block();
	} /* Endif */

#if MQX_COMPONENT_DESTRUCTION
	if (event_ptr->VALID == 0)
	{ /* We've been deleted */
		result = MQX_LWEVENT_INVALID;
	} /* Endif */
#endif

	_INT_ENABLE();
	return (result);
}
/*! \endcond */

/*!
 * \brief Used by a task to wait until the the event for specified time (in tick time).
 *
 * \param[in] event_ptr Pointer to the lightweight event.
 * \param[in] bit_mask  Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all       TRUE (wait for all bits in bit_mask to be set), FALSE
 * (wait for any bit in bit_mask to be set).
 * \param[in] tick_ptr  Pointer to time until the functions waits.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_wait_for
 * \see _lwevent_wait_ticks
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 * \see MQX_TICK_STRUCT
 */
_mqx_uint _lwevent_wait_until
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	MQX_TICK_STRUCT_PTR tick_ptr
)
{
	_KLOGM(KERNEL_DATA_STRUCT_PTR kernel_data);
	_mqx_uint result;

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_wait_until(event_ptr, bit_mask, all, tick_ptr);
	}
#endif

	_KLOGM(_GET_KERNEL_DATA(kernel_data));

	_KLOGE5(KLOG_lwevent_wait_until, event_ptr, bit_mask, all, tick_ptr);

	result = _lwevent_wait_internal(event_ptr, bit_mask, all, tick_ptr, TRUE);

	_KLOGX2(KLOG_lwevent_wait_until, result);
	return (result);
}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to wait until the specified time (in tick time).
 *
 * This function is an equivalent to the _lwevent_wait_until() API call but it can
 * be executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer are required to meet the memory
 * protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr Read only. Pointer to the lightweight event.
 * \param[in] bit_mask  Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all       TRUE (wait for all bits in bit_mask to be set), FALSE
 * (wait for any bit in bit_mask to be set).
 * \param[in] tick_ptr  Pointer to time until the function waits.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_wait_until
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_ticks
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 * \see MQX_TICK_STRUCT
 */
_mqx_uint _usr_lwevent_wait_until
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	MQX_TICK_STRUCT_PTR tick_ptr
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr, (uint32_t)bit_mask,
									(uint32_t)all,       (uint32_t)tick_ptr, 0};

	return _mqx_api_call(MQX_API_LWEVENT_WAIT_UNTIL, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

/*!
 * \brief Used by a task to wait for the event for the number of ticks.
 *
 * \param[in] event_ptr        Pointer to the lightweight event.
 * \param[in] bit_mask         Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all              TRUE (wait for all bits in bit_mask to be set),
 * FALSE (wait for any bit in bit_mask to be set).
 * \param[in] timeout_in_ticks The maximum number of ticks to wait for the events
 * to be set. If the value is 0, then the timeout will be infinite.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_create
 * \see _lwevent_destroy
 * \see _lwevent_set
 * \see _lwevent_set_auto_clear
 * \see _lwevent_clear
 * \see _lwevent_wait_for
 * \see _lwevent_wait_until
 * \see _lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _lwevent_wait_ticks
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	_mqx_uint           timeout_in_ticks
)
{
	MQX_TICK_STRUCT ticks;
	_KLOGM(KERNEL_DATA_STRUCT_PTR kernel_data);
	_mqx_uint result;

#if MQX_ENABLE_USER_MODE && MQX_ENABLE_USER_STDAPI
	if (MQX_RUN_IN_USER_MODE)
	{
		return _usr_lwevent_wait_ticks(event_ptr, bit_mask, all, timeout_in_ticks);
	}
#endif

	_KLOGM(_GET_KERNEL_DATA(kernel_data));

	_KLOGE5(KLOG_lwevent_wait_ticks, event_ptr, bit_mask, all,
					timeout_in_ticks);

	if (timeout_in_ticks)
	{
		ticks = _mqx_zero_tick_struct;

		PSP_ADD_TICKS_TO_TICK_STRUCT(&ticks, timeout_in_ticks, &ticks);
		result = _lwevent_wait_internal(event_ptr, bit_mask, all, &ticks, FALSE);
	}
	else
	{
		result = _lwevent_wait_internal(event_ptr, bit_mask, all, NULL, FALSE);
	} /* Endif */

	_KLOGX2(KLOG_lwevent_wait_ticks, result);
	return (result);

}

#if MQX_ENABLE_USER_MODE
/*!
 * \brief Used by a task to wait for the event for the number of ticks.
 *
 * This function is an equivalent to the _lwevent_wait_ticks() API call but it can be\
 * executed from within the User task or other code running in the CPU User mode.
 * Parameters passed to this function by pointer are required to meet the memory
 * protection requirements as described in the parameter list below.
 *
 * \param[in] event_ptr        Read only. Pointer to the lightweight event.
 * \param[in] bit_mask         Bit mask. Each set bit represents an event bit to wait for.
 * \param[in] all              TRUE (wait for all bits in bit_mask to be set),
 * FALSE (wait for any bit in bit_mask to be set).
 * \param[in] timeout_in_ticks The maximum number of ticks to wait for the events
 * to be set. If the value is 0, then the timeout will be infinite.
 *
 * \return MQX_OK
 * \return LWEVENT_WAIT_TIMEOUT (The time elapsed before an event signalled.)
 * \return MQX_LWEVENT_INVALID (Lightweight event is no longer valid or was never
 * valid.)
 * \return MQX_CANNOT_CALL_FUNCTION_FROM_ISR (Function cannot be called from an ISR.)
 *
 * \see _lwevent_wait_ticks
 * \see _usr_lwevent_create
 * \see _usr_lwevent_destroy
 * \see _usr_lwevent_set
 * \see _usr_lwevent_set_auto_clear
 * \see _usr_lwevent_clear
 * \see _usr_lwevent_wait_for
 * \see _usr_lwevent_wait_until
 * \see _usr_lwevent_get_signalled
 * \see LWEVENT_STRUCT
 */
_mqx_uint _usr_lwevent_wait_ticks
(
	LWEVENT_STRUCT_PTR  event_ptr,
	_mqx_uint           bit_mask,
	bool             all,
	_mqx_uint           timeout_in_ticks
)
{
	MQX_API_CALL_PARAMS params = {  (uint32_t)event_ptr,
									(uint32_t)bit_mask,
									(uint32_t)all,
									(uint32_t)timeout_in_ticks, 0};

	return _mqx_api_call(MQX_API_LWEVENT_WAIT_FOR_TICKS, &params);
}

#endif /* MQX_ENABLE_USER_MODE */

#endif /* MQX_USE_LWEVENTS */

/* EOF */
