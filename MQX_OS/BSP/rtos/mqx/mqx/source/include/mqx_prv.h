
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
*   This file contains information private to the mqx kernel.
*
*
*END************************************************************************/


#ifndef __mqx_prv_h__
#define __mqx_prv_h__


/*--------------------------------------------------------------------------*/
/*
 *                        KERNEL CONFIGURATION
 *
 * Each mqx kernel configuration item has an associated bit that is written
 * into the configuration field of the kernel data structure at run time,
 * so that the compiled configuration of the kernel is available to software
 * and the debuggers.
 */

#define MQX_CNFG1_INCLUDE_FLOATING_POINT_IO            (0x0001)
#define MQX_CNFG1_USE_INLINE_MACROS                    (0x0002)
#define MQX_CNFG1_KERNEL_LOGGING                       (0x0004)
#define MQX_CNFG1_MONITOR_STACK                        (0x0008)
#define MQX_CNFG1_TASK_CREATION_BLOCKS                 (0x0010)
#define MQX_CNFG1_TASK_DESTRUCTION                     (0x0020)
#define MQX_CNFG1_COMPONENT_DESTRUCTION                (0x0040)
#define MQX_CNFG1_CHECK_ERRORS                         (0x0080)
#define MQX_CNFG1_CHECK_VALIDITY                       (0x0100)
#define MQX_CNFG1_CHECK_MEMORY_ALLOCATION_ERRORS       (0x0200)
#define MQX_CNFG1_USE_32BIT_MESSAGE_QIDS               (0x0400)
#define MQX_CNFG1_MEMORY_FREE_LIST_SORTED              (0x0800)
#define MQX_CNFG1_DEFAULT_TIME_SLICE_IN_TICKS          (0x1000)
#define MQX_CNFG1_LWLOG_TIME_STAMP_IN_TICKS            (0x2000)
#define MQX_CNFG1_PROFILING_ENABLE                     (0x4000)
#define MQX_CNFG1_RUN_TIME_ERR_CHECK_ENABLE            (0x8000)

#define MQX_CNFG2_USE_32BIT_TYPES                      (0x0001)
#define MQX_CNFG2_TIMER_USES_TICKS_ONLY                (0x0002)
#define MQX_CNFG2_EXIT_ENABLED                         (0x0004)
#define MQX_CNFG2_USE_IDLE_TASK                        (0x0008)
#define MQX_CNFG2_IS_MULTI_PROCESSOR                   (0x0010)
#define MQX_CNFG2_HAS_TIME_SLICE                       (0x0020)
#define MQX_CNFG2_MUTEX_HAS_POLLING                    (0x0040)
#define MQX_CNFG2_USE_LWMEM_ALLOCATOR                  (0x0080)
#define MQX_CNFG2_USE_TYPED_MEMORY                     (0x0100)

/* OBSOLETE
 * These bits will be set by dispatch.s
 *#define MQX_CNFG2_PSP_ASM_KERNEL_LOGGING               (0x8000)
 *#define MQX_CNFG2_PSP_ASM_FP_REGISTERS_EXIST           (0x4000)
 *#define MQX_CNFG2_PSP_ASM_MMU_CONTEXT_EXIST            (0x2000)
 *#define MQX_CNFG2_PSP_ASM_PROFILING_ENABLED            (0x1000)
 *#define MQX_CNFG2_PSP_ASM_RUN_TIME_ERR_CHECK_ENABLED   (0x0800)
 *#define MQX_CNFG2_PSP_ASM_DSP_REGISTERS_EXIST          (0x0400)
 */

#ifndef __ASM__

/* Compute value for kernel to write to CONFIG fields */
#define MQX_CNFG1 ( \
   (MQX_INCLUDE_FLOATING_POINT_IO ? MQX_CNFG1_INCLUDE_FLOATING_POINT_IO : 0) | \
   (MQX_USE_INLINE_MACROS ? MQX_CNFG1_USE_INLINE_MACROS : 0) | \
   (MQX_KERNEL_LOGGING ? MQX_CNFG1_KERNEL_LOGGING : 0) | \
   (MQX_MONITOR_STACK ? MQX_CNFG1_MONITOR_STACK : 0) | \
   (MQX_TASK_CREATION_BLOCKS ? MQX_CNFG1_TASK_CREATION_BLOCKS : 0) | \
   (MQX_TASK_DESTRUCTION ? MQX_CNFG1_TASK_DESTRUCTION : 0) | \
   (MQX_COMPONENT_DESTRUCTION ? MQX_CNFG1_COMPONENT_DESTRUCTION : 0) | \
   (MQX_CHECK_ERRORS ? MQX_CNFG1_CHECK_ERRORS : 0) | \
   (MQX_CHECK_VALIDITY ? MQX_CNFG1_CHECK_VALIDITY : 0) | \
   (MQX_CHECK_MEMORY_ALLOCATION_ERRORS ? MQX_CNFG1_CHECK_MEMORY_ALLOCATION_ERRORS : 0) | \
   (MQX_USE_32BIT_MESSAGE_QIDS ? MQX_CNFG1_USE_32BIT_MESSAGE_QIDS : 0) | \
   (MQX_MEMORY_FREE_LIST_SORTED ? MQX_CNFG1_MEMORY_FREE_LIST_SORTED : 0) | \
   (MQX_DEFAULT_TIME_SLICE_IN_TICKS ? MQX_CNFG1_DEFAULT_TIME_SLICE_IN_TICKS : 0) | \
   (MQX_LWLOG_TIME_STAMP_IN_TICKS ? MQX_CNFG1_LWLOG_TIME_STAMP_IN_TICKS : 0) | \
   (MQX_PROFILING_ENABLE ? MQX_CNFG1_PROFILING_ENABLE : 0) | \
   (MQX_RUN_TIME_ERR_CHECK_ENABLE ? MQX_CNFG1_RUN_TIME_ERR_CHECK_ENABLE : 0) \
   )

#define MQX_CNFG2 ( \
   (MQX_USE_32BIT_TYPES ? MQX_CNFG2_USE_32BIT_TYPES : 0) | \
   (MQX_TIMER_USES_TICKS_ONLY ? MQX_CNFG2_TIMER_USES_TICKS_ONLY : 0) | \
   (MQX_EXIT_ENABLED ? MQX_CNFG2_EXIT_ENABLED : 0) | \
   (MQX_USE_IDLE_TASK ? MQX_CNFG2_USE_IDLE_TASK : 0) | \
   (MQX_IS_MULTI_PROCESSOR ? MQX_CNFG2_IS_MULTI_PROCESSOR : 0) | \
   (MQX_HAS_TIME_SLICE ? MQX_CNFG2_HAS_TIME_SLICE : 0) | \
   (MQX_MUTEX_HAS_POLLING ? MQX_CNFG2_MUTEX_HAS_POLLING : 0) | \
   (MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM ? MQX_CNFG2_USE_LWMEM_ALLOCATOR : 0) | \
   (MQX_ALLOW_TYPED_MEMORY ? MQX_CNFG2_USE_TYPED_MEMORY : 0) \
   )

#endif /* __ASM__ */

/*--------------------------------------------------------------------------*/
/*
 *                            INTERNAL TASK FLAGS
 *
 * These bits are set in the task descriptor FLAGS field.
 * Note that the bits from the task template ATTRIBUTES field are also copied
 * to the task descriptor FLAGS field.
 */

/* This bit indicates that the task is running at a boosted priority level */
#define TASK_PRIORITY_BOOSTED             (0x8000)

/* This bit indicates that the task has kernel logging enabled */
#define TASK_LOGGING_ENABLED              (0x4000)

/* This bit indicates that the watchdog is active for this task */
#define TASK_WATCHDOG_STARTED             (0x2000)

/* This bit indicates that preemption has been disabled for this task */
#define TASK_PREEMPTION_DISABLED          (0x1000)

/* This bit indicates that the watchdog is currently running for this task */
#define TASK_WATCHDOG_RUNNING             (0x800)

/* This bit indicates that the task has a MMU context associated with it */
#define TASK_MMU_CONTEXT_EXISTS           (0x0400)

/*
 * This bit indicates that the floating point registers have been saved
 * on the stack of the blocked task
 */
#define TASK_FLOATING_POINT_CONTEXT_SAVED (0x0200)

/*
 * This bit indicates that the DSP registers have been saved
 * on the stack of the blocked task
 */
#define TASK_DSP_CONTEXT_SAVED            (0x0100)

/*
 * This bit indicates that the task is waiting on a LWEvent and wants
 * all bits to be set
 */
#define TASK_LWEVENT_ALL_BITS_WANTED      (0x0080)

/*
 * This bit indicates that the PSP context switch/ISR code has saved the
 * scratch registers on the task's stack
 */

#define PSP_SCRATCH_REGISTERS_SAVED       (0x040)

/*
 * This bit indicates that the tasks TD and stack were provided by the
 * application
 */
#define TASK_STACK_PREALLOCATED           (0x020)

/*
 * This bit indicates that the task runs in restricted USER mode
 */
#define TASK_USER_MODE                    (0x10000)

/*
 * This bit cooperates with the _task_allow_preemption() function
 */
#define SWITCH_REQUEST_WHEN_TASK_PREEMPTION_DISABLED (0x20000)



#ifndef __ASM__

/*--------------------------------------------------------------------------*/
/*
 *                    INTERNAL API NUMBERS FOR USERMODE
 */

typedef enum {
	MQX_API_LWSEM_POLL,
	MQX_API_LWSEM_POST,
	MQX_API_LWSEM_WAIT,
	MQX_API_LWSEM_WAIT_FOR,
	MQX_API_LWSEM_WAIT_TICKS,
	MQX_API_LWSEM_WAIT_UNTIL,
	MQX_API_LWSEM_CREATE,
	MQX_API_LWSEM_DESTROY,

	MQX_API_LWEVENT_CLEAR,
	MQX_API_LWEVENT_SET,
	MQX_API_LWEVENT_SET_AUTO_CLEAR,
	MQX_API_LWEVENT_WAIT_FOR,
	MQX_API_LWEVENT_WAIT_FOR_TICKS,
	MQX_API_LWEVENT_WAIT_UNTIL,
	MQX_API_LWEVENT_GET_SIGNALLED,
	MQX_API_LWEVENT_CREATE,
	MQX_API_LWEVENT_DESTROY,

	MQX_API_LWMSGQ_INIT,
	MQX_API_LWMSGQ_DEINIT,
	MQX_API_LWMSGQ_SEND,
	MQX_API_LWMSGQ_RECEIVE,

	MQX_API_TASK_CREATE,
	MQX_API_TASK_DESTROY,
	MQX_API_TASK_SET_ERROR,
	MQX_API_TASK_GET_TD,
	MQX_API_TASK_ABORT,
	MQX_API_TASK_READY,
	MQX_API_TASK_SET_PRIORITY,

	MQX_API_LWMEM_ALLOC,
	MQX_API_LWMEM_ALLOC_FROM,
	MQX_API_LWMEM_FREE,
	MQX_API_LWMEM_CREATE_POOL,
	MQX_API_LWMEM_REALLOC,

	MQX_API_TIME_DELAY,
	MQX_API_TIME_DELAY_TICKS,
	MQX_API_TIME_GET_ELAPSED_TICKS
} MQX_API_NUMBER_ENUM;

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Data structure for calling API functions.
 */
typedef struct {
	/*! \brief Parameter 0. */
	uint32_t param0;
	/*! \brief Parameter 1. */
	uint32_t param1;
	/*! \brief Parameter 2. */
	uint32_t param2;
	/*! \brief Parameter 3. */
	uint32_t param3;
	/*! \brief Parameter 4. */
	uint32_t param4;
} MQX_API_CALL_PARAMS, * MQX_API_CALL_PARAMS_PTR;
/*! \endcond */


/*--------------------------------------------------------------------------*/
/*
 *                    STRUCTURE OFFSET MACRO
 *
 *  This macro was used to calculate the offset of a structure field from
 *  the start of that structure.
 */
#define FIELD_OFFSET(item,field) (_mqx_uint)&(((item *)0)->field)


/*--------------------------------------------------------------------------*/
/*
 *                       MQX INITIALIZATION DEFINITIONS
 */

/*
 * The maximum number of task templates for a MQX application.
 */
#define MQX_MAXIMUM_NUMBER_OF_TASK_TEMPLATES (0xFFFF)

/*
 * The scaler to use to initialize the default time slice value. It is used
 * in conjunction with MQX's tick rate (ticks per second). The default
 * time slice frequency = ticks per second / MQX_DEFAULT_TIME_SLICE
 */
#define MQX_DEFAULT_TIME_SLICE  ((_mqx_uint)10)

#ifndef MQX_SETJMP
#define MQX_SETJMP(b)    setjmp(b)
#endif
#ifndef MQX_LONGJMP
#define MQX_LONGJMP(b,n) longjmp(b,n)
#endif


/*--------------------------------------------------------------------------*/
/*
 *                        KERNEL LOGGING CONTROL
 *
 * If MQX_KERNEL_LOGGING is configured to non-zero, code will be compiled in
 * that logs kernel activity to the MQX kernel log.
 */
#if MQX_KERNEL_LOGGING
# if MQX_CRIPPLED_EVALUATION
#  define MQX_KLOG_KILL_COUNT    (10000000)             /*C*/
#  include <setjmp.h>                                   /*C*/
   extern jmp_buf _mqx_exit_jump_buffer_internal;       /*C*/
#  define _KLOGM(x) x                                   /*C*/
#  if (PSP_ENDIAN == MQX_BIG_ENDIAN)                    /*C*/
#    define _KLOG(x) \
	  kernel_data->MQX_KLOG_COUNT++;                    /*C*/\
	  if ((kernel_data->TIME.TICKS[1] > 240000) ||      /*C*/\
		 (kernel_data->MQX_KLOG_COUNT > MQX_KLOG_KILL_COUNT)) \
	  {  _INT_DISABLE();                                /*C*/\
		 MQX_LONGJMP(_mqx_exit_jump_buffer_internal,1); /*C*/\
	  } else if (kernel_data->LOG_CONTROL & 1) x        /*C*/
#  else /*PSP is LITTLE ENDIAN */
#    define _KLOG(x)                                    /*C*/\
	  kernel_data->MQX_KLOG_COUNT++;                    /*C*/\
	  if ((kernel_data->TIME.TICKS[0] > 240000) ||      /*C*/\
		 (kernel_data->MQX_KLOG_COUNT > MQX_KLOG_KILL_COUNT)) \
	  {  _INT_DISABLE();                                /*C*/\
		 MQX_LONGJMP(_mqx_exit_jump_buffer_internal,1); /*C*/\
	  } else if (kernel_data->LOG_CONTROL & 1) x        /*C*/
#  endif /*PSP is LITTLE ENDIAN */
# else /* MQX_CRIPPLED_EVALUATION */
#  define _KLOGM(x) x
#  define _KLOG(x) if (kernel_data->LOG_CONTROL & 1) x
# endif /* MQX_CRIPPLED_EVALUATION */
#else /* MQX_KERNEL_LOGGING */
# if MQX_CRIPPLED_EVALUATION
#  define MQX_KLOG_KILL_COUNT    (10000000)             /*C*/
#  include <setjmp.h>                                   /*C*/
   extern jmp_buf _mqx_exit_jump_buffer_internal;       /*C*/
#  define _KLOGM(x) x
#  if (PSP_ENDIAN == MQX_BIG_ENDIAN)                    /*C*/
#    define _KLOG(x) \
	  kernel_data->MQX_KLOG_COUNT++;      /*C*/\
	  if ((kernel_data->TIME.TICKS[1] > 240000) ||      /*C*/\
		 (kernel_data->MQX_KLOG_COUNT > MQX_KLOG_KILL_COUNT)) \
	  {  _INT_DISABLE();                                /*C*/\
		 MQX_LONGJMP(_mqx_exit_jump_buffer_internal,1); /*C*/\
	  }                                                 /*C*/
#  else /*PSP is LITTLE ENDIAN */
#    define _KLOG(x)                                    /*C*/\
	  kernel_data->MQX_KLOG_COUNT++;                    /*C*/\
	  if ((kernel_data->TIME.TICKS[0] > 240000) ||      /*C*/\
		 (kernel_data->MQX_KLOG_COUNT > MQX_KLOG_KILL_COUNT)) \
	  {  _INT_DISABLE();                                /*C*/\
		 MQX_LONGJMP(_mqx_exit_jump_buffer_internal,1); /*C*/\
	  }                                                 /*C*/
#  endif /*PSP is LITTLE ENDIAN */
# else /* MQX_CRIPPLED_EVALUATION */
#define _KLOGM(x)
#define _KLOG(x)
# endif /* MQX_CRIPPLED_EVALUATION */
#endif /* MQX_KERNEL_LOGGING */

/* NOTE: _klog_log now only accepts 6 parameters */
/* Function entry logging macros, */
#define _KLOGE1(fn) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGE2(fn,p1) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGE3(fn,p1,p2) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGE4(fn,p1,p2,p3) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)(p3), (_mqx_max_type)0);)
#define _KLOGE5(fn,p1,p2,p3,p4) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)(p3), (_mqx_max_type)(p4));)
#define _KLOGE6(fn,p1,p2,p3,p4,p5) _KLOG(_klog_log(KLOG_FUNCTION_ENTRY, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)(p3), (_mqx_max_type)(p4));)

/* Function exit logging macros */
#define _KLOGX1(fn) _KLOG(_klog_log(KLOG_FUNCTION_EXIT, \
   (_mqx_max_type)(fn), (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGX2(fn,p1) _KLOG(_klog_log(KLOG_FUNCTION_EXIT, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)0, (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGX3(fn,p1,p2) _KLOG(_klog_log(KLOG_FUNCTION_EXIT, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)0, (_mqx_max_type)0);)
#define _KLOGX4(fn,p1,p2,p3) _KLOG(_klog_log(KLOG_FUNCTION_EXIT, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)(p3), (_mqx_max_type)0);)
#define _KLOGX5(fn,p1,p2,p3,p4) _KLOG(_klog_log(KLOG_FUNCTION_EXIT, \
   (_mqx_max_type)(fn), (_mqx_max_type)(p1), (_mqx_max_type)(p2), (_mqx_max_type)(p3), (_mqx_max_type)(p4));)

/*--------------------------------------------------------------------------*/
/*
 *                             MEMORY ALIGNMENTS
 *
 * It is important to maintain proper memory alignment of the kernel
 * data structures in order to avoid the penalty of mis-aligned memory
 * accesses.
 */

#if PSP_MEMORY_ALIGNMENT   /* We must align memory */

/* Make sure that the address is a multiple of the memory alignment */
#define _MEMORY_ALIGN_VAL_LARGER(val) \
   val = (((_mem_size)(val) + PSP_MEMORY_ALIGNMENT) & PSP_MEMORY_ALIGNMENT_MASK)
#define _MEMORY_ALIGN_VAL_SMALLER(val) \
   val = ((_mem_size)(val) & PSP_MEMORY_ALIGNMENT_MASK)

#define _ALIGN_ADDR_TO_HIGHER_MEM(mem_ptr) \
   (((_mem_size)(mem_ptr) + PSP_MEMORY_ALIGNMENT) & PSP_MEMORY_ALIGNMENT_MASK)
#define _ALIGN_ADDR_TO_LOWER_MEM(mem_ptr) \
   ((_mem_size)(mem_ptr) & PSP_MEMORY_ALIGNMENT_MASK)

/* Make sure that the stack address is a multiple of the stack alignment */
#define _STACK_ALIGN_VAL_LARGER(val) \
   val = (((_mem_size)(val) + PSP_STACK_ALIGNMENT) & PSP_STACK_ALIGNMENT_MASK)
#define _STACK_ALIGN_VAL_SMALLER(val) \
   val = ((_mem_size)(val) & PSP_STACK_ALIGNMENT_MASK)

#define _ALIGN_STACK_TO_HIGHER_MEM(mem_ptr) \
   (((_mem_size)(mem_ptr) + PSP_STACK_ALIGNMENT) & PSP_STACK_ALIGNMENT_MASK)
#define _ALIGN_STACK_TO_LOWER_MEM(mem_ptr) \
   ((_mem_size)(mem_ptr) & PSP_STACK_ALIGNMENT_MASK)


/* Is the memory address a multiple of the memory alignment */
#define _MEMORY_ALIGNED(mem_ptr) \
   (((_mem_size)(mem_ptr) & PSP_MEMORY_ALIGNMENT) ? FALSE : TRUE)

#define _STACK_ALIGNED(mem_ptr) \
   (((_mem_size)(mem_ptr) & PSP_STACK_ALIGNMENT) ? FALSE : TRUE)

#else
#define _ALIGN_ADDR_TO_HIGHER_MEM(val) val
#define _ALIGN_ADDR_TO_LOWER_MEM(val) val
#define _MEMORY_ALIGNED(mem_ptr) TRUE
#define _ALIGN_STACK_TO_HIGHER_MEM(val) val
#define _ALIGN_STACK_TO_LOWER_MEM(val) val
#define _STACK_ALIGNED(mem_ptr) TRUE
#endif

/*--------------------------------------------------------------------------*/
/*
 *                        KERNEL STACK MONITORING
 *
 * If MQX_MONITOR_STACK is defined as non-zero, then all stacks
 * are filled with the following value at stack initialization time.
 */
#define MQX_STACK_MONITOR_VALUE          (_mqx_uint)(0x7374616B)  /* "stak" */

/*--------------------------------------------------------------------------*/
/*
 *                          STACK MACROS
 *
 * These macros determine the BASE and LIMIT of the stack, depending upon
 * which direction the stack grow
 */
#if PSP_STACK_GROWS_TO_LOWER_MEM

#define _GET_STACK_BASE(mem_ptr, size)              ((void *)_ALIGN_STACK_TO_LOWER_MEM(((unsigned char *)_PSP_NORMALIZE_MEMORY(mem_ptr) + size - 1)))
#define _GET_STACK_LIMIT(mem_ptr, size)             ((void *)_ALIGN_STACK_TO_HIGHER_MEM(_PSP_NORMALIZE_MEMORY(mem_ptr)))

#else

#define _GET_STACK_BASE(mem_ptr, size)              ((void *)_ALIGN_STACK_TO_HIGHER_MEM(_PSP_NORMALIZE_MEMORY(mem_ptr)))
#define _GET_STACK_LIMIT(mem_ptr, size)             ((void *)_ALIGN_STACK_TO_LOWER_MEM(((unsigned char *)_PSP_NORMALIZE_MEMORY(mem_ptr) + size - 1)))

#endif /* PSP_STACK_GROWS_TO_LOWER_MEM */

 /*--------------------------------------------------------------------------*/
/*
 *                             TASK STATES
 *
 * The following define all of the states that a task have.
 */

/* All blocked states have this bit set */
#define IS_BLOCKED                      (0x001)

/* Some states may also have the task on a timeout queue when blocked */
#define IS_ON_TIMEOUT_Q                 (0x100)

/* Some states may also have their task descriptors on a queue when blocked */
#define TD_IS_ON_QUEUE                  (0x200)

/*
 * Some states may also have their task descriptors on a queue using their
 * AUX pointer fields in the TD
 */
#define TD_IS_ON_AUX_QUEUE              (0x400)

/* Mask to remove queueing and timeout bits */
#define STATE_MASK                      (0x0ff)

#define READY                           (0x02)
#define BLOCKED                         (0x02 | IS_BLOCKED)
#define RCV_SPECIFIC_BLOCKED            (0x04 | IS_BLOCKED)
#define RCV_ANY_BLOCKED                 (0x06 | IS_BLOCKED)
#define DYING                           (0x08 | IS_BLOCKED)
#define UNHANDLED_INT_BLOCKED           (0x0A | IS_BLOCKED)
#define SEND_BLOCKED                    (0x0C | IS_BLOCKED)
#define BREAKPOINT_BLOCKED              (0x0E | IS_BLOCKED)
#define IO_BLOCKED                      (0x10 | IS_BLOCKED | TD_IS_ON_QUEUE)

#define SEM_BLOCKED                     (0x20 | IS_BLOCKED)
#define MUTEX_BLOCKED                   (0x22 | IS_BLOCKED | TD_IS_ON_QUEUE)
#define EVENT_BLOCKED                   (0x24 | IS_BLOCKED)
#define TASK_QUEUE_BLOCKED              (0x28 | IS_BLOCKED | TD_IS_ON_QUEUE)
#define BLOCKED_ON_AUX_QUEUE            (TD_IS_ON_AUX_QUEUE|IS_BLOCKED)
#define LWSEM_BLOCKED                   (0x2A | BLOCKED_ON_AUX_QUEUE)
#define LWEVENT_BLOCKED                 (0x2C | BLOCKED_ON_AUX_QUEUE)

/*
 * The task ID of a task that is in the DYING state
 */
#define TASK_ID_DYING                   (0x80000000)


/*--------------------------------------------------------------------------*/
/*
 *                             TASK ID MACROS
 *
 * These macros manipulate the internals of a TASKID
 */

/* Create a new task id from processor number and task number */
#define BUILD_TASKID(procnum, tasknum) (_task_id)\
   (((uint32_t)(procnum) << 16) | ((uint32_t)(tasknum) & 0xFFFF))

/* Obtain the processor number from the task ID */
#define PROC_NUMBER_FROM_TASKID(taskid) ((uint16_t)((taskid) >> 16))

/* Obtain the task number from the task ID */
#define TASK_NUMBER_FROM_TASKID(taskid) ((uint16_t)((taskid) & 0xFFFF))

/* Increment task number being careful not to wrap to 0 */
#if (PSP_MEMORY_ADDRESSING_CAPABILITY > 16)
#define INC_TASK_NUMBER(n)                      \
   n++;                                         \
   n &= 0xFFFF;                                 \
   if (n == 0) {                                \
	  n = 1;                                    \
   }
#else
#define INC_TASK_NUMBER(n)                      \
   n++;                                         \
   if (n == 0) {                                \
	  n = 1;                                    \
   }
#endif


/*--------------------------------------------------------------------------*/
/*
 *                            MQX INTERNAL TASK FLAGS
 *
 * These bits are set in the kernel data FLAGS field.
 */


/*
 * This flag indicates to _int_kernel_isr that the _int_exception_isr has
 * been installed.  Thus _int_kernel_isr will save and restore all
 * registers upon entry/exit.
 */
#define MQX_FLAGS_EXCEPTION_HANDLER_INSTALLED   (0x0001)

/*--------------------------------------------------------------------------*/
/*
 *                            TIME CONSTANTS
 */
#define LEAP_YEAR                (1)
#define NON_LEAP_YEAR            (0)

/*
 * Time normalization macro.
 * Ensures that output milliseconds are in the range of 0-999,
 * and that out_secs are adjusted if required
 */
#define MQX_TIME_NORMALIZE(in_secs, in_msecs, out_secs, out_msecs) \
   (out_secs)  = 0;                                                \
   (out_msecs) = (in_msecs);                                       \
   if ((out_msecs) >= MILLISECS_IN_SECOND) {                       \
	  (out_secs)   = (out_msecs) / MILLISECS_IN_SECOND;            \
	  (out_msecs) -= ((out_secs) * MILLISECS_IN_SECOND);           \
   } /* Endif */                                                   \
   (out_secs) += (in_secs)

/*
 * This macro performs the same function as above but uses a single
 * TIME_STRUCT
 */
#define MQX_NORMALIZE_TIME_STRUCT(t_ptr)                     \
   if ( ((TIME_STRUCT_PTR)(t_ptr))->MILLISECONDS >= 1000 ) { \
	  ((TIME_STRUCT_PTR)(t_ptr))->SECONDS      +=            \
		 ((TIME_STRUCT_PTR)(t_ptr))->MILLISECONDS / 1000;    \
	  ((TIME_STRUCT_PTR)(t_ptr))->MILLISECONDS  =            \
		 ((TIME_STRUCT_PTR)(t_ptr))->MILLISECONDS % 1000;    \
   }

/* This macro initializes a TICK structure to zero */
#if MQX_NUM_TICK_FIELDS == 1
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 2
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 3
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 4
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->TICKS[3] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 5
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->TICKS[3] = 0; \
   (tick_ptr)->TICKS[4] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 6
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->TICKS[3] = 0; \
   (tick_ptr)->TICKS[4] = 0; \
   (tick_ptr)->TICKS[5] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 7
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->TICKS[3] = 0; \
   (tick_ptr)->TICKS[4] = 0; \
   (tick_ptr)->TICKS[5] = 0; \
   (tick_ptr)->TICKS[6] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#elif MQX_NUM_TICK_FIELDS == 8
#define MQX_ZERO_TICK_STRUCT(tick_ptr) \
   (tick_ptr)->TICKS[0] = 0; \
   (tick_ptr)->TICKS[1] = 0; \
   (tick_ptr)->TICKS[2] = 0; \
   (tick_ptr)->TICKS[3] = 0; \
   (tick_ptr)->TICKS[4] = 0; \
   (tick_ptr)->TICKS[5] = 0; \
   (tick_ptr)->TICKS[6] = 0; \
   (tick_ptr)->TICKS[7] = 0; \
   (tick_ptr)->HW_TICKS = 0;
#else
#error missing option for zeroing mqx tick structure
#endif



/*--------------------------------------------------------------------------*/
/*
 *                      QUEUE MANIPULATION MACROS
 */

/*
 *  CR171 & CR172
 *
 *  For a time delay queues we insert the task into the queue after
 *  all tasks of the same delay period.  We do not need to do this, we
 *  can just insert the task into the queue before the task waiting
 *  for the same time, as all tasks will be activated anyways.
 *  Running the queues takes a long time, and is done with interrupts
 *  disabled.  There will be a slight behavioural difference in that
 *  two tasks that delay for the same time at the same priority level,
 *  will not run in the order that they called delay.
 *
 *  Set the following define to 1 if you want the original FIFO
 *  behavior where tasks are activated from the time delay queue into
 *  the ready queue in FIFO order (longer enqueue search with
 *  interrupts disabled).
 */
#ifndef MQX_DELAY_ENQUEUE_FIFO_ORDER
#define MQX_DELAY_ENQUEUE_FIFO_ORDER    0  /* 0=LIFO, 1=FIFO */
#endif
#if MQX_DELAY_ENQUEUE_FIFO_ORDER
#define MQX_DELAY_ENQUEUE_POLICY(result)  (result > 0)  /* FIFO original */
#else
#define MQX_DELAY_ENQUEUE_POLICY(result)  (result >= 0) /* LIFO faster */
#endif

/*--------------------------------------------------------------------------*/
/*
 *                      INLINE MACROS FOR SPEED
 */

/*
 * This macro checks if the current task should invoke the scheduler
 * to allow a higher priority task to run (it may just have been added).
 */
#define _CHECK_RUN_SCHEDULER()  _sched_check_scheduler_internal()

#if MQX_ENABLE_USER_MODE
#define MQX_RUN_IN_USER_MODE    ((__get_CONTROL() & 1) & !(__get_xPSR() & 0x1f))
#endif

/*
 * This macro simulates the _task_ready function call. It is in the form of a
 * macro to optimize for speed.
 * NOTE: it MUST BE USED DISABLED
 */
#if MQX_HAS_TIME_SLICE
#define ZERO_TICK_STRUCT_INTERNAL(x) MQX_ZERO_TICK_STRUCT(x)
#else
#define ZERO_TICK_STRUCT_INTERNAL(x)
#endif

#if MQX_USE_INLINE_MACROS
#define _TASK_READY(the_td,the_kernel_data)                \
 _KLOGE2(KLOG_task_ready, the_td);                         \
 if ( ((unsigned char *) (the_td)->MY_QUEUE) >                 \
 (unsigned char *) (the_kernel_data->CURRENT_READY_Q) ) {      \
	the_kernel_data->CURRENT_READY_Q = (the_td)->MY_QUEUE; \
 } /* Endif */                                             \
 (the_td)->STATE = READY;                                  \
 (the_td)->TD_PREV = ((the_td)->MY_QUEUE)->TAIL_READY_Q;   \
 (the_td)->TD_NEXT = ((the_td)->TD_PREV)->TD_NEXT;         \
 ((the_td)->TD_PREV)->TD_NEXT = (the_td);                  \
 ((the_td)->MY_QUEUE)->TAIL_READY_Q = (the_td);            \
 /*(the_td)->CURRENT_TIME_SLICE = _mqx_zero_tick_struct;*/ \
  ZERO_TICK_STRUCT_INTERNAL(&(the_td)->CURRENT_TIME_SLICE) \
 _KLOGX1(KLOG_task_ready);
#else
#define _TASK_READY(the_td,the_kernel_data) _task_ready_internal(the_td)
#endif

#if MQX_USE_INTERRUPTS

#define _INT_DISABLE_CODE()                             \
   if (kernel_data->ACTIVE_PTR->DISABLED_LEVEL == 0)  { \
	  _PSP_SET_DISABLE_SR(kernel_data->DISABLE_SR);     \
   } /* Endif */                                        \
   ++kernel_data->ACTIVE_PTR->DISABLED_LEVEL;

#define _INT_ENABLE_CODE()                                                     \
   if (kernel_data->ACTIVE_PTR->DISABLED_LEVEL) {                              \
	  if (--kernel_data->ACTIVE_PTR->DISABLED_LEVEL == 0) {                    \
		 if (kernel_data->IN_ISR) {                                            \
			_PSP_SET_ENABLE_SR(kernel_data->INTERRUPT_CONTEXT_PTR->ENABLE_SR); \
		 } else {                                                              \
			_PSP_SET_ENABLE_SR(kernel_data->ACTIVE_SR);                        \
		 } /* Endif */                                                         \
	  } /* Endif */                                                            \
   } /* Endif */
#else
	#define _INT_DISABLE_CODE()
	#define _INT_ENABLE_CODE()
#endif

#if MQX_USE_INLINE_MACROS
#define _INT_DISABLE() _INT_DISABLE_CODE()
#define _INT_ENABLE()  _INT_ENABLE_CODE()
#else
#define _INT_DISABLE() _int_disable()
#define _INT_ENABLE()  _int_enable()
#endif

/* This macro dequeues a td from the timeout queue */
#if MQX_HAS_TICK
#define _TIME_DEQUEUE(td,kd)                             \
   if ( ((TD_STRUCT_PTR)td)->STATE & IS_ON_TIMEOUT_Q ) { \
	  _QUEUE_REMOVE(&kd->TIMEOUT_QUEUE,td);              \
	  ((TD_STRUCT_PTR)td)->STATE &= ~IS_ON_TIMEOUT_Q;    \
	  ((TD_STRUCT_PTR)td)->STATE |= BLOCKED;             \
   } /* Endif */
#else
#define _TIME_DEQUEUE(td,kd)
#endif

/*
 * Macro to help with de-queueing, it subtracts the offset of
 * field f of type t from pointer p, so that p points to the
 * start of the structure
 */
#define _BACKUP_POINTER(p,t,f) \
   p = (t *)((unsigned char *)p - FIELD_OFFSET(t, f))

/*--------------------------------------------------------------------------*/
/*
 *                      KERNEL TASKING CONSTANTS
 */


#define SYSTEM_TASK_NUMBER    0xffff

/* This bit signifies a special task template index. */
#define SYSTEM_TASK_FLAG       ((_mqx_uint)(((_mqx_uint)0x1) << (MQX_INT_SIZE_IN_BITS - 1)))

/* The task descriptor for the system task */
#define SYSTEM_TD_PTR(kd)      ((TD_STRUCT_PTR)(&(kd)->SYSTEM_TD))

/* The task id of the system task */
#define SYSTEM_TASK_ID(kd)     (SYSTEM_TD_PTR(kd)->TASK_ID)

/*
 * The following are task indexes for the system tasks: idle task
 * and the ipc task.
 */
#define IPC_TASK               (SYSTEM_TASK_FLAG | 0x2)
#define IDLE_TASK              (SYSTEM_TASK_FLAG | 0x3)
#define INIT_TASK              (SYSTEM_TASK_FLAG | 0x4)

/*--------------------------------------------------------------------------*/
/*
 *                          IPC CONSTANTS
 *
 * Inter-Processor Communication requests are made by calling the IPC
 * function. This function is installed into the kernel data 'IPC' field
 * by the IPC initialization code.
 * An IPC request is made by calling this function.
 * The first parameter indicates which processor this request is for.
 * The second parameter indicates which component is sending the request.
 * The third parameter indicates which function of the component is requesting
 * service.
 * The fourth parameter is the number of further parameters for the function call.
 * And following this parameter are the parameters required for the remote
 * function call.
 *
 */

/*
 * The following definitions apply to the inter-processor communications.
 * These constants are used as the third parameter of the IPC_FUNCTION
 * (installed in the kernel data structure).  This parameter indicates what
 * kernel operation is required on a remote processor.
 */
#define IPC_ACTIVATE                  (0x1000)
#define IPC_TASK_CREATE               (0x1001)
#define IPC_TASK_CREATE_WITH_TEMPLATE (0x1002)
#define IPC_TASK_DESTROY              (0x1003)
#define IPC_TASK_RESTART              (0x1004)
#define IPC_TASK_ABORT                (0x1005)

/*--------------------------------------------------------------------------*/
/*
 *                    KERNEL COMPONENT CONSTANTS
 */

/*
 * The MQX component indexes, used to index into the component
 * arrays to access component specific data,
 * task destruction handlers and IPC handlers
 */
#define KERNEL_NAME_MANAGEMENT        (0)
#define KERNEL_SEMAPHORES             (1)
#define KERNEL_EVENTS                 (2)
#define KERNEL_MUTEXES                (3)
#define KERNEL_TIMER                  (4)
#define KERNEL_LOG                    (5)
#define KERNEL_WATCHDOG               (6)
#define KERNEL_MESSAGES               (7)
#define KERNEL_PARTITIONS             (8)
#define KERNEL_IPC                    (9)
#define KERNEL_IPC_MSG_ROUTING        (10)
#define KERNEL_LWLOG                  (11)

/* The maximum number of components */
#define MAX_KERNEL_COMPONENTS         (12)


/*--------------------------------------------------------------------------*/
/*
 *                       KERNEL SCHEDULER CONSTANTS
 */

/* Scheduling policy constants */
#define SCHED_MAX_POLICY  MQX_SCHED_RR

/*--------------------------------------------------------------------------*/
/*
 *                      QUEUE MANIPULATION MACROS
 */

/*
 * These macros assume a data structure that looks like
 * a double linked list.
 *     element.NEXT is first field and points to next item in q
 *     element.PREV is second field and points to prev item in q
 * The queue head actually looks like a queue element!, but is initialized
 * with values such that a queue with 1 element on it can have that element
 * removed, and a queue with 0 elements on it can have an element added to
 * the queue without any special checking.
 */

/* Initialize a queue of doubly linked elements */
#if MQX_USE_INLINE_MACROS
#define _QUEUE_INIT(queue,max)                                      \
   ((QUEUE_STRUCT_PTR)(queue))->NEXT =                              \
	  (QUEUE_ELEMENT_STRUCT_PTR)((void *)&((QUEUE_STRUCT_PTR)(queue))->NEXT); \
   ((QUEUE_STRUCT_PTR)(queue))->PREV =                              \
	  (QUEUE_ELEMENT_STRUCT_PTR)((void *)&((QUEUE_STRUCT_PTR)(queue))->NEXT); \
   ((QUEUE_STRUCT_PTR)(queue))->SIZE = (uint16_t)0;                  \
   ((QUEUE_STRUCT_PTR)(queue))->MAX  = (uint16_t)max; \
   ((QUEUE_STRUCT_PTR)(queue))->ATOMIC = 0;
#else
#define _QUEUE_INIT(queue, max) _queue_init((QUEUE_STRUCT_PTR)(queue), max)
#endif

/* How big is the queue */
#define _QUEUE_GET_SIZE(queue)  ((QUEUE_STRUCT_PTR)(queue))->SIZE


/* Is the queue empty */
#define _QUEUE_IS_EMPTY(queue) (_QUEUE_GET_SIZE(queue) == 0)

/* Is the queue full */
#define _QUEUE_IS_FULL(queue)  \
   (_QUEUE_GET_SIZE(queue) == (((QUEUE_STRUCT_PTR)(queue))->MAX))

/* links a queue element into a doubly linked list, after queue_member */
#define _QUEUE_LINK(queue_member,element)               \
{                                                       \
   QUEUE_ELEMENT_STRUCT_PTR nxt = (queue_member)->NEXT; \
	  (element)->NEXT = nxt;                            \
	  (queue_member)->NEXT = element;                   \
	  (element)->PREV = queue_member;                   \
	  nxt->PREV = element;                              \
}

/*
 * Inserts a queue element, after queue_member
 * NOTE: this does not _ENQUEUE at the end of the queue.
 */
#if MQX_USE_INLINE_MACROS
#define _QUEUE_INSERT(queue,queue_member,element)           \
	  _QUEUE_LINK((QUEUE_ELEMENT_STRUCT_PTR)((void *)(queue_member)), \
		 (QUEUE_ELEMENT_STRUCT_PTR)((void *)(element)));              \
	  ++((QUEUE_STRUCT_PTR)(queue))->SIZE;
#else
#define _QUEUE_INSERT(queue,queue_member,element) \
   _queue_insert((QUEUE_STRUCT_PTR)(queue), \
	  (QUEUE_ELEMENT_STRUCT_PTR)((void *)(queue_member)), \
	  (QUEUE_ELEMENT_STRUCT_PTR)((void *)(element)))
#endif

/* Enqueue an element at the end of the queue */
#if MQX_USE_INLINE_MACROS
#define _QUEUE_ENQUEUE(queue,element) \
   _QUEUE_INSERT((queue),((QUEUE_STRUCT_PTR)((void *)(queue)))->PREV, \
   (element))
#else
#define _QUEUE_ENQUEUE(queue,element) \
   _queue_enqueue((QUEUE_STRUCT_PTR)(queue), \
	  (QUEUE_ELEMENT_STRUCT_PTR)((void *)(element)))
#endif


/* Unlink a queue member from any doubly linked queue */
#define _QUEUE_UNLINK(queue_member) \
{ \
   QUEUE_ELEMENT_STRUCT_PTR prev_ptr = \
	  ((QUEUE_ELEMENT_STRUCT_PTR)((void *)(queue_member)))->PREV; \
   QUEUE_ELEMENT_STRUCT_PTR next_ptr = \
	  ((QUEUE_ELEMENT_STRUCT_PTR)((void *)(queue_member)))->NEXT; \
   prev_ptr->NEXT = next_ptr; \
   next_ptr->PREV = prev_ptr; \
}


/*
 * Remove an element from the queue
 * NOTE: this does not dequeue an element from the front of the list
 * the element must be known
 */
#define _QUEUE_REMOVE(queue,element)        \
	  _QUEUE_UNLINK(element);               \
	  --((QUEUE_STRUCT_PTR)(queue))->SIZE;

/* Dequeue an element from the front of the queue, the element is not known */
#if MQX_USE_INLINE_MACROS
#define _QUEUE_DEQUEUE(queue,element)                    \
   element = (void *)((QUEUE_STRUCT_PTR)(queue))->NEXT; \
   _QUEUE_REMOVE((queue),(element))
#else
#define _QUEUE_DEQUEUE(queue,element) element = (void *)_queue_dequeue(queue)
#endif


/*--------------------------------------------------------------------------*/
/*                     TASK QUEUE CONSTANTS                                 */

/* The correct value for the task queue VALID field */
#define TASK_QUEUE_VALID           (_mqx_uint)(0x74736b71)  /* "tskq" */


/*--------------------------------------------------------------------------*/
/*                LIGHT WEIGHT SEMAPHORE CONSTANTS                          */

/* The correct value for the lwsem VALID field */
#define LWSEM_VALID                (_mqx_uint)(0x6C77736D)    /* "lwsm" */


/*--------------------------------------------------------------------------*/
/*                     MISC. CONSTANTS                                      */

/* A macros to convert a pre-processor constant to a string */

#define NUM_TO_STR(x) #x
#define REAL_NUM_TO_STR(x) NUM_TO_STR(x)

/*--------------------------------------------------------------------------*/
/*                SPECIAL MEMORY ALLOCATOR OPTION                           */

#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM

#ifndef __MEMORY_MANAGER_COMPILE__
#define _mem_get_next_block_internal _lwmem_get_next_block_internal
#define _mem_init_internal _lwmem_init_internal
#define _mem_transfer_td_internal _lwmem_transfer_td_internal
#define _mem_transfer_internal _lwmem_transfer_internal

#define GET_MEMBLOCK_TYPE(ptr)  _GET_LWMEMBLOCK_TYPE(ptr)
#endif /* __MEMORY_MANAGER_COMPILE__ */

#elif MQXCFG_ALLOCATOR == MQX_ALLOCATOR_TLSF

#ifndef __MEMORY_MANAGER_COMPILE__
#define _mem_get_next_block_internal _tlsf_get_next_block_internal
#define _mem_init_internal _tlsf_init_internal
#define _mem_transfer_td_internal _tlsf_transfer_td_internal
#define _mem_transfer_internal _tlsf_transfer_internal

#define GET_MEMBLOCK_TYPE(ptr)  _GET_TLSFBLOCK_TYPE(ptr)
#endif /* __MEMORY_MANAGER_COMPILE__ */

#else /* MQXCFG_ALLOCATOR */

#define GET_MEMBLOCK_TYPE(ptr)  _GET_MEMBLOCK_TYPE(ptr)

#endif /* MQXCFG_ALLOCATOR */

/*==========================================================================*/
/*                        DATA STRUCTURES                                   */

#if MQXCFG_SPARSE_ISR_TABLE
/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Interrupt sparse record structure.
 */
typedef struct interrupt_sparse_rec_struct {
	/*! \brief Vector number. */
	int32_t VEC_NUM;

	/*! \bief The application ISR to call for this interrupt. */
	INT_ISR_FPTR APP_ISR;

#if MQXCFG_EXCEPTION_HANDLING
	/*!
	 * \brief The exception handler for this ISR.
	 *
	 * If the exception handling has been installed as the default ISR, then when
	 * an exception occurs during an ISR, the ISR (and the exception) are aborted,
	 * and this function is called. This function is passed the ISR vector number,
	 * the exception vector number and the parameter for the application ISR, and
	 * the an exception frame pointer.
	 */
	INT_EXCEPTION_FPTR APP_ISR_EXCEPTION_HANDLER;
#endif /* MQXCFG_EXCEPTION_HANDLING */

	/*! \br1ief The parameter to pass to this ISR. */
	void   *APP_ISR_DATA;

	/*! \brief Pointer to the next structure. */
	struct interrupt_sparse_rec_struct *NEXT;
} INTERRUPT_SPARSE_REC_STRUCT, * INTERRUPT_SPARSE_REC_STRUCT_PTR;
/*! \endcond */

typedef void *INTERRUPT_TABLE_STRUCT, ** INTERRUPT_TABLE_STRUCT_PTR;

#else
/*--------------------------------------------------------------------------*/
/*                      INTERRUPT TABLE STRUCTURE */

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Interrupt table structure.
 *
 * An array of these context structures is created at initialization time.
 * \n The array is bounded by the FIRST_USER_ISR_VECTOR and LAST_USER_ISR_VECTOR
 * fields in the kernel data structure.
 * \n When an interrupt occurs, the interrupt number is checked to be within
 * these stated bounds. If it is, the interrupt table structure is indexed with
 * the interrupt number to obtain the 'C' function to call.
 * \n The interrupt table also contains a parameter to pass to the users ISR, and
 * an exception handler for the users ISR.
 */
typedef struct interrupt_table_struct
{
  /*! \brief The application ISR to call for this interrupt. */
   INT_ISR_FPTR        APP_ISR;

#if MQXCFG_EXCEPTION_HANDLING
   /*!
	* \brief The exception handler for this ISR.
	*
	* If the exception handling has been installed as the default ISR, then when
	* an exception occurs during an ISR, the ISR (and the exception) are aborted,
	* and this function is called. This function is passed the ISR vector number,
	* the exception vector number and the parameter for the application ISR, and
	* the an exception frame pointer.
	*/
   INT_EXCEPTION_FPTR  APP_ISR_EXCEPTION_HANDLER;
#endif /* MQXCFG_EXCEPTION_HANDLING */

   /*! \brief The parameter to pass to this ISR. */
   void               *APP_ISR_DATA;

} INTERRUPT_TABLE_STRUCT, * INTERRUPT_TABLE_STRUCT_PTR;
/*! \endcond */
#endif

/*--------------------------------------------------------------------------*/
/*                         TASK QUEUE STRUCTURE */
/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This structure defines a task queue.
 *
 * These task queues are elements on the KERNEL_TASK_QUEUE queue, a field of the
 * kernel data structure.
 */
typedef struct task_queue_struct
{
   /*! \brief The next task queue. */
   struct task_queue_struct       *NEXT;

   /*! \brief The previous task queue. */
   struct task_queue_struct       *PREV;

   /*! \brief A validation stamp. */
   _mqx_uint                       VALID;

   /*! \brief The policy of the queue (FIFO or priority). */
   _mqx_uint                       POLICY;

   /*! \brief The queue of task descriptors. */
   QUEUE_STRUCT                    TD_QUEUE;

} TASK_QUEUE_STRUCT, * TASK_QUEUE_STRUCT_PTR;
/*! \endcond */

/*--------------------------------------------------------------------------*/
/* READY QUEUE STRUCTURE */

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This structure defines the structure of a ready queue element.
 *
 * The list is allocated as one large array of elements.
 * \n ALSO the list is ordered so that higher priority ready queue elements are
 * located at higher addresses in memory.
 * \n The CURRENT_READY_Q field of the kernel data structure points to the highest
 * priority ready queue that contains a task descriptor.
 * \n These elements are linked together via the NEXT_Q field.
 * \n This linked list is pointed to by the READY_Q_LIST field of the kernel data
 * structure.  The list is terminated by a NULL field.
 * \n The list is ordered, with the highest priority ready queue element
 * first.
 */

typedef struct ready_q_struct
{

   /*!
	* \brief The head of the ready queue for this priority level.
	*
	* Points to itself if empty. Otherwise points to first task descriptor ready
	* to run. The queue is circular, so the first task descriptor points to the
	* next task descriptor ready to run.
	*/
   TD_STRUCT_PTR                HEAD_READY_Q;

   /*! \brief The tail of the ready queue.
	*
	* Points to itself if empty. Otherwise, it points to the last task descriptor
	* on the queue, to which new tasks are added as they become ready.
	*/
   TD_STRUCT_PTR                TAIL_READY_Q;

   /*! \brief The address of the next priority queue, of lower priority. */
   struct ready_q_struct       *NEXT_Q;

   /*!
	* \brief The hardware priority of this priority queue.
	*
	* This field is copied to the tasks TASK_SR field when this task is assigned
	* to the ready queue (ie. when the task is created, or when the task changes
	* priority levels).
	* The task TASK_SR field is assigned to the ACTIVE_SR field in the kernel data
	* structure when the task becomes the ACTIVE(running) task.
	* \n The ACTIVE_SR field in the kernel data is used by _int_enable, to set
	* the correct hardware interrupt level for this task.
	*/
   uint16_t                      ENABLE_SR;

   /*! \brief The software priority of this queue, 0 being the highest priority. */
   uint16_t                      PRIORITY;

} READY_Q_STRUCT, * READY_Q_STRUCT_PTR;
/*! \endcond */

/*! \cond DOXYGEN_IGNORE */
typedef void (_CODE_PTR_ MQX_COMPONENT_CLEANUP_FPTR)(TD_STRUCT_PTR);
/*! \endcond */

/*--------------------------------------------------------------------------*/
/* THE KERNEL DATA STRUCTURE */

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Defines the kernel data structure used by MQX for all of it's state and
 * dynamic variables.
 *
 * This structure is created starting at the address provided by the
 * START_OF_KERNEL_MEMORY field of the MQX_INITIALIZATION_STRUCT provided by the
 * user as a parameter to _mqx().
 * \n The address of this structure is kept in the global variable _mqx_kernel_data.
 */

#ifndef __ASM__

typedef struct  kernel_data_struct
{
   /*
	* ----------------------------------------------------------
	* Configuration information. Used by MQX tools and debuggers
	* ----------------------------------------------------------
	*/

   /*! \brief The addressing capability of the processor. */
   uint32_t                         ADDRESSING_CAPABILITY;

   /*! \brief The endianess of the processor. */
   uint32_t                         ENDIANESS;

   /*! \brief The type of CPU that this kernel is running upon. */
   uint16_t                         CPU_TYPE;

   /* PSP Configuration data. */

   /*! \brief How a PSP aligns a memory block. */
   uint16_t                         PSP_CFG_MEMORY_ALIGNMENT;

   /*! \brief How a PSP aligns a task's stack. */
   uint16_t                         PSP_CFG_STACK_ALIGNMENT;

   /*! \brief How a PSP pads the STOREBLOCK_STRUCT. Number of reserved fields. */
   uint16_t                         PSP_CFG_MEM_STOREBLOCK_ALIGNMENT;

   /*!
	* \brief The configuration used to compile this kernel.
	*
	* Written to at mqx initialization time.
	*/
   uint16_t                         CONFIG1;
   /*!
	* \brief The configuration used to compile this kernel.
	*
	* Written to at mqx initialization time.
	*/
   uint16_t                         CONFIG2;

   /*
	* ----------------------------------------------------------
	*/

   /*! \brief This field is used to store bit flags for use by MQX primitives. */
   uint16_t                         FLAGS;

   /*!
	* \brief This field is used to by _int_disable to program the hardware correctly
	* when interrupts are to be disabled.
	*
	* The value of this field is set by the psp upon system initialization.
	* This field is equivalent to the ENABLE_SR field for the highest
	* priority MQX task (ie tasks at priority 0) which run with interrupts
	* disabled.
	*/
   uint16_t                         DISABLE_SR;

   /*!
	* \brief A count incremented when an interrupt service routine is entered.
	*
	* It thus indicates the interrupt nesting level for the current
	* interrupt being serviced.
	*/
   uint16_t                         IN_ISR;

   /*!
	* \brief This field is used by _int_enable to program the hardware correctly
	* when interrupts are to be re-enabled.
	*
	* When a READY task becomes the ACTIVE task the TASK_SR field from the
	* task descriptor is copied to this field.
	*/
   uint16_t                         ACTIVE_SR;

   /*!
	* \brief The address of the task descriptor of the currently running task.
	*
	* Note that the currently running task (the ACTIVE task) is the
	* first task on the highest priority ready queue and will be in the
	* READY state.
	*/
   TD_STRUCT_PTR                   ACTIVE_PTR;

   /*! \brief The address of the highest priority ready q. */
   READY_Q_STRUCT_PTR              READY_Q_LIST;

   /*! \brief The address of the highest priority occupied ready queue. */
   READY_Q_STRUCT_PTR              CURRENT_READY_Q;


   /*!
	* \brief This is the default ISR that is called whenever a users ISR is
	* not available
	*/
   INT_ISR_FPTR                    DEFAULT_ISR;

   /*!
	* \brief The interrupt vector for the first interrupt that the
	* application wants to have a 'C' ISR for.
	*/
   _mqx_uint                       FIRST_USER_ISR_VECTOR;

   /*! \brief The last interrupt vector that the application wants to handle. */
   _mqx_uint                       LAST_USER_ISR_VECTOR;

   /*!
	* \brief A pointer to the CURRENT interrupt handler context.
	*
	* This is a link list of context information kept on the interrupt stack
	* which keeps an error code and a status register for each nested interrupt.
	*/
   PSP_INT_CONTEXT_STRUCT_PTR      INTERRUPT_CONTEXT_PTR;

   /*! \brief A pointer to a table of 'C' handlers for interrupts. */
   INTERRUPT_TABLE_STRUCT_PTR      INTERRUPT_TABLE_PTR;

   /*! \brief The address of the base of the interrupt stack. */
   void                           *INTERRUPT_STACK_PTR;

   /*! \brief The kernel log control variable for KLOG entry control. */
   uint32_t                         LOG_CONTROL;

   /*! \brief The address of the last task descriptor being logged. */
   void                           *LOG_OLD_TD;

   /*!
	* \brief The address of the task descriptor of the currently running task
	* that uses the floating point co-processor.
	*
	* Note this may be different from the ACTIVE_PTR, as the floating point
	* register context switches are only performed between FLOATING_POINT_TASKS
	* (where applicable).
	*/
   TD_STRUCT_PTR                   FP_ACTIVE_PTR;



   /*!
	* \brief The address of the task descriptor of the currently running task
	* that uses the DSP co-processor.
	*
	* Note this may be different from the ACTIVE_PTR, as the DSP register
	* context switches are only performed between DSP_TASKS (where applicable).
	*/
   TD_STRUCT_PTR                   DSP_ACTIVE_PTR;


   /*! \brief TD for the system task. */
   TD_STRUCT                       SYSTEM_TD;


/*************************************************/

   /*! \brief This field contains the address of _int_kernel_isr. */
   INT_KERNEL_ISR_FPTR             INT_KERNEL_ISR_ADDR;

   /*! \brief The number of task templates on this processor. */
   _mqx_uint                       NUM_TASK_TEMPLATES;

   /*! \brief The address of the local task template list. */
   TASK_TEMPLATE_STRUCT_PTR        TASK_TEMPLATE_LIST_PTR;

   /*! \brief Kernel Scheduler constants. */
   _mqx_uint                       LOWEST_TASK_PRIORITY;

   /*! \brief The kernel counter (used to provide unique numbers). */
   _mqx_uint                       COUNTER;


   /*! \brief A queue of all allocated TDs. */
   QUEUE_STRUCT                    TD_LIST;

   /*! \brief A counter used to build a unique Task ID. */
   _mqx_uint                       TASK_NUMBER;

   /*! \brief Pointer in TD list where to insert the next TD after. */
   TD_STRUCT_PTR                   INSERT_TD_PTR;

   /*! \brief The list of task queue structures for explicit scheduling. */
   QUEUE_STRUCT                    KERNEL_TASK_QUEUES;

   /*! \brief A queue of lightweight semaphores that have been created. */
   QUEUE_STRUCT                    LWSEM;

   /*! \brief Semaphore for serializing create/destroy. */
   LWSEM_STRUCT                    TASK_CREATE_LWSEM;

#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM
   /*! \brief Context information for kernel memory. */
   MEMPOOL_STRUCT                  KD_POOL;

   /*! \brief Used to keep track of all memory pools. */
   MEMORY_COMPONENT_STRUCT         MEM_COMP;
#endif /* MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM */

#if MQX_USE_UNCACHED_MEM
#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM
   /*! \brief Context information for kernel memory. */
   MEMPOOL_STRUCT_PTR              UNCACHED_POOL;
#else
   /*! \brief Context information for kernel memory. */
   void*                           UNCACHED_POOL;
#endif /* MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM */
#endif /* MQX_USE_UNCACHED_MEM */





   /*! \brief The current internal time, from start of processor. */
   MQX_TICK_STRUCT                 TIME;

   /*! \brief The current time offset, generated by _time_set. */
   MQX_TICK_STRUCT                 TIME_OFFSET;

   /*!
	* \brief The MQX timeout queue.
	*
	* Tasks waiting for a timeout are placed into this queue.
	*/
   QUEUE_STRUCT                    TIMEOUT_QUEUE;

   /*! \brief The system clock interrupt vector number. */
   _mqx_uint                       SYSTEM_CLOCK_INT_NUMBER;

   /*! \brief This is the period in milliseconds of the clock interrupt. */
   _mqx_uint                       KERNEL_ALARM_RESOLUTION;

   /*! \brief The number of times the timer interrupts or ticks in a second. */
   _mqx_uint                       TICKS_PER_SECOND;

   /*! \brief The system clock hardware register reference value. */
   _mqx_uint                       TIMER_HW_REFERENCE;

   /*! \brief The number of hardware ticks in a ticks. */
   uint32_t                         HW_TICKS_PER_TICK;

   /*!
	* \brief This is the function that is called when the number of HW ticks
	* is needed.
	*/
   MQX_GET_HWTICKS_FPTR            GET_HWTICKS;

   /*! \brief This is the parameter that is passed to the get HW ticks function. */
   void                           *GET_HWTICKS_PARAM;

   /*
	* \brief This field holds a function pointer for a secondary timer,
	* if installed.
	*/
   // UNUSED: void                (_CODE_PTR_ TIMER2)(void *);




   /*! \brief A pointer to MQX initialization structure. */
  MQX_INITIALIZATION_STRUCT       INIT;




   /*! \brief Lightweight semaphore to protect component creation. */
   LWSEM_STRUCT                    COMPONENT_CREATE_LWSEM;

   /*!
	* \brief Kernel component data pointers.
	*
	* When a kernel component is installed, it allocates memory and stores a
	* pointer to this memory in this array.
	*/
   void                           *KERNEL_COMPONENTS[MAX_KERNEL_COMPONENTS];


#if MQX_COMPONENT_DESTRUCTION
   /*!
	* \brief Kernel component task destruction cleanup functions.
	*
	* When a task is destroyed, each component's cleanup function is called to
	* free any resources that the task may have acquired.
	*/
   MQX_COMPONENT_CLEANUP_FPTR      COMPONENT_CLEANUP[MAX_KERNEL_COMPONENTS];

#endif

   /*! \brief Semaphore for serializing open/close. */
   LWSEM_STRUCT                    IO_LWSEM;

   /*!
	* \brief When I/O device drivers are added, their initialization tables are
	* linked onto this queue by the I/O driver installation function.
	*/
   QUEUE_STRUCT                    IO_DEVICES;

   /*!
	* \brief The default stdin I/O FILE pointer constructed at initialization
	* time from the IO_FUNCTION field of the MQX_INITIALIZATION_STRUCT.
	*/
   void                           *PROCESSOR_STDIN;
   /*!
	* \brief The default stdout I/O FILE pointer constructed at initialization
	* time from the IO_FUNCTION field of the MQX_INITIALIZATION_STRUCT.
	*/
   void                           *PROCESSOR_STDOUT;
   /*!
	* \brief The default stderr I/O FILE pointer constructed at initialization
	* time from the IO_FUNCTION field of the MQX_INITIALIZATION_STRUCT.
	*/
   void                           *PROCESSOR_STDERR;

   /*!
	* \brief IO component data pointers.
	*
	* When an IO component is installed, it allocates memory and stores a
	* pointer to this memory in this array.
	*/
   void                           *IO_COMPONENTS[MAX_IO_COMPONENTS];

#if MQX_USE_IDLE_TASK

   /*!
	* \brief Idle loop counters, incremented by the idle task as it executes.
	*
	* IDLE_LOOP1 in incremented until it reaches 0. When IDLE_LOOP1 wraps to 0,
	* IDLE_LOOP2 is incremented etc.
	*/
   IDLE_LOOP_STRUCT                IDLE_LOOP;

   /*! \brief System Task Templates. */
   TASK_TEMPLATE_STRUCT            IDLE_TASK_TEMPLATE;
#endif

#if MQXCFG_INIT_TASK
   TASK_TEMPLATE_STRUCT            INIT_TASK_TEMPLATE;
#endif

#if MQX_EXIT_ENABLED || MQX_CRIPPLED_EVALUATION
   /*! \brief An address used by _mqx_exit. */
   MQX_EXIT_FPTR                   EXIT_HANDLER;

   /*! \brief used by MQX exit. */
   void                           *USERS_STACK;
   /*! \brief used by MQX exit. */
   _mqx_max_type                   USERS_VBR;
   /*! \brief used by MQX exit. */
   _mqx_uint                       USERS_ERROR;
#endif


#if MQX_HAS_TIME_SLICE
   /*! \brief Kernel Scheduler default task creation attribute. */
   _mqx_uint                       SCHED_POLICY;
   /*! \brief Kernel Scheduler default task creation attribute. */
   MQX_TICK_STRUCT                 SCHED_TIME_SLICE;
#endif


#if MQX_USE_TIMER

   /*!
	* \brief When this field is not NULL, the kernel timer ISR will call it at
	* each timer interrupt.
	*
	* It is set by the timer component.
	*/
   TIMER_COMPONENT_ISR_FPTR        TIMER_COMPONENT_ISR;
#endif

#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM
   /*! \brief Queue for storing Light weight memory pools. */
   QUEUE_STRUCT                    LWMEM_POOLS;

   /*! \brief The light weight memory pool for default memory allocation. */
   void                           *KERNEL_LWMEM_POOL;
#endif

#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_TLSF
   /*! \brief tlsf instance pointer for default memory allocation. */
   void*                          KERNEL_TLSF_POOL;
#endif


#if MQX_USE_LWEVENTS
   /*! \brief Queue for storing Light weight events. */
   QUEUE_STRUCT                    LWEVENTS;
#endif

#if MQX_USE_LWMSGQ
   /*! \brief Queue for storing Light weight message queues. */
   QUEUE_STRUCT                    LWMSGQS;
#endif

#if MQX_USE_LWTIMER
   /*! \brief Queue for storing Light weight timers. */
   QUEUE_STRUCT                    LWTIMERS;

   /*! \brief The lwtimer ISR called from the kernel timer ISR. */
   TIMER_COMPONENT_ISR_FPTR        LWTIMER_ISR;
#endif

#if MQX_IS_MULTI_PROCESSOR
   /*!
	* \brief This function is called to handle IPC functionality.
	*
	* When a component determines that a request is for a different processor,
	* it calls this function.
	* The first parameter indicates which processor number to send
	*    the message to,
	* the second parameter indicates which component is to handle the
	*    IPC message on the remote CPU.
	* The third parameter indicates which function in the component to perform,
	* the fourth parameter indicates the number of additional parameters
	*    (All _mqx_uint).
	* Following are the additional parameters.
	*/
   MQX_IPC_FPTR                    IPC;

   /*! \brief A pointer to the context information required by the IPC. */
   void                           *IPC_COMPONENT_PTR;

   /*! \brief The IPC task ID. */
   _task_id                        MY_IPC_ID;

   /*! \brief The IPC task descriptor. */
   TD_STRUCT_PTR                   MY_IPC_TD_PTR;

   /*! \brief Message pool_id of the named pool, for IPC requests.*/
   void                           *IPC_NAMED_POOL;

   /*! \brief Queue for storing PCB pools. */
   QUEUE_STRUCT                    IO_PCB_POOLS;

#endif

#if PSP_HAS_SUPPORT_STRUCT
   /*! \brief A pointer for use by any PSP Support functions. */
   void                           *PSP_SUPPORT_PTR;
#endif


#if MQX_EXTRA_TASK_STACK_ENABLE
   /*!
	* \brief Size of extra memory reserved at the top of each task
	* (user-defined.).
	*/
   _mqx_uint                       TOS_RESERVED_SIZE;
   /*!
	* \brief Align mask of extra memory reserved at the top of each task
	* (user-defined).
	*/
   uint8_t                          TOS_RESERVED_ALIGN_MASK;
   /*! \brief Spare fields. */
   uint8_t                          RESERVED_FOR_ALIGNMENT_FILL[3];
#endif /* MQX_EXTRA_TASK_STACK_ENABLE */

#if MQXCFG_SPARSE_ISR_TABLE
   /*! \brief Sparse table ISR count.*/
	uint16_t                        SPARSE_ISR_COUNT;
   /*! \brief Sparse table ISR shift.*/
	uint16_t                        SPARSE_ISR_SHIFT;
#endif
#if MQX_KERNEL_LOGGING
# if MQX_CRIPPLED_EVALUATION
   /*! \brief Count of klog logs.*/
   _mqx_uint                       MQX_KLOG_COUNT;
# endif
#endif

#if MQX_ENABLE_USER_MODE

#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM
   /*! \brief The light weight memory pool for default memory allocation. */
   void                           *KD_USER_POOL;
#endif

   /*! \brief A queue of user light weight semaphores that have been created. */
   QUEUE_STRUCT                    USR_LWSEM;

#if MQX_USE_LWEVENTS
   /*! \brief Queue for storing user light weight events. */
   QUEUE_STRUCT                    USR_LWEVENTS;
#endif

#if MQX_USE_LWMSGQ
   /*! \brief Queue for storing user light weight message queues. */
   QUEUE_STRUCT                    USR_LWMSGQS;
#endif
   /*! \brief Number of user task running.*/
   _mqx_uint                       USR_TASK_RUN_COUNT;

#endif
   /*! \brief FD table presentation - pointer to file descriptors table. This pointer is used only for TAD */
   void *FD_TABLE;

} KERNEL_DATA_STRUCT, * KERNEL_DATA_STRUCT_PTR;
/*! \endcond */

#endif

/*--------------------------------------------------------------------------*/
/*
 * KERNEL INTERNAL FUNCTION PROTOTYPES
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __TAD_COMPILE__
extern struct kernel_data_struct      *_mqx_kernel_data;

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _int_kernel_isr_return_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_block_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_context_switch_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_execute_scheduler_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern char *_klog_get_function_name_internal(uint32_t index);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _klog_get_task_stack_usage_internal(
	TD_STRUCT_PTR td_ptr,
	_mem_size_ptr stack_size_ptr,
	_mem_size_ptr stack_used_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_isr_end_internal(_mqx_uint vector_number);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_isr_start_internal(_mqx_uint vector_number);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_log(
	_mqx_uint type,
	_mqx_max_type p1,
	_mqx_max_type p2,
	_mqx_max_type p3,
	_mqx_max_type p4,
	_mqx_max_type p5
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _klog_yield_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _lwsem_wait_timed_internal(
	LWSEM_STRUCT_PTR sem_ptr,
	TD_STRUCT_PTR td_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _mqx_init_kernel_data_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _task_id _task_create_internal(
	_processor_number processor_number,
	_mqx_uint template_index,
	uint32_t parameter,
	bool user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern TD_STRUCT_PTR _task_alloc_td_internal(
	_mem_size stack_size,
	_mem_size_ptr overhead,
	void *input_stack_ptr,
	_mqx_uint user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern TD_STRUCT_PTR _task_build_internal(
	_mqx_uint template_index,
	uint32_t parameter,
	void *stack_ptr,
	_mqx_uint stack_size,
	bool user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _task_fill_stack_internal(
	_mqx_uint_ptr addr_ptr,
	_mqx_uint size
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern TD_STRUCT_PTR _task_init_internal(
	TASK_TEMPLATE_STRUCT_PTR template_ptr,
	_task_id creator_task_id,
	uint32_t create_parameter,
	bool dynamic,
	void *input_stack_ptr,
	_mem_size input_stack_size
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _task_ready_internal(TD_STRUCT_PTR td_ptr);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _task_set_error_td_internal(
	TD_STRUCT_PTR td_ptr,
	_mqx_uint new_error_code
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _task_destroy_internal(
	_task_id task_id,
	bool  user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _task_abort_internal(
	_task_id task_id,
	bool user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_boost_priority_internal(
	register TD_STRUCT_PTR td_ptr,
	register _mqx_uint priority
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _sched_get_max_priority_on_q_internal(
	register QUEUE_STRUCT_PTR queue_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_insert_priorityq_internal(
	register QUEUE_STRUCT_PTR queue_ptr,
	register TD_STRUCT_PTR td_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_set_priority_internal(
	register TD_STRUCT_PTR td_ptr,
	register _mqx_uint new_priority
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_unboost_priority_internal(
	register TD_STRUCT_PTR td_ptr,
	register _mqx_uint number_of_boosts
);
/*! \endcond */

/*
 * Prototypes from psp specific directories
 */
#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void *_mem_alloc_internal(
	_mem_size requested_size,
	TD_STRUCT_PTR td_ptr,
	MEMPOOL_STRUCT_PTR mem_pool_ptr,
	_mqx_uint_ptr error_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void *_mem_alloc_at_internal(
	_mem_size          requested_size,
	void              *requested_addr,
	TD_STRUCT_PTR      td_ptr,
	MEMPOOL_STRUCT_PTR mem_pool_ptr,
	_mqx_uint_ptr      error_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void *_mem_alloc_internal_align(
	_mem_size requested_size,
	_mem_size req_align,
	TD_STRUCT_PTR td_ptr,
	MEMPOOL_STRUCT_PTR mem_pool_ptr,
	_mqx_uint_ptr error_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _mem_alloc_extend_internal(
	void *mem_ptr,
	_mem_size size
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _mem_free_part_internal(
	void *mem_ptr,
	_mem_size requested_size
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void *_mem_get_next_block_internal(
	TD_STRUCT_PTR td_ptr,
	void *memory_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _mem_init_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _mem_transfer_td_internal(
	void *memory_ptr,
	TD_STRUCT_PTR source_td,
	TD_STRUCT_PTR target_td
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _mem_transfer_internal(
	void *memory_ptr,
	TD_STRUCT_PTR target_td
);
/*! \endcond */

#endif/* MQXCFG_ALLOCATOR == MQX_ALLOCATOR_MEM */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_check_scheduler_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_execute_scheduler_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_run_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _sched_set_rr_interval_internal(
	_task_id task_id,
	MQX_TICK_STRUCT_PTR new_rr_tick_ptr,
	MQX_TICK_STRUCT_PTR old_rr_tick_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _sched_get_rr_interval_internal(
	_task_id task_id,
	MQX_TICK_STRUCT_PTR tick_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _sched_start_internal(_mqx_uint addr_ptr, _mqx_uint size);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _task_exit_function_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern uint32_t _task_get_parameter_internal(
	TD_STRUCT_PTR td_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern uint32_t _task_set_parameter_internal(
	uint32_t new_value,
	TD_STRUCT_PTR td_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _task_sync_priority_internal(
	TD_STRUCT_PTR td_ptr
 );
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern bool _time_check_if_leap(uint16_t year);
/*! \endcond */

extern const uint16_t _time_days_in_month_internal[2][13];
extern const int32_t  _timezone;

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _time_delay_internal(
	register TD_STRUCT_PTR td_ptr
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern bool _psp_build_stack_frame(
	  TD_STRUCT_PTR td_ptr,
	  void *stack_ptr,
	  _mem_size stack_size,
	  TASK_TEMPLATE_STRUCT_PTR template_ptr,
	  _mqx_uint status_register,
	  uint32_t create_parameter
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _psp_destroy_stack_frame(TD_STRUCT_PTR td_ptr);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern PSP_STACK_START_STRUCT_PTR _psp_get_stack_start(TD_STRUCT_PTR td_ptr);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _psp_init_readyqs(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _psp_set_kernel_disable_level(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _psp_save_fp_context_internal(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _lwsem_create_internal(
	LWSEM_STRUCT_PTR sem_ptr,
	_mqx_int initial_number,
	bool hidden,
	bool user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _lwsem_destroy_internal(
	LWSEM_STRUCT_PTR sem_ptr,
	bool user
);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern _mqx_uint _mqx_api_call(MQX_API_NUMBER_ENUM, MQX_API_CALL_PARAMS_PTR);
/*! \endcond */

#if PSP_BYPASS_P3_WFI

/*!
 * \cond DOXYGEN_PRIVATE
 */
void _sleep_p3(uint32_t*);
/*! \endcond */

#endif

/*
 * General prototypes from bsp specific directories, called by MQX
 */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Function install BSP specific drivers to MQX.
 * This function is called from init task and run befor any application tasks.
 */
int _bsp_init(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 */
int _bsp_pre_init(void);
int _bsp_MQX_tick_timer_init(void);
/*! \endcond */

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Post initial function.
 * This function is called with disabled preemtion!!!
 * Any task block function can switch to another task and it can cause the init
 * task will not be finished (destroyed) and resources (stack, ...) will remain
 * allocated.
 */
int _bsp_post_init(void);
/*! \endcond */


/*
 * General prototypes from bsp specific directories, internal to BSP
 */

/*!
 * \cond DOXYGEN_PRIVATE
 */
extern void _bsp_exit_handler(void);
/*! \endcond */

#endif /* __TAD_COMPILE__ */

#ifdef __cplusplus
}
#endif

#endif /* __ASM__ */

#endif
