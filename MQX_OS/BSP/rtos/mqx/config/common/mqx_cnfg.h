
/*HEADER**********************************************************************
*
* Copyright 2008-2010 Freescale Semiconductor, Inc.
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
*   This file contains the default configuration definitions for configuring
*   the various optional features of MQX. Individual config. Defines can be
*   overridden in the "user_config.h" file.
*
*   By default, MQX is configured as follows:
*    MQX_ALLOW_TYPED_MEMORY               1
*    MQX_COMPONENT_DESTRUCTION            1
*    MQX_DEFAULT_TIME_SLICE_IN_TICKS      0
*    MQX_EXIT_ENABLED                     1
*    MQX_EXTRA_TASK_STACK_ENABLE          1
*    MQX_HAS_TIME_SLICE                   1
*    MQX_CHECK_ERRORS                     1
*    MQX_CHECK_MEMORY_ALLOCATION_ERRORS   1
*    MQX_CHECK_VALIDITY                   1
*    MQX_INCLUDE_FLOATING_POINT_IO        0
*    MQX_IS_MULTI_PROCESSOR               1
*    MQX_KERNEL_LOGGING                   1
*    MQX_LWLOG_TIME_STAMP_IN_TICKS        1
*    MQX_MEMORY_FREE_LIST_SORTED          1
*    MQX_MONITOR_STACK                    1
*    MQX_MUTEX_HAS_POLLING                1
*    MQX_PROFILING_ENABLE                 0
*    MQX_RUN_TIME_ERR_CHECK_ENABLE        0
*    MQX_TASK_CREATION_BLOCKS             1
*    MQX_TASK_DESTRUCTION                 1
*    MQX_TIMER_USES_TICKS_ONLY            0
*    MQX_USE_32BIT_MESSAGE_QIDS           0
*    MQX_USE_32BIT_TYPES                  0
*    MQX_USE_EVENTS                       1
*    MQX_USE_IDLE_TASK                    1
*    MQX_USE_INLINE_MACROS                1
*    MQX_USE_IPC                          1
*    MQX_USE_LOGS                         1
*    MQX_USE_LWEVENTS                     1
*    MQX_USE_LWLOGS                       1
*    MQX_USE_LWMSGQ                       1
*    MQX_USE_LWTIMER                      1
*    MQX_USE_MESSAGES                     1
*    MQX_USE_MUTEXES                      1
*    MQX_USE_NAME                         1
*    MQX_USE_PARTITIONS                   1
*    MQX_USE_SEMAPHORES                   1
*    MQX_USE_SW_WATCHDOGS                 1
*    MQX_USE_TIMER                        1
*    MQX_VERIFY_KERNEL_DATA               1
*    MQX_ENABLE_USER_MODE                 0
*    MQX_ENABLE_LOW_POWER                 0
*    MQXCFG_ALLOCATOR                     MQX_ALLOCATOR_TLSF
*    MQXCFG_MAX_FD                        5
*    MQX_CUSTOM_MAIN                      0
*    BSPCFG_ENABLE_IO_SUBSYSTEM           1
*    MQXCFG_PREALLOCATED_SYSTEM_STACKS    0
*    MQXCFG_PREALLOCATED_READYQ           0
*    MQXCFG_STATIC_MUTEX                  0
*    MQXCFG_STATIC_LWLOG                  0
*    MQXCFG_STATIC_KLOG                   0
*    MQXCFG_STATIC_ISR_TABLE              0
*    MQXCFG_STATIC_FP_CONTEXT             0
*    MQXCFG_PRINT_MEM_DUMP                0
*    MQXCFG_SPARSE_ISR_TABLE              1
*    MFSCFG_MINIMUM_FOOTPRINT             1
*    MQX_HAS_EXCEPTION_HANDLER            1
*    MQXCFG_LOWEST_TASK_PRIORITY          16
*    BSP_FIRST_INTERRUPT_VECTOR_USED      (PSP_MAXIMUM_INTERRUPT_VECTORS)
*
*END************************************************************************/

#ifndef __mqx_cnfg_h__
#define __mqx_cnfg_h__

/***************************************************************************
**
** ENUM DEFINITIONS SECTION
**
** Warning:
** this section IS NOT INTENDED TO CONFIGURE MQX.
** Please see the "CONFIGURATION" section below for default settings.
**
** MGCT: <category name="Enum Definitions">
**
****************************************************************************/

/* Default Configuration options
 * MQX_COMMON_CONFIG could contain one of following values:
*/
#define MQX_LITE_CONFIG       1
#define MQX_SMALL_RAM_CONFIG  2

/* Default Configuration options
 * MQXCFG_ALLOCATOR could contain one of following values:
*/
#define MQX_ALLOCATOR_NONE    0 // no allocators
#define MQX_ALLOCATOR_TOOL    1 // toolchain allocators (IAR, MDK, GCC)
#define MQX_ALLOCATOR_LWMEM   2
#define MQX_ALLOCATOR_MEM     3
#define MQX_ALLOCATOR_TLSF    4

/** MGCT: </category> */

/***************************************************************************
**
** CONFIGURATION SECTION
**
****************************************************************************/

/*
** get user configuration constants.
*/
#include "mqx_sdk_config.h"

/*
** some setting may be forced by source files (before including this file)
*/
#ifdef MQX_FORCE_USE_INLINE_MACROS
#undef  MQX_USE_INLINE_MACROS
#define MQX_USE_INLINE_MACROS  MQX_FORCE_USE_INLINE_MACROS
#endif

/***************************************************************************
**
** SCHEDULER OPTIONS
**
** MGCT: <category name="Scheduler Options">
**
****************************************************************************/

/*
** When MQX_HAS_TICK is defined as 1, MQX includes support for tick time and all related
** functionality of delaying tasks, waiting for synchronization objects with timeout etc.
** Only change this option to 0 if you know what you are doing.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_TICK
#define MQX_HAS_TICK 1
#endif


/*
** When MQX_DEFAULT_TIME_SLICE_IN_TICKS is define as 1,
** then the default time slice in the task template structure
** is in units of ticks. Changing this to 0 will change the units
** to milliseconds. This also affects the time slice field in the
** task template since this is used to set a task's default time
** slice.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_DEFAULT_TIME_SLICE_IN_TICKS
#define MQX_DEFAULT_TIME_SLICE_IN_TICKS  0
#endif


/*
** When MQX_HAS_TIME_SLICE is defined as 1,
** then code is compiled in to support time sliced tasks.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_TIME_SLICE
#define MQX_HAS_TIME_SLICE 1
#endif

/*
** When MQX_HAS_DYNAMIC_PRIORITIES is defined as 1, MQX includes code to change
** task priorities dynamically by _task_set_priority() call or by priority inheritance or priority boosting.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_DYNAMIC_PRIORITIES
#define MQX_HAS_DYNAMIC_PRIORITIES 1
#endif


/*
** When MQX_USE_IDLE_TASK is defined as 1,
** the kernel will create the idle task which will execute when no other tasks are ready,
** otherwise, the processor will stop when there are no tasks to run.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_IDLE_TASK
#define MQX_USE_IDLE_TASK 1
#endif



/***************************************************************************
**
** SYNCHRONIZATION COMPONENT OPTIONS
**
** MGCT: <category name="Synchronization Objects Options">
**
****************************************************************************/

/*
** When MQX_USE_EVENTS is defined as 1,
** then mqx will compile in the support code for event component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_EVENTS
#define MQX_USE_EVENTS 1
#endif

/*
** When MQX_USE_LWEVENTS is defined as 1,
** then mqx will compile in the support code for light weight event component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_LWEVENTS
#define MQX_USE_LWEVENTS 1
#endif

/*
** When MQX_USE_MESSAGES is defined as 1,
** then mqx will compile in the support code for messages.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_MESSAGES
#define MQX_USE_MESSAGES 1
#endif

/*
** Enables message receive calls to set MSGQ_MESSAGE_NOT_AVAILABLE task
** error code when message is not available.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_ENABLE_MSG_TIMEOUT_ERROR
#define MQXCFG_ENABLE_MSG_TIMEOUT_ERROR 0
#endif

/*
** When MQX_USE_LWMSGQ is defined as 1,
** then mqx will compile in the support code for light weight message queues.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_LWMSGQ
#define MQX_USE_LWMSGQ 1
#endif

/*
** When MQX_USE_MUTEXES is defined as 1,
** then mqx will compile in the support code for mutex component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_MUTEXES
#define MQX_USE_MUTEXES 1
#endif

/*
** When MQX_MUTEX_HAS_POLLING is defined as 1,
** then extra code will be compiled in to support the
** mutex options: MUTEX_SPIN_ONLY and MUTEX_LIMITED_SPIN
** MGCT: <option type="bool"/>
*/
#ifndef MQX_MUTEX_HAS_POLLING
#define MQX_MUTEX_HAS_POLLING 1
#endif

/*
** When MQX_USE_SEMAPHORES is defined as 1,
** then mqx will compile in the support code for Semaphores
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_SEMAPHORES
#define MQX_USE_SEMAPHORES 1
#endif

/** MGCT: </category> */

/***************************************************************************
**
** OTHER MQX OBJECTS OPTIONS
**
** MGCT: <category name="Other MQX Objects Options">
**
****************************************************************************/

/*
** When MQX_USE_LOGS is defined as 1,
** then mqx will compile in the support code for log component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_LOGS
#define MQX_USE_LOGS 1
#endif

/*
** When MQX_USE_LWLOGS is defined as 1,
** then mqx will compile in the support code for light weight log component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_LWLOGS
#define MQX_USE_LWLOGS 1
#endif

/*
** When MQX_LWLOG_TIME_STAMP_IN_TICKS is define as 0,
** then the time stamp in the light weight log component
** is in seconds, milliseconds, and microseconds. Changing this
** to 1 will causes the time stamp to be in ticks which is not
** as readable, but has much less overhead.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_LWLOG_TIME_STAMP_IN_TICKS
#define MQX_LWLOG_TIME_STAMP_IN_TICKS  1
#endif

/*
** When MQX_USE_IO_COMPONENTS is defined as 1, the MQX maintains the list of additional
** functional components like RTCS, MFS or USB in kernel data. Disable this feature
** only with MQX-only applications to save some RAM.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_IO_COMPONENTS
#define MQX_USE_IO_COMPONENTS 1
#endif

/*
** When MQX_USE_TIMER is defined as 1,
** then mqx will compile in the support code for timer component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_TIMER
#define MQX_USE_TIMER 1
#endif

/*
** When MQX_TIMER_USES_TICKS_ONLY, the timer task will *only* process
** timer periodic and one-shot requests using ticks for timeout
** reporting, rather than the MQX2.40 milliseconds/seconds.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TIMER_USES_TICKS_ONLY
#define MQX_TIMER_USES_TICKS_ONLY 0
#endif

/*
** When MQX_USE_LWTIMER is defined as 1,
** then mqx will compile in the support code for light weight timers.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_LWTIMER
#define MQX_USE_LWTIMER 1
#endif

/*
** When MQX_USE_NAME is defined as 1,
** then mqx will compile in the support code for name component.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_NAME
#define MQX_USE_NAME 1
#endif

/*
** When MQX_USE_SW_WATCHDOGS is defined as 1,
** then mqx will compile in the support code for software watchdog timers.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_SW_WATCHDOGS
#define MQX_USE_SW_WATCHDOGS 1
#endif

/** MGCT: </category> */

/***************************************************************************
**
** MEMORY ALLOCATION OPTIONS
**
** MGCT: <category name="Memory Allocation Options">
**
****************************************************************************/

/*
** When MQXCFG_ALLOCATOR is defined as 1,
** then mqx will use the dynamic allocations.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_ALLOCATOR
#define MQXCFG_ALLOCATOR  MQX_ALLOCATOR_LWMEM
#endif

/*
** When MQX_ALLOCATOR_GARBAGE_COLLECTING is defined as 1,
** then mqx will do the garbage collectin on task destroy.
*/
#ifndef MQX_ALLOCATOR_GARBAGE_COLLECTING
#define MQX_ALLOCATOR_GARBAGE_COLLECTING  1
#endif

/*
** When MQX_ALLOCATOR_ALLOW_IN_ISR is defined as 1,
** then the allocations are allowed in ISR context
*/
#ifndef MQX_ALLOCATOR_ALLOW_IN_ISR
#define MQX_ALLOCATOR_ALLOW_IN_ISR 1
#endif

/*
** When MQX_USE_UNCACHED_MEM is defined as 1,
** then mqx will compile in the support for uncached memory allocation.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_UNCACHED_MEM
	#if MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM
		#define MQX_USE_UNCACHED_MEM         0
	#else
		#define MQX_USE_UNCACHED_MEM         1
	#endif
#endif

/*
** When MQX_USE_PARTITIONS is defined as 1,
** then mqx will compile in the support code for memory with fixed-size blocks.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_PARTITIONS
#define MQX_USE_PARTITIONS 1
#endif

/*
** When MQX_MEMORY_FREE_LIST_SORTED is defined as 1,
** then the free memory blocks are stored in order of address
** to reduce fragmentation. This can increase memory freeing time
** and code size.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_MEMORY_FREE_LIST_SORTED
#define MQX_MEMORY_FREE_LIST_SORTED   1
#endif

/*
** When MQX_ALLOW_TYPED_MEMORY is defined as 1,
** then mqx will include code that allows typed memory blocks.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_ALLOW_TYPED_MEMORY
#define MQX_ALLOW_TYPED_MEMORY 1
#endif

/** MGCT: </category> */

/***************************************************************************
**
** INTERRUPT SUPPORT AND HANDLING
**
** MGCT: <category name="Interrupt Support and Handling Options">
**
****************************************************************************/

/*
** When MQX_USE_INTERRUPTS is defined as 1, the support for interrupts is
** enabled in MQX.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_INTERRUPTS
#define MQX_USE_INTERRUPTS 1
#endif

#ifdef MQX_ROM_VECTORS
#error MQX_ROM_VECTORS is not used anymore to decide between RAM and ROM vector table. Linker symbol __ram_vector_table__  is used instead.
#endif
/*
** When MQXCFG_SPARSE_ISR_TABLE is defined as 1, the MQX interrupt service
** routine table is allocated as a "array of linked lists" instead of linear array.
** This option is independent on the MQX_ROM_VECTORS as it deals with the
** "logical" table managed by the interrupt dispatcher in MQX. With the sparse ISR
** table, only the ISRs installed by _int_install_isr call consume RAM memory.
** Interrupt latency increases as MQX needs to walk the list to find user ISR
** to be invoked.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_SPARSE_ISR_TABLE
#define MQXCFG_SPARSE_ISR_TABLE 0
#endif

/*
** When MQXCFG_SPARSE_ISR_TABLE is defined as 1, this option determines the number of bits
** the vector number is shifted to determine index of ISR linked list root.
** For example with 256 potential interrupt sources and with shift value
** of 3, it makes 256>>3=32 lists each with maximum depth of eight ISR entries.
** Shift value of 8 would yield one big linked list of all ISR entries.
** MGCT: <option type="number"/>
*/
#ifndef MQX_SPARSE_ISR_SHIFT
#define MQX_SPARSE_ISR_SHIFT 3
#endif

/** MGCT: </category> */

/***************************************************************************
**
** TERMINATION CONTROL
**
** MGCT: <category name="Task and System Termination Options">
**
****************************************************************************/

/*
** When MQX_TASK_DESTRUCTION is defined as 1,
** the kernel will allow for the destruction of tasks,
** releasing all resources owned by the task when it is destroyed
** (memory, events, semaphores, mutexes etc.).
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TASK_DESTRUCTION
#define MQX_TASK_DESTRUCTION 1
#endif

/*
** When MQX_COMPONENT_DESTRUCTION is defined as 1,
** the kernel will handle the destruction of a kernel component object
** (such as a semaphore or event).
** MGCT: <option type="bool"/>
*/
#ifndef MQX_COMPONENT_DESTRUCTION
#define MQX_COMPONENT_DESTRUCTION 1
#endif


/*
** When MQX_EXIT_ENABLED is defined as 1,
** code is compiled in to allow the application to return from _mqx().
** MGCT: <option type="bool"/>
*/
#ifndef MQX_EXIT_ENABLED
#define MQX_EXIT_ENABLED 1
#endif

/** MGCT: </category> */

/***************************************************************************
**
** RUN-TIME DIAGNOSTICS OPTIONS
**
** MGCT: <category name="Run-time Diagnostics Options">
**
****************************************************************************/

/*
** When MQX_KERNEL_LOGGING is defined as 1,
** calls to write to the kernel log will be added to some kernel functions at
** function entry (with parameters) and function exit (with error codes).
**
** The MQX functions that have logging compiled in will run slower, however
** it is only when the logging is enabled for the function that performance
** will be affected.
**
** The logging of specific functions can be controlled using the
** _klog_control function.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_KERNEL_LOGGING
#define MQX_KERNEL_LOGGING   1
#endif

/*
** When MQX_CHECK_ERRORS is defined as 1,
** kernel functions will perform error checking on their parameters.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_CHECK_ERRORS
#define MQX_CHECK_ERRORS 1
#endif


/*
** When MQX_CHECK_MEMORY_ALLOCATION_ERRORS is defined as 1,
** kernel functions will check all memory allocations for errors.
** The functions will verify that the memory allocation
** was successful.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_CHECK_MEMORY_ALLOCATION_ERRORS
#define MQX_CHECK_MEMORY_ALLOCATION_ERRORS 1
#endif


/*
** When MQX_CHECK_VALIDITY is defined as 1,
** kernel functions will check the the VALIDITY fields of structures
** as they are accessed.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_CHECK_VALIDITY
#define MQX_CHECK_VALIDITY 1
#endif

/*
** When MQX_VERIFY_KERNEL_DATA is defined as 1,
** then mqx will perform a memory check at startup to
** verify it can correcly read and write kernel memory.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_VERIFY_KERNEL_DATA
#define MQX_VERIFY_KERNEL_DATA 1
#endif

/*
** When MQX_MONITOR_STACK is defined as 1,
** the kernel will fill all stacks with a known value when the stack is
** initialized.
** This allows utility software and debuggers to calculate how much of the
** stack has been used.  This will only have an impact at task creation time.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_MONITOR_STACK
#define MQX_MONITOR_STACK 1
#endif

/*
** When MQX_RUN_TIME_ERR_CHECK_ENABLE is defined as 1,
** code to support an external run-time error checking tool will be compiled
** into the kernel.  This will add to the size of the compiled image,
** and will cause MQX to run slower
**
** Note: You can only use run time error checking if the toolset being used
**       supports it
** MGCT: <option type="bool"/>
*/
#ifndef MQX_RUN_TIME_ERR_CHECK_ENABLE
#define MQX_RUN_TIME_ERR_CHECK_ENABLE 0
#endif

/** MGCT: </category> */

/***************************************************************************
**
** PROCESSOR FEATURES
**
** MGCT: <category name="Processor Hardware Feaures Options">
**
****************************************************************************/

#ifndef MQX_AUX_CORE
#define MQX_AUX_CORE    0
#endif

/*
** When MQXCFG_ENABLE_FP is defined as 1, MQX support for floating point context saving is
** enabled if the PSP supports it. The task which make use of the floating point unit must
** still be declared with MQX_FLOATING_POINT_TASK flag.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_ENABLE_FP
#define MQXCFG_ENABLE_FP  0
#endif

/*
** When MQX_SAVE_FP_ALWAYS is defined as 1, MQX always store and restore floating
** point context in scheduler and store and restore fpu registers in ISR.
** The task which make use of the floating point unit must not be declared
** with MQX_FLOATING_POINT_TASK flag.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_SAVE_FP_ALWAYS
#define MQX_SAVE_FP_ALWAYS      0
#endif

/*
** When MQX_FP_CONTEXT_CHECK is defined as 1, MQX support for floating point context
** checking is enabled. The task which make use of the floating point unit must still
** be declared with MQX_FLOATING_POINT_TASK flag.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_FP_CONTEXT_CHECK
#define MQX_FP_CONTEXT_CHECK  0
#endif

/*
** When MQX_INCLUDE_FLOATING_POINT_IO is defined as 1,
** _io_printf and _io_scanf will include floating point I/O code.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_INCLUDE_FLOATING_POINT_IO
#define MQX_INCLUDE_FLOATING_POINT_IO 0
#endif

/*
** When MQXCFG_ENABLE_DSP is defined as 1, MQX support for dsp  context saving is
** enabled if the PSP supports it. The task which make use of thedsp registers must
** still be declared with MQX_DSP_TASK flag.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_ENABLE_DSP
#define MQXCFG_ENABLE_DSP  0
#endif

/*
** When this option is defined as 1, support for USER-mode restricted tasks is enabled
** (ARM/CortexM only).
** MGCT: <option type="bool"/>
*/
#ifndef MQX_ENABLE_USER_MODE
#define MQX_ENABLE_USER_MODE        0
#endif

/*
** When this option is defined as 1, and if MQX_ENABLE_USER_MODE is enabled, the
** default access to global variables is read-write for User tasks.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_DEFAULT_USER_ACCESS_RW
#define MQX_DEFAULT_USER_ACCESS_RW  1
#endif

/*
** When this option is defined as 1, and if MQX_ENABLE_USER_MODE is enabled, the
** classic MQX API is available also for User-mode. The use of User-mode API is
** determined dynamically, based on if the calling task runs in User or Privilege
** mode.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_ENABLE_USER_STDAPI
#define MQX_ENABLE_USER_STDAPI      1
#endif

/*
** When this option is defined as 1, support for various low power modes is enabled
** (Kinetis platform so far).
** MGCT: <option type="bool"/>
*/
#ifndef MQX_ENABLE_LOW_POWER
#define MQX_ENABLE_LOW_POWER        0
#endif

/*
** Disable cache support.
*/
#ifndef MQX_DISABLE_CACHE
#define MQX_DISABLE_CACHE           0
#endif

/** MGCT: </category> */

/***************************************************************************
**
** MULTI-PROCESSOR OPTIONS
**
** MGCT: <category name="Multi-processor Options">
**
****************************************************************************/

/*
** When MQX_IS_MULTI_PROCESSOR is defined as 1,
** then code is compiled in to support multiple processor MQX systems.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_IS_MULTI_PROCESSOR
#define MQX_IS_MULTI_PROCESSOR 1
#endif

/*
** When MQX_USE_IPC is defined as 1,
** then mqx will compile in the support code for Interprocessor communication.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_IPC
#define MQX_USE_IPC 1
#endif

/*
** When MQX_TASK_CREATION_BLOCKS is defined as 1,
** the creating task will be suspended when _task_create is called for a task
** residing on a different processor.
**
** The creator task will be suspended until the target task is created,
** and an error code is returned.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TASK_CREATION_BLOCKS
#define MQX_TASK_CREATION_BLOCKS 1
#endif

/*
** When MQX_USE_32BIT_MESSAGE_QIDS is defined as 1,
** the message component datatypes (_queue_numbe & _queue_id) will be uint_32s
** wide instead of uint_16s.
** This will allow for more than 256 message queues on a cpu, and
** more than 256 processors in a multi-processor network.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_32BIT_MESSAGE_QIDS
#define MQX_USE_32BIT_MESSAGE_QIDS 0
#endif

/** MGCT: </category> */

/***************************************************************************
**
** OTHER CODE-SIZE OPTIMIZATION OPTIONS
**
** MGCT: <category name="Other Code-size and RAM-size Optimization Options">
**
****************************************************************************/

/*
** When MQXCFG_EXCEPTION_HANDLING is defined as 1, MQX includes code to handle
** exceptions.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_EXCEPTION_HANDLING
#define MQXCFG_EXCEPTION_HANDLING 1
#endif

/*
** When MQX_HAS_EXIT_HANDLER is defined as 1, MQX includes code to execute task
** exit handler before the task exits. Also the _task_set_exit_handler/_task_get_exit_handler
** calls are also included.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_EXIT_HANDLER
#define MQX_HAS_EXIT_HANDLER 1
#endif

/*
** When MQX_HAS_HW_TICKS is defined as 1, MQX includes support for hardware ticks and
** associated calls: _time_get_hwticks, _time_get_hwticks_per_tick and _psp_usecs_to_ticks.
** Note that hardware ticks also need to be supported by BSP.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_HW_TICKS
#define MQX_HAS_HW_TICKS 1
#endif

/*
** When MQX_HAS_TASK_ENVIRONMENT is defined as 1, MQX includes code to set and get
** task environment data pointer: _task_set_environment/_task_get_environment.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_TASK_ENVIRONMENT
#define MQX_HAS_TASK_ENVIRONMENT 1
#endif

/*
** When MQX_KD_HAS_COUNTER is defined as 1, the MQX kernel maintains the counter value
** which is automatically increamented any time the value is queried by _mqx_get_counter
** call.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_KD_HAS_COUNTER
#define MQX_KD_HAS_COUNTER 1
#endif

/*
** When MQX_TD_HAS_ERROR_CODE is defined as 1, the MQX task descriptors maintain the
** error code which is accessible with _task_set_error/_task_get_error calls.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TD_HAS_ERROR_CODE
#define MQX_TD_HAS_ERROR_CODE 1
#endif

/*
** When MQX_TD_HAS_PARENT is defined as 1, the MQX task descriptors maintain the task's
** creator ID which is available through _task_get_creator call.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TD_HAS_PARENT
#define MQX_TD_HAS_PARENT 1
#endif

/*
** When MQX_TD_HAS_STACK_LIMIT is defined as 1, the MQX task descriptors maintain the
** task limit value which is needed by various stack overflow checking calls like
** _task_check_stack.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TD_HAS_STACK_LIMIT
#define MQX_TD_HAS_STACK_LIMIT 1
#endif

/*
** When MQX_TD_HAS_TASK_TEMPLATE_PTR is defined as 1, the MQX task descriptors maintain the
** pointer to original TASK_TEMPLATE_STRUCT used for task creation. This pointer is used
** by task restart call (_task_restart()) and by several lookup functions like
** _task_get_id_from_name().
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TD_HAS_TASK_TEMPLATE_PTR
#define MQX_TD_HAS_TASK_TEMPLATE_PTR 1
#endif

/*
** When MQX_TD_HAS_TEMPLATE_INDEX is defined as 1, the MQX task descriptors maintain the
** original index value coming from the TASK_TEMPLATE_STRUCT. This value is maintained for
** backward compatiblity only and is not used by MQX kernel.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TD_HAS_TEMPLATE_INDEX
#define MQX_TD_HAS_TEMPLATE_INDEX 1
#endif

/*
** When this option is defined as 1, private MQX implementation of memcpy is used instead
** the one from standard library.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_MEM_COPY
#define MQXCFG_MEM_COPY  0
#endif

/*
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_MEM_COPY_NEON
#define MQXCFG_MEM_COPY_NEON  0
#endif

#if MQXCFG_MEM_COPY_NEON
#undef MQX_SAVE_FP_ALWAYS
#undef MQXCFG_ENABLE_FP
#define MQXCFG_ENABLE_FP        1
#define MQX_SAVE_FP_ALWAYS      1
#endif

/*
** When this option is defined as 0, MQX use private mem_zero function, otherwise
** memset from standard library is used
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_MEM_ZERO
#define MQX_USE_MEM_ZERO  0
#endif

/*
** When MQX_USE_SMALL_MEM_COPY is defined as 1, a simple/short memory
** copy function is compiled.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_SMALL_MEM_COPY
#define MQX_USE_SMALL_MEM_COPY 0
#endif

/*
** When MQX_USE_SMALL_MEM_ZERO is defined as 1, a simple/short memory
** zeroing function is compiled.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_SMALL_MEM_ZERO
#define MQX_USE_SMALL_MEM_ZERO 0
#endif

/*
** When MQX_USE_SMALL_MEM_COPY is defined as 0 and MQX_USE_BLOCK_MEM_COPY as 1,
** a stronger _mem_copy speed optimization is enabled.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_BLOCK_MEM_COPY
#define MQX_USE_BLOCK_MEM_COPY 1
#endif

/*
** When MQX_USE_SMALL_MEM_ZERO is defined as 0 and MQX_USE_BLOCK_MEM_ZERO as 1,
** a stronger _mem_zero speed optimization is enabled.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_BLOCK_MEM_ZERO
#define MQX_USE_BLOCK_MEM_ZERO 1
#endif

/*
** When MQX_USE_INLINE_MACROS is defined as 1,
** certain internal utility functions called by MQX will
** be made "inline" rather than by function calls.  This allows the user
** to optimize for time or code space.  Inlining optimizes for time,
** not inlining optimizes for space.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_INLINE_MACROS
#define MQX_USE_INLINE_MACROS 1
#endif

/** MGCT: </category> */

/***************************************************************************
**
** COMPILER AND C-CODE OPTIONS
**
** MGCT: <category name="Compiler and C-code Options">
**
****************************************************************************/

/*
** When MQX_ENABLE_CPP is defined as 1, MQX support for C++ is enabled.
** On some Power Architecture and ColdFire platforms C++ is enabled by
** BSPCFG_ENABLE_CPP macro.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_ENABLE_CPP
#define MQX_ENABLE_CPP  0
#endif

/*
** The FILE and MQX_FILE_PTR types are deprecated since MQX 3.6 (replaced by MQX_FILE
** and MQX_FILE_PTR). The symbols are still available for backward compatibility.
** When this option is defined as 1 the old symbols are not declared/defined.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_SUPPRESS_FILE_DEF
#define MQX_SUPPRESS_FILE_DEF 0
#endif

/*
** The same for <stdio.h> definitions.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_SUPPRESS_STDIO_MACROS
#define MQX_SUPPRESS_STDIO_MACROS 0
#endif


/*
** The same for time_t definitions.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_STD_TIME_API
#define MQX_STD_TIME_API 1
#endif


/*
** The same for <string.h> definitions. By default, the string.h macros are
** suppressed in IAR EWARM 6.x to avoid conflict between MQX and system C libraries.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_SUPPRESS_STRINGH_MACROS
	#if defined(__ICCARM__) /* IAR */
		#if __VER__ >= 6000000
			#define MQX_SUPPRESS_STRINGH_MACROS 1
		#else
			#define MQX_SUPPRESS_STRINGH_MACROS 0
		#endif
	#elif defined(__CC_ARM)
			#define MQX_SUPPRESS_STRINGH_MACROS 1
	#else
		#define MQX_SUPPRESS_STRINGH_MACROS 0
	#endif
#endif

/** MGCT: </category> */

/***************************************************************************
**
** OTHER OPTIONS
**
** MGCT: <category name="Other">
**
****************************************************************************/

/*
** When MQX_EXTRA_TASK_STACK_ENABLE is defined as 1,
** code is compiled in to reserve extra memory at the top of stack
** in every task, if desired. Both "OS Changer" and the MetaWare C/C++
** runtime want additional per-task variables.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_EXTRA_TASK_STACK_ENABLE
#define MQX_EXTRA_TASK_STACK_ENABLE 1
#endif

/*
** When MQX_PROFILING_ENABLE is defined as 1,
** code to support an external profiling tool will be compiled into the kernel
** Profiling will add to the size of the compiled image, and it will run slower.
**
** Note: You can only use profiling if the toolset being used supports it.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_PROFILING_ENABLE
#define MQX_PROFILING_ENABLE 0
#endif

/*
** When MQX_THREAD_LOCAL_STORAGE_ENABLE is defined as 1,
** additional space will be reserved on the task's stack for local storage. This may be required by some tool-chains,
** but is not required for CodeWarrior.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_THREAD_LOCAL_STORAGE_ENABLE
#define MQX_THREAD_LOCAL_STORAGE_ENABLE 0
#endif

/*
** When MQX_TAD_RESERVED_ENABLE is defined as 1,
** additional space will be reserved in the task descriptor for TAD. This may be required by some tool-chains,
** but is not required for CodeWarrior.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_TAD_RESERVED_ENABLE
#define MQX_TAD_RESERVED_ENABLE 0
#endif

/*
** When MQX_USE_32BIT_TYPES is defined as 1, the kernel will be forced
** to 32 bit mode regardless of the natural size of the processor. This will
** reduce the number of warning generated by the compiler when linking 2.4x
** applications against versio 2.5x of MQX.
**
** Obsolete in MQX3.x
** MGCT: <option type="bool"/>
*/
#ifndef MQX_USE_32BIT_TYPES
#define MQX_USE_32BIT_TYPES 0
#endif

/*
** MGCT: <option type="bool"/>
*/
#ifndef MQX_GUERRILLA_INTERRUPTS_EXIST
#define MQX_GUERRILLA_INTERRUPTS_EXIST  0
#endif

/*
** Macro MQXCFG_MAX_FD defines maximal number of file descriptors.
** MGCT: <option type="uint32_t"/>
*/
#ifndef MQXCFG_MAX_FD
#define MQXCFG_MAX_FD 5
#endif

/*
** Macro MQX_CUSTOM_MAIN enable main function in application for lite configuration.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_CUSTOM_MAIN
#define MQX_CUSTOM_MAIN 0
#endif

/*
** Macro BSPCFG_ENABLE_IO_SUBSYSTEM enable IO subsystem.
** MGCT: <option type="bool"/>
*/
#ifndef BSPCFG_ENABLE_IO_SUBSYSTEM
#define BSPCFG_ENABLE_IO_SUBSYSTEM 1
#endif

/*
** When MQXCFG_PREALLOCATED_SYSTEM_STACKS is defined as 1, the MQX will use preallocated system stacks.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_PREALLOCATED_SYSTEM_STACKS
#define MQXCFG_PREALLOCATED_SYSTEM_STACKS 0
#endif

/*
** When MQXCFG_PREALLOCATED_READYQ is defined as 1, the MQX will use preallocated ready queues.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_PREALLOCATED_READYQ
#define MQXCFG_PREALLOCATED_READYQ 0
#endif

/*
** When MQXCFG_STATIC_MUTEX is defined as 1, the MQX will use MUTEX without dynamic allocations.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_STATIC_MUTEX
#define MQXCFG_STATIC_MUTEX 0
#endif

/*
** When MQXCFG_STATIC_LWLOG is defined as 1, the MQX will use lwlog without dynamic allocations.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_STATIC_LWLOG
#define MQXCFG_STATIC_LWLOG 0
#endif

/*
** When MQXCFG_STATIC_KLOG is defined as 1, the MQX will use klog without dynamic allocations.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_STATIC_KLOG
#define MQXCFG_STATIC_KLOG 0
#endif

/*
** When MQXCFG_STATIC_ISR_TABLE is defined as 1, the MQX will use static ISR table.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_STATIC_ISR_TABLE
#define MQXCFG_STATIC_ISR_TABLE 0
#endif

/*
** When MQXCFG_STATIC_FP_CONTEXT is defined as 1, the MQX will use preallocated  stack space for FP context.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_STATIC_FP_CONTEXT
#define MQXCFG_STATIC_FP_CONTEXT 0
#endif

/*
** When MQXCFG_PRINT_MEM_DUMP is defined as 1, the unhandled ISR will dump memory.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_PRINT_MEM_DUMP
#define MQXCFG_PRINT_MEM_DUMP 0
#endif

/*\
** When MQXCFG_PRINT_MEM_DUMP is defined as 1, the MQX will reduce MFS memory requirements.
** MGCT: <option type="bool"/>
*/
#ifndef MFSCFG_MINIMUM_FOOTPRINT
#define MFSCFG_MINIMUM_FOOTPRINT 1
#endif

/*
** Default interrupt stack size.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_INTERRUPT_STACK_SIZE
#define MQX_INTERRUPT_STACK_SIZE 1024
#endif

/*
** When MQX_HAS_EXCEPTION_HANDLER is defined as 1, the MQX will use exception handler.
** MGCT: <option type="bool"/>
*/
#ifndef MQX_HAS_EXCEPTION_HANDLER
#define MQX_HAS_EXCEPTION_HANDLER 1
#endif

/*
** Default lowest task priority.
** MGCT: <option type="bool"/>
*/
#ifndef MQXCFG_LOWEST_TASK_PRIORITY
#define MQXCFG_LOWEST_TASK_PRIORITY 16
#endif

/*
** Default first interrupt  vector.
** MGCT: <option type="uint32_t"/>
*/
#ifndef BSP_FIRST_INTERRUPT_VECTOR_USED
#define BSP_FIRST_INTERRUPT_VECTOR_USED 0
#endif

/*
** Default last interrupt  vector.
** MGCT: <option type="uint32_t"/>
*/
#ifndef BSP_LAST_INTERRUPT_VECTOR_USED
#define BSP_LAST_INTERRUPT_VECTOR_USED (PSP_MAXIMUM_INTERRUPT_VECTORS)
#endif

/** MGCT: </category> */

/*
** Any MQX component used?
*/
#define MQX_USE_COMPONENTS MQX_USE_NAME || MQX_USE_SEMAPHORES  || MQX_USE_EVENTS || MQX_USE_MUTEXES || MQX_USE_TIMER || \
	MQX_USE_LOGS || MQX_USE_SW_WATCHDOGS || MQX_USE_MESSAGES || MQX_USE_PARTITIONS ||        \
	MQX_USE_IPC || MQX_IS_MULTI_PROCESSOR || MQX_USE_LWLOGS

/*
** Check for dependencies.
*/
#if MQX_USE_EVENTS || MQX_USE_SEMAPHORES
#if !MQX_USE_NAME
#error Enable MQX_USE_NAME for Semaphores and Events
#endif
#endif

#if MQX_USE_IPC
#if !MQX_USE_PARTITIONS
#error enable MQX_USE_PARTITIONS for IPC
#endif
#endif

#ifdef MQX_CRIPPLED_EVALUATION

#if MQX_CRIPPLED_EVALUATION
#if MQX_KERNEL_LOGGING && !MQX_EXIT_ENABLED
#error Enable MQX_EXIT_ENABLED for MQX crippled evaluation
#elif !MQX_KERNEL_LOGGING && MQX_EXIT_ENABLED
#error Disable MQX_EXIT_ENABLED for MQX crippled evaluation without MQX Kernel logging
#endif
#endif

#endif

#if MQX_KERNEL_LOGGING && !MQX_USE_LWLOGS
#error Set MQX_USE_LWLOGS to 1 when using MQX_KERNEL_LOGGING
#endif

#if (MQXCFG_ALLOCATOR != MQX_ALLOCATOR_LWMEM) && (MQXCFG_ALLOCATOR != MQX_ALLOCATOR_MEM) && (MQXCFG_ALLOCATOR != MQX_ALLOCATOR_TLSF)
#if MQXCFG_ALLOCATOR
#error Set MQXCFG_ALLOCATOR to: MQX_ALLOCATOR_LWMEM, MQX_ALLOCATOR_MEM or MQX_ALLOCATOR_TLSF
#endif
#endif

#if (MQXCFG_ALLOCATOR == MQX_ALLOCATOR_LWMEM) && MQX_USE_UNCACHED_MEM
#error Set MQX_USE_UNCACHED_MEM to 0 when is MQXCFG_ALLOCATOR set to MQX_ALLOCATOR_LWMEM
#endif

#if MQXCFG_SPARSE_ISR_TABLE && (MQX_SPARSE_ISR_SHIFT < 1 || MQX_SPARSE_ISR_SHIFT > 7)
#error MQX_SPARSE_ISR_SHIFT out of range <1, 7>
#endif

#if MQX_ENABLE_USER_MODE && (MQXCFG_ALLOCATOR != MQX_ALLOCATOR_LWMEM)
#error Set MQXCFG_ALLOCATOR to MQX_ALLOCATOR_LWMEM when using MQX_ENABLE_USER_MODE
#endif

#if (MQXCFG_ENABLE_FP == 1)&&(MQXCFG_ALLOCATOR == MQX_ALLOCATOR_NONE)
	#error FP tasks need enabled allocator. Please enable allocator or disable MQXCFG_ENABLE_FP.
#endif

/* CHECK DELETED MACROS */
#if MQX_USE_MEM
	#error Macro MQX_USE_MEM is not used anymore, define MQXCFG_ALLOCATOR to MQX_ALLOCATOR_MEM for selection of MEM allocator.
#endif

#if MQX_USE_LWMEM_ALLOCATOR
	#error Macro MQX_USE_LWMEM_ALLOCATOR is not used anymore, define MQXCFG_ALLOCATOR to MQX_ALLOCATOR_LWMEM for selection of LWMEM allocator.
#endif

#if MQX_USE_LWMEM
	#error Macro MQX_USE_LWMEM is not used anymore, define MQXCFG_ALLOCATOR to MQX_ALLOCATOR_LWMEM for selection of LWMEM allocator.
#endif

#if MQX_USE_TLSF_ALLOCATOR
	#error Macro MQX_USE_TLSF_ALLOCATOR is not used anymore, define MQXCFG_ALLOCATOR to MQX_ALLOCATOR_TLSF for selection of TLSF allocator.
#endif

#endif
