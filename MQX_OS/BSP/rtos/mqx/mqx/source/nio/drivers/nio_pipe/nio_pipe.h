/*HEADER**********************************************************************
*
* Copyright 2013 Freescale Semiconductor, Inc.
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
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*
*END************************************************************************/
#ifndef _NIO_PIPE_H_
#define _NIO_PIPE_H_

#include <stdint.h>
#include "nio.h"

extern const NIO_DEV_FN_STRUCT nio_pipe_dev_fn;

typedef struct {
	uint32_t LEN;
} NIO_PIPE_INIT_DATA_STRUCT;

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

#endif // _NPIPE_H_
