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
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the source for an MFS shell function.
*
*
*END************************************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <mqx.h>
#include <shell.h>
#include <sh_prv.h>

#include "nio.h"
#include "fcntl.h"
#include "fs_supp.h"

#if SHELLCFG_USES_MFS
#include <mfs.h>

static const char * const BYTE_SIZES[] = { "bytes", "kB", "MB", "GB", "TB" };

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_format
* Returned Value   :  int32_t error code
* Comments  :  mount a filesystem on a device.
*
* Usage:  format <drive:> [<volume label>]
*
*END*---------------------------------------------------------------------*/

int32_t  Shell_format(int32_t argc, char *argv[] )
{ /* Body */
   bool                    print_usage, shorthelp = FALSE;
   int32_t                     return_code = SHELL_EXIT_SUCCESS;
   int32_t                     error_code = -1;
   int fd = -1;
   char                   *volume_ptr[SFILENAME_SIZE];
   uint64_t                    big_number;
   uint32_t                    small_number;
   SHELL_CONTEXT_PTR shell_ptr = Shell_get_context(argv);

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
	  if (argc < 2 || argc >  3) {
		 fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
		 return_code = SHELL_EXIT_ERROR;
		 print_usage=TRUE;
	  } else  {
		 if (argc == 3)  {
			memcpy (volume_ptr, argv[2], SFILENAME_SIZE);
		 } else  {
			volume_ptr[0] = 0;
		 }
		 fd = _io_get_fs_by_name(argv[1]);
		 if (0 > fd)  {
			fprintf(shell_ptr->STDOUT, "Error, file system %s not found\n", argv[1] );
			return_code = SHELL_EXIT_ERROR;
		 } else {
			fprintf(shell_ptr->STDOUT, "\nFormatting...\n");
			error_code = ioctl(fd, IO_IOCTL_DEFAULT_FORMAT,  NULL);
			if (0 <= error_code && (*volume_ptr)) {
			   error_code = ioctl(fd, IO_IOCTL_SET_VOLUME,  (void*)volume_ptr);
			}

			if (0 > error_code) {
			   fprintf(shell_ptr->STDOUT, "Error while formatting: 0x%x\n", error_code);
			} else  {
			   /* print disk information */
			   error_code = ioctl(fd, IO_IOCTL_GET_VOLUME, (uint32_t*)volume_ptr);
			   fprintf(shell_ptr->STDOUT, "Done. Volume name is %s\n", volume_ptr);
			   error_code = ioctl(fd, IO_IOCTL_FREE_SPACE, &big_number);
			   if (0 <= error_code) {
				   for (small_number = 0; big_number > 128 * 1024; big_number >>= 10) small_number++;
				   fprintf(shell_ptr->STDOUT, "Free disk space: %lu %s\n", (uint32_t)big_number, BYTE_SIZES[small_number]);
			   }
			   else {
				   fprintf(shell_ptr->STDOUT, "Error while retrieving free space.\n");
			   }
			}
		 }
	  }
   }

   if (print_usage)  {
	  if (shorthelp)  {
		 fprintf(shell_ptr->STDOUT, "%s <drive:> [<volume label>]\n", argv[0]);
	  } else  {
		 fprintf(shell_ptr->STDOUT, "Usage: %s <drive:> [<volume label>]\n", argv[0]);
		 fprintf(shell_ptr->STDOUT, "   <drive:> = specifies the drive name (followed by a colon)\n");
		 fprintf(shell_ptr->STDOUT, "   <volume label>  = specifies the volume label\n");
	  }
   }
   return return_code;
}

#endif // SHELLCFG_USES_MFS
