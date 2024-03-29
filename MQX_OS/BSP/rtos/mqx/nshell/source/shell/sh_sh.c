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
#include <stdio.h>
#include <nio.h>
#include <fcntl.h>
#include <fs_supp.h>
#include "shell.h"
#include "sh_prv.h"

#if SHELLCFG_USES_MFS
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_sh
* Returned Value   :  int32_t error code
* Comments  :  Reads from a file .
*
*END*---------------------------------------------------------------------*/

int32_t  Shell_sh(int32_t argc, char *argv[] )
{ /* Body */
   bool              print_usage, shorthelp = FALSE;
   int32_t               return_code = SHELL_EXIT_SUCCESS, error;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   FILE *f;
   int                  fs;
   char                 abs_path[SHELL_MAX_FILELEN];

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
	  if (argc != 2) {
		 fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
		 return_code = SHELL_EXIT_ERROR;
		 print_usage=TRUE;
	  } else  {
		fs = Shell_get_current_filesystem(argv);

		/* check if filesystem is mounted */
		if (0 > fs)  {
		   fprintf(shell_ptr->STDOUT, "Error, file system not mounted\n");
		   return_code = SHELL_EXIT_ERROR;
		} else {
		  error = _io_rel2abs(abs_path,shell_ptr->CURRENT_DIR,(char *) argv[1],sizeof(abs_path),shell_ptr->CURRENT_DEVICE_NAME);
		  if (!error)
		  {
			f = fopen(abs_path, "r");
			if (NULL != f)  {
			   shell_ptr = Shell_get_context( argv );
			   if (shell_ptr->COMMAND_FP != stdin)  {
				 fclose(shell_ptr->COMMAND_FP);
			   }
			   shell_ptr->COMMAND_FP = f;
			} else  {
			   fprintf(shell_ptr->STDOUT, "Error, unable to open file %s.\n", argv[1] );
			   return_code = SHELL_EXIT_ERROR;
			}
		 } else  {
			fprintf(shell_ptr->STDOUT, "Error, unable to open file %s.\n", argv[1] );
			return_code = SHELL_EXIT_ERROR;
		 }
		}
	  }
   }

   if (print_usage)  {
	  if (shorthelp)  {
		 fprintf(shell_ptr->STDOUT, "%s <filename>\n", argv[0]);
	  } else  {
		 fprintf(shell_ptr->STDOUT, "Usage: %s <filename>\n", argv[0]);
		 fprintf(shell_ptr->STDOUT, "   <filename>   = filename to execute\n");
	  }
   }
   return return_code;


} /* Endbody */
#endif //SHELLCFG_USES_MFS
/* EOF*/
