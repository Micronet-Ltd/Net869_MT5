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
*   This file contains the RTCS shell.
*
*
*END************************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <mqx.h>
#include <mqx_inc.h> /* internal kernel structures */
#include "shell.h"
#include "sh_prv.h"

#include "fs/fs_supp.h"

static int shell_strtolower(char *arg) {
	char *p = arg;

	if (p) {
		while (*p) {
			*p = tolower((unsigned char)*p);
			p++;
		}
	}

	return 0;
}
/*FUNCTION*-------------------------------------------------------------
*
*  Function Name :  Shell
*  Returned Value:  none
*  Comments  :  The RTCS shell
*
*END*-----------------------------------------------------------------*/

int32_t Shell
   (
	  const SHELL_COMMAND_STRUCT   shell_commands[],
	  char            *start_file
   )
{ /* Body */
   SHELL_CONTEXT_PTR    shell_ptr;
   // int32_t               return_code;
   uint32_t              i;
   register KERNEL_DATA_STRUCT_PTR  kernel_data;
   register TD_STRUCT_PTR           active_ptr;
   FILE * this_stdin;
   _GET_KERNEL_DATA(kernel_data);

   shell_ptr = _mem_alloc_zero( sizeof( SHELL_CONTEXT ));
   if (shell_ptr == NULL)  {
	  return SHELL_EXIT_ERROR;
   }
   _mem_set_type(shell_ptr, MEM_TYPE_SHELL_CONTEXT);

   active_ptr = kernel_data->ACTIVE_PTR;
   /* only for Shell via "telnetio:" */
   if(active_ptr->STDOUT_STREAM)
   {
	 shell_ptr->STDOUT = active_ptr->STDOUT_STREAM;
   }
   else
   {
	 shell_ptr->STDOUT = stdout;
   }
   shell_ptr->STDERR = shell_ptr->STDOUT;

   fprintf(shell_ptr->STDOUT, "\nShell (build: %s)\n", __DATE__);
   fprintf(shell_ptr->STDOUT, "Copyright (c) 2013-2014 Freescale Semiconductor;\n");

   shell_ptr->COMMAND_LIST_PTR=(SHELL_COMMAND_PTR)shell_commands;

   if (start_file != NULL)  {
	  if (start_file[0] == '\'') {
		 strncpy(  shell_ptr->CMD_LINE, &start_file[1],sizeof(shell_ptr->CMD_LINE  ));
	  } else {
		 shell_ptr->COMMAND_FP = fopen(start_file, "r");
	  }
   }

   /* only for Shell via "telnetio:" */
   if(active_ptr->STDIN_STREAM)
   {
	 this_stdin = active_ptr->STDIN_STREAM;
	 shell_ptr->STDIN = active_ptr->STDIN_STREAM;
   }
   else
   {
	 this_stdin = stdin;
	 shell_ptr->STDIN = stdin;
   }
   if (!shell_ptr->COMMAND_FP)  {
	 shell_ptr->COMMAND_FP = this_stdin;
   }

#if SHELLCFG_USES_MFS
   // initialize current fs ptr, fs name and path
   Shell_set_current_filesystem((void *)shell_ptr, -1);
#endif //SHELLCFG_USES_MFS

   if (shell_ptr->CMD_LINE){
	  fprintf(shell_ptr->STDOUT, "shell> %s\n",shell_ptr->CMD_LINE);
   } else {
	  fprintf(shell_ptr->STDOUT, "shell> ");
   }
   fflush(shell_ptr->STDOUT);

   while (!shell_ptr->EXIT) {

	  if ((!shell_ptr->EXIT) && (shell_ptr->CMD_LINE[0] != '\0'))  {

		 if (shell_ptr->COMMAND_FP != this_stdin)  {
			fprintf(shell_ptr->STDOUT, "%s\n", shell_ptr->CMD_LINE);
		 }

		 if (shell_ptr->CMD_LINE[0] != '#') {

			if (strcmp(shell_ptr->CMD_LINE, "!") == 0)  {
			   strncpy(shell_ptr->CMD_LINE,shell_ptr->HISTORY,sizeof(shell_ptr->CMD_LINE));
			} else if (strcmp(shell_ptr->CMD_LINE, "\340H") == 0)  {
			   strncpy(shell_ptr->CMD_LINE,shell_ptr->HISTORY,sizeof(shell_ptr->CMD_LINE));
			} else  {
			   strncpy(shell_ptr->HISTORY,shell_ptr->CMD_LINE,sizeof(shell_ptr->HISTORY));
			}

			shell_ptr->ARGC = Shell_parse_command_line(shell_ptr->CMD_LINE, shell_ptr->ARGV );

			if (shell_ptr->ARGC > 0) {

				shell_strtolower(shell_ptr->ARGV[0]);
			   for (i=0;shell_commands[i].COMMAND != NULL;i++)  {
				  if (strcmp(shell_ptr->ARGV[0], shell_commands[i].COMMAND) == 0)  {
					 /* return_code = */ (*shell_commands[i].SHELL_FUNC)(shell_ptr->ARGC, shell_ptr->ARGV);
					 break;
				  }
			   }

			   if (shell_commands[i].COMMAND == NULL)  {
				 fprintf(shell_ptr->STDOUT, "Invalid command.  Type 'help' for a list of commands.\n");
			   }
			}
		 }
	  }

	  if (!shell_ptr->EXIT) {
		 fprintf(shell_ptr->STDOUT, "shell> ");
		 fflush(shell_ptr->STDOUT);

		 do {
			if (!fgets(shell_ptr->CMD_LINE, sizeof(shell_ptr->CMD_LINE  ), shell_ptr->COMMAND_FP)) {
			   if (shell_ptr->COMMAND_FP != this_stdin)  {
				  fclose(shell_ptr->COMMAND_FP);
				  shell_ptr->COMMAND_FP = this_stdin;
				  shell_ptr->HISTORY[0]=0;
				  shell_ptr->CMD_LINE[0]=0;
				  fprintf(shell_ptr->STDOUT, "\n");
			   } else  {
				  shell_ptr->EXIT=TRUE;
				  break;
			   }
			}
		 } while ((shell_ptr->CMD_LINE[0] == '\0') && (shell_ptr->COMMAND_FP != this_stdin)) ;
	  }
   }

   fprintf(shell_ptr->STDOUT, "Terminating shell.\n");
   _mem_free(shell_ptr);
   return SHELL_EXIT_SUCCESS;
} /* Endbody */


/* EOF */
