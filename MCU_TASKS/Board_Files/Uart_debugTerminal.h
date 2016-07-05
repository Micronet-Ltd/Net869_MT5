/*
 * Uart_debugTerminal.h
 *
 *  Created on: Dec 16, 2015
 *      Author: ruslans
 */

#ifndef BOARD_FILES_UART_DEBUGTERMINAL_H_
#define BOARD_FILES_UART_DEBUGTERMINAL_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifdef BSPCFG_ENABLE_IO_SUBSYSTEM
#define MIC_DEBUG_UART_PRINTF           UART_debug_printf
#define MIC_DEBUG_UART_SCANF            UART_debug_scanf
#define MIC_DEBUG_UART_PUTCHAR          UART_debug_putchar
#define MIC_DEBUG_UART_GETCHAR          UART_debug_getchar
#else
#define MIC_DEBUG_UART_PRINTF
#define MIC_DEBUG_UART_SCANF
#define MIC_DEBUG_UART_PUTCHAR
#define MIC_DEBUG_UART_GETCHAR
#endif

#if defined(__cplusplus)
extern "C"
{
#endif

#ifdef BSPCFG_ENABLE_IO_SUBSYSTEM
extern int UART_debug_printf( const char  *fmt_s, ... );
extern int UART_debug_putchar( int ch );
extern int UART_debug_scanf( const char  *fmt_ptr, ... );
extern int UART_debug_getchar( void );

#endif //~BSPCFG_ENABLE_IO_SUBSYSTEM

#if defined(__cplusplus)
}
#endif

#endif /* BOARD_FILES_UART_DEBUGTERMINAL_H_ */
