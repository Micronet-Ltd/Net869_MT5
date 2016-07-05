/*
 * Uart_debugTerminal.c
 *
 *  Created on: Dec 16, 2015
 *      Author: ruslans
 */

#include "print_prv.h"
//#include "print_scan.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>


#include "Uart_debugTerminal.h"

#define MAX_DEBUG_OUT_STR_SIZE  0x80
#define IO_MAXLINE  20

#if defined(__cplusplus)
extern "C"
{
#endif

#if BSPCFG_ENABLE_IO_SUBSYSTEM

int UART_debug_printf( const char  *fmt_s, ... ) {
    return 0;

    {
        va_list  ap;
        int  result;
        char tmpBuff[MAX_DEBUG_OUT_STR_SIZE];

        char *str_ptr = tmpBuff;

        va_start(ap, fmt_s);
        result = _io_doprint((FILE *)((void *)&str_ptr), _io_sputc, MAX_DEBUG_OUT_STR_SIZE, (char *)fmt_s, ap);
        *str_ptr = '\0';
        va_end(ap);

        str_ptr = tmpBuff;

        result = write(0, (const void *)str_ptr, strlen(str_ptr));
        
		return result;
	}
}

static int UART_debug_putc( int ch ) {
    const unsigned char c = (unsigned char)ch;

    return write(0, (const void *)&c, sizeof(c));
}

int UART_debug_putchar( int ch ) {
    return UART_debug_putc(ch);
}

int UART_debug_scanf( const char  *fmt_ptr, ... ) {
    char    temp_buf[IO_MAXLINE];
    va_list ap;
    uint32_t i;
    char result;

    va_start(ap, fmt_ptr);
    temp_buf[0] = '\0';

    for ( i = 0; i < IO_MAXLINE; i++ ) {
        temp_buf[i] = result = UART_debug_getchar();

        if ( (result == '\r') || (result == '\n') ) {
            /* End of Line */
            if ( i == 0 ) {
                i = (uint32_t)-1;
            } else {
                break;
            }
        }

        temp_buf[i + 1] = '\0';
    }

    result = scan_prv(temp_buf, (char *)fmt_ptr, ap);
    va_end(ap);

    return result;
}

int UART_debug_getchar( void ) {
    unsigned char c;

    if ( -1 ==  read(0, (void *)&c, sizeof(c)) ) return -1;

    return c;
}

#endif //~BSPCFG_ENABLE_IO_SUBSYSTEM

#if defined(__cplusplus)
}
#endif

