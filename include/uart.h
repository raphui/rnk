#ifndef UART_H
#define UART_H

#include <io.h>

void uart_init( void );
void uart_print( unsigned char byte );
int uart_printl( const char *string );

#endif /* UART_H */
