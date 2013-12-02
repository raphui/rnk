#ifndef UART_H
#define UART_H

void uart_init( void );
void uart_print( unsigned char byte );
void uart_printl( const char *string );

#endif /* UART_H */
