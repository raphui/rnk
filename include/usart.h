#ifndef USART_H
#define USART_H

#include <io.h>

void usart_init( void );
void usart_print( unsigned char byte );
int usart_printl( const char *string );

#endif /* USART_H */
