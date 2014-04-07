#ifndef BOARD_SAM7S_H
#define BOARD_SAM7S_H

#include <sam7s.h>
#include <sam7s-reg.h>

struct uart_operations
{
	void ( *init )( void );
	void ( *print )( unsigned char byte );
	int ( *printl )( const char *string );

};

struct uart_operations uart_ops;

#endif /* BOARD_SAM7S_H */
