#include <board.h>
#include <utils.h>
#include <uart.h>

static void uart_register_functions( void )
{

    uart_ops.init = &uart_init;
	uart_ops.print = &uart_print;
	uart_ops.printl = &uart_printl;	

} 

void sam7s_uart_init( void )
{

}

void sam7s_uart_print( unsigned char byte )
{

}


int sam7s_uart_printl( const char *string )
{
	return 0;
}
