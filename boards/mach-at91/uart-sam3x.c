#include <board.h>
#include <utils.h>
#include <uart.h>

static void uart_register_functions( void )
{

    uart_ops.init = &uart_init;
	uart_ops.print = &uart_print;
	uart_ops.printl = &uart_printl;	

} 

void sam3x_uart_init( void )
{
	unsigned int cd = 0;

	/* Reset and disable receiver and transmitter */
	writel( UART_BASE + U_CR , U_CR_RSTRX
								| U_CR_RSTTX
								| U_CR_TXDIS
								| U_CR_RXDIS );

	/* Configure baudrate */

	/* Register io functions */
    uart_register_functions();
}

void sam3x_uart_print( unsigned char byte )
{

	/* Polling right now */

	while( !( readl( UART_BASE + U_SR ) & U_SR_TXRDY ) )
		;

	writel( UART_BASE + U_THR , byte );
}


int sam3x_uart_printl( const char *string )
{
	while( *string )
	{
		uart_print( *string++ );
	}

	return 0;
}
