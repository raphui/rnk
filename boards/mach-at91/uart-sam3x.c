#include <board.h>
#include <utils.h>
#include <uart.h>

static void uart_register_functions( void )
{

        io_op.write = &uart_printl;        

} 

void uart_init( void )
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

void uart_print( unsigned char byte )
{

	/* Polling right now */

	while( !( readl( UART_BASE + U_SR ) & US_SR_TXRDY ) )
		;

	writel( UART_BASE + U_THR , byte );
}


int uart_printl( const char *string )
{
	while( *string )
	{
		uart_print( *string++ );
	}

	return 0;
}
