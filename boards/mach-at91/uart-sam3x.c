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
	unsigned int mode = 0x0; /* Normal mode */
	
	/* Enable clock */
	writel( PMC_BASE + PMC_PCER0 , ( 1 << 8 ) );

	/* Reset and disable receiver and transmitter */
	writel( UART_BASE + U_CR , U_CR_RSTRX
								| U_CR_RSTTX
								| U_CR_TXDIS
								| U_CR_RXDIS );
	/* Configure mode */
	writel( UART_BASE + U_MR , mode );

	/* Configure baudrate */
	writel( UART_BASE + U_BRGR , ( BOARD_MCK / UART_BAUDRATE ) / 16 );

	/* Disable PDC channel */
//	writel( UART_BASE + U_ )

	/* Enable receiver and transmitter */
	writel( UART_BASE + U_CR , U_CR_RXEN | U_CR_TXEN );

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
