#include <board.h>
#include <utils.h>
#include <uart.h>

static void uart_register_functions( void )
{

	io_functions.write = &uart_printl;	

}

void uart_init( void )
{
	/* Disable UART */
	writel( UART0_CR , 0x00000000 );
	
	/*Disable pull up/down for all GPIO */
	writel( GPIO_GPPUD , 0x00000000 );
	delay( 150 );

	/* Disable pull up/down for GPIO 14 and 15 */
	writel( GPIO_GPPUDCLK0 , ( 1 << 14 ) | ( 1 << 15 ) );
	delay( 150 );

	/* Enable */
	writel( GPIO_GPPUDCLK0 , 0x00000000 );

	/* Clear pending interrupt */
	writel( UART0_ICR , 0x7F );

	/* Set divider */
	writel( UART0_IBRD , 1 );
	writel( UART0_FBRD , 40 );

	/* Enable FIFO & 8 bit data transmission */
	writel( UART0_LCRH , ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) );
	
	/* Mask all interrupts */
	writel( UART0_IMSC , ( 1 << 1 ) | ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 )
						| ( 1 << 8 ) | ( 1 << 9 ) | ( 1 << 10 ) );

	/* Enable UART0 */
	writel( UART0_CR , ( 1 << 0 ) | ( 1 << 8 ) | ( 1 << 9 ) );

	/* Register io functions */
	uart_register_functions();
}

void uart_print( unsigned char byte )
{
	while( 1 )
	{
		if( !( readl( UART0_FR ) & ( 1 << 5 ) ) )
			break;
	}
	
	writel( UART0_DR , byte );
}


int uart_printl( const char *string )
{
	while( *string )
	{
		uart_print( *string++ );
	}

	return 0;
}
