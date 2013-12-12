#include <board.h>
#include <utils.h>
#include <usart.h>

static void usart_register_functions( void )
{

        io_op.write = &usart_printl;        

} 

void usart_init( void )
{
	unsigned int cd = 0;

	/* Reset and disable receiver and transmitter */
	writel( USART3_BASE + US_CR , US_CR_RSTRX
								| US_CR_RSTTX
								| US_CR_TXDIS
								| US_CR_RXDIS );

	/* Configure baudrate */

	/* Register io functions */
    usart_register_functions();
}

void usart_print( unsigned char byte )
{
	while( 1 )
	{
		if( !( readl( UART0_FR ) & ( 1 << 5 ) ) )
			break;
	}
	
	writel( UART0_DR , byte );
}


int usart_printl( const char *string )
{
	while( *string )
	{
		usart_print( *string++ );
	}

	return 0;
}
