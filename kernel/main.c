#include <stdint.h>
#include <uart.h>

int main( void )
{

	uart_init();
	
	uart_printl("\r\nHello World from RNK ( Raphio new kernel )\r\n");
	
	while( 1 )
		;

	return 0; //Never reach
}
