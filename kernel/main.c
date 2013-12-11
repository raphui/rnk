#include <stdint.h>
#include <uart.h>
#include <io.h>

extern int get_value_cp( void );

int main( void )
{

	uart_init();
	
	printk("\r\nHello World from RNK ( Raphio new kernel )\r\n");

	printk("CP15: %x\r\n" , get_value_cp() );

	while( 1 )
		;

	return 0; //Never reach
}