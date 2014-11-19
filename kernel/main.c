#include <stdint.h>
#include <board.h>
#include <uart.h>
#include <io.h>
#include <scheduler.h>

extern int get_value_cp(void);

int main(void)
{

//	low_level_init();

	uart_init();

//	while(1)
		printk("\r\nHello World from RNK ( Raphio new kernel )\r\n");

//	printk("CP15: %x\r\n" , get_value_cp());

	while(1)
		;

	return 0; //Never reach
}
