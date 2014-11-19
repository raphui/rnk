#include <stdint.h>
#include <board.h>
#include <uart.h>
#include <io.h>
#include <scheduler.h>

extern int get_value_cp(void);

int main(void)
{
	uart_init();

	printk("\r\nHello World from RNK ( Raphio new kernel )\r\n");

	while(1)
		;

	return 0; //Never reach
}
