#include <stdint.h>
#include <board.h>
#include <uart.h>
#include <io.h>
#include <scheduler.h>
#include <task.h>

void first_task(void)
{
	printk("\r\nTASK A\r\n");
	while (1) {
		printk("cool\r\n");
	}
}

void second_task(void)
{
	printk("\r\nTASK B\r\n");
	while (1)
		;
}

int main(void)
{
	uart_init();

	printk("\r\nHello World from RNK ( Raphio new kernel )\r\n");

	add_task(&first_task, 1);
	add_task(&second_task, 1);

	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
