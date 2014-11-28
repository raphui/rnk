#include <stdint.h>
#include <board.h>
#include <uart.h>
#include <io.h>
#include <scheduler.h>
#include <task.h>
#include <pio.h>

int toogle = 0;
void first_task(void)
{

	printk("starting task A\r\n");
//	pio_set_output(AT91C_BASE_PIOA, (1 << 0));
//			pio_set_value(AT91C_BASE_PIOA, (1 << 0));
	while (1) {
		if (toogle) {
			pio_clear_value(AT91C_BASE_PIOA, (1 << 0));
		} else {
			pio_set_value(AT91C_BASE_PIOA, (1 << 0));
		}
//
		toogle != toogle;
//		printk("A");
	}
}

void second_task(void)
{
//	int toogle = 0;
	printk("starting task B\r\n");
	while (1) {
		if (toogle) {
			pio_clear_value(AT91C_BASE_PIOA, (1 << 2));
		} else {
			pio_set_value(AT91C_BASE_PIOA, (1 << 2));
		}
		toogle != toogle;
//		printk("B");
	}
}

int main(void)
{
	uart_init();
	pio_set_output(AT91C_BASE_PIOA, (1 << 0) | (1 << 2));
	pio_set_value(AT91C_BASE_PIOA, (1 << 0) | (1 << 2));
	pio_clear_value(AT91C_BASE_PIOA, (1 << 0) | (1 << 2));

	printk("Hello World from RNK ( Raphio new kernel )\r\n");

	add_task(&first_task, 1);
	add_task(&second_task, 1);

	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
