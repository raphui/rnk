#include <board.h>
#include <utils.h>
#include <uart.h>

void uart_init(void)
{
	/* Register io functions */
	uart_ops.init();
}

void uart_print(unsigned char byte)
{
	uart_ops.print(byte);
}


int uart_printl(const char *string)
{
	return uart_ops.printl(string);
}

struct io_operations io_op = {
	.write = uart_printl,
};
