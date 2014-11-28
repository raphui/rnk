#include <board.h>
#include <pio.h>

void pio_set_output(unsigned int port, unsigned int mask)
{
	pio_ops.set_output(port, mask);
}

void pio_set_input(unsigned int port, unsigned int mask)
{
	pio_ops.set_input(port, mask);
}

void pio_set_value(unsigned int port, unsigned int mask)
{
	pio_ops.set_value(port, mask);
}

void pio_clear_value(unsigned int port, unsigned int mask)
{
	pio_ops.clear_value(port, mask);
}
