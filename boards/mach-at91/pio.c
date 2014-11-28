#include <board.h>
#include <utils.h>

static void pio_set_output(unsigned int port, unsigned int mask)
{
	writel(port + PIO_OER, mask);
	writel(port + PIO_PER, mask);
}

static void pio_set_input(unsigned int port, unsigned int mask)
{
	writel(port + PIO_ODR, mask);
	writel(port + PIO_PER, mask);
}

static void pio_set_value(unsigned int port, unsigned int mask)
{
	writel(port + PIO_SODR, mask);
}

static void pio_clear_value(unsigned int port, unsigned int mask)
{
	writel(port + PIO_CODR, mask);
}

struct pio_operations pio_ops = {
	.set_output = &pio_set_output,
	.set_input = &pio_set_input,
	.set_value = &pio_set_value,
	.clear_value = &pio_clear_value,
};
