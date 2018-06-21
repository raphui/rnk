#include <pit.h>
#include <board.h>

void pit_init(unsigned int period, unsigned int pit_frequency)
{
	pit_ops.init(period, pit_frequency);
}

void pit_enable(void)
{
	pit_ops.enable();
}

void pit_enable_it(void)
{
	pit_ops.enable_it();
}

void pit_disable_it(void)
{
	pit_ops.disable_it();
}
