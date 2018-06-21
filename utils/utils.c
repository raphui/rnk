#include <utils.h>

void writel(unsigned int reg, unsigned int val)
{
	unsigned int *p = (unsigned int *)reg;

	asm volatile("str %[val], [%[reg]]"
			: : [reg]"r"(p), [val]"r"(val));
}


unsigned int readl(unsigned int reg)
{
	unsigned int *p = (unsigned int *)reg;
	unsigned int val;

	asm volatile("ldr %[val], [%[reg]]"
			: [val]"=r"(val) : [reg]"r"(p));

	return val;
}

unsigned int next_power_of_2(unsigned int n)
{
	n--;
	n |= n >> 1;
	n |= n >> 2;
	n |= n >> 4;
	n |= n >> 8;
	n |= n >> 16;
	n++;
	return n;
}
