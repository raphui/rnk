#include "pit.h"

void pit_init(unsigned int period, unsigned int pit_frequency)
{
	unsigned int tmp = 0;
	writel(AT91C_BASE_PITC + PITC_PIMR, (period * pit_frequency + 8));
	
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp |= AT91C_PITC_PITEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}

void pit_enable(void)
{
	writel(AT91C_BASE_PITC + PITC_PIMR, AT91C_PITC_PITEN);
}

void pit_enable_it(void)
{
	writel(AT91C_BASE_PITC + PITC_PIMR, AT91C_PITC_PITIEN);
}

void pit_disable_it(void)
{
	unsigned int tmp = 0;
	
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp &= ~AT91C_PITC_PITIEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}
