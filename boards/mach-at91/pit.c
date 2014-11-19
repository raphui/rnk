#include "pit.h"

void pit_init(unsigned int period, unsigned int pit_frequency)
{
	unsigned int tmp = 0;
	writel(base_pit + PITC_PIMR, (period * pit_frequency + 8));
	
	tmp = readl(base_pit + PITC_IMR);
	tmp |= AT91C_PITC_PITEN
	writel(base_pit + PITC_PIMR, tmp);
}

void pit_enable(void)
{
	writel(base_pit + PITC_PIMR, AT91C_PITC_PITEN);
}

void pit_enable_it(void)
{
	writel(base_pit + PITC_PIMR, AT91C_PITC_PITIEN);
}

void pit_disable_it(void)
{
	unsigned int tmp = 0;
	
	tmp = readl(base_pit + PITC_PIMR);
	tmp &= ~AT91C_PITC_PITIEN;
	writel(base_pit + PITC_PIMR, tmp);
}
