#include <board.h>
#include <utils.h>

/*

PIV 4808
MCK = 48000000 = 48MHz
MCK/16 = 3000000 = 3 MHz
1/(MCK/16) = 0.000000333 = 3,33us
One interruption every => 4808 * 3,33 = 16010.64us = 16ms

*/

static void pit_init(unsigned int period, unsigned int pit_frequency)
{
	unsigned int tmp = 0;
	writel(AT91C_BASE_PITC + PITC_PIMR, (period * pit_frequency + 8));
	
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp |= AT91C_PITC_PITEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}

static void pit_enable(void)
{
	unsigned int tmp = 0;
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp |= AT91C_PITC_PITEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}

static void pit_enable_it(void)
{
	unsigned int tmp = 0;
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp |= AT91C_PITC_PITIEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}

static void pit_disable_it(void)
{
	unsigned int tmp = 0;
	
	tmp = readl(AT91C_BASE_PITC + PITC_PIMR);
	tmp &= ~AT91C_PITC_PITIEN;
	writel(AT91C_BASE_PITC + PITC_PIMR, tmp);
}


void pit_read_pivr(void)
{
	if (readl(AT91C_BASE_PITC + PITC_PISR) & AT91C_PITC_PITS) {
		readl(AT91C_BASE_PITC + PITC_PIVR);
	}
}

struct pit_operations pit_ops = {
	.init = &pit_init,
	.enable = &pit_enable,
	.enable_it = &pit_enable_it,
	.disable_it = &pit_disable_it,
	.read_pivr = &pit_read_pivr,
};
