#include <board.h>
#include <utils.h>

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
	writel(AT91C_BASE_PITC + PITC_PIMR, AT91C_PITC_PITEN);
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

struct pit_operations pit_ops = {
	.init = &pit_init,
	.enable = &pit_enable,
	.enable_it = &pit_enable_it,
	.disable_it = &pit_disable_it,
};
