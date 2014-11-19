#include "aic.h"

void aic_register_handler(unsigned int source, unsigned int mode, void (*handler)(void))
{
	unsigned int source_shift = source * 0x4;

	/* Disable the interrupt */
	writel(AT91C_BASE_AIC + AIC_IDCR, (1 << source));

	/* Configure mode and handler */
	writel(AT91C_BASE_AIC + (AIC_SMR + source_shift), mode);
	writel(AT91C_BASE_AIC + (AIC_SVR + source_shift), (unsigned int)handler);

	/* Clear the interrupt */
	writel(AT91C_BASE_AIC + AIC_ICCR, (1 << source));
}

void aic_enable_it(unsigned int source)
{
	writel(AT91C_BASE_AIC + AIC_IECR, (1 << source));
}

void aic_disable_it(unsigned int source)
{
	writel(AT91C_BASE_AIC + AIC_IDCR, (1 << source));
}
