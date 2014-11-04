#include <board.h>
#include <utils.h>

// Startup time of main oscillator (in number of slow clock ticks).
#define BOARD_OSCOUNT           (AT91C_CKGR_OSCOUNT & (0x40 << 8))

// USB PLL divisor value to obtain a 48MHz clock.
#define BOARD_USBDIV            AT91C_CKGR_USBDIV_1

// PLL frequency range.
#define BOARD_CKGR_PLL          AT91C_CKGR_OUT_0

// PLL startup time (in number of slow clock ticks).
#define BOARD_PLLCOUNT          (16 << 8)

// PLL MUL value.
#define BOARD_MUL               (AT91C_CKGR_MUL & (72 << 16))

// PLL DIV value.
#define BOARD_DIV               (AT91C_CKGR_DIV & 14)

// Master clock prescaler value.
#define BOARD_PRESCALER         AT91C_PMC_PRES_CLK_2

void low_level_init(void)
{
	unsigned int base_pmc = 0xFFFFFC00;
	unsigned int base_aic = 0xFFFFF000;
	unsigned int tmp = 0;

	/* Initialise the main oscillator */
	writel(base_pmc + AT91C_PMC_MOR, BOARD_OSCOUNT | AT91C_CKGR_MOSCEN);
	while ((readl(base_pmc + AT91C_PMC_SR) & AT91C_PMC_MOSCS))
		;

	/* Initliase the PLL at 96MHz and USB clock to 48MHz */
	writel(base_pmc + AT91C_PMC_PLLR, BOARD_USBDIV | BOARD_CKGR_PLL | BOARD_PLLCOUNT | BOARD_MUL | BOARD_DIV);
	while ((readl(base_pmc + AT91C_PMC_SR) & AT91C_PMC_LOCK))
		;

	/* Wait for the master clock if it was already initialised */
	while ((readl(base_pmc + AT91C_PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Switch to slow clock + prescaler */
	writel(base_pmc + AT91C_PMC_MCKR, BOARD_PRESCALER);
	while ((readl(base_pmc + AT91C_PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Switch to fast clock + prescaler */
	tmp = readl(base_pmc + AT91C_PMC_MCKR);
	writel(base_pmc + AT91C_PMC_MCKR, tmp | AT91C_PMC_CSS_PLL_CLK);
	while ((readl(base_pmc + AT91C_PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Disable all interrupts */
	writel(base_aic + AT91C_AIC_IDCR, 0xFFFFFFFF);

	/* Clear pending interrupts */
	writel(base_aic + AT91C_AIC_ICCR, 0xFFFFFFFF);
}
