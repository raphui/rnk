#include <board.h>
#include <utils.h>
#include <scheduler.h>
#include "aic.h"
#include "pit.h"

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

/// Master clock frequency in Hz.
#define BOARD_MCK               48000000

// PIT period value in µseconds.
#define PIT_PERIOD          1000

void low_level_init(void)
{
	unsigned int tmp = 0;
	unsigned int i = 0;

	/* Initialise the main oscillator */
	writel(AT91C_BASE_CKGR + CKGR_MOR, BOARD_OSCOUNT | AT91C_CKGR_MOSCEN);
	while (!(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_MOSCS))
		;

	/* Initliase the PLL at 96MHz and USB clock to 48MHz */
	writel(AT91C_BASE_CKGR + CKGR_PLLR, BOARD_USBDIV | BOARD_CKGR_PLL | BOARD_PLLCOUNT | BOARD_MUL | BOARD_DIV);
	while (!(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_LOCK))
		;

	/* Wait for the master clock if it was already initialised */
	while (!(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Switch to slow clock + prescaler */
	writel(AT91C_BASE_PMC + PMC_MCKR, BOARD_PRESCALER);
	while (!(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Switch to fast clock + prescaler */
	tmp = readl(AT91C_BASE_PMC + PMC_MCKR);
	writel(AT91C_BASE_PMC + PMC_MCKR, tmp | AT91C_PMC_CSS_PLL_CLK);
	while (!(readl(AT91C_BASE_PMC + PMC_SR) & AT91C_PMC_MCKRDY))
		;

	/* Disable all interrupts */
	writel(AT91C_BASE_AIC + AIC_IDCR, 0xFFFFFFFF);

	/* Clear pending interrupts */
	writel(AT91C_BASE_AIC + AIC_ICCR, 0xFFFFFFFF);

	/* Initialise AIC */

	/* Unstack nested interrupts */
	for (i = 0; i < 8; i++) {
		writel(AT91C_BASE_AIC + AIC_EOICR, 0x0);
	}

	aic_disable_it(AT91C_ID_SYS);
	aic_register_handler(AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST, schedule);
	aic_enable_it(AT91C_ID_SYS);

	/* Initialise and start PIT */
	pit_init(PIT_PERIOD, BOARD_MCK / 1000000);
	pit_enable_it();
	pit_enable();
}
