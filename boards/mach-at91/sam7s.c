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

void default_spurious_handler( void )
{
    while( 1 )
		;
}

void default_fiq_handler( void )
{
    while( 1 )
		;
}

void default_irq_handler( void )
{
    while( 1 )
		;
}

void low_level_init( void )
{
    AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_1FWS;

	// Initialize main oscillator
    AT91C_BASE_PMC->PMC_MOR = BOARD_OSCOUNT | AT91C_CKGR_MOSCEN;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS));

    // Initialize PLL at 96MHz (96.109) and USB clock to 48MHz
    AT91C_BASE_PMC->PMC_PLLR = BOARD_USBDIV | BOARD_CKGR_PLL | BOARD_PLLCOUNT
                               | BOARD_MUL | BOARD_DIV;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK));

    // Wait for the master clock if it was already initialized
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    // Switch to slow clock + prescaler
    AT91C_BASE_PMC->PMC_MCKR = BOARD_PRESCALER;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    // Switch to fast clock + prescaler
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY));

    // Initialize AIC
    AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;
    AT91C_BASE_AIC->AIC_SVR[0] = (unsigned int) default_fiq_handler;
    for (i = 1; i < 31; i++) {

        AT91C_BASE_AIC->AIC_SVR[i] = (unsigned int) default_irq_handler;
    }
    AT91C_BASE_AIC->AIC_SPU = (unsigned int) default_spurious_handler;

    // Unstack nested interrupts
    for (i = 0; i < 8 ; i++) {

        AT91C_BASE_AIC->AIC_EOICR = 0;
    }

    // Enable Debug mode
    AT91C_BASE_AIC->AIC_DCR = AT91C_AIC_DCR_PROT;

    // Watchdog initialization
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;

    // Remap the internal SRAM at 0x0
    BOARD_RemapRam();

    // Disable RTT and PIT interrupts (potential problem when program A
    // configures RTT, then program B wants to use PIT only, interrupts
    // from the RTT will still occur since they both use AT91C_ID_SYS)
    AT91C_BASE_RTTC->RTTC_RTMR &= ~(AT91C_RTTC_ALMIEN | AT91C_RTTC_RTTINCIEN);
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN;

}
