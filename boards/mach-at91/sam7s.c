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
}
