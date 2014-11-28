#ifndef BOARD_SAM7S_H
#define BOARD_SAM7S_H

#include <sam7s.h>
#include <sam7s-reg.h>

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


struct uart_operations
{
	void (*init)(void);
	void (*print)(unsigned char byte);
	int (*printl)(const char *string);

};

struct uart_operations uart_ops;

struct pit_operations
{
	void (*init)(unsigned int period, unsigned int pit_frequency);
	void (*enable)(void);
	void (*enable_it)(void);
	void (*disable_it)(void);
	void (*read_pivr)(void);
};

struct pit_operations pit_ops;

#endif /* BOARD_SAM7S_H */
