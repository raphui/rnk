#include <board.h>
#include <utils.h>
#include <uart.h>

/* This UART driver is based on DGBU Block */

#define BOARD_MCK       48000000
#define BAUDRATE	115200

static unsigned int base_dbgu = 0xFFFFF200;

static void sam7s_uart_init(void)
{
	/* Reset and disable receiver and transmitter */
	writel(base_dbgu + AT91C_DBGU_CR, AT91C_US_RSTRX | AT91C_US_RSTTX);

	/* Disable interrupts */
	writel(base_dbgu + AT91C_DBGU_IDR, 0xFFFFFFFF);

	/* Configure baud rate */
	writel(base_dbgu + AT91C_DBGU_BRGR, BOARD_MCK / (BAUDRATE * 16));

	/* Disable DMA channel */
	writel(base_dbgu + AT91C_DBGU_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

	/* Enable receiver and transmitter */
	writel(base_dbgu + AT91C_DBGU_CR, AT91C_US_RXEN | AT91C_US_TXEN);
}

static void sam7s_uart_print(unsigned char byte)
{
	/* Wait for transmitter to be ready */
	while ((readl(base_dbgu + AT91C_DBGU_CR) & AT91C_US_TXEMPTY) == 0)
		;

	/* Send byte */
	writel(base_dbgu + AT91C_DBGU_THR, byte);

	/* Wait for the transfer to complete */
	while ((readl(base_dbgu + AT91C_DBGU_CR) & AT91C_US_TXEMPTY) == 0)
		;
}


static int sam7s_uart_printl(const char *string)
{
	while (*string++) {
		sam7s_uart_print(*string);
	}

	return 0;
}


struct uart_operations uart_ops = {
	.init = &sam7s_uart_init,
	.print = &sam7s_uart_print,
	.printl = &sam7s_uart_printl,
};
