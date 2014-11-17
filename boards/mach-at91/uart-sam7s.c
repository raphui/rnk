#include <board.h>
#include <utils.h>
#include <uart.h>

/* This UART driver is based on DGBU Block */

#define BOARD_MCK       48000000
#define BAUDRATE	115200
#define DBGU_CR         (  	0x00000000) // (DBGU_CR) Control Register
#define DBGU_MR         (  	0x00000004) // (DBGU_MR) Mode Register
#define DBGU_IER        (  	0x00000008) // (DBGU_IER) Interrupt Enable Register
#define DBGU_IDR        (  	0x0000000C) // (DBGU_IDR) Interrupt Disable Register
#define DBGU_IMR        (  	0x00000010) // (DBGU_IMR) Interrupt Mask Register
#define DBGU_CSR        (  	0x00000014) // (DBGU_CSR) Channel Status Register
#define DBGU_RHR        (  	0x00000018) // (DBGU_RHR) Receiver Holding Register
#define DBGU_THR        (  	0x0000001C) // (DBGU_THR) Transmitter Holding Register
#define DBGU_BRGR       (  	0x00000020) // (DBGU_BRGR) Baud Rate Generator Register
#define DBGU_CIDR       (  	0x00000040) // (DBGU_CIDR) Chip ID Register
#define DBGU_EXID       (  	0x00000044) // (DBGU_EXID) Chip ID Extension Register
#define DBGU_FNTR       (  	0x00000048) // (DBGU_FNTR) Force NTRST Register
static unsigned int base_dbgu = 0xFFFFF200;
static unsigned int dbgu_ptcr = 0xFFFFF320;

static void sam7s_uart_init(void)
{
	/* Reset and disable receiver and transmitter */
	writel(base_dbgu + DBGU_CR, AT91C_US_RSTRX | AT91C_US_RSTTX);

	/* Disable interrupts */
	writel(base_dbgu + DBGU_IDR, 0xFFFFFFFF);

	/* Configure baud rate */
	writel(base_dbgu + DBGU_BRGR, BOARD_MCK / (BAUDRATE * 16));

	/* Disable DMA channel */
	writel(dbgu_ptcr, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

	/* Enable receiver and transmitter */
	writel(base_dbgu + DBGU_CR, AT91C_US_RXEN | AT91C_US_TXEN);
}

static void sam7s_uart_print(unsigned char byte)
{
	/* Wait for transmitter to be ready */
	while ((readl(base_dbgu + DBGU_CR) & AT91C_US_TXEMPTY) == 0)
		;

	/* Send byte */
	writel(base_dbgu + AT91C_DBGU_THR, byte);

	/* Wait for the transfer to complete */
	while ((readl(base_dbgu + DBGU_CR) & AT91C_US_TXEMPTY) == 0)
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
