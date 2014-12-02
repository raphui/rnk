/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <utils.h>
#include <uart.h>

/* This UART driver is based on DGBU Block */

#define BOARD_MCK       48000000
#define BAUDRATE	115200

static void sam7s_uart_init(void)
{
	/* Reset and disable receiver and transmitter */
	writel(AT91C_BASE_DBGU + DBGU_CR, AT91C_US_RSTRX | AT91C_US_RSTTX);

	/* Disable interrupts */
	writel(AT91C_BASE_DBGU + DBGU_IDR, 0xFFFFFFFF);

	/* Configure baud rate */
	writel(AT91C_BASE_DBGU + DBGU_BRGR, BOARD_MCK / (BAUDRATE * 16));

	/* Disable DMA channel */
	writel(AT91C_BASE_PDC_DBGU + PDC_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

	/* Enable receiver and transmitter */
	writel(AT91C_BASE_DBGU + DBGU_CR, AT91C_US_RXEN | AT91C_US_TXEN);
}

static void sam7s_uart_print(unsigned char byte)
{
	/* Wait for transmitter to be ready */
	while ((readl(AT91C_BASE_DBGU + DBGU_CSR) & AT91C_US_TXEMPTY) == 0)
		;

	/* Send byte */
	writel(AT91C_BASE_DBGU + DBGU_THR, byte);

	/* Wait for the transfer to complete */
	while ((readl(AT91C_BASE_DBGU + DBGU_CSR) & AT91C_US_TXEMPTY) == 0)
		;
}


static int sam7s_uart_printl(const char *string)
{
	while (*string) {
		sam7s_uart_print(*string++);
	}

	return 0;
}


struct uart_operations uart_ops = {
	.init = &sam7s_uart_init,
	.print = &sam7s_uart_print,
	.printl = &sam7s_uart_printl,
};
