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

static void uart_register_functions( void )
{

    uart_ops.init = &uart_init;
	uart_ops.print = &uart_print;
	uart_ops.printl = &uart_printl;	

} 

void sam3x_uart_init( void )
{
	unsigned int cd = 0;
	unsigned int mode = 0x0; /* Normal mode */
	
	/* Enable clock */
	writel( PMC_BASE + PMC_PCER0 , ( 1 << 8 ) );

	/* Reset and disable receiver and transmitter */
	writel( UART_BASE + U_CR , U_CR_RSTRX
								| U_CR_RSTTX
								| U_CR_TXDIS
								| U_CR_RXDIS );
	/* Configure mode */
	writel( UART_BASE + U_MR , mode );

	/* Configure baudrate */
	writel( UART_BASE + U_BRGR , ( BOARD_MCK / UART_BAUDRATE ) / 16 );

	/* Disable PDC channel */
//	writel( UART_BASE + U_ )

	/* Enable receiver and transmitter */
	writel( UART_BASE + U_CR , U_CR_RXEN | U_CR_TXEN );

	/* Register io functions */
    uart_register_functions();
}

void sam3x_uart_print( unsigned char byte )
{

	/* Polling right now */

	while( !( readl( UART_BASE + U_SR ) & U_SR_TXRDY ) )
		;

	writel( UART_BASE + U_THR , byte );
}


int sam3x_uart_printl( const char *string )
{
	while( *string )
	{
		uart_print( *string++ );
	}

	return 0;
}
