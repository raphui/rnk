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

#ifndef BOARD_BCM2835_H
#define BOARD_BCM2835_H

/* GPIO Register address */
#define GPIO_BASE		0x20200000
	#define GPIO_GPPUD		( GPIO_BASE + 0x94 )
	#define GPIO_GPPUDCLK0	( GPIO_BASE + 0x98 )

/* UART Register */
//#define UART_BASE_ADDRESS	0x7E20100
#define UART0_BASE		0x20201000
	#define UART0_DR		( UART0_BASE + 0x0	)
	#define UART0_RSRECR	( UART0_BASE + 0x4	)
	#define UART0_FR		( UART0_BASE + 0x18	)
	#define UART0_ILPR		( UART0_BASE + 0x20	)
	#define UART0_IBRD		( UART0_BASE + 0x24	)
	#define UART0_FBRD		( UART0_BASE + 0x28	)
	#define UART0_LCRH		( UART0_BASE + 0x2C	)
	#define UART0_CR		( UART0_BASE + 0x30	)
	#define UART0_IFLS		( UART0_BASE + 0x34	)
	#define UART0_IMSC		( UART0_BASE + 0x38	)
	#define UART0_RIS		( UART0_BASE + 0x3C	)
	#define UART0_MIS		( UART0_BASE + 0x40	)
	#define UART0_ICR		( UART0_BASE + 0x44	)
	#define UART0_DMACR		( UART0_BASE + 0x48	)
	#define UART0_ITCR		( UART0_BASE + 0x80	)
	#define UART0_ITIP		( UART0_BASE + 0x84	)
	#define UART0_ITOP		( UART0_BASE + 0x88	)
	#define UART0_TDR		( UART0_BASE + 0x8C	)


#endif /* BOARD_BCM2835_H */
