/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef USART_PL010_H
#define USART_PL010_H


#define UART_PL010_RSR_OE	(1 << 3)
#define UART_PL010_RSR_BE	(1 << 2)
#define UART_PL010_RSR_PE	(1 << 1)
#define UART_PL010_RSR_FE	(1 << 0)

#define UART_PL010_FR_TXFE	(1 << 7)
#define UART_PL010_FR_RXFF	(1 << 6)
#define UART_PL010_FR_TXFF	(1 << 5)
#define UART_PL010_FR_RXFE	(1 << 4)
#define UART_PL010_FR_BUSY	(1 << 3)
#define UART_PL010_FR_TMSK	(UART_PL010_FR_TXFF + UART_PL010_FR_BUSY)

#define UART_PL010_CR_LPE	(1 << 7)
#define UART_PL010_CR_RTIE	(1 << 6)
#define UART_PL010_CR_TIE	(1 << 5)
#define UART_PL010_CR_RIE	(1 << 4)
#define UART_PL010_CR_MSIE	(1 << 3)
#define UART_PL010_CR_IIRLP	(1 << 2)
#define UART_PL010_CR_SIREN	(1 << 1)
#define UART_PL010_CR_UARTEN	(1 << 0)

#define UART_PL010_LCRH_WLEN_8	(3 << 5)
#define UART_PL010_LCRH_WLEN_7	(2 << 5)
#define UART_PL010_LCRH_WLEN_6	(1 << 5)
#define UART_PL010_LCRH_WLEN_5	(0 << 5)
#define UART_PL010_LCRH_FEN	(1 << 4)
#define UART_PL010_LCRH_STP2	(1 << 3)
#define UART_PL010_LCRH_EPS	(1 << 2)
#define UART_PL010_LCRH_PEN	(1 << 1)
#define UART_PL010_LCRH_BRK	(1 << 0)

#define UART_PL010_BAUD_460800	1
#define UART_PL010_BAUD_230400	3
#define UART_PL010_BAUD_115200	7
#define UART_PL010_BAUD_57600	15
#define UART_PL010_BAUD_38400	23
#define UART_PL010_BAUD_19200	47
#define UART_PL010_BAUD_14400	63
#define UART_PL010_BAUD_9600	95
#define UART_PL010_BAUD_4800	191
#define UART_PL010_BAUD_2400	383
#define UART_PL010_BAUD_1200	767

#endif /* USART_PL010_H */
