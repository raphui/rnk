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

#ifndef BOARD_SAM3X_H
#define BOARD_SAM3X_H

#include <sam3x.h>

struct uart_operations
{
	void ( *init )( void );
	void ( *print )( unsigned char byte );
	int ( *printl )( const char *string );

};

struct uart_operations uart_ops;

#define OSC_12MHZ	12000000
#define BOARD_MCK   84000000

#define EEFC0_BASE	0x400E0A00
#define EEFC1_BASE	0x400E0C00
	#define EFFC_FMR	0x0
		#define E_FMR_FRDY		( 1 << 0 )
		#define E_FMR_FWS(x)	( x << 8 )
		#define E_FMR_FAM		( 1 << 24 )
	#define EEFC_FCR	0x4
		#define E_FCR_FCMD(x)	( x << 0 )
		#define E_FCR_FARG(x)	( x << 8 )
		#define E_FCR_FKEY(x)	( x << 24 )
	#define EEFC_FSR	0x8
		#define E_FSR_FRDY		( 1 << 0 )
		#define E_FSR_FCMDE		( 1 << 1 )
		#define E_FSR_FLOCKE	( 1 << 2 )
	#define EEFC_FRR	0xC
		#define E_FRR_FVALUE(x)	( x << 0 )

#define PMC_BASE	0x400E0600
	#define PMC_SCER		0x0
	#define PMC_SCDR		0x4
	#define PMC_SCSR		0x8
	#define PMC_PCER0		0x10
	#define PMC_PCDR0		0x14
	#define PMC_PCSR0		0x18
	#define PMC_CKGR_UCKR	0x1C
		#define P_CKGR_UCKR_UPLLEN			( 1 << 16 )
		#define P_CKGR_UCKR_UPLLCOUNT(x)	( x << 20 )
	#define PMC_CKGR_MOR	0x20
		#define P_CKGR_MOR_MOSCXTEN		( 1 << 0 )
		#define P_CKGR_MOR_MOSCXTBY		( 1 << 1 )
		#define P_CKGR_MOR_MOSCRCEN		( 1 << 3 )
		#define P_CKGR_MOR_MOSCRCF(x)	( x << 4 )
		#define P_CKGR_MOR_MOSCXTST(x)	( x << 8 )
		#define P_CKGR_MOR_KEY(x)		( x << 16 )
		#define P_CKGR_MOR_MOSCSEL		( 1 << 24 )
		#define P_CKGR_MOR_CFDEN		( 1 << 25 )
	#define PMC_CKGR_MCFR	0x24
		#define P_CKGR_MCFR_MAINF(x)	( x << 0 )
		#define P_CKGR_MCFR_MAINFRDY	( 1 << 16 )
	#define PMC_CKGR_PLLAR	0x28
		#define P_CKGR_PLLAR_DIVA(x)		( x << 0 )
		#define P_CKGR_PLLAR_PLLACOUNT(x)	( x << 8 )
		#define P_CKGR_PLLAR_MULA(x)		( x << 16 )
		#define P_CKGR_PLLAR_ONE			( 1 << 29 )
	#define PMC_MCKR		0x30
		#define P_MCKR_CSS(x)	( x << 0 )
		#define P_MCKR_PRES(x)	( x << 4 )
		#define P_MCKR_PLLADIV2	( 1 << 12 )
		#define P_MCKR_UPLLDIV2	( 1 << 13 )
	#define PMC_USB			0x38
	#define PMC_PCK0		0x40
	#define PMC_PCK1		0x44
	#define PMC_PCK2		0x48
	#define PMC_IER			0x60
		#define P_IER_MOSCXTS	( 1 << 0 )
		#define P_IER_LOCKA		( 1 << 1 )
		#define P_IER_MCKRDY	( 1 << 3 )
		#define P_IER_LOCKU		( 1 << 6 )
		#define P_IER_PCKRDY0	( 1 << 8 )
		#define P_IER_PCKRDY1	( 1 << 9 )
		#define P_IER_PCKRDY2	( 1 << 10 )
		#define P_IER_MOSCSELS	( 1 << 16 )
		#define P_IER_MOSCRCS	( 1 << 17 )
		#define P_IER_CFDEV		( 1 << 18 )
	#define PMC_IDR			0x64
		#define P_IDR_MOSCXTS	( 1 << 0 )
		#define P_IDR_LOCKA		( 1 << 1 )
		#define P_IDR_MCKRDY	( 1 << 3 )
		#define P_IDR_LOCKU		( 1 << 6 )
		#define P_IDR_PCKRDY0	( 1 << 8 )
		#define P_IDR_PCKRDY1	( 1 << 9 )
		#define P_IDR_PCKRDY2	( 1 << 10 )
		#define P_IDR_MOSCSELS	( 1 << 16 )
		#define P_IDR_MOSCRCS	( 1 << 17 )
		#define P_IDR_CFDEV		( 1 << 18 )
	#define PMC_SR			0x68
		#define P_SR_MOSCXTS	( 1 << 0 )
		#define P_SR_LOCKA		( 1 << 1 )
		#define P_SR_MCKRDY	( 1 << 3 )
		#define P_SR_LOCKU		( 1 << 6 )
		#define P_SR_PCKRDY0	( 1 << 8 )
		#define P_SR_PCKRDY1	( 1 << 9 )
		#define P_SR_PCKRDY2	( 1 << 10 )
		#define P_SR_MOSCSELS	( 1 << 16 )
		#define P_SR_MOSCRCS	( 1 << 17 )
		#define P_SR_CFDEV		( 1 << 18 )
		#define P_SR_CFDS		( 1 << 19 )
		#define P_SR_FOS		( 1 << 20 )
	#define PMC_IMR			0x6C
		#define P_IMR_MOSCXTS	( 1 << 0 )
		#define P_IMR_LOCKA		( 1 << 1 )
		#define P_IMR_MCKRDY	( 1 << 3 )
		#define P_IMR_LOCKU		( 1 << 6 )
		#define P_IMR_PCKRDY0	( 1 << 8 )
		#define P_IMR_PCKRDY1	( 1 << 9 )
		#define P_IMR_PCKRDY2	( 1 << 10 )
		#define P_IMR_MOSCSELS	( 1 << 16 )
		#define P_IMR_MOSCRCS	( 1 << 17 )
		#define P_IMR_CFDEV		( 1 << 18 )
	#define PMC_FSMR		0x70
	#define PMC_FSPR		0x74
	#define PMC_FOCR		0x76
	#define PMC_WPMR		0xE4
	#define PMC_WPSR		0xE8
	#define PMC_PCER1		0x100
	#define PMC_PCDR1		0x104
	#define PMC_PCSR1		0x108
	#define PMC_PCR			0x10C

#define UART_BAUDRATE	115200

#define UART_BASE	0x400E0800
	#define U_CR	0x0
		#define U_CR_RSTRX	( 1 << 2 )
		#define U_CR_RSTTX	( 1 << 3 )
		#define U_CR_RXEN	( 1 << 4 )
		#define U_CR_RXDIS	( 1 << 5 )
		#define U_CR_TXEN	( 1 << 6 )
		#define U_CR_TXDIS	( 1 << 7 )
		#define U_CR_RSTSTA	( 1 << 8 )
	#define U_MR	0x4
		#define U_MR_PAR(x)		( x << 9 )
		#define U_MR_CHMODE(x)	( x << 14 )
	#define U_IER	0x8
		#define U_IER_RXRDY		( 1 << 0 )
		#define U_IER_TXRDY		( 1 << 1 )
		#define U_IER_ENDRX		( 1 << 3 )
		#define U_IER_ENDTX		( 1 << 4 )
		#define U_IER_OVRE		( 1 << 5 )
		#define U_IER_FRAME		( 1 << 6 )
		#define U_IER_PARE		( 1 << 7 )
		#define U_IER_TXEMPTY	( 1 << 9 )
		#define U_IER_TXBUFE	( 1 << 11 )
		#define U_IER_RXBUFF	( 1 << 12 )
	#define U_IDR	0xC
		#define U_IDR_RXRDY		( 1 << 0 )
		#define U_IDR_TXRDY		( 1 << 1 )
		#define U_IDR_ENDRX		( 1 << 3 )
		#define U_IDR_ENDTX		( 1 << 4 )
		#define U_IDR_OVRE		( 1 << 5 )
		#define U_IDR_FRAME		( 1 << 6 )
		#define U_IDR_PARE		( 1 << 7 )
		#define U_IDR_TXEMPTY	( 1 << 9 )
		#define U_IDR_TXBUFE	( 1 << 11 )
		#define U_IDR_RXBUFF	( 1 << 12 )
	#define U_IMR	0x10
		#define U_IMR_RXRDY		( 1 << 0 )
		#define U_IMR_TXRDY		( 1 << 1 )
		#define U_IMR_ENDRX		( 1 << 3 )
		#define U_IMR_ENDTX		( 1 << 4 )
		#define U_IMR_OVRE		( 1 << 5 )
		#define U_IMR_FRAME		( 1 << 6 )
		#define U_IMR_PARE		( 1 << 7 )
		#define U_IMR_TXEMPTY	( 1 << 9 )
		#define U_IMR_TXBUFE	( 1 << 11 )
		#define U_IMR_RXBUFF	( 1 << 12 )
	#define U_SR	0x14
		#define U_SR_RXRDY		( 1 << 0 )
		#define U_SR_TXRDY		( 1 << 1 )
		#define U_SR_ENDRX		( 1 << 3 )
		#define U_SR_ENDTX		( 1 << 4 )
		#define U_SR_OVRE		( 1 << 5 )
		#define U_SR_FRAME		( 1 << 6 )
		#define U_SR_PARE		( 1 << 7 )
		#define U_SR_TXEMPTY	( 1 << 9 )
		#define U_SR_TXBUFE		( 1 << 11 )
		#define U_SR_RXBUFF		( 1 << 12 )
	#define U_RHR	0x18
		#define U_RHR_RXCHR	/* data */
	#define U_THR	0x1C
		#define U_THR_TXCHR	/* data */
	#define U_BRGR	0x20
		#define U_BRGR_CD	/* clock divisor */

#define USART0_BASE	0x40098000
#define USART1_BASE	0x4009C000
#define USART2_BASE	0x400A0000
#define USART3_BASE	0x400A4000
	#define US_CR		0x00
		#define US_CR_RSTRX		( 1 << 2 )
		#define US_CR_RSTTX		( 1 << 3 )
		#define US_CR_RXEN		( 1 << 4 )
		#define US_CR_RXDIS		( 1 << 5 )
		#define US_CR_TXEN		( 1 << 6 )
		#define US_CR_TXDIS 	( 1 << 7 )
		#define US_CR_RSTSTA 	( 1 << 8 )
		#define US_CR_STTBRK 	( 1 << 9 )
		#define US_CR_STPBRK 	( 1 << 10 )
		#define US_CR_STTTO 	( 1 << 11 )
		#define US_CR_SENDA 	( 1 << 12 )
		#define US_CR_RSTIT		( 1 << 13 )
		#define US_CR_RSTNACK  	( 1 << 14 )
		#define US_CR_RETTO  	( 1 << 15 )
		#define US_CR_RTSEN  	( 1 << 18 )
		#define US_CR_RTSDIS  	( 1 << 19 )
		#define US_CR_LINABT  	( 1 << 20 )
		#define US_CR_LINWKUP  	( 1 << 21 )

	#define US_MR		0x04
		#define US_MR_USART_MODE	0x0
			#define US_MR_USART_MODE_NORMAL			0x0
			#define US_MR_USART_MODE_RS485			0x1
			#define US_MR_USART_MODE_HW_HANDSHAKING	0x2
			#define US_MR_USART_MODE_ISO7816_T_0	0x4
			#define US_MR_USART_MODE_ISO7816_T_1	0x6
			#define US_MR_USART_MODE_IRDA			0x8
			#define US_MR_USART_MODE_LIN_MASTER		0xA
			#define US_MR_USART_MODE_LIN_SLAVE		0xB
			#define US_MR_USART_MODE_SPI_MASTER		0xE
			#define US_MR_USART_MODE_SPI_SLAVE		0xF

	#define US_IER		0x08
	#define US_IDR		0x0C
	#define US_IMR		0x10
	#define US_CSR		0x14
		#define US_CSR_RXRDY	( 1 << 0 )
		#define US_CSR_TXRDY	( 1 << 1 )
		#define US_CSR_RXBRK	( 1 << 2 )
		#define US_CSR_ENDRX	( 1 << 3 )
		#define US_CSR_ENDTX	( 1 << 4 )
		#define US_CSR_OVRE		( 1 << 5 )
		#define US_CSR_FRAME	( 1 << 6 )
		#define US_CSR_PARE		( 1 << 7 )
		#define US_CSR_TIMEOUT	( 1 << 8 )
		#define US_CSR_TXEMPTY	( 1 << 9 )
		#define US_CSR_ITER		( 1 << 10 )
		#define US_CSR_TXBUFE	( 1 << 11 )
		#define US_CSR_RXBUFF	( 1 << 12 )
		#define US_CSR_NACK		( 1 << 13 )
		#define US_CSR_LINID	( 1 << 14 )
		#define US_CSR_LINTC	( 1 << 15 )
		#define US_CSR_CTSIC	( 1 << 19 )
		#define US_CSR_CTS		( 1 << 23 )
		#define US_CSR_MANERR	( 1 << 24 )
		#define US_CSR_LINBE	( 1 << 25 )
		#define US_CSR_LINISFE	( 1 << 26 )
		#define US_CSR_LINIPE	( 1 << 27 )
		#define US_CSR_LINCE	( 1 << 28 )
		#define US_CSR_LINSNRE	( 1 << 29 )

	#define US_RHR		0x18
		#define RXCHR	/* data */ 
		#define RXSYNH	( 1 << 15 )

	#define US_THR		0x1C
		#define TXCHR	/* data */
		#define TXSYNH	( 1 << 15 )

	#define US_BRGR		0x20
	#define US_RTOR		0x24
	#define US_TTGR		0x28
	#define US_FIDI		0x40
	#define US_NER		0x44
	#define US_IF		0x4C
	#define US_MAN		0x50
	#define US_LINMR	0x54
	#define US_LINIR	0x58
	#define US_WPMR		0xE4
	#define US_WPSR		0xE8
	#define US_VERSION	0xFC

#endif /* BOARD_SAM3X_H */
