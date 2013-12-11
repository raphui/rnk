#ifndef BOARD_SAM3X_H
#define BOARD_SAM3X_H

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
	#define US_RHR		0x18
	#define US_THR		0x1C
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
