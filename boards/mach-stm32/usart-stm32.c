/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>

/* Calculates the value for the USART_BRR */
static unsigned short stm32_baud_rate(long clock, unsigned int baud)
{
	unsigned int divisor = 16 * baud;
	unsigned short mantissa = clock / divisor;
	unsigned int remainder = clock % divisor;
	unsigned short fraction = (16 * remainder) / divisor;

	return (mantissa << 4) | (fraction & 0xf);
}

/* TODO: init GPIO */
static void stm32_uart_init(void)
{
	/* Enable USART3 Clock */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable GPIOB Clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	/* Configure PB10 and PB11 */
	GPIOB->MODER = 0xA00000;
	GPIOB->OTYPER = 0x0000;
	GPIOB->OSPEEDR = 0xF00000;
	GPIOB->PUPDR = 0x000000;
	GPIOB->AFR[1] = 0x7700;


	USART3->CR1 |= USART_CR1_UE;
	USART3->CR1 &= (1 << 12);
	USART3->CR1 &= (3 << 12);
	USART3->BRR = stm32_baud_rate(APB1_CLK, 115200);
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_TE;

}

static void stm32_uart_print(unsigned char byte)
{
	while(!(USART3->SR & USART_SR_TXE))
		;

	USART3->DR = byte;
}

static int stm32_uart_printl(const char *string)
{
	while (*string)
		stm32_uart_print(*string++);

	return 0;
}

struct uart_operations uart_ops = {
	.init = &stm32_uart_init,
	.print = &stm32_uart_print,
	.printl = &stm32_uart_printl,
};
