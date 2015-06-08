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
#include <ltdc.h>
#include <utils.h>
#include <stdio.h>

static short stm32_spi_find_best_pres(unsigned long parent_rate, unsigned long rate)
{
	unsigned int i;
	unsigned short pres[] = {2, 4, 8, 16, 32, 64, 128, 256};
	unsigned short best_pres;
	unsigned int diff;
	unsigned int best_diff;
	unsigned long curr_rate;


	best_diff = parent_rate - rate;

	for (i = 0; i < 8; i++) {
		curr_rate = parent_rate / pres[i];

		if (curr_rate < rate)
			diff = rate - curr_rate;
		else
			diff = curr_rate - rate;

		if (diff < best_diff) {
			best_pres = pres[i];
			best_diff = diff;
		}

		if (!best_diff || curr_rate < rate)
			break;
	}

	return best_pres;
}

void stm32_spi_init(struct spi *spi)
{

	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

	spi->rate = APB2_CLK;
	spi->rate = stm32_spi_find_best_pres(spi->rate, spi->speed);

	SPI->CR1 &= ~SPI_CR1_SPE;

//	SPI->CR1 |= (spi->rate << 3);
	SPI->CR1 |= (0x2 << 3);

	/* Set master mode */
	SPI->CR1 |= SPI_CR1_MSTR;

	/* Handle slave selection via software */
	SPI->CR1 |= SPI_CR1_SSM;
	SPI->CR1 |= SPI_CR1_SSI;

//	SPI->CR1 |= SPI_CR1_BIDIMODE;
	SPI->CR1 |= SPI_CR1_BIDIOE;

//	SPI->CR2 |= (1 << 4);

	SPI->CR1 |= SPI_CR1_SPE;
}

void stm32_spi_write(struct spi *spi, unsigned short data)
{
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	SPI->DR = data;

	while (!(SPI->SR & SPI_SR_TXE))
		;
}

unsigned short stm32_spi_read(struct spi *spi)
{
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;
	unsigned short data = 0;

	while (!(SPI->SR & SPI_SR_RXNE))
		;

	data = SPI->DR;

	return data;
}

struct spi_operations spi_ops = {
	.init = stm32_spi_init,
	.write = stm32_spi_write,
	.read = stm32_spi_read,
};
