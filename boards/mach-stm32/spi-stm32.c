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

	spi->rate = APB2_CLK;
	spi->rate = stm32_spi_find_best_pres(spi->rate, spi->speed);

	SPI->CR1 |= (spi->rate << 3);
}

void stm32_spi_write(struct spi *spi, unsigned short data)
{

}

unsigned short stm32_spi_read(struct spi *spi)
{
	return 0;
}

struct spi_operations spi_ops = {
	.init = stm32_spi_init,
	.write = stm32_spi_write,
	.read = stm32_spi_read,
};
