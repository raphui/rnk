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
#include <mach/dma-stm32.h>
#include <arch/nvic.h>
#include <errno.h>
#include <queue.h>
#include <common.h>

static int stm32_spi_get_nvic_number(struct spi *spi)
{
	int nvic = 0;

	switch (spi->base_reg) {
		case SPI1_BASE:
			nvic = SPI1_IRQn;
			break;
		case SPI2_BASE:
			nvic = SPI2_IRQn;
			break;
		case SPI3_BASE:
			nvic = SPI3_IRQn;
			break;
		case SPI4_BASE:
			nvic = SPI4_IRQn;
			break;
		case SPI5_BASE:
			nvic = SPI5_IRQn;
			break;
	}


	return nvic;
}

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

static void stm32_spi_init_dma(struct spi *spi)
{
	struct dma *dma = &spi->dma;
	dma->num = 2;
	dma->stream_base = DMA2_Stream4_BASE;
	dma->stream_num = 4;
	dma->channel = 2;
	dma->dir = DMA_M_P;
	dma->mdata_size = DATA_SIZE_HALF_WORD;
	dma->pdata_size = DATA_SIZE_HALF_WORD;
	dma->mburst = INCR0;
	dma->pburst = INCR0;
	dma->minc = 0;
	dma->pinc = 0;
	dma->use_fifo = 0;

	stm32_dma_init(dma);
}

void stm32_spi_init(struct spi *spi)
{

	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

	spi->rate = APB2_CLK;
	spi->rate = stm32_spi_find_best_pres(spi->rate, spi->speed);

	SPI->CR1 &= ~SPI_CR1_SPE;

	SPI->CR1 |= (0x2 << 3);

	/* Set master mode */
	SPI->CR1 |= SPI_CR1_MSTR;

	/* Handle slave selection via software */
	SPI->CR1 |= SPI_CR1_SSM;
	SPI->CR1 |= SPI_CR1_SSI;

	SPI->CR1 |= SPI_CR1_BIDIOE;

	SPI->CR2 |= SPI_CR2_TXEIE;
	SPI->CR2 |= SPI_CR2_ERRIE;

	stm32_spi_init_dma(spi);

	SPI->CR1 |= SPI_CR1_SPE;
}

unsigned short stm32_spi_write(struct spi *spi, unsigned short data)
{
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;

	struct dma *dma = &spi->dma;
	struct dma_transfer *dma_trans = &spi->dma_trans;

	int nvic = stm32_spi_get_nvic_number(spi);
	int ready = 0;
	int ret = 0;

	stm32_dma_disable(dma);
	dma_trans->src_addr = &data;
	dma_trans->dest_addr = &SPI->DR;
	dma_trans->size = sizeof(unsigned short);

	nvic_enable_interrupt(nvic);

	stm32_dma_enable(dma);
	queue_receive(&queue, &ready, 1000);

	if (ready) {
		printk("spi ready !\r\n");

		stm32_dma_transfer(dma, dma_trans);
		if (spi->only_tx)
			SPI->CR2 |= SPI_CR2_TXDMAEN;

		stm32_dma_disable(dma);
		SPI->CR2 &= ~SPI_CR2_TXDMAEN;
	} else {
		error_printk("spi not ready\r\n");
		ret = -EIO;
	}

	return ret;
}

unsigned short stm32_spi_read(struct spi *spi)
{
	SPI_TypeDef *SPI = (SPI_TypeDef *)spi->base_reg;
	unsigned short data = 0;

	/* write dummy bytes */
	SPI->DR = 0xFF;

	while (!(SPI->SR & SPI_SR_TXE))
		;

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
