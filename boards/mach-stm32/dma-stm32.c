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
#include <dma.h>
#include <utils.h>
#include <stdio.h>
#include <arch/nvic.h>

static int stm32_dma_get_nvic_number(struct dma *dma)
{
	int nvic = 0;

	if (dma->num == 1) {

		switch (dma->stream_num) {
			case 0:
				nvic = DMA1_Stream0_IRQn;
				break;
			case 1:
				nvic = DMA1_Stream1_IRQn;
				break;
			case 2:
				nvic = DMA1_Stream2_IRQn;
				break;
			case 3:
				nvic = DMA1_Stream3_IRQn;
				break;
			case 4:
				nvic = DMA1_Stream4_IRQn;
				break;
			case 5:
				nvic = DMA1_Stream5_IRQn;
				break;
			case 6:
				nvic = DMA1_Stream6_IRQn;
				break;
			case 7:
				nvic = DMA1_Stream7_IRQn;
				break;
		}

	} else if (dma->num == 2) {

		switch (dma->stream_num) {
			case 0:
				nvic = DMA2_Stream0_IRQn;
				break;
			case 1:
				nvic = DMA2_Stream1_IRQn;
				break;
			case 2:
				nvic = DMA2_Stream2_IRQn;
				break;
			case 3:
				nvic = DMA2_Stream3_IRQn;
				break;
			case 4:
				nvic = DMA2_Stream4_IRQn;
				break;
			case 5:
				nvic = DMA2_Stream5_IRQn;
				break;
			case 6:
				nvic = DMA2_Stream6_IRQn;
				break;
			case 7:
				nvic = DMA2_Stream7_IRQn;
				break;
		}

	}

	return nvic;
}

void stm32_dma_init(struct dma *dma)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;

	if (dma->num == 1)
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	else
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	if ((dma->num == 1) && (dma->dir == DMA_M_M)) {
		debug_printk("DMA1 does not support mem to mem transfer\r\n");
		return;
	}


	DMA_STREAM->CR = (dma->channel << 25) | (dma->mburst << 23)
			| (dma->pburst << 21) | (dma->mdata_size << 13)
			| (dma->pdata_size << 11) | (dma->inc_mode << 10) 
			| (dma->dir << 6);

}

void stm32_dma_transfer(struct dma *dma, struct dma_transfer *dma_trans)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;

	DMA_STREAM->NDTR = dma_trans->size;

	if (dma->dir == DMA_P_M) {
		DMA_STREAM->PAR = dma_trans->src_addr;
		DMA_STREAM->M0AR = dma_trans->dest_addr;

	} else if (dma->dir == DMA_M_P) {
		DMA_STREAM->M0AR = dma_trans->src_addr;
		DMA_STREAM->PAR = dma_trans->dest_addr;

	} else if (dma->dir == DMA_M_M) {
		DMA_STREAM->PAR = dma_trans->src_addr;
		DMA_STREAM->M0AR = dma_trans->dest_addr;
	}
}

void stm32_dma_enable(struct dma *dma)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma);

	nvic_enable_interrupt(nvic);

	DMA_STREAM->CR |= (1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1);

	DMA_STREAM->CR |= (1 << 0);
}

void stm32_dma_disable(struct dma *dma)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma);

	DMA_STREAM->CR &= ~(1 << 0);

	DMA_STREAM->CR &= ~((1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1));

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);
}

struct dma_operations dma_ops = {
	.init = stm32_dma_init,
	.transfer = stm32_dma_transfer,
	.enable = stm32_dma_enable,
	.disable = stm32_dma_disable,
};
