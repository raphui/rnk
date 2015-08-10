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
#include <errno.h>

#include <mach/rcc-stm32.h>

#define MAX_DMA_SIZE 0xFFFF

static unsigned int stm32_dma_get_base(struct dma *dma)
{
	unsigned int base = 0;

	if (dma->num == 1)
		base = DMA1_BASE;
	else if (dma->num == 2)
		base = DMA2_BASE;
	else
		base = -EINVAL;

	return base;
}

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

static int stm32_dma_get_interrupt_flags(struct dma *dma)
{
	int mask = 0;

	switch (dma->stream_num) {
		case 0:
			mask = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0;
			if (dma->use_fifo)
				mask |= DMA_LIFCR_CFEIF0;
			break;
		case 1:
			mask = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1;
			if (dma->use_fifo)
				mask |= DMA_LIFCR_CFEIF1;
			break;
		case 2:
			mask = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2;
			if (dma->use_fifo)
				mask |= DMA_LIFCR_CFEIF2;
			break;
		case 3:
			mask = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3;
			if (dma->use_fifo)
				mask |= DMA_LIFCR_CFEIF3;
			break;
		case 4:
			mask = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4;
			if (dma->use_fifo)
				mask |= DMA_HIFCR_CFEIF4;
			break;
		case 5:
			mask = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5;
			if (dma->use_fifo)
				mask |= DMA_HIFCR_CFEIF5;
			break;
		case 6:
			mask = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6;
			if (dma->use_fifo)
				mask |= DMA_HIFCR_CFEIF6;
			break;
		case 7:
			mask = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7;
			if (dma->use_fifo)
				mask |= DMA_HIFCR_CFEIF7;
			break;
		default:
			error_printk("invalid channel number\r\n");
			mask = -EINVAL;
			break;
	}

	return mask;
}

int stm32_dma_init(struct dma *dma)
{
	int ret = 0;
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;

	if (dma->num == 1)
		ret = stm32_rcc_enable_clk(DMA1_BASE);
	else
		ret = stm32_rcc_enable_clk(DMA2_BASE);

	if (ret < 0) {
		error_printk("cannot enable DMA periph clock\r\n");
		return ret;
	}

	if ((dma->num == 1) && (dma->dir == DMA_M_M)) {
		debug_printk("DMA1 does not support mem to mem transfer\r\n");
		return -EINVAL;
	}


	DMA_STREAM->CR = (dma->channel << 25) | (dma->mburst << 23)
			| (dma->pburst << 21) | (dma->mdata_size << 13)
			| (dma->pdata_size << 11) | (dma->minc << 10) 
			| (dma->pinc << 9) | (dma->dir << 6);

	if (dma->use_fifo) {
		DMA_STREAM->FCR &= DMA_SxFCR_DMDIS;
		DMA_STREAM->FCR |= DMA_SxFCR_FTH;
	}

	return 0;
}

int stm32_dma_transfer(struct dma *dma, struct dma_transfer *dma_trans)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;

	if (dma_trans->size > MAX_DMA_SIZE) {
		error_printk("invalid dma transfer size\r\n");
		return -EINVAL;
	}
	else
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

	return 0;
}

void stm32_dma_enable(struct dma *dma)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;
	DMA_TypeDef *DMA_BASE;

	unsigned int base = stm32_dma_get_base(dma);
	int nvic = stm32_dma_get_nvic_number(dma);
	int mask = stm32_dma_get_interrupt_flags(dma);

	if (base < 0) {
		error_printk("invalid dma num\r\n");
		return;
	}

	DMA_BASE = (DMA_TypeDef *)base;

	if (mask < 0) {
		error_printk("cannot enable dma\r\n");
		return;
	}

	if (dma->stream_num > 3 )
		DMA_BASE->HIFCR = mask;
	else
		DMA_BASE->LIFCR = mask;

	nvic_enable_interrupt(nvic);

	DMA_STREAM->CR |= (1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1);

	if (dma->use_fifo)
		DMA_STREAM->FCR |= (1 << 7);

	DMA_STREAM->CR |= (1 << 0);
}

void stm32_dma_disable(struct dma *dma)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma);

	DMA_STREAM->CR &= ~(1 << 0);

	DMA_STREAM->CR &= ~((1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1));

	if (dma->use_fifo)
		DMA_STREAM->FCR &= ~(1 << 7);

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);
}

struct dma_operations dma_ops = {
	.init = stm32_dma_init,
	.transfer = stm32_dma_transfer,
	.enable = stm32_dma_enable,
	.disable = stm32_dma_disable,
};
