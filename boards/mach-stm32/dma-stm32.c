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
#include <mach/rcc-stm32.h>
#include <dma.h>
#include <utils.h>
#include <stdio.h>
#include <arch/nvic.h>
#include <errno.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <fdtparse.h>

#define MAX_DMA_SIZE 0xFFFF

static inline unsigned int stm32_dma_get_base(struct dma_stream *dma_stream)
{
	return dma_stream->dma->base_reg;
}

static int stm32_dma_get_nvic_number(struct dma_stream *dma_stream)
{
	int nvic = 0;

	if (dma_stream->dma->num == 1) {

		switch (dma_stream->stream_num) {
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

	} else if (dma_stream->dma->num == 2) {

		switch (dma_stream->stream_num) {
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

static int stm32_dma_get_interrupt_flags(struct dma_stream *dma_stream)
{
	int mask = 0;

	switch (dma_stream->stream_num) {
		case 0:
			mask = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0;
			if (dma_stream->use_fifo)
				mask |= DMA_LIFCR_CFEIF0;
			break;
		case 1:
			mask = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1;
			if (dma_stream->use_fifo)
				mask |= DMA_LIFCR_CFEIF1;
			break;
		case 2:
			mask = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2;
			if (dma_stream->use_fifo)
				mask |= DMA_LIFCR_CFEIF2;
			break;
		case 3:
			mask = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3;
			if (dma_stream->use_fifo)
				mask |= DMA_LIFCR_CFEIF3;
			break;
		case 4:
			mask = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4;
			if (dma_stream->use_fifo)
				mask |= DMA_HIFCR_CFEIF4;
			break;
		case 5:
			mask = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5;
			if (dma_stream->use_fifo)
				mask |= DMA_HIFCR_CFEIF5;
			break;
		case 6:
			mask = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6;
			if (dma_stream->use_fifo)
				mask |= DMA_HIFCR_CFEIF6;
			break;
		case 7:
			mask = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7;
			if (dma_stream->use_fifo)
				mask |= DMA_HIFCR_CFEIF7;
			break;
		default:
			error_printk("invalid channel number\r\n");
			mask = -EINVAL;
			break;
	}

	return mask;
}

static int stm32_dma_transfer(struct dma_stream *dma_stream, struct dma_transfer *dma_trans)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma_stream->stream_base;

	if (dma_trans->size > MAX_DMA_SIZE) {
		error_printk("invalid dma transfer size\r\n");
		return -EINVAL;
	}
	else
		DMA_STREAM->NDTR = dma_trans->size;

	if (dma_stream->dir == DMA_P_M) {
		DMA_STREAM->PAR = dma_trans->src_addr;
		DMA_STREAM->M0AR = dma_trans->dest_addr;

	} else if (dma_stream->dir == DMA_M_P) {
		DMA_STREAM->M0AR = dma_trans->src_addr;
		DMA_STREAM->PAR = dma_trans->dest_addr;

	} else if (dma_stream->dir == DMA_M_M) {
		DMA_STREAM->PAR = dma_trans->src_addr;
		DMA_STREAM->M0AR = dma_trans->dest_addr;
	}

	return 0;
}

static int stm32_dma_enable(struct dma_stream *dma_stream)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma_stream->stream_base;
	DMA_TypeDef *DMA_BASE;

	unsigned int base = stm32_dma_get_base(dma_stream);
	int nvic = stm32_dma_get_nvic_number(dma_stream);
	int mask = stm32_dma_get_interrupt_flags(dma_stream);

	if (base < 0) {
		error_printk("invalid dma num\r\n");
		return -EINVAL;
	}

	DMA_BASE = (DMA_TypeDef *)base;

	if (mask < 0) {
		error_printk("cannot enable dma\r\n");
		return mask;
	}

	if (dma_stream->stream_num > 3 )
		DMA_BASE->HIFCR = mask;
	else
		DMA_BASE->LIFCR = mask;

	nvic_enable_interrupt(nvic);

	DMA_STREAM->CR |= (1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1);

	if (dma_stream->use_fifo)
		DMA_STREAM->FCR |= (1 << 7);

	DMA_STREAM->CR |= (1 << 0);

	return 0;
}

static int stm32_dma_disable(struct dma_stream *dma_stream)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma_stream->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma_stream);

	DMA_STREAM->CR &= ~(1 << 0);

	DMA_STREAM->CR &= ~((1 <<  4) | (1 << 3) | (1 << 2) | (1 << 1));

	if (dma_stream->use_fifo)
		DMA_STREAM->FCR &= ~(1 << 7);

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return 0;
}

int stm32_dma_stream_init(struct dma_stream *dma_stream)
{
	int ret = 0;
	struct dma_controller *dma_ctrl = dma_stream->dma;
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma_stream->stream_base;

	if (!dma_ctrl->mem2mem) {
		debug_printk("dma controller does not support mem to mem transfer\r\n");
		return -EINVAL;
	}


	DMA_STREAM->CR = (dma_stream->channel << 25) | (dma_stream->mburst << 23)
			| (dma_stream->pburst << 21) | (dma_stream->mdata_size << 13)
			| (dma_stream->pdata_size << 11) | (dma_stream->minc << 10)
			| (dma_stream->pinc << 9) | (dma_stream->dir << 6);

	if (dma_stream->use_fifo) {
		DMA_STREAM->FCR &= DMA_SxFCR_DMDIS;
		DMA_STREAM->FCR |= DMA_SxFCR_FTH;
	}

	return ret;
}

struct dma_operations dma_ops = {
	.stream_init = stm32_dma_stream_init,
	.transfer = stm32_dma_transfer,
	.enable = stm32_dma_enable,
	.disable = stm32_dma_disable,
};

int stm32_dma_of_init(struct dma_controller *dma)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dma->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, dma->dev.of_compat);
	if (ret < 0)
		goto out;

	dma->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!dma->base_reg) {
		error_printk("failed to retrieve dma controller base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

out:
	return ret;
}

int stm32_dma_init(struct device *dev)
{
	int ret = 0;
	struct dma_controller *dma = NULL;

	dma = dma_new_controller();
	if (!dma) {
		error_printk("failed to request new dma controller\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&dma->dev, dev, sizeof(struct device));
	dma->dma_ops = &dma_ops;

	ret = stm32_dma_of_init(dma);
	if (ret < 0) {
		error_printk("failed to init dma controller with fdt data\n");
		goto err;
	}

	ret = stm32_rcc_enable_clk(dma->base_reg);
	if (ret < 0) {
		error_printk("cannot enable clk for dma controller\n");
		goto err;
	}

	ret = dma_register_controller(dma);
	if (ret < 0) {
		error_printk("failed to register stm32 dma controller\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(dma->base_reg);
err:
	return ret;
}

struct device stm32_dma_driver = {
	.of_compat = "st,stm32f4xx-dma",
	.probe = stm32_dma_init,
};

static int stm32_dma_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_dma_driver);
	if (ret < 0)
		error_printk("failed to register stm32_dma device\n");
	return ret;
}
postarch_initcall(stm32_dma_register);
