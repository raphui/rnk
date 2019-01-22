#include <board.h>
#include <mach/rcc-stm32.h>
#include <drv/dma.h>
#include <utils.h>
#include <kernel/printk.h>
#include <armv7m/nvic.h>
#include <errno.h>
#include <drv/device.h>
#include <init.h>
#include <string.h>
#include <fdtparse.h>
#include <mm/mm.h>
#include <drv/irq.h>
#include <armv7m/vector.h>

#define MAX_DMA_SIZE 0xFFFF

#define DMA_ISR_TCIF	0x8200820

static inline unsigned int stm32_dma_get_base(struct dma_stream *dma_stream)
{
	return dma_stream->dma->base_reg;
}

static int stm32_dma_get_nvic_number(struct dma_stream *dma_stream)
{

	return dma_stream->dma->interrupts[dma_stream->stream_num];
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
			error_printk("invalid stream number\r\n");
			mask = -EINVAL;
			break;
	}

	return mask;
}

static void stm32_dma_isr(void *arg)
{
	struct dma_stream *stream = (struct dma_stream *)arg;
	unsigned int base = stm32_dma_get_base(stream);
	DMA_TypeDef *dma_base = (DMA_TypeDef *)base;
	DMA_Stream_TypeDef *dma_stream = (DMA_Stream_TypeDef *)stream->stream_base;
	int flags;

	if (stream->stream_num > 3)
		flags = dma_base->HISR;
	else
		flags = dma_base->LISR;

	debug_printk("dma->sr: 0x%x\n", flags);
	debug_printk("dma->ndtr: 0x%x\n", dma_stream->NDTR);

	if (stream->stream_num > 3)
		dma_base->HIFCR = flags;
	else
		dma_base->LIFCR = flags;

	if (flags & DMA_ISR_TCIF)
		stream->handler(stream->arg);
}

int stm32_dma_transfer(struct dma_stream *dma_stream, struct dma_transfer *dma_trans)
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

int stm32_dma_enable(struct dma_stream *dma_stream)
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

	if (dma_stream->stream_num > 3)
		DMA_BASE->HIFCR = mask;
	else
		DMA_BASE->LIFCR = mask;

	nvic_enable_interrupt(nvic);

	if (dma_stream->enable_interrupt)
		DMA_STREAM->CR |= DMA_SxCR_TCIE;

	if (dma_stream->use_fifo)
		DMA_STREAM->FCR |= DMA_SxFCR_FEIE;

	DMA_STREAM->CR |= DMA_SxCR_EN;

	return 0;
}

int stm32_dma_disable(struct dma_stream *dma_stream)
{
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)dma_stream->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma_stream);

	DMA_STREAM->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_EN);

	if (dma_stream->use_fifo)
		DMA_STREAM->FCR &= ~(DMA_SxFCR_FEIE);

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return 0;
}

int stm32_dma_stream_init(struct dma_stream *stream)
{
	int ret = 0;
	struct dma_controller *dma_ctrl = stream->dma;
	DMA_Stream_TypeDef *DMA_STREAM = (DMA_Stream_TypeDef *)stream->stream_base;

	if (stream->dir == DMA_M_M && !dma_ctrl->mem2mem) {
		debug_printk("dma controller does not support mem to mem transfer\r\n");
		return -EINVAL;
	}


	DMA_STREAM->CR = (stream->channel << 25) | (stream->mburst << 23)
			| (stream->pburst << 21) | (stream->priority) | (stream->mdata_size << 13)
			| (stream->pdata_size << 11) | (stream->minc)
			| (stream->pinc) | (stream->dir << 6);

	if (stream->use_fifo) {
		DMA_STREAM->FCR &= DMA_SxFCR_DMDIS;
		DMA_STREAM->FCR |= DMA_SxFCR_FTH;
	}

	ret = irq_request(stream->irq, stm32_dma_isr, stream);

	return ret;
}
