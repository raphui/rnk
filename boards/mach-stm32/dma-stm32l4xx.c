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

#define DMA_ISR_TCIF	0x2222222
#define DMA_ISR_TEIF	0x8888888

static inline unsigned int stm32_dma_get_base(struct dma_stream *dma_stream)
{
	return dma_stream->dma->base_reg;
}

static int stm32_dma_get_nvic_number(struct dma_stream *dma_stream)
{

	return dma_stream->dma->interrupts[dma_stream->stream_num - 1];
}

static void stm32_dma_isr(void *arg)
{
	struct dma_stream *stream = (struct dma_stream *)arg;
	unsigned int base = stm32_dma_get_base(stream);
	DMA_TypeDef *dma_base = (DMA_TypeDef *)base;
	DMA_Channel_TypeDef *dma_stream = (DMA_Channel_TypeDef *)stream->stream_base;
	int flags;
	int mask = 0xA << (4 * (stream->stream_num - 1));

	flags = dma_base->ISR & mask;

	debug_printk("dma->sr: 0x%x\n", flags);
	debug_printk("dma->ndtr: 0x%x\n", dma_stream->CNDTR);

	dma_base->IFCR = flags;

	if (flags & DMA_ISR_TCIF)
		stream->handler(stream->arg);
}


int stm32_dma_transfer(struct dma_stream *dma_stream, struct dma_transfer *dma_trans)
{
	int ret = 0;
	DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef *)dma_stream->stream_base;

	if (dma_trans->size > MAX_DMA_SIZE) {
		error_printk("invalid dma transfer size\r\n");
		return -EINVAL;
	}
	else
		dma->CNDTR = dma_trans->size;

	if (dma_stream->dir == DMA_P_M) {
		dma->CPAR = dma_trans->src_addr;
		dma->CMAR = dma_trans->dest_addr;

	} else if (dma_stream->dir == DMA_M_P) {
		dma->CMAR = dma_trans->src_addr;
		dma->CPAR = dma_trans->dest_addr;

	} else if (dma_stream->dir == DMA_M_M) {
		dma->CPAR = dma_trans->src_addr;
		dma->CMAR = dma_trans->dest_addr;
	}

	return ret;
}

int stm32_dma_enable(struct dma_stream *dma_stream)
{
	int ret = 0;
	DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef *)dma_stream->stream_base;
	DMA_TypeDef *DMA_BASE;

	unsigned int base = stm32_dma_get_base(dma_stream);
	int nvic = stm32_dma_get_nvic_number(dma_stream);

	if (base < 0) {
		error_printk("invalid dma num\r\n");
		return -EINVAL;
	}

	DMA_BASE = (DMA_TypeDef *)base;

	DMA_BASE->IFCR = DMA_ISR_TCIF | DMA_ISR_TEIF;

	nvic_enable_interrupt(nvic);

	if (dma_stream->enable_interrupt)
		dma->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

	dma->CCR |= DMA_CCR_EN;

	return ret;
}

int stm32_dma_disable(struct dma_stream *dma_stream)
{
	int ret = 0;
	DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef *)dma_stream->stream_base;
	int nvic = stm32_dma_get_nvic_number(dma_stream);

	dma->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_EN);

	nvic_clear_interrupt(nvic);
	nvic_disable_interrupt(nvic);

	return ret;
}

int stm32_dma_stream_init(struct dma_stream *stream)
{
	int ret = 0;
	struct dma_controller *dma_ctrl = stream->dma;
	DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef *)stream->stream_base;
	unsigned int base = stm32_dma_get_base(stream);

	if (stream->dir == DMA_M_M && !dma_ctrl->mem2mem) {
		debug_printk("dma controller does not support mem to mem transfer\r\n");
		return -EINVAL;
	}

	if (stream->dir == DMA_M_M) {
		dma->CCR |= DMA_CCR_MEM2MEM;
	}


	dma->CCR |= stream->dir << DMA_CCR_DIR_Pos;
	dma->CCR |= stream->pinc << DMA_CCR_PINC_Pos;
	dma->CCR |= stream->minc << DMA_CCR_MINC_Pos;
	dma->CCR |= stream->pdata_size << DMA_CCR_PSIZE_Pos;
	dma->CCR |= stream->mdata_size << DMA_CCR_MSIZE_Pos;
	dma->CCR |= stream->priority << DMA_CCR_PL_Pos;

	/* channel selection */
	*((volatile unsigned int *)(base + 0xA8)) |= (stream->channel << (4 * (stream->stream_num - 1)));

	ret = irq_request(stream->irq, stm32_dma_isr, stream);

	return ret;
}
