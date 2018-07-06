#ifndef DMA_STM32_H
#define DMA_STM32_H

#include <drv/dma.h>

int stm32_dma_transfer(struct dma_stream *dma, struct dma_transfer *dma_trans);
int stm32_dma_enable(struct dma_stream *dma);
int stm32_dma_disable(struct dma_stream *dma);
int stm32_dma_stream_of_configure(int fdt_offset, void (*handler)(void *arg), void *arg, struct dma_stream *dma_stream, int size);
int stm32_dma_stream_init(struct dma_stream *dma_stream);

#endif /* DMA_STM32_H */
