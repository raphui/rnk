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

#ifndef DMA_STM32_H
#define DMA_STM32_H

#include <dma.h>

int stm32_dma_transfer(struct dma_stream *dma, struct dma_transfer *dma_trans);
int stm32_dma_enable(struct dma_stream *dma);
int stm32_dma_disable(struct dma_stream *dma);
int stm32_dma_stream_of_configure(struct dma_stream *dma_stream, void (*handler)(struct device *dev), int fdt_offset);
int stm32_dma_stream_init(struct dma_stream *dma_stream);

#endif /* DMA_STM32_H */
