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

void stm32_dma_init(struct dma *dma);
void stm32_dma_transfer(struct dma *dma, struct dma_transfer *dma_trans);
void stm32_dma_enable(struct dma *dma);
void stm32_dma_disable(struct dma *dma);


#endif /* DMA_STM32_H */
