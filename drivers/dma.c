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

void dma_init(struct dma *dma)
{
	dma_ops.init(dma);
}

void dma_transfer(struct dma *dma, struct dma_transfer *dma_trans)
{
	dma_ops.transfer(dma, dma_trans);
}

void dma_enable(struct dma *dma)
{
	dma_ops.enable(dma);
}

void dma_disable(struct dma *dma)
{
	dma_ops.disable(dma);
}