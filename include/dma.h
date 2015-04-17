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

#ifndef DMA_H
#define DMA_H

#define INCR0	0x0
#define INCR4	0x1
#define INCR8	0x2
#define INCR16	0x3

#define DATA_SIZE_BYTE		0x0
#define DATA_SIZE_HALF_WORD	0x0
#define DATA_SIZE_WORD		0x0

#define DMA_P_M		0x0
#define DMA_M_P		0x1
#define DMA_M_M		0x2

struct dma {
	unsigned char num;
	unsigned int stream_base;
	unsigned char stream_num;
	unsigned char channel;
	unsigned char dir;
	unsigned char mdata_size;
	unsigned char pdata_size;
	unsigned char mburst;
	unsigned char pburst;
	unsigned char inc_mode;	
	unsigned char use_fifo;
};

struct dma_transfer {
	unsigned int src_addr;
	unsigned int dest_addr;
	unsigned short size;
};

void dma_init(struct dma *dma);
void dma_transfer(struct dma *dma, struct dma_transfer *dma_trans);
void dma_enable(struct dma *dma);
void dma_disable(struct dma *dma);

#endif /* DMA_H */
