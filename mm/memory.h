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

#ifndef MEMORY_H
#define MEMORY_H

#define GRAIN_PER_BLOCK		32
#define GRAIN_SIZE		(1 << 4)
#define MAGIC			0xABCD
#define BLOCK_SIZE		(GRAIN_PER_BLOCK * GRAIN_SIZE)
#define KERNEL_NUM_BLOCKS	((0x230000 - 0x220000) / BLOCK_SIZE)
#define MAX_KERNEL_SIZE		(0x230000 - 0x220000)

#define NULL			((void *)0)

struct alloc_header
{
	unsigned int magic;
	unsigned int grains;
};

struct memory_block
{
	unsigned int free_grains;
};

struct memory_block kernel_heap[KERNEL_NUM_BLOCKS];




#endif /* MEMORY_H */
