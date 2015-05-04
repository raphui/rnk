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

#include <stdio.h>
#include "memory.h"

static void free_mem(void *mem, struct memory_block *heap, void *base)
{
	struct alloc_header *header = (struct alloc_header *)((unsigned int *)mem - sizeof(struct alloc_header));
	unsigned int chunks = header->chunks;
	unsigned int index_block = addr_to_block((void *)header, base);
	unsigned int block_to_free;
	int i;

	if (header->magic != MAGIC)
		debug_printk("Corrupted or invalid object to be free !\n");

	if (chunks < CHUNK_PER_BLOCK) {
		heap[index_block].free_mask &= ~(MASK(chunks) << addr_to_chunks_offset((void *)header, base));
		heap[index_block].free_chunks += chunks;
	} else {
		block_to_free = chunks / CHUNK_PER_BLOCK;
		chunks = chunks % CHUNK_PER_BLOCK;

		for (i = 0; i < block_to_free; i++) {
			heap[i].free_mask = 0;
			heap[i].free_chunks = CHUNK_PER_BLOCK;
		}

		heap[index_block + block_to_free].free_mask &= ~MASK(chunks);
		heap[index_block + block_to_free].free_chunks += chunks;
	}
}

void kfree(void *mem)
{
	free_mem(mem, kernel_heap, (void *)KERNEL_HEAP_START);
}
