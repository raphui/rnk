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

#include "memory.h"
#include <init.h>

int heap_size = MAX_KERNEL_HEAP_SIZE;

int heap_init(void)
{
	int ret = 0;
#ifdef CONFIG_CUSTOM_MALLOC
	int i;

	for (i = 0; i < KERNEL_NUM_BLOCKS; i++) {
		kernel_heap[i].free_chunks = CHUNK_PER_BLOCK;
		kernel_heap[i].free_mask = 0;
	}
#elif defined(CONFIG_TLSF)
	tlsf_mem_kernel_pool = tlsf_create_with_pool((void *)KERNEL_HEAP_START, MAX_KERNEL_HEAP_SIZE);
	tlsf_mem_user_pool = tlsf_create_with_pool((void *)USER_HEAP_START, MAX_USER_HEAP_SIZE);
#endif

	return ret;
}
pure_initcall(heap_init);
