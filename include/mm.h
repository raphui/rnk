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

#ifndef MM_H
#define MM_H

#include <stddef.h>

extern int heap_size;
extern int mem_alloc;
#define mem_available (heap_size - mem_alloc)

extern int heap_init(void);
extern void *kmalloc(size_t size);
extern void kfree(void *mem);
extern void umalloc(size_t size, void *m);
extern void ufree(void *mem);

#ifdef CONFIG_DLMALLOC
extern void *sbrk(int increment);
#endif /* CONFIG_DLMALLOC */


#endif /* MM_H */
