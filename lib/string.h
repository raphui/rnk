/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef STRING_H
#define STRING_H

#include <stddef.h>

void memcpy(void *dst, const void *src, unsigned int num);
void *memset(void *s, int c, unsigned int count);
int strcmp(const char *cs, const char *ct);
void *memchr(const void *ptr, int value, size_t num);
int memcmp(const void *ptr1, const void *ptr2, size_t num);
void memmove(void *dst, const void *src, size_t n);
size_t strlen(const char *s);

#endif /* STRING_H */
