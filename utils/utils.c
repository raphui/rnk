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

#include <utils.h>

void writel(unsigned int reg, unsigned int val)
{
	unsigned int *p = (unsigned int *)reg;

	asm volatile("str %[val], [%[reg]]"
			: : [reg]"r"(p), [val]"r"(val));
}


unsigned int readl(unsigned int reg)
{
	unsigned int *p = (unsigned int *)reg;
	unsigned int val;

	asm volatile("ldr %[val], [%[reg]]"
			: [val]"=r"(val) : [reg]"r"(p));

	return val;
}

unsigned int next_power_of_2(unsigned int n)
{
	n--;
	n |= n >> 1;
	n |= n >> 2;
	n |= n >> 4;
	n |= n >> 8;
	n |= n >> 16;
	n++;
	return n;
}
