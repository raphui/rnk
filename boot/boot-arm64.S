/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <asm/linkage.h>

ENTRY(start)
	adr	x0, vectors
	mrs	x1, CurrentEL
	cmp	x1, #0xC		/* Check EL3 state */
	b.eq	1f
	cmp	x1, #0x8		/* Check EL2 state */
	b.eq	2f
	cmp	x1, #0x4		/* Check EL1 state */
	b.eq	3f

1:
	msr	vbar_el3, x0
	mov	x0, #1			/* Non-Secure EL0/1 */
	orr	x0, x0, #(1 << 10)	/* 64-bit EL2 */
	msr	scr_el3, x0
	msr	cptr_el3, xzr
	b	done

2:
	msr	vbar_el2, x0
	mov	x0, #0x33ff		/* Enable FP/SIMD */
	msr	cptr_el2, x0
	b	done


3:
	msr	vbar_el1, x0
	mov	x0, #(3 << 20)		/* Enable FP/SIMD */
	msr	cpacr_el1, x0
	b	done

done:
	bl low_level_init
	b main

END(start)
