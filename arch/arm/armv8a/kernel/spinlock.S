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

#include <asm.h>

ENTRY(arch_spin_trylock)
	mov	x2, x0
	mov	x1, #1
	ldaxr	x0, [x2]
	cbnz	x0, 1f
	stxr	w0, x1, [x2]
1:
	ret
END(arch_spin_trylock)

ENTRY(arch_spin_lock)
	mov	x1, #1
	sevl
1:
	wfe
	ldaxr	x2, [x0]
	cbnz	x2, 1b
	stxr	w2, x1, [x0]
	cbnz	w2, 1b
	ret
END(arch_spin_lock)

ENTRY(arch_spin_unlock)
	stlr	xzr, [x0]
	ret
END(arch_spin_unlock)
