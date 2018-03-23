/*
 * Copyright (C) 2018 Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef ARMV7M_THREAD_H
#define ARMV7M_THREAD_H

struct arch_sw_context_frame {
	unsigned int sp;
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
	unsigned int exc_lr;
};

struct arch_short_context_frame {
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r12;
	unsigned int lr;
	unsigned int pc;
	unsigned int xpsr;
};

struct arch_thread {
	struct arch_sw_context_frame ctx_frame;
	struct arch_short_context_frame hw_frame;
};

void arch_create_context(struct arch_thread *arch, unsigned int func, unsigned int *stack, unsigned int param1, unsigned int param2);
void arch_switch_context(struct arch_thread *old, struct arch_thread *new);;

#endif /* ARMV7M_THREAD_H */
