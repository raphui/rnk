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

#include <armv7m/thread.h>
#include <armv7m/system.h>
#include <string.h>

struct arch_sw_context_frame *current_ctx_frame;

void arch_create_context(struct arch_thread *arch, unsigned int func, unsigned int *stack, unsigned int param1, unsigned int param2)
{

	stack = (unsigned int *)((unsigned int)stack & 0xFFFFFFF8);

	/* HW frame */
	arch->hw_frame.r0 = param1;
	arch->hw_frame.r1 = param2;
	arch->hw_frame.r2 = 2;
	arch->hw_frame.r3 = 3;
	arch->hw_frame.r12 = 12;
	arch->hw_frame.lr = 0xFFFFFFFF;
	arch->hw_frame.pc = func | 1; 
	arch->hw_frame.xpsr = 0x01000000;

	stack -= sizeof(struct arch_short_context_frame) / sizeof(unsigned int);

	memcpy(stack, &arch->hw_frame, sizeof(struct arch_short_context_frame));

	/* SW frame */
	arch->ctx_frame.r4 = 4;
	arch->ctx_frame.r5 = 5;
	arch->ctx_frame.r6 = 6;
	arch->ctx_frame.r7 = 7;
	arch->ctx_frame.r8 = 8;
	arch->ctx_frame.r9 = 9;
	arch->ctx_frame.r10 = 10;
	arch->ctx_frame.r11 = 11;
	arch->ctx_frame.exc_lr = 0xFFFFFFFD;

	stack -= sizeof(struct arch_sw_context_frame) / sizeof(unsigned int);

	memcpy(stack, &arch->ctx_frame, sizeof(struct arch_sw_context_frame));

	arch->ctx_frame.sp = (unsigned int)stack;

	*stack = arch->ctx_frame.sp;
}

unsigned int arch_get_thread_stack(void)
{
	return PSP();
}

void arch_set_thread_stack(struct arch_thread *arch)
{
	//SET_PSP((void *)t->regs->sp);
}

void arch_switch_context(struct arch_thread *old, struct arch_thread *new)
{
	unsigned int *old_sp = arch_get_thread_stack();

	if (old) {

		*--old_sp = old_sp;

		memcpy(&old->ctx_frame, (void *)old_sp, sizeof(struct arch_sw_context_frame));

		old_sp += sizeof(struct arch_sw_context_frame);

		memcpy(&old->hw_frame, (void *)old_sp, sizeof(struct arch_short_context_frame));
	}

	current_ctx_frame = &new->ctx_frame;
}