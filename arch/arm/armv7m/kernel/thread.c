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

#include <armv7m/mpu.h>
#include <armv7m/thread.h>
#include <armv7m/system.h>
#include <string.h>


struct arch_sw_context_frame *current_ctx_frame;

static struct arch_thread *current_thread_frame;

int *current_thread_mode;

void arch_create_context(struct arch_thread *arch, unsigned int func, unsigned int return_func, unsigned int *stack, unsigned int param1, unsigned int param2)
{
	stack = (unsigned int *)ALIGN((unsigned int)stack, 8);

	arch->mpu.top_sp = (unsigned int)stack;

	/* HW frame */
	arch->hw_frame.r0 = param1;
	arch->hw_frame.r1 = param2;
	arch->hw_frame.r2 = 2;
	arch->hw_frame.r3 = 3;
	arch->hw_frame.r12 = 12;
	arch->hw_frame.lr = return_func | 1;
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

	arch->mpu.start_pc = func | 1;
}

static unsigned int arch_get_thread_stack(void)
{
	return PSP();
}

void arch_switch_context(struct arch_thread *old, struct arch_thread *new)
{
	unsigned int *old_sp = (unsigned int *)arch_get_thread_stack();

	if (old) {

		old_sp--;
	        *old_sp = (unsigned int)old_sp;

		memcpy(&old->ctx_frame, (void *)old_sp, sizeof(struct arch_sw_context_frame));

		old_sp += sizeof(struct arch_sw_context_frame) / sizeof(unsigned int);

		memcpy(&old->hw_frame, (void *)old_sp, sizeof(struct arch_short_context_frame));

		mpu_unmap_prio(old->mpu.prio);
	}

	current_ctx_frame = &new->ctx_frame;

	new->mpu.prio = mpu_map_from_high((void *)(new->mpu.top_sp - CONFIG_THREAD_STACK_SIZE), CONFIG_THREAD_STACK_SIZE, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);

	current_thread_frame = new;
	current_thread_mode = &new->privileged;
}

void arch_request_sched(void)
{
	pendsv_request();
}

void arch_thread_set_return(void *ret)
{
	struct arch_short_context_frame *frame;

	/* minus 4 because SP is not push on PSP at the stage */
	frame = (struct arch_short_context_frame *)(arch_get_thread_stack() + sizeof(struct arch_sw_context_frame) - 4);

	frame->r0 = (unsigned int)ret;
}

void arch_thread_switch_unpriv(void)
{
	asm volatile (
		"mrs r1, control\n"
		"orr r1, r1, #1\n"
		"msr control, r1\n"
		:::);

	*current_thread_mode = ARCH_TREAD_UNPRIVILEGED;
}

void arch_thread_switch_priv(void)
{
	asm volatile (
		"mrs r1, control\n"
		"bic r1, r1, #1\n"
		"msr control, r1\n"
		:::);

	*current_thread_mode = ARCH_TREAD_PRIVILEGED;
}
