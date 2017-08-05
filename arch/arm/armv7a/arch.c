/*
 * Copyright (C) 2017 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <armv7a/system.h>
#include <armv7a/svc.h>

#include <string.h>
#include <thread.h>


extern void low_level_init(void);

unsigned char svc_stack[ARCH_STACK_SIZE] __attribute__((aligned(CACHE_LINE)));
unsigned char irq_stack[ARCH_STACK_SIZE] __attribute__((aligned(CACHE_LINE)));

extern int main(void);

void arch_init(void)
{
	memset(svc_stack, 0, ARCH_STACK_SIZE);
	memset(irq_stack, 0, ARCH_STACK_SIZE);

	arm_set_irq_stack(irq_stack + ARCH_STACK_SIZE);
	arm_set_stack(svc_stack + ARCH_STACK_SIZE);

	memset((void *)&__bss_start, 0, &__bss_stop - &__bss_start);

	low_level_init();

	__enable_it();

	main();
}

void arch_init_tick(void)
{
#ifdef CONFIG_MACH_HAVE_GENERIC_TIMER

#endif
}

void arch_request_sched(void)
{
	SVC(SVC_THREAD_SWITCH);
}

unsigned int arch_get_thread_stack(void)
{
	return arm_get_user_stack();
}

void arch_set_thread_stack(struct thread *t)
{
	arm_set_user_stack((void *)t->regs->sp);
}
