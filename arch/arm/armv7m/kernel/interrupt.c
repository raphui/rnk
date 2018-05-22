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

#include <board.h>
#include <printk.h>
#include <utils.h>
#include <interrupt.h>
#include <scheduler.h>
#include <syscall.h>
#include <armv7m/system.h>
#include <armv7m/thread.h>
#include <arch/system.h>
#include <symbols.h>
#include <backtrace.h>

static void dump_stack(unsigned int *stack)
{
	volatile unsigned int lr;
	volatile unsigned int pc;

	lr = stack[5];
	pc = stack[6];

#ifdef CONFIG_UNWIND
	/* fp = 0, because we don't care about it */
	unwind_backtrace(0, (unsigned int)stack, lr, pc);
#endif /* CONFIG_UNWIND */

	while (1)
		;
}


void hardfault_handler(void)
{
	asm volatile (
		"tst lr, #4			\n"
		"ite eq				\n"
		"mrseq r0, msp			\n"
		"mrsne r0, psp			\n"
//		"ldr r1, [r0, #24]		\n"
//		"ldr r2, _dump_stack		\n"
		"b dump_stack			\n"
//		"_dump_stack: .word dump_stack	\n"
	);	

	while (1)
		;
}

void memmanage_handler(void)
{
	while (1)
		;
}

void busfault_handler(void)
{
	while (1)
		;
}

void usagefault_handler(void)
{
	while (1)
		;
}

void systick_handler(void)
{
	system_tick++;

	arch_request_sched();
}

void pendsv_handler(void)
{
	schedule_thread(NULL);
}

int svc_handler(unsigned int svc_number, void *arg1, void *arg2, void *arg3)
{
	void *ret;
	void * (*handler)(void *, void *, void *);
	unsigned char type;

	debug_printk("svc_handler: got call %d\r\n", svc_number);

	handler = (void (*))syscall_table[svc_number].handler;
	type = syscall_table[svc_number].type;

	ret = (*handler)(arg1, arg2, arg3);

	arch_thread_set_return(ret);

	if (type == SYSCALL_PRIVILEGE_ELEVATION)
		arch_thread_switch_unpriv();

	return ret;
}
