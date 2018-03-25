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
#include <stdio.h>
#include <utils.h>
#include <interrupt.h>
#include <scheduler.h>
#include <armv7m/system.h>
#include <arch/nvic.h>
#include <arch/system.h>
#include <arch/svc.h>
#include <time.h>
#include <pio.h>
#include <irq.h>
#include <common.h>
#include <queue.h>
#include <symbols.h>
#include <backtrace.h>
#include <elfloader.h>

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

void svc_handler(unsigned int call, void *arg)
{
	unsigned int svc_number;
	unsigned int *psp = (unsigned int *)call;

	svc_number = ((char *)psp[6])[-2];

	debug_printk("svc_handler: got call %d with arg (%x)\r\n", svc_number, arg);

	switch (svc_number) {
		schedule_thread((struct thread *)arg);
		break;
	case SVC_ACQUIRE_MUTEX:
		debug_printk("SVC call ask for acquiring mutex\r\n");
		svc_mutex_lock((struct mutex *)arg);
		break;
	case SVC_RELEASE_MUTEX:
		debug_printk("SVC call ask for releasing mutex\r\n");
		svc_mutex_unlock((struct mutex *)arg);
		break;
	case SVC_WAIT_SEM:
		debug_printk("SVC call ask for wait semaphore\r\n");
		svc_sem_wait((struct semaphore *)arg);
		break;
	case SVC_POST_SEM:
		debug_printk("SVC call ask for post semaphore\r\n");
		svc_sem_post((struct semaphore *)arg);
		break;
	case SVC_USLEEP:
		debug_printk("SVC call ask for usleep\r\n");
		svc_usleep((struct timer *)arg);
		break;
	case SVC_QUEUE_POST:
		debug_printk("SVC call ask for post in queue\r\n");
		svc_queue_post((struct queue *)psp[1], (void *)psp[2]);
		break;
	case SVC_QUEUE_RECEIVE:
		debug_printk("SVC call ask for receive from queue\r\n");
		svc_queue_receive((struct queue *)psp[1], (void *)psp[2]);
		break;
	case SVC_TIMER_ONESHOT:
		debug_printk("SVC call ask for oneshot timer\r\n");
		svc_timer_soft_oneshot(*(int *)psp[1], (void (*)(void *))psp[2], (void *)psp[3]);
		break;
	case SVC_ELF_LOAD:
		debug_printk("SVC call ask for elf load\r\n");
		svc_elf_exec(*(int *)psp[1], (void (*)(void *))psp[2], (void *)psp[3]);
		break;
	default:
		debug_printk("Invalid svc call\r\n");
		break;
	}
}
