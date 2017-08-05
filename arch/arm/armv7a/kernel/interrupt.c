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

#include <board.h>
#include <stdio.h>
#include <utils.h>
#include <interrupt.h>
#include <scheduler.h>
#include <arch/system.h>
#include <arch/svc.h>
#include <time.h>
#include <irq.h>
#include <common.h>
#include <queue.h>
#include <symbols.h>
#include <backtrace.h>


static void show_regs(struct pt_regs *regs)
{
#if 0
	unsigned long flags;
	const char *processor_modes[] = {
		"USER_26",	"FIQ_26",	"IRQ_26",	"SVC_26",
		"UK4_26",	"UK5_26",	"UK6_26",	"UK7_26",
		"UK8_26",	"UK9_26",	"UK10_26",	"UK11_26",
		"UK12_26",	"UK13_26",	"UK14_26",	"UK15_26",
		"USER_32",	"FIQ_32",	"IRQ_32",	"SVC_32",
		"UK4_32",	"UK5_32",	"UK6_32",	"ABT_32",
		"UK8_32",	"UK9_32",	"UK10_32",	"UND_32",
		"UK12_32",	"UK13_32",	"UK14_32",	"SYS_32",
	};

	flags = condition_codes (regs);

	printf ("pc : [<%08lx>]    lr : [<%08lx>]\n"
			"sp : %08lx  ip : %08lx  fp : %08lx\n",
			instruction_pointer (regs),
			regs->ARM_lr, regs->ARM_sp, regs->ARM_ip, regs->ARM_fp);
	printf ("r10: %08lx  r9 : %08lx  r8 : %08lx\n",
			regs->ARM_r10, regs->ARM_r9, regs->ARM_r8);
	printf ("r7 : %08lx  r6 : %08lx  r5 : %08lx  r4 : %08lx\n",
			regs->ARM_r7, regs->ARM_r6, regs->ARM_r5, regs->ARM_r4);
	printf ("r3 : %08lx  r2 : %08lx  r1 : %08lx  r0 : %08lx\n",
			regs->ARM_r3, regs->ARM_r2, regs->ARM_r1, regs->ARM_r0);
	printf ("Flags: %c%c%c%c",
			flags & PSR_N_BIT ? 'N' : 'n',
			flags & PSR_Z_BIT ? 'Z' : 'z',
			flags & PSR_C_BIT ? 'C' : 'c', flags & PSR_V_BIT ? 'V' : 'v');
	printf ("  IRQs %s  FIQs %s  Mode %s%s\n",
			interrupts_enabled (regs) ? "on" : "off",
			fast_interrupts_enabled (regs) ? "on" : "off",
			processor_modes[processor_mode (regs)],
			thumb_mode (regs) ? " (T)" : "");
#ifdef CONFIG_UNWIND
	unwind_backtrace(regs);
#endif
#endif
}

static void exception(struct pt_regs *pt_regs)
{
	show_regs(pt_regs);

	while(1)
		;
}

void undefined_instruction(struct pt_regs *pt_regs)
{
	exception(pt_regs);
}

void software_interrupt(unsigned int svc_number, unsigned int *arg)
{
	__disable_it();

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
		svc_queue_post((struct queue *)arg[1], (void *)arg[2]);
		break;
	case SVC_QUEUE_RECEIVE:
		debug_printk("SVC call ask for receive from queue\r\n");
		svc_queue_receive((struct queue *)arg[1], (void *)arg[2]);
		break;
	case SVC_TIMER_ONESHOT:
		debug_printk("SVC call ask for oneshot timer\r\n");
		svc_timer_soft_oneshot(*(int *)arg[1], (void (*)(void *))arg[2], (void *)arg[3]);
		break;
	default:
		debug_printk("Invalid svc call\r\n");
		break;
	}

	__enable_it();
}

void prefetch_abort(struct pt_regs *pt_regs)
{
	exception(pt_regs);
}

void data_abort(struct pt_regs *pt_regs)
{
	exception(pt_regs);
}

void irq_interrupt(struct pt_regs *pt_regs)
{

	exception(pt_regs);
}

void fiq_interrupt(struct pt_regs *pt_regs)
{
	exception(pt_regs);
}
