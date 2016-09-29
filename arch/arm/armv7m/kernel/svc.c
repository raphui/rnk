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

#include <stdio.h>
#include <scheduler.h>
#include <mutex.h>
#include <semaphore.h>
#include <time.h>
#include <queue.h>

#include <arch/svc.h>
#include <arch/system.h>

void arch_system_call(unsigned int call, void *arg1, void *arg2, void *arg3)
{
	switch (call) {
	case SVC_THREAD_SWITCH:
		debug_printk("System call ask for a thread switch\r\n");
		arch_request_sched();
		break;
	case SVC_ACQUIRE_MUTEX:
		debug_printk("System call ask for acquiring mutex\r\n");
		SVC_ARG(SVC_ACQUIRE_MUTEX, arg1);
		break;
	case SVC_RELEASE_MUTEX:
		debug_printk("System call ask for releasing mutex\r\n");
		SVC_ARG(SVC_RELEASE_MUTEX, arg1);
		break;
	case SVC_WAIT_SEM:
		debug_printk("System call ask for wait semaphore\r\n");
		SVC_ARG(SVC_WAIT_SEM, arg1);
		break;
	case SVC_POST_SEM:
		debug_printk("System call ask for post semaphore\r\n");
		SVC_ARG(SVC_POST_SEM, arg1);
		break;
	case SVC_USLEEP:
		debug_printk("System call ask for usleep\r\n");
		SVC_ARG(SVC_USLEEP, arg1);
		break;
	case SVC_QUEUE_POST:
		debug_printk("System call ask for post in queue\r\n");
		SVC_ARG2(SVC_QUEUE_POST, arg1, arg2);
		break;
	case SVC_QUEUE_RECEIVE:
		debug_printk("System call ask for receive from queue\r\n");
		SVC_ARG2(SVC_QUEUE_RECEIVE, arg1, arg2);
		break;
	default:
		debug_printk("Invalid system call\r\n");
		break;
	};

}
