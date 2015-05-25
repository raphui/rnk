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

void svc_handler(unsigned int call, void *arg)
{
	unsigned int svc_number;
	unsigned int *psp = (unsigned int *)call;

	svc_number = ((char *)psp[6])[-2];

	debug_printk("svc_handler: got call %d with arg (%x)\r\n", svc_number, arg);

	switch (svc_number) {
	case SVC_TASK_SWITCH:
		debug_printk("SVC call ask for a task switch\r\n");
		schedule_task((struct task *)arg);
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
	default:
		debug_printk("Invalid svc call\r\n");
		break;
	}
}
