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

#include <arch/svc.h>

void svc_handler(unsigned int call, void *arg)
{
	unsigned int svc_number;

	printk("svc_handler: got call %d with arg (%x)\r\n", call, arg);

	svc_number = ((char *)call)[-2];

	switch (svc_number) {
	case SVC_TASK_SWITCH:
		printk("SVC call ask for a task switch\r\n");
		schedule_task((struct task *)arg);
		break;
	case SVC_ACQUIRE_MUTEX:
		printk("SVC call ask for acquiring mutex\r\n");
		mutex_lock(arg);
		break;
	case SVC_RELEASE_MUTEX:
		printk("SVC call ask for releasing mutex\r\n");
		mutex_unlock(arg);
		break;
	default:
		printk("Invalid svc call\r\n");
		break;
	}
}
