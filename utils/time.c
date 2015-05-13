/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <task.h>
#include <time.h>
#include <timer.h>
#include <arch/svc.h>
#include <stdio.h>

LIST_HEAD(, task) sleeping_tasks = LIST_HEAD_INITIALIZER(sleeping_tasks);

struct timer timer;

static void remove_sleeping_task(struct task *task)
{
	LIST_REMOVE(task, next);
}

void time_init(void)
{
	LIST_INIT(&sleeping_tasks);
}

void usleep(unsigned int usec)
{
	struct task *first = NULL;
	struct task *task = NULL;

	task = get_current_task();
	task->delay = usec;

	remove_runnable_task(task);
	task->state = TASK_BLOCKED;

	if (LIST_EMPTY(&sleeping_tasks))
		LIST_INSERT_HEAD(&sleeping_tasks, task, next);
	else {
		first = LIST_FIRST(&sleeping_tasks);
		LIST_INSERT_AFTER(first, task, next);
	}

	timer.num = 2;
	timer.one_pulse = 1;
	timer.count_up = 0;

	timer_init(&timer);
	timer_set_rate(&timer, 1000000);
	timer_set_counter(&timer, usec);
	timer_enable(&timer);

	SVC_ARG(SVC_TASK_SWITCH, NULL);
}

void decrease_task_delay(void)
{
	struct task *task;

	LIST_FOREACH(task, &sleeping_tasks, next) {
		task->delay--;
		printk("%d: %d usec remaining\r\n", task->pid, task->delay);
		if (!task->delay) {
			remove_sleeping_task(task);
			SVC_ARG(SVC_TASK_SWITCH, task);
		}
	}
}
