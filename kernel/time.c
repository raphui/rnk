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
#include <scheduler.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

LIST_HEAD(, task) sleeping_tasks = LIST_HEAD_INITIALIZER(sleeping_tasks);

struct timer timer;

static void remove_sleeping_task(struct task *task)
{
	LIST_REMOVE(task, next);
}

int time_init(void)
{
	int ret = 0;

	LIST_INIT(&sleeping_tasks);

	return ret;
}
#ifdef CONFIG_INITCALL
core_initcall(time_init);
#endif /* CONFIG_INITCALL */

void svc_usleep(struct timer *timer)
{
	struct task *first = NULL;
	struct task *task = NULL;

	task = get_current_task();
	task->delay = timer->counter;

	task->state = TASK_BLOCKED;
	remove_runnable_task(task);

	if (LIST_EMPTY(&sleeping_tasks))
		LIST_INSERT_HEAD(&sleeping_tasks, task, next);
	else {
		first = LIST_FIRST(&sleeping_tasks);
		LIST_INSERT_AFTER(first, task, next);
	}

#ifdef CONFIG_HR_TIMER
	timer_init(timer);
	timer_set_rate(timer, 1000000);
	timer_set_counter(timer, timer->counter);
	timer_enable(timer);
#endif /* CONFIG_HR_TIMER */

	schedule_task(NULL);
}

void usleep(unsigned int usec)
{
#ifdef CONFIG_HR_TIMER
	timer.num = 2;
	timer.one_pulse = 1;
	timer.count_up = 0;
	timer.counter = usec;
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_BW_DELAY
	int end = system_tick + (usec / 1000);

	do {
		asm("wfi");
	} while (end > system_tick);
#else
	timer.counter = usec / 1000;
	SVC_ARG(SVC_USLEEP, &timer);
#endif /* CONFIG_BW_DELAY */
}

void decrease_task_delay(void)
{
	struct task *task;
	struct task *tmp;
	struct task *curr = get_current_task();

	task = LIST_FIRST(&sleeping_tasks);

	while (task) {
		if (!task->delay) {
			tmp = task;
			task = LIST_NEXT(task, next);
			tmp->state = TASK_RUNNABLE;
			remove_sleeping_task(tmp);
			insert_runnable_task(tmp);

#ifdef CONFIG_HR_TIMER
			if (LIST_EMPTY(&sleeping_tasks))
				timer_disable(&timer);
#endif /* CONFIG_HR_TIMER */

			if (curr->priority < tmp->priority)
				schedule_isr();
		} else {

			task->delay--;
			verbose_printk("%d: %d usec remaining\r\n", task->pid, task->delay);
			task = LIST_NEXT(task, next);
		}
	}
}
