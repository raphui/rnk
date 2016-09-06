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
#include <armv7m/system.h>
#include <spinlock.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

static struct list_node sleeping_tasks;

struct timer timer;

static void remove_sleeping_task(struct task *task)
{
	list_delete(&task->node);
}

int time_init(void)
{
	int ret = 0;

	list_initialize(&sleeping_tasks);

	return ret;
}
#ifdef CONFIG_INITCALL
core_initcall(time_init);
#endif /* CONFIG_INITCALL */

void svc_usleep(struct timer *timer)
{
	struct task *first = NULL;
	struct task *task = NULL;

	task_lock(state);

	task = get_current_task();
	task->delay = timer->counter;

	task->state = TASK_BLOCKED;
	remove_runnable_task(task);

	list_add_tail(&sleeping_tasks, &task->node);

#ifdef CONFIG_HR_TIMER
	timer_init(timer);
	timer_set_rate(timer, 1000000);
	timer_set_counter(timer, timer->counter);
	timer_enable(timer);
#endif /* CONFIG_HR_TIMER */

	schedule_task(NULL);

	task_unlock(state);
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
		wait_for_interrupt();
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

	task_lock(state);

	list_for_every_entry(&sleeping_tasks, task, struct task, node) {
		if (task->state == TASK_RUNNABLE) {
			tmp = task;
			tmp->delay = 0;
			remove_sleeping_task(tmp);
			insert_runnable_task(tmp);
#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < tmp->priority)
				schedule_isr();
#endif /* CONFIG_SCHEDULE_PRIORITY */

		} else if (!task->delay) {
			tmp = task;
			tmp->state = TASK_RUNNABLE;
			remove_sleeping_task(tmp);
			insert_runnable_task(tmp);

#ifdef CONFIG_HR_TIMER
			if (list_is_empty(&sleeping_tasks))
				timer_disable(&timer);
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < tmp->priority)
				schedule_isr();
#endif /* CONFIG_SCHEDULE_PRIORITY */
		} else {

			task->delay--;
			verbose_printk("%d: %d usec remaining\r\n", task->pid, task->delay);
		}
	}

	task_unlock(state);
}
