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

#include <thread.h>
#include <ktime.h>
#include <timer.h>
#include <syscall.h>
#include <stdio.h>
#include <scheduler.h>
#include <arch/system.h>
#include <spinlock.h>
#include <init.h>
#include <export.h>

static struct list_node sleeping_threads;

static void remove_sleeping_thread(struct thread *thread)
{
	list_delete(&thread->node);
}

int time_init(void)
{
	int ret = 0;

	list_initialize(&sleeping_threads);

	return ret;
}
core_initcall(time_init);

void ktime_usleep(unsigned int usec)
{
	struct thread *thread = NULL;
	struct timer timer;

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

	thread = get_current_thread();
	thread->delay = timer.counter;

	thread->state = THREAD_BLOCKED;
	remove_runnable_thread(thread);

	list_add_tail(&sleeping_threads, &thread->node);

	arch_request_sched();
#endif /* CONFIG_BW_DELAY */
}

void ktime_oneshot(int delay, void (*handler)(void *), void *arg)
{
	int ret;

	ret = timer_oneshot_soft(delay, handler, arg);
	if (ret < 0)
		error_printk("failed to create software timer oneshot\n");
}

void decrease_thread_delay(void)
{
	struct thread *thread;
	struct thread *tmp;
	struct thread *curr = get_current_thread();

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, node) {
		if (thread->state == THREAD_RUNNABLE) {
			thread->delay = 0;
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);
#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < thread->priority)
				arch_request_sched();
#endif /* CONFIG_SCHEDULE_PRIORITY */

		} else if (!thread->delay) {
			thread->state = THREAD_RUNNABLE;
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

#ifdef CONFIG_HR_TIMER
			if (list_is_empty(&sleeping_threads))
				timer_disable(&timer);
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < thread->priority)
				arch_request_sched();
#endif /* CONFIG_SCHEDULE_PRIORITY */
		} else {

			thread->delay--;
			verbose_printk("%d: %d usec remaining\r\n", thread->pid, thread->delay);
		}
	}
}

void decrease_timer_delay(void)
{
	timer_soft_decrease_delay();
}
