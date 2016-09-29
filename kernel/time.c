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
#include <time.h>
#include <timer.h>
#include <arch/svc.h>
#include <stdio.h>
#include <scheduler.h>
#include <arch/system.h>
#include <spinlock.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

static struct list_node sleeping_threads;

struct timer timer;

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
#ifdef CONFIG_INITCALL
core_initcall(time_init);
#endif /* CONFIG_INITCALL */

void svc_usleep(struct timer *timer)
{
	struct thread *thread = NULL;

	thread_lock(state);

	thread = get_current_thread();
	thread->delay = timer->counter;

	thread->state = THREAD_BLOCKED;
	remove_runnable_thread(thread);

	list_add_tail(&sleeping_threads, &thread->node);

#ifdef CONFIG_HR_TIMER
//	timer_init(timer);
//	timer_set_rate(timer, 1000000);
//	timer_set_counter(timer, timer->counter);
//	timer_enable(timer);
#endif /* CONFIG_HR_TIMER */

	arch_system_call(SVC_THREAD_SWITCH, NULL, NULL, NULL);

	thread_unlock(state);
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
	arch_system_call(SVC_USLEEP, &timer, NULL, NULL);
#endif /* CONFIG_BW_DELAY */
}

void svc_timer_oneshot(int delay, void (*handler)(void *), void *arg)
{
	thread_lock(state);


	thread_unlock(state);
}

void decrease_thread_delay(void)
{
	struct thread *thread;
	struct thread *tmp;
	struct thread *curr = get_current_thread();

	thread_lock(state);

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, node) {
		if (thread->state == THREAD_RUNNABLE) {
			thread->delay = 0;
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);
#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < thread->priority)
				arch_system_call(SVC_THREAD_SWITCH, NULL, NULL, NULL);
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
				arch_system_call(SVC_THREAD_SWITCH, NULL, NULL, NULL);
#endif /* CONFIG_SCHEDULE_PRIORITY */
		} else {

			thread->delay--;
			verbose_printk("%d: %d usec remaining\r\n", thread->pid, thread->delay);
		}
	}

	thread_unlock(state);
}
