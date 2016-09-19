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

#include <scheduler.h>
#include <thread.h>
#include <stdio.h>
#include <pit.h>
#include <board.h>
#include <arch/svc.h>
#include <arch/system.h>
#include <time.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

int thread_switching = 0;
unsigned int system_tick = 0;

int schedule_init(void)
{
	int ret = 0;

	thread_init();

	return ret;
}
#ifdef CONFIG_INITCALL
core_initcall(schedule_init);
#endif /* CONFIG_INITCALL */

void start_schedule(void)
{
	arch_init_tick();
	arch_request_sched();
}

void schedule(void)
{
	struct thread *t;

	t = find_next_thread();
	switch_thread(t);
	thread_switching = 1;
}

void schedule_thread(struct thread *thread)
{
	struct thread *t;

	t = get_current_thread();
#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_PREEMPT)
	if (t)
		t->quantum--;
#endif /* CONFIG_SCHEDULE_ROUND_ROBIN */

	if (t && t->state != THREAD_BLOCKED) {
		t->regs->sp = arch_get_thread_stack();
		insert_runnable_thread(t);
	}

	if (thread)
		switch_thread(thread);
	else {
#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
		if (!t || !t->quantum || (t->state == THREAD_BLOCKED)) {
			t = find_next_thread();
			switch_thread(t);
		}
#elif defined(CONFIG_SCHEDULE_PRIORITY) || defined (CONFIG_SCHEDULE_PREEMPT)
		t = find_next_thread();
		switch_thread(t);
#endif
	}
	
	thread_switching = 1;

#if !defined(CONFIG_HR_TIMER) && !defined(CONFIG_BW_DELAY)
	decrease_thread_delay();
#endif
}

/* Since threads cannot end, if we jump into this functions it's mean that the context switch is buggy */
void end_thread(void)
{
	struct thread *thread = get_current_thread();

	thread->state = THREAD_STOPPED;

	remove_runnable_thread(thread);
}
