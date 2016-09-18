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
#include <task.h>
#include <stdio.h>
#include <pit.h>
#include <board.h>
#include <arch/svc.h>
#include <arch/system.h>
#include <time.h>

#ifdef CONFIG_INITCALL
#include <init.h>
#endif /* CONFIG_INITCALL */

int task_switching = 0;
unsigned int system_tick = 0;

int schedule_init(void)
{
	int ret = 0;

	task_init();

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
	struct task *t;

	t = find_next_task();
	switch_task(t);
	task_switching = 1;
}

void schedule_task(struct task *task)
{
	struct task *t;

#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_PREEMPT)
	t = get_current_task();
	if (t)
		t->quantum--;
#endif /* CONFIG_SCHEDULE_ROUND_ROBIN */

	if (t && t->state != TASK_BLOCKED)
		insert_runnable_task(t);

	if (task)
		switch_task(task);
	else {
#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
		if (!t || !t->quantum || (t->state == TASK_BLOCKED)) {
			t = find_next_task();
			switch_task(t);
		}
#elif defined(CONFIG_SCHEDULE_PRIORITY) || defined (CONFIG_SCHEDULE_PREEMPT)
		t = find_next_task();
		switch_task(t);
#endif
	}
	
	task_switching = 1;

#if !defined(CONFIG_HR_TIMER) && !defined(CONFIG_BW_DELAY)
	decrease_task_delay();
#endif
}

/* Since tasks cannot end, if we jump into this functions it's mean that the context switch is buggy */
void end_task(void)
{
	struct task *task = get_current_task();

	task->state = TASK_STOPPED;

	remove_runnable_task(task);
}
