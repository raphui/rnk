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
#include <stdio.h>
#include <mm.h>
#include <utils.h>
#include <arch/svc.h>
#include <armv7m/system.h>

static struct task *current_task = NULL;
static int task_count = 0;
LIST_HEAD(, task) runnable_tasks = LIST_HEAD_INITIALIZER(runnable_tasks);

static void idle_task(void)
{
	while(1)
		;
}

static void insert_task(struct task *t)
{
	struct task *task;

	task = LIST_FIRST(&runnable_tasks);

	if (!task_count) {
		debug_printk("runnable list is empty\r\n");
		LIST_INSERT_HEAD(&runnable_tasks, task, next);
		return;
	}

	LIST_FOREACH(task, &runnable_tasks, next) {

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
		if (!LIST_NEXT(task, next)) {
			debug_printk("inserting task %d\r\n", t->pid);
			LIST_INSERT_AFTER(task, t, next);
			break;
		}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
		debug_printk("t->priority: %d, task->priority; %d\r\n", t->priority, task->priority);
		if (t->priority > task->priority) {
			debug_printk("inserting task %d\r\n", t->pid);
			LIST_INSERT_BEFORE(task, t, next);
			break;
		}
#endif
	}
}

void task_init(void)
{
	LIST_INIT(&runnable_tasks);
	add_task(&idle_task, 0);
}

void add_task(void (*func)(void), unsigned int priority)
{
	struct task *task = (struct task *)kmalloc(sizeof(struct task));
	task->state = TASK_RUNNABLE;
	task->pid = task_count;

#ifdef CONFIG_SCHEDULE_PRIORITY
	task->priority = priority;
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	task->quantum = CONFIG_TASK_QUANTUM;
#endif

	task->start_stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task->delay = 0;
	task->func = func;
	task->regs = (struct registers *)kmalloc(sizeof(struct registers));
	task->regs->sp = task->start_stack;
	task->regs->lr = (unsigned int)end_task;
	task->regs->pc = (unsigned int)func;

	/* Creating task context */
	create_context(task->regs, task);

	if (task_count)
		insert_task(task);
	else
		LIST_INSERT_HEAD(&runnable_tasks, task, next);

	task_count++;
}

void first_switch_task(void)
{
	SVC_ARG(SVC_TASK_SWITCH, NULL);
}

void switch_task(struct task *task)
{
	if (current_task && (current_task->state != TASK_BLOCKED)) {
		current_task->regs->sp = PSP();
		insert_task(current_task);
	}

	task->state = TASK_RUNNING;
	SET_PSP((void *)task->regs->sp);
	current_task = task;

	if (task->pid != 0)
		remove_runnable_task(task);
}

struct task *get_current_task(void)
{
	return current_task;
}

struct task *find_next_task(void)
{
	struct task *task = NULL;

#ifdef CONFIG_SCHEDULE_PRIORITY
	task = LIST_FIRST(&runnable_tasks);
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	LIST_FOREACH(task, &runnable_tasks, next)
		if ((task->quantum > 0) && (task->pid != 0))
			break;

	if (current_task)
		current_task->quantum = TASK_QUANTUM;

	/* Only idle task is eligible */
	if (!task)
		task = LIST_FIRST(&runnable_tasks);
#endif

	debug_printk("next task: %d\r\n", task->pid);

	return task;
}

void insert_runnable_task(struct task *task)
{
	insert_task(task);
	task->state = TASK_RUNNABLE;
}

void remove_runnable_task(struct task *task)
{
	task->regs->sp = PSP();

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	current_task->quantum = TASK_QUANTUM;
#endif

	LIST_REMOVE(task, next);
}
