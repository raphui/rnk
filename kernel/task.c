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
#include <string.h>
#include <mm.h>
#include <utils.h>
#include <arch/svc.h>
#include <armv7m/system.h>
#include <spinlock.h>

static struct task *current_task = NULL;
static int task_count = 0;

#ifdef CONFIG_SCHEDULE_PREEMPT
#define NB_RUN_QUEUE		32
#define MAX_PRIORITIES		32
#define HIGHEST_PRIORITY	(MAX_PRIORITIES - 1)
#else
#define NB_RUN_QUEUE	1
#endif /* CONFIG_SCHEDULE_PREEMPT */

static unsigned int run_queue_bitmap;
static struct list_node run_queue[NB_RUN_QUEUE];

unsigned long task_lock = SPIN_LOCK_INITIAL_VALUE;

static void idle_task(void)
{
	while(1)
		wait_for_interrupt();
}

static void insert_in_run_queue_head(struct task *t)
{
	list_add_head(&run_queue[t->priority], &t->node);
	run_queue_bitmap |= (1 << t->priority);
}

static void insert_in_run_queue_tail(struct task *t)
{
	list_add_tail(&run_queue[t->priority], &t->node);
	run_queue_bitmap |= (1 << t->priority);
}

static void insert_task(struct task *t)
{
	struct task *task = NULL;

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	insert_in_run_queue_tail(t);
	verbose_printk("inserting task %d\r\n", t->pid);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	if (list_is_empty(&run_queue[0])) {
		list_add_head(&run_queue[0], &t->node);
	} else {
		list_for_every_entry(&run_queue[0], task, struct task, node) {
			verbose_printk("t->priority: %d, task->priority; %d\r\n", t->priority, task->priority);
			if (t->priority > task->priority) {
				verbose_printk("inserting task %d\r\n", t->pid);
				list_add_before(&task->node, &t->node);
				break;
			}
		}
	}
#elif defined(CONFIG_SCHEDULE_PREEMPT)
	if (task->quantum > 0)
		insert_in_run_queue_head(task);
	else {
		task->quantum = CONFIG_TASK_QUANTUM;
		insert_in_run_queue_tail(task);
	}
#endif
}

void task_init(void)
{
	int i;

	for (i = 0; i < NB_RUN_QUEUE; i++)
		list_initialize(&run_queue[i]);

	add_task(&idle_task, 0);
}

void add_task(void (*func)(void), unsigned int priority)
{
	struct task *task = (struct task *)kmalloc(sizeof(struct task));

	memset(task, 0, sizeof(struct task));

	task->state = TASK_RUNNABLE;
	task->pid = task_count;

#ifdef CONFIG_SCHEDULE_PRIORITY
	task->priority = priority;
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	task->quantum = CONFIG_TASK_QUANTUM;
#elif defined(CONFIG_SCHEDULE_PREEMPT)
	task->priority = priority;
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

	insert_task(task);

	task_count++;
}

void switch_task(struct task *task)
{
	task->state = TASK_RUNNING;
	arch_set_task_stack(task);
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

#ifdef CONFIG_SCHEDULE_PREEMPT
	unsigned int bitmap = run_queue_bitmap;
	unsigned int next_queue;

	while (bitmap)	{
		next_queue = HIGHEST_PRIORITY - __builtin_clz(bitmap) 
			- (sizeof(run_queue_bitmap) * 8 - MAX_PRIORITIES);

		debug_printk("next_queue: %d\n", next_queue);

		list_for_every_entry(&run_queue[next_queue], task, struct task, node) {

			debug_printk("next task: %d\n", task->pid);
			if (list_is_empty(&run_queue[next_queue]))
				run_queue_bitmap &= ~(1 << next_queue);

			return task;
		}

		bitmap &= ~(1 << next_queue);
	}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	task = list_peek_head_type(&run_queue[0], struct task, node);

	if (current_task && current_task->state != TASK_BLOCKED)
		if (task->priority < current_task->priority)
			task = current_task;
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	list_for_every_entry(&run_queue[0], task, struct task, node)
		if ((task->quantum > 0) && (task->pid != 0))
			break;

	if (current_task)
		current_task->quantum = CONFIG_TASK_QUANTUM;

	/* Only idle task is eligible */
	if (!task) {
		task = list_peek_head_type(&run_queue[0], struct task, node);
	}
#endif

	verbose_printk("next task: %d\r\n", task->pid);

	return task;
}

void insert_runnable_task(struct task *task)
{
	insert_task(task);
	task->state = TASK_RUNNABLE;
}

void remove_runnable_task(struct task *task)
{
	task->regs->sp = arch_get_task_stack();

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	current_task->quantum = CONFIG_TASK_QUANTUM;
#endif

	list_delete(&task->node);
}
