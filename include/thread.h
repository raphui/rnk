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

#ifndef THREAD_H
#define THREAD_H

#include <list.h>
#include <stddef.h>
#include <arch/thread.h>

#define THREAD_RUNNING		0
#define THREAD_RUNNABLE		1
#define THREAD_STOPPED		2
#define THREAD_INTERRUPTIBLE	3
#define THREAD_BLOCKED		4

#define THREAD_STACK_START	CONFIG_THREAD_STACK_START
#define THREAD_STACK_OFFSET	CONFIG_THREAD_STACK_SIZE

#define NUM_PRIORITIES		32
#define MAX_PRIORITIES		32
#define LOWEST_PRIORITY		0
#define HIGHEST_PRIORITY	(NUM_PRIORITIES - 1)
#define DPC_PRIORITY		(NUM_PRIORITIES - 2)
#define IDLE_PRIORITY		LOWEST_PRIORITY
#define LOW_PRIORITY		(NUM_PRIORITIES / 4)
#define DEFAULT_PRIORITY	(NUM_PRIORITIES / 2)
#define HIGH_PRIORITY		((NUM_PRIORITIES / 4) * 3)

#ifdef CONFIG_SCHEDULE_RR_PRIO
#define NB_RUN_QUEUE	32
#else
#define NB_RUN_QUEUE	1
#endif /* CONFIG_SCHEDULE_RR_PRIO */

extern unsigned long thread_lock;

struct thread
{
	unsigned int state;
	unsigned int pid;
	unsigned int priority;
	unsigned int quantum;
	unsigned int start_stack;
	unsigned int delay;
	void (*func)(void);
	struct arch_thread *arch;
	struct list_node node;
	struct list_node event_node;
};

void thread_init(void);
void add_thread(void (*func)(void), unsigned int priority);
void switch_thread(struct thread *thread);
struct thread *get_current_thread(void);
struct thread *find_next_thread(void);
void insert_runnable_thread(struct thread *thread);
void remove_runnable_thread(struct thread *thread);

#endif /* THREAD_H */
