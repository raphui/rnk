#include "task.h"

void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count].state = TASK_STOPPED;
	task[task_count].counter = priority;
	task[task_count].stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task[task_count].func = func;

	task_count++;
}

void switch_task(struct task *task)
{
	if (current_task) {
		current_task.state = TASK_STOPPED;
		task[i].state = TASK_RUNNING;
		current_task = task[i];
	} else {
		task[i].state = TASK_RUNNING;
		current_task = task[i];
	}
}
