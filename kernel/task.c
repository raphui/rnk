#include <task.h>

static struct task *current_task = NULL;

void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count].state = TASK_STOPPED;
	task[task_count].counter = priority;
	task[task_count].start_stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task[task_count].func = func;

	task_count++;
}

void switch_task(struct task _task)
{
	if (current_task) {
		current_task->state = TASK_STOPPED;
		_task.state = TASK_RUNNING;
		current_task = &_task;
	} else {
		_task.state = TASK_RUNNING;
		current_task = &_task;
	}
}
