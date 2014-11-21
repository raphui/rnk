#include <task.h>
#include <scheduler.h>
#include <io.h>

static struct task *current_task = NULL;
static int task_count;


void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count].state = TASK_STOPPED;
	task[task_count].counter = priority;
	task[task_count].start_stack = TASK_STACK_START - (task_count * TASK_STACK_OFFSET);
	task[task_count].func = func;
	task[task_count].regs = &task_regs[task_count];
	task[task_count].regs->sp = task[task_count].start_stack;

	/* Creating task context */
	create_context(task[task_count]);

	task_count++;
}

void first_switch_task(struct task _task)
{
	_task.state = TASK_RUNNING;
	current_task = &_task;
	/* Active first task */
	activate_context(*current_task);
}

void switch_task(struct task _task)
{
	if (current_task) {
		current_task->state = TASK_STOPPED;
		_task.state = TASK_RUNNING;

		/* Switch context */
		switch_context(current_task->regs, _task.regs);
		current_task = &_task;
	} else {
		_task.state = TASK_RUNNING;
		current_task = &_task;
		
		/* Active first task */
		activate_context(*current_task);
	}

}

int get_task_count(void)
{
	return task_count;
}
