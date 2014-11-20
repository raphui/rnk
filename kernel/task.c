#include <task.h>
#include <scheduler.h>
#include <io.h>

static struct task *current_task = NULL;

void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count].state = TASK_STOPPED;
	task[task_count].counter = priority;
	task[task_count].start_stack = TASK_STACK_START + (task_count * TASK_STACK_OFFSET);
	task[task_count].func = func;
	task[task_count].regs.sp = task[task_count].start_stack;

	/* Creating task context */
	create_context(task[task_count]);

	task_count++;
}

void switch_task(struct task _task)
{
	printk("#\r\n");
	if (current_task) {
		current_task->state = TASK_STOPPED;
		_task.state = TASK_RUNNING;
		current_task = &_task;

		/* Switch context */
		switch_context(*current_task);
	} else {
		_task.state = TASK_RUNNING;
		current_task = &_task;
		
		/* Active first task */
		activate_context(*current_task);
	}

}
