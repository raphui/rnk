#include <task.h>
#include <scheduler.h>
#include <io.h>

static struct task *current_task = NULL;
static int index_current_task = -1;
static int task_count;


void add_task(void (*func)(void), unsigned int priority)
{
	task[task_count].state = TASK_STOPPED;
	task[task_count].counter = priority;
	task[task_count].start_stack = TASK_STACK_START - (task_count * TASK_STACK_OFFSET);
	task[task_count].func = func;
	task[task_count].regs = &task_regs[task_count];
	task[task_count].regs->sp = task[task_count].start_stack;
	task[task_count].regs->lr = (unsigned int)end_task;
	task[task_count].regs->pc = (unsigned int)func;

	/* Creating task context */
	create_context(task[task_count].regs, task[task_count]);

	task_count++;
}

void first_switch_task(int index_task)
{
	task[index_task].state = TASK_RUNNING;
	index_current_task = index_task;
	/* Active first task */
	activate_context(task[index_current_task]);
}

void switch_task(int index_task)
{
	if (index_current_task > -1) {
		task[index_current_task].state = TASK_STOPPED;
		task[index_task].state = TASK_RUNNING;

		/* Switch context */
		switch_context(task[index_current_task].regs, task[index_task].regs);
		index_current_task = index_task;
	} else {
		task[index_task].state = TASK_RUNNING;
		index_current_task = index_task;
		
		/* Active first task */
		activate_context(*current_task);
	}

}

int get_task_count(void)
{
	return task_count;
}
