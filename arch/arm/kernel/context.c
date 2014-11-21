#include <task.h>
#include <scheduler.h>
#include "svc.h"

void create_context(struct task _task)
{
	svc_create_context(_task.start_stack, (unsigned int)_task.func, (unsigned int)end_task, TASK_STACK_OFFSET);
}

void activate_context(struct task _task)
{
	svc_activate_context(_task.regs);
}

void switch_context(struct registers *_current_regs, struct registers *_task_regs)
{
	svc_switch_context(_current_regs, _task_regs);
}
