#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <task.h>

void create_context(struct task _task);
void activate_context(struct task _task);
void switch_context(struct registers _current_regs, struct registers _task_regs);
void start_schedule(void);
void schedule(void);
void end_task(void);

#endif /* SCHEDULER_H */
