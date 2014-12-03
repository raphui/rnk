#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <task.h>

extern int task_switching;

void create_context(struct registers *_regs, struct task _task);
void activate_context(struct task _task);
void switch_context(struct registers *_current_regs, struct registers *_task_regs);
void save_user_context(void);
void get_user_context(void);
void start_schedule(void);
void schedule(void);
void end_task(void);

#endif /* SCHEDULER_H */
