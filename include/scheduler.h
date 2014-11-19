#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <task.h>

void create_context(struct task _task);
void schedule(void);
void end_task(void);

#endif /* SCHEDULER_H */
