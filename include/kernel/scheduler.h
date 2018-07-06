#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <kernel/thread.h>

extern int thread_switching;
extern unsigned int system_tick;

void start_schedule(void);
int schedule_init(void);
void schedule(void);
void schedule_thread(struct thread *thread);
void schedule_thread_stop(struct thread *thread);
void schedule_isr(void);
void schedule_yield(void);

#endif /* SCHEDULER_H */
