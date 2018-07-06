#ifndef SYSTEM_H
#define SYSTEM_H

#include <kernel/thread.h>

extern void arch_init(void);
extern void arch_init_tick(void);
extern void arch_request_sched(void);
extern unsigned int arch_get_thread_stack(void);
extern void arch_set_thread_stack(struct thread *t);

#endif /* SYSTEM_H */
