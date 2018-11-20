#ifndef SYSTEM_H
#define SYSTEM_H

#ifdef CONFIG_CPU_ARMV7M
#include <armv7m/system.h>
#endif /* CONFIG_CPU_ARMV7M */

#include <kernel/thread.h>

extern void arch_init(void);
extern void arch_init_tick(void);
extern void arch_idle(void);
extern void arch_request_sched(void);
extern unsigned int arch_get_thread_stack(void);
extern void arch_set_thread_stack(struct thread *t);

#endif /* SYSTEM_H */
