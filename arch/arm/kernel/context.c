#include <task.h>
#include <scheduler.h>
#include "svc.h"

void create_context(struct task _task)
{
	unsigned int *stack = (unsigned int *)_task.start_stack;

	asm volatile(
		 "stmdb   %[stack]!, {%[psr]}   /* xPSR */                      \n\
                  stmdb   %[stack]!, {%[pc]}    /* PC */                        \n\
                  stmdb   %[stack]!, {%[lr]}    /* LR */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R12 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R3 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R2 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R1 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R0 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R11 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R10 */                       \n\
                  stmdb   %[stack]!, {%[zero]}  /* R9 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R8 */                        \n\
                  stmdb   %[stack]!, {%[frame]} /* R7 - Frame Pointer*/         \n\
                  stmdb   %[stack]!, {%[zero]}  /* R6 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R5 */                        \n\
                  stmdb   %[stack]!, {%[zero]}  /* R4 */                        \n"

		/* Output */
                :[stack] "+r" (_task.start_stack)
                /* Input */
                :[pc] "r" (_task.func), [lr] "r" (end_task), [frame] "r" (TASK_STACK_OFFSET),
                [zero] "r" (0), [psr] "r" (0x01000000) /* Set the Thumb bit */
		/* Clobbers */
                :);
}

void activate_context(struct task _task)
{
	svc_activate_context(_task.regs.sp);
}

void switch_context(struct task _task)
{
	svc_switch_context(_task.regs.sp);
}
