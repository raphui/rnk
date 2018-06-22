#ifndef ARMV7M_THREAD_H
#define ARMV7M_THREAD_H

#define ARCH_TREAD_UNPRIVILEGED	0
#define ARCH_TREAD_PRIVILEGED	1

struct arch_sw_context_frame {
	unsigned int sp;
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
	unsigned int exc_lr;
};

struct arch_short_context_frame {
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r12;
	unsigned int lr;
	unsigned int pc;
	unsigned int xpsr;
};

struct arch_mpu_priv {
	int prio;
	unsigned int start_pc;
	unsigned int top_sp;
};

struct arch_thread {
	struct arch_sw_context_frame ctx_frame;
	struct arch_short_context_frame hw_frame;
	struct arch_mpu_priv mpu;
	int privileged;
};

void arch_create_context(struct arch_thread *arch, unsigned int func, unsigned int return_func, unsigned int *stack, unsigned int param1, int privileged);
void arch_switch_context(struct arch_thread *old, struct arch_thread *new);;
void arch_thread_set_return(void *ret);
void arch_thread_switch_unpriv(void);
void arch_thread_switch_priv(void);

#endif /* ARMV7M_THREAD_H */
