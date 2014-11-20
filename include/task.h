#ifndef TASK_H
#define TASK_H

#define NR_TASK	8

#define TASK_RUNNING		0
#define TASK_STOPPED		1
#define TASK_INTERRUPTIBLE	3

#ifndef NULL
#define NULL	((void *)0)
#endif

#define TASK_STACK_START	0x002E0000
#define TASK_STACK_OFFSET	0x00001000

struct registers
{
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
	unsigned int r12;
	unsigned int sp;
	unsigned int lr;
	unsigned int pc;

};

struct task
{
	int state;
	unsigned int counter;
	unsigned int start_stack;
	void (*func)(void);
	struct registers regs;
};


struct task task[NR_TASK];

void add_task(void (*func)(void), unsigned int priority);
void switch_task(struct task _task);
int get_task_count(void);

#endif /* TASK_H */
