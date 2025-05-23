#ifndef THREAD_H
#define THREAD_H

#include <list.h>
#include <stddef.h>
#include <arch/thread.h>
#include <kernel/wait.h>
#include <kernel/ktime.h>

#define THREAD_RUNNING		0
#define THREAD_RUNNABLE		1
#define THREAD_STOPPED		2
#define THREAD_INTERRUPTIBLE	3
#define THREAD_BLOCKED		4
#define THREAD_SUSPEND		5

#define THREAD_WAIT_NONE	0
#define THREAD_WAIT_SEM		1
#define THREAD_WAIT_MUTEX	2
#define THREAD_WAIT_QUEUE	3
#define THREAD_WAIT_TIMEOUT	4

#define NUM_PRIORITIES		32
#define MAX_PRIORITIES		32
#define LOWEST_PRIORITY		0
#define HIGHEST_PRIORITY	(NUM_PRIORITIES - 1)
#define DPC_PRIORITY		(NUM_PRIORITIES - 2)
#define IDLE_PRIORITY		LOWEST_PRIORITY
#define LOW_PRIORITY		(NUM_PRIORITIES / 4)
#define DEFAULT_PRIORITY	(NUM_PRIORITIES / 2)
#define HIGH_PRIORITY		((NUM_PRIORITIES / 4) * 3)

#define USER_THREAD		0
#define PRIVILEGED_THREAD	1

#ifdef CONFIG_SCHEDULE_RR_PRIO
#define NB_RUN_QUEUE	32
#else
#define NB_RUN_QUEUE	1
#endif /* CONFIG_SCHEDULE_RR_PRIO */

extern unsigned long thread_lock;

struct thread
{
	unsigned int state;
	unsigned int pid;
	unsigned int priority;
	unsigned int quantum;
	unsigned int start_stack;
	unsigned int delay;
#ifdef CONFIG_TRACE
	char name[16];
#endif /* CONFIG_TRACE */
	void (*func)(void);
	struct arch_thread *arch;
	struct list_node node;
	struct list_node state_node;
	struct list_node event_node;
	struct wait_queue wait_exit;
	struct wait_queue *wait_queue;
	struct ktimer wait_timer;
	int err_wait;
	int wait_reason;
};

void thread_init(void);
struct thread *thread_create(void (*func)(void), void *arg, unsigned int priority);
int thread_destroy(struct thread *thread);
int thread_suspend(struct thread *t);
int thread_resume(struct thread *t);
int thread_join(struct thread *t);
struct thread *add_thread(void (*func)(void), void *arg, unsigned int priority, int privileged);
void switch_thread(struct thread *thread);
struct thread *get_current_thread(void);
struct thread *find_next_thread(void);
void insert_runnable_thread(struct thread *thread);
void remove_runnable_thread(struct thread *thread);
int is_thread_runnable(struct thread *thread);

#endif /* THREAD_H */
