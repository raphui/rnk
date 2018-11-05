#include <kernel/scheduler.h>
#include <kernel/printk.h>
#include <string.h>
#include <mm/mm.h>
#include <utils.h>
#include <arch/system.h>
#include <kernel/spinlock.h>
#include <kernel/syscall.h>
#include <kernel/ktime.h>
#include <unistd.h>
#include <errno.h>

static struct thread *current_thread = NULL;
static int thread_count = 0;

#if defined(CONFIG_SCHEDULE_RR_PRIO) || defined(CONFIG_SCHEDULE_ROUND_ROBIN)
static unsigned int run_queue_bitmap;
#endif

static struct list_node run_queue[NB_RUN_QUEUE];

unsigned long thread_lock = SPIN_LOCK_INITIAL_VALUE;

static void idle_thread(void)
{
	unsigned long irqstate;

	while(1) {
		arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);
#ifdef CONFIG_TICKLESS
		ktime_wakeup_next_delay();
#endif
		arch_idle();

		arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	}
}

#if defined(CONFIG_SCHEDULE_RR_PRIO) || defined(CONFIG_SCHEDULE_ROUND_ROBIN)
static void insert_in_run_queue_head(struct thread *t)
{
	list_add_head(&run_queue[t->priority], &t->node);
	run_queue_bitmap |= (1 << t->priority);
}

static void insert_in_run_queue_tail(struct thread *t)
{
	list_add_tail(&run_queue[t->priority], &t->node);
	run_queue_bitmap |= (1 << t->priority);
}
#endif

static void insert_thread(struct thread *t)
{
	int inserted = 0;
	struct thread *thread = NULL;

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	insert_in_run_queue_tail(t);
	verbose_printk("inserting thread %d\r\n", t->pid);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	if (list_is_empty(&run_queue[0])) {
		list_add_head(&run_queue[0], &t->node);
	} else {
		list_for_every_entry(&run_queue[0], thread, struct thread, node) {
			verbose_printk("t->priority: %d, thread->priority; %d\r\n", t->priority, thread->priority);
			if (t->priority > thread->priority) {
				verbose_printk("inserting thread %d\r\n", t->pid);
				list_add_before(&thread->node, &t->node);
				inserted = 1;
				break;
			}
			else if (thread->node.next == &run_queue[0])
				break;
		}

		if (!inserted)
			list_add_after(&thread->node, &t->node);
	}
#elif defined(CONFIG_SCHEDULE_RR_PRIO)
	if (t->quantum > 0)
		insert_in_run_queue_head(t);
	else {
		t->quantum = CONFIG_THREAD_QUANTUM;
		insert_in_run_queue_tail(t);
	}
#endif
}

static void end_thread(void)
{
	syscall(SYSCALL_THREAD_STOP, NULL);
}

struct thread *add_thread(void (*func)(void), void *arg, unsigned int priority, int privileged)
{
	struct thread *thread = (struct thread *)kmalloc(sizeof(struct thread));
	if (!thread) {
		error_printk("failed to allocate thread: %p\n", func);
		return NULL;
	}

	memset(thread, 0, sizeof(struct thread));

	thread->state = THREAD_RUNNABLE;
	thread->pid = thread_count;

#ifdef CONFIG_SCHEDULE_PRIORITY
	thread->priority = priority;
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	thread->quantum = CONFIG_THREAD_QUANTUM;
#elif defined(CONFIG_SCHEDULE_RR_PRIO)
	thread->priority = priority;
	thread->quantum = CONFIG_THREAD_QUANTUM;
#endif

	thread->start_stack = THREAD_STACK_START + (thread_count * THREAD_STACK_OFFSET);
	thread->delay = 0;
	thread->func = func;
	thread->arch = (struct arch_thread *)kmalloc(sizeof(struct arch_thread));
	if (!thread->arch) {
		error_printk("failed to allocate arch thread for: %p\n", func);
		kfree(thread);
		return NULL;
	}

	memset(thread->arch, 0, sizeof(struct arch_thread));

	wait_queue_init(&thread->wait_exit);

	/* Creating thread context */
	arch_create_context(thread->arch, (unsigned int)thread->func, (unsigned int)&end_thread, (unsigned int *)thread->start_stack, (unsigned int )arg, privileged);

	insert_thread(thread);

	thread_count++;

	return thread;
}

void thread_init(void)
{
	int i;

	for (i = 0; i < NB_RUN_QUEUE; i++)
		list_initialize(&run_queue[i]);

	add_thread(&idle_thread, NULL, IDLE_PRIORITY, PRIVILEGED_THREAD);
}

struct thread *thread_create(void (*func)(void), void *arg, unsigned int priority)
{
	return add_thread(func, arg, priority, USER_THREAD);
}

void switch_thread(struct thread *thread)
{
	thread->state = THREAD_RUNNING;

	arch_switch_context(current_thread->arch, thread->arch);

	remove_runnable_thread(thread);

	current_thread = thread;
}

struct thread *get_current_thread(void)
{
	return current_thread;
}

struct thread *find_next_thread(void)
{
#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	int found = 0;
#endif

	struct thread *thread = NULL;

#ifdef CONFIG_SCHEDULE_RR_PRIO
	unsigned int bitmap = run_queue_bitmap;
	unsigned int next_queue;

	while (bitmap)	{
		next_queue = HIGHEST_PRIORITY - __builtin_clz(bitmap) 
			- (sizeof(run_queue_bitmap) * 8 - MAX_PRIORITIES);

		debug_printk("next_queue: %d\n", next_queue);

		list_for_every_entry(&run_queue[next_queue], thread, struct thread, node) {

			debug_printk("next thread: %d\n", thread->pid);
			if (list_is_empty(&run_queue[next_queue]))
				run_queue_bitmap &= ~(1 << next_queue);

			return thread;
		}

		bitmap &= ~(1 << next_queue);
	}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	thread = list_peek_head_type(&run_queue[0], struct thread, node);
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	list_for_every_entry(&run_queue[0], thread, struct thread, node) {
		if ((thread->quantum > 0) && (thread->pid != 0)) {
			found = 1;
			break;
		}
	}

	if (current_thread && is_thread_runnable(current_thread))
		current_thread->quantum = CONFIG_THREAD_QUANTUM;

	/* Only idle thread is eligible */
	if (!found) {
		thread = list_peek_head_type(&run_queue[0], struct thread, node);
	}
#endif

	verbose_printk("next thread: %d\r\n", thread->pid);

	return thread;
}

void insert_runnable_thread(struct thread *thread)
{
	thread_lock(state);

	insert_thread(thread);
	thread->state = THREAD_RUNNABLE;

	thread_unlock(state);
}

void remove_runnable_thread(struct thread *thread)
{
	thread_lock(state);

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	current_thread->quantum = CONFIG_THREAD_QUANTUM;
#endif

	list_delete(&thread->node);

	thread_unlock(state);
}

int is_thread_runnable(struct thread *thread)
{
	return ((thread->state != THREAD_BLOCKED) && (thread->state != THREAD_STOPPED)) ? 1 : 0;
}

int thread_join(struct thread *t)
{
	if (t->state == THREAD_STOPPED)
		return -ENOENT;

	wait_queue_block(&t->wait_exit);

	return 0;
}
