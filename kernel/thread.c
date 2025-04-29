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
#include <trace.h>
#include <rflat/rflat.h>

#ifdef CONFIG_MAX_THREADS
static struct thread threads[CONFIG_MAX_THREADS];
#else
static struct list_node threads;
#endif

static struct thread *current_thread = NULL;
static int thread_count = 0;

#if defined(CONFIG_SCHEDULE_RR_PRIO) || defined(CONFIG_SCHEDULE_ROUND_ROBIN)
static unsigned int run_queue_bitmap;
#endif

static struct list_node run_queue[NB_RUN_QUEUE];
static struct list_node suspend_queue;

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
	list_add_head(&run_queue[t->priority], &t->state_node);
	run_queue_bitmap |= (1 << t->priority);
}

static void insert_in_run_queue_tail(struct thread *t)
{
	list_add_tail(&run_queue[t->priority], &t->state_node);
	run_queue_bitmap |= (1 << t->priority);
}
#endif

static void insert_thread(struct thread *t)
{
	int inserted = 0;
	struct thread *thread = NULL;

	assert(t != NULL);

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	insert_in_run_queue_tail(t);
	verbose_printk("inserting thread %d\r\n", t->pid);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	if (list_is_empty(&run_queue[0])) {
		list_add_head(&run_queue[0], &t->state_node);
	} else {
		list_for_every_entry(&run_queue[0], thread, struct thread, state_node) {
			verbose_printk("t->priority: %d, thread->priority; %d\r\n", t->priority, thread->priority);
			if (t->priority > thread->priority) {
				verbose_printk("inserting thread %d\r\n", t->pid);
				list_add_before(&thread->state_node, &t->state_node);
				inserted = 1;
				break;
			}
			else if (thread->state_node.next == &run_queue[0])
				break;
		}

		if (!inserted)
			list_add_after(&thread->state_node, &t->state_node);
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
	struct thread *thread;
	int platform_register = 0;

#ifdef CONFIG_MAX_THREADS
	if (thread_count < CONFIG_MAX_THREADS)
		thread  = &threads[thread_count];
	else
		thread = NULL;
#else
	thread = (struct thread *)kmalloc(sizeof(struct thread));
#endif
	if (!thread) {
		error_printk("failed to allocate thread: %p\n", func);
		goto err;
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

	thread->start_stack = (unsigned int)kmalloc_align(8, CONFIG_THREAD_STACK_SIZE);
	if (!thread->start_stack) {
		error_printk("failed to allocate thread stack\n");
		goto err_alloc_stack;
	}

	thread->start_stack += CONFIG_THREAD_STACK_SIZE;

	thread->delay = 0;
	thread->func = func;
	thread->arch = (struct arch_thread *)kmalloc(sizeof(struct arch_thread));
	if (!thread->arch) {
		error_printk("failed to allocate arch thread for: %p\n", func);
		goto err_alloc_arch;
	}

	memset(thread->arch, 0, sizeof(struct arch_thread));

	wait_queue_init(&thread->wait_exit);

#ifdef CONFIG_RFLAT_LOADER
	platform_register = rflat_get_app_header()->got_loc;
#endif
	/* Creating thread context */
	arch_create_context(thread->arch, (unsigned int)thread->func, (unsigned int)&end_thread, (unsigned int *)thread->start_stack, (unsigned int )arg, privileged, platform_register);

#ifdef CONFIG_TRACE
	snprintf(thread->name, sizeof(thread->name), "thread %d\n", thread->pid);
#endif /* CONFIG_TRACE */

	thread->wait_reason = THREAD_WAIT_NONE;

	trace_thread_create(thread);

	insert_thread(thread);

	list_add_tail(&threads, &thread->node);

	thread_count++;

	return thread;

err_alloc_arch:
	thread->start_stack -= CONFIG_THREAD_STACK_SIZE;
	kfree((void *)thread->start_stack);
err_alloc_stack:
	kfree(thread);
err:
	return NULL;
}

void thread_init(void)
{
	int i;

	for (i = 0; i < NB_RUN_QUEUE; i++)
		list_initialize(&run_queue[i]);

	list_initialize(&suspend_queue);
	list_initialize(&threads);

	add_thread(&idle_thread, NULL, IDLE_PRIORITY, PRIVILEGED_THREAD);
}

struct thread *thread_create(void (*func)(void), void *arg, unsigned int priority)
{
#ifdef CONFIG_USER
	return add_thread(func, arg, priority, USER_THREAD);
#else
	return add_thread(func, arg, priority, PRIVILEGED_THREAD);
#endif
}

int thread_destroy(struct thread *thread)
{
	if (!thread)
		return -EINVAL;

//#ifndef CONFIG_MAX_THREADS
#if 0
	kfree(thread->arch);
	kfree(thread);
#endif

	return 0;
}

void switch_thread(struct thread *thread)
{
	if (current_thread)
		arch_switch_context(current_thread->arch, thread->arch);
	else
		arch_switch_context(NULL, thread->arch);

	remove_runnable_thread(thread);

	thread->state = THREAD_RUNNING;

	current_thread = thread;
}

#ifdef CONFIG_TRACE
void switch_thread_trace(void)
{
	trace_sched_thread(current_thread);
}
#endif

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

		list_for_every_entry(&run_queue[next_queue], thread, struct thread, state_node) {

			debug_printk("next thread: %d\n", thread->pid);
			if (list_is_empty(&run_queue[next_queue]))
				run_queue_bitmap &= ~(1 << next_queue);

			return thread;
		}

		bitmap &= ~(1 << next_queue);
	}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	thread = list_peek_head_type(&run_queue[0], struct thread, state_node);
#elif defined(CONFIG_SCHEDULE_ROUND_ROBIN)
	list_for_every_entry(&run_queue[0], thread, struct thread, state_node) {
		if ((thread->quantum > 0) && (thread->pid != 0)) {
			found = 1;
			break;
		}
	}

	if (current_thread && is_thread_runnable(current_thread))
		current_thread->quantum = CONFIG_THREAD_QUANTUM;

	/* Only idle thread is eligible */
	if (!found) {
		thread = list_peek_head_type(&run_queue[0], struct thread, state_node);
	}
#endif

	assert(thread != NULL);

	verbose_printk("next thread: %d\r\n", thread->pid);

	return thread;
}

void insert_runnable_thread(struct thread *thread)
{
	thread_lock(state);

	assert(thread != NULL);

	if (!list_in_list(&thread->event_node) && !list_in_list(&thread->state_node)) {
		trace_thread_runnable(thread);

		insert_thread(thread);
		thread->state = THREAD_RUNNABLE;
	}

	thread_unlock(state);
}

void remove_runnable_thread(struct thread *thread)
{
	thread_lock(state);

	assert(thread != NULL);

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	current_thread->quantum = CONFIG_THREAD_QUANTUM;
#endif

	if (thread->state != THREAD_RUNNING && thread->state != THREAD_STOPPED)
		list_delete(&thread->state_node);

	thread_unlock(state);
}

int is_thread_runnable(struct thread *thread)
{
	assert(thread != NULL);

	return ((thread->state != THREAD_BLOCKED) && (thread->state != THREAD_STOPPED)) ? 1 : 0;
}

int thread_suspend(struct thread *t)
{
	assert(t != NULL);

	if (t->state != THREAD_SUSPEND) {
		remove_runnable_thread(t);
		t->state = THREAD_SUSPEND;
		list_add_tail(&suspend_queue, &t->state_node);
		schedule_thread(NULL);
	}

	return 0;
}

int thread_resume(struct thread *t)
{
	assert(t != NULL);

	if (t->state == THREAD_SUSPEND) {
		t->state = THREAD_RUNNABLE;
		list_delete(&t->state_node);
		insert_runnable_thread(t);
		schedule_thread(NULL);
	}

	return 0;
}

int thread_join(struct thread *t)
{
	if (t->state == THREAD_STOPPED)
		return -ENOENT;

	wait_queue_block(&t->wait_exit);

	return 0;
}
