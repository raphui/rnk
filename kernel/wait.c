#include <kernel/wait.h>
#include <list.h>
#include <kernel/thread.h>
#include <errno.h>
#include <kernel/scheduler.h>
#include <kernel/spinlock.h>
#include <kernel/printk.h>
#include <kernel/ktime.h>
#include <trace.h>

static void insert_waiting_thread(struct wait_queue *wait, struct thread *t)
{
#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	struct thread *thread;
#endif

	if (wait->count) {
#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
		list_for_every_entry(&wait->list, thread, struct thread, event_node) {

			if (!list_next(&wait->list, &thread->event_node)) {
				list_add_after(&thread->event_node, &t->event_node);
				break;
			}
		}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
		list_add_tail(&wait->list, &t->event_node);
#endif
	} else {
		list_add_head(&wait->list, &t->event_node);
	}

	trace_thread_blocked(t);
}

static void remove_waiting_thread(struct wait_queue *wait, struct thread *t)
{
	if (t->state == THREAD_BLOCKED)
		list_delete(&t->event_node);
}

static void wait_queue_timeout(void *arg)
{
	struct thread *thread = (struct thread *)arg;

	if (thread->state == THREAD_BLOCKED) {
		thread->err_wait = -ETIMEDOUT;
		wait_queue_wake_thread(thread);
	}
}

int wait_queue_init(struct wait_queue *wait)
{
	if (!wait)
		return -EINVAL;

	list_initialize(&wait->list);
	wait->count = 0;

	return 0;
}

static int __wait_queue_block(struct wait_queue *wait, unsigned long *irqstate, struct thread *thread)
{
	remove_runnable_thread(thread);

	if (thread->state == THREAD_RUNNING)
		thread->state = THREAD_BLOCKED;

	thread->wait_queue = wait;
	insert_waiting_thread(wait, thread);
	wait->count++;
			
	schedule_yield();

	arch_interrupt_restore(*irqstate, SPIN_LOCK_FLAG_IRQ);

	return thread->err_wait;
}

static int __wait_queue_wake(struct wait_queue *wait, unsigned long *irqstate)
{
	struct thread *thread;

	if (!wait)
		return -EINVAL;

	if (wait->count) {
			wait->count--;

			thread = list_peek_head_type(&wait->list, struct thread, event_node);

			remove_waiting_thread(wait, thread);
			insert_runnable_thread(thread);
			thread->wait_queue = NULL;

			schedule_yield();
	}

	arch_interrupt_restore(*irqstate, SPIN_LOCK_FLAG_IRQ);

	return 0;
}

int wait_queue_block_irqstate(struct wait_queue *wait, unsigned long *irqstate)
{
	int ret;
	struct thread *thread = get_current_thread();

	if (!wait)
		return -EINVAL;

	ret = __wait_queue_block(wait, irqstate, thread);

	arch_interrupt_save(irqstate, SPIN_LOCK_FLAG_IRQ);

	return ret;
}

int wait_queue_block(struct wait_queue *wait)
{
	int ret;
	unsigned long irqstate;
	struct thread *thread = get_current_thread();

	if (!wait)
		return -EINVAL;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	ret = __wait_queue_block(wait, &irqstate, thread);

	return ret;
}

int wait_queue_block_thread(struct wait_queue *wait, struct thread *thread)
{
	int ret;
	unsigned long irqstate;

	if (!wait)
		return -EINVAL;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	ret = __wait_queue_block(wait, &irqstate, thread);

	return ret;
}

int wait_queue_block_timed(struct wait_queue *wait, int timeout, unsigned long *irqstate)
{
	int ret;
	unsigned long _irqstate;
	struct thread *thread = get_current_thread();
	struct ktimer timer;

	if (!wait)
		return -EINVAL;

	if (!irqstate) {
		irqstate = &_irqstate;
		arch_interrupt_save(&_irqstate, SPIN_LOCK_FLAG_IRQ);
	}

	thread->err_wait = 0;

	ktime_oneshot(&timer, timeout, wait_queue_timeout, thread);

	ret = __wait_queue_block(wait, irqstate, thread);

	ktime_oneshot_cancel(&timer);

	return ret;
}

int wait_queue_wake_irqstate(struct wait_queue *wait, unsigned long *irqstate)
{
	int ret;

	if (!wait)
		return -EINVAL;

	ret = __wait_queue_wake(wait, irqstate);

	arch_interrupt_save(irqstate, SPIN_LOCK_FLAG_IRQ);

	return ret;
}

int wait_queue_wake(struct wait_queue *wait)
{
	int ret;
	unsigned long irqstate;

	if (!wait)
		return -EINVAL;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	ret = __wait_queue_wake(wait, &irqstate);

	return ret;	
}

int wait_queue_wake_thread(struct thread *thread)
{
	int ret = 0;
	unsigned long irqstate;

	if (!thread)
		return -EINVAL;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	thread->wait_queue->count--;

	remove_waiting_thread(thread->wait_queue, thread);
	insert_runnable_thread(thread);

	thread->wait_queue = NULL;

	schedule_thread(NULL);

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);

	return ret;
}

int wait_queue_wake_isr(struct wait_queue *wait)
{
	struct thread *thread;
	struct thread *current_thread = get_current_thread();

	if (!wait)
		return -EINVAL;

	if (wait->count) {
			wait->count--;

			thread = list_peek_head_type(&wait->list, struct thread, event_node);

			remove_waiting_thread(wait, thread);
			insert_runnable_thread(thread);

			if (thread->priority > current_thread->priority)
				schedule_thread(NULL);
	}

	return 0;
}
