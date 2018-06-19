#include <ksem.h>
#include <thread.h>
#include <errno.h>
#include <scheduler.h>
#include <spinlock.h>
#include <printk.h>
#include <syscall.h>
#include <export.h>

static void insert_waiting_thread(struct semaphore *sem, struct thread *t)
{
	struct thread *thread;

#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_RR_PRIO)
	list_add_tail(&sem->waiting_threads, &t->event_node);
#elif defined(CONFIG_SCHEDULE_PRIORITY)
	if (list_is_empty(&sem->waiting_threads))
		list_add_head(&sem->waiting_threads, &t->event_node);
	else {
		list_for_every_entry(&sem->waiting_threads, thread, struct thread, event_node)
			if (t->priority > thread->priority)
				list_add_before(&thread->event_node, &t->event_node);
	}
#endif
}

static void remove_waiting_thread(struct semaphore *sem, struct thread *t)
{
	list_delete(&t->event_node);
}

int ksem_init(struct semaphore *sem, unsigned int value)
{
	int ret = 0;

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->value = value;
	sem->count = 0;
	sem->waiting = 0;

	list_initialize(&sem->waiting_threads);

err:
	return ret;
}

int ksem_wait(struct semaphore *sem)
{
	int ret = 0;
	struct thread *current_thread;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	if (--sem->count < 0) {
		debug_printk("unable to got sem (%p)(%d)\r\n", sem, sem->count);

		current_thread = get_current_thread();
		current_thread->state = THREAD_BLOCKED;

		insert_waiting_thread(sem, current_thread);
		sem->waiting++;

		schedule_yield();
	}

err:
	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	return ret;
}

int ksem_post(struct semaphore *sem)
{
	int ret = 0;
	struct thread *thread;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count++;

	if (sem->count > sem->value)
		sem->count = sem->value;

	if (sem->count <= 0) {
		if (!list_is_empty(&sem->waiting_threads)) {
			sem->waiting--;

			thread = list_peek_head_type(&sem->waiting_threads, struct thread, event_node);

			debug_printk("waking up thread: %d\n", thread->pid);

			remove_waiting_thread(sem, thread);
			insert_runnable_thread(thread);
			schedule_yield();
		}
	}

err:
	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	return ret;
}
