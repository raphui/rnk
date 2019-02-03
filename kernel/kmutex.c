#include <kernel/printk.h>
#include <kernel/kmutex.h>
#include <errno.h>
#include <kernel/scheduler.h>
#include <utils.h>
#include <kernel/thread.h>
#include <kernel/spinlock.h>
#include <export.h>
#include <kernel/syscall.h>
#include <kernel/wait.h>
#include <trace.h>

int kmutex_init(struct mutex *mutex)
{
	int ret = 0;

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	mutex->lock = 0;
	mutex->owner = NULL;
	mutex->old_prio = 0;

	trace_mutex_create(mutex);

	wait_queue_init(&mutex->wait);

err:
	return ret;
}

int kmutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct thread *current_thread = get_current_thread();

	thread_lock(state);

	trace_mutex_lock(mutex);

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	if (++mutex->lock > 1) {
		debug_printk("mutex %x already locked by %d [%x]\n", mutex, mutex->owner->pid, mutex->owner);

#ifdef CONFIG_PRIORITY_INHERITANCE
		if (current_thread->priority > mutex->owner->priority) {
			if (!mutex->old_prio)
				mutex->old_prio = mutex->owner->priority;

			mutex->owner->priority = current_thread->priority;
		}
#endif /* CONFIG_PRIORITY_INHERITANCE */

		ret = wait_queue_block_irqstate(&mutex->wait, &state);
	}

	mutex->owner = current_thread;
err:
	thread_unlock(state);
	return ret;
}

int kmutex_unlock(struct mutex *mutex)
{
	int ret = 0;
	struct thread *current_thread = get_current_thread();

	thread_lock(state);

	trace_mutex_unlock(mutex);

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_thread) {
		mutex->owner = NULL;

#ifdef CONFIG_PRIORITY_INHERITANCE
		if (mutex->old_prio) {
				current_thread->priority = mutex->old_prio;
				mutex->old_prio = 0;
		}
#endif /* CONFIG_PRIORITY_INHERITANCE */

		if (--mutex->lock >= 1)
			ret = wait_queue_wake_irqstate(&mutex->wait, &state);
	} else {
		debug_printk("mutex cannot be unlock, thread %d [%x] is not the owner\n", mutex->owner->pid, mutex->owner);
	}

err:
	thread_unlock(state);
	return ret;
}
