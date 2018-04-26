#include <printk.h>
#include <kmutex.h>
#include <errno.h>
#include <scheduler.h>
#include <utils.h>
#include <thread.h>
#include <spinlock.h>
#include <export.h>
#include <syscall.h>

static void insert_waiting_thread(struct mutex *m, struct thread *t)
{
	struct thread *thread;

	if (m->waiting) {
		list_for_every_entry(&m->waiting_threads, thread, struct thread, event_node) {

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
			if (!list_next(&m->waiting_threads, &thread->event_node)) {
				list_add_after(&thread->event_node, &t->event_node);
				break;
			}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
			if (t->priority > thread->priority)
				list_add_before(&thread->event_node, &t->event_node);
#endif
		}

	} else {
		list_add_head(&m->waiting_threads, &t->event_node);
	}


}

static void remove_waiting_thread(struct mutex *mutex, struct thread *t)
{
	list_delete(&t->event_node);
}

int kmutex_init(struct mutex *mutex)
{
	int ret = 0;

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	mutex->lock = 0;
	mutex->owner = NULL;
	mutex->waiting = 0;

	list_initialize(&mutex->waiting_threads);

err:
	return ret;
}

int kmutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct thread *current_thread = get_current_thread();

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	if (mutex->lock) {
		debug_printk("mutex already locked\r\n");

		if (mutex->owner) {
			debug_printk("mutex has owner: %d\r\n", mutex->owner->pid);

			current_thread->state = THREAD_BLOCKED;
			remove_runnable_thread(current_thread);

			insert_waiting_thread(mutex, current_thread);
			mutex->waiting++;
			schedule_yield();

		} else {
			debug_printk("No owner for mutex (%x)\r\n", mutex);
		}

	} else {
		mutex->lock = 1;
		mutex->owner = current_thread;
		mutex->waiting = 0;
	}
err:
	return ret;
}

int kmutex_unlock(struct mutex *mutex)
{
	int ret = 0;
	struct thread *current_thread = get_current_thread();
	struct thread *thread;

	if (!mutex) {
		ret = -EINVAL;
		goto err;
	}

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_thread) {
		mutex->lock = 0;
		mutex->owner = NULL;

		if (mutex->waiting) {
			mutex->waiting--;

			if (!list_is_empty(&mutex->waiting_threads)) {
				thread = list_peek_head_type(&mutex->waiting_threads, struct thread, event_node);
				thread->state = THREAD_RUNNABLE;

				remove_waiting_thread(mutex, thread);
				insert_runnable_thread(thread);
				schedule_yield();
			}
		}

		debug_printk("mutex (%x) unlock\r\n", mutex);

	} else {
		debug_printk("mutex cannot be unlock, thread is not the owner\r\n");
	}
err:
	return ret;
}
