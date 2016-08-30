#include <stdio.h>
#include <mutex.h>
#include <errno.h>
#include <scheduler.h>
#include <utils.h>
#include <task.h>
#include <spinlock.h>

#include <arch/svc.h>

static void insert_waiting_task(struct mutex *m, struct task *t)
{
	struct task *task;

	if (m->waiting) {
		LIST_FOREACH(task, &m->waiting_tasks, event_next) {

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
			if (!LIST_NEXT(task, event_next)) {
				LIST_INSERT_AFTER(task, t, event_next);
				break;
			}
#elif defined(CONFIG_SCHEDULE_PRIORITY)
			if (t->priority > task->priority)
				LIST_INSERT_BEFORE(task, t, event_next);
#endif
		}

	} else {
		LIST_INSERT_HEAD(&m->waiting_tasks, t, event_next);
	}


}

static void remove_waiting_task(struct mutex *mutex, struct task *t)
{
	LIST_REMOVE(t, event_next);
}

static int __mutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct task *current_task = get_current_task();

	task_lock(state);

	if (mutex->lock) {
		debug_printk("mutex already locked\r\n");
		ret = -EDEADLOCK;

		if (mutex->owner) {
			debug_printk("mutex has owner: %d\r\n", mutex->owner->pid);

			current_task->state = TASK_BLOCKED;
			remove_runnable_task(current_task);

			insert_waiting_task(mutex, current_task);
			mutex->waiting++;

		} else {
			debug_printk("No owner for mutex (%x)\r\n", mutex);
		}

	} else {
		mutex->lock = 1;
		mutex->owner = current_task;
		mutex->waiting = 0;
	}

	task_unlock(state);

	return ret;
}

void init_mutex(struct mutex *mutex) {
	mutex->lock = 0;
	mutex->owner = NULL;
	mutex->waiting = 0;

	LIST_INIT(&mutex->waiting_tasks);
}

void svc_mutex_lock(struct mutex *mutex)
{
	int ret;

	ret = __mutex_lock(mutex);
	if (ret < 0) {
		debug_printk("mutex_lock FAILED !\r\n");

	} else {
		debug_printk("mutex (%x) lock\r\n", mutex);
	}
}

void mutex_lock(struct mutex *mutex)
{
	SVC_ARG(SVC_ACQUIRE_MUTEX, mutex);
}

void svc_mutex_unlock(struct mutex *mutex)
{
	struct task *current_task = get_current_task();
	struct task *task;

	task_lock(state);

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_task) {
		mutex->lock = 0;
		mutex->owner = NULL;

		if (mutex->waiting) {
			mutex->waiting--;

			if (!LIST_EMPTY(&mutex->waiting_tasks)) {
				task = LIST_FIRST(&mutex->waiting_tasks);
				task->state = TASK_RUNNABLE;

				remove_waiting_task(mutex, task);
				insert_runnable_task(task);
			}
		}

		debug_printk("mutex (%x) unlock\r\n", mutex);

	} else {
		debug_printk("mutex cannot be unlock, task is not the owner\r\n");
	}

	task_unlock(state);
}

void mutex_unlock(struct mutex *mutex)
{
	SVC_ARG(SVC_RELEASE_MUTEX, mutex);
}
