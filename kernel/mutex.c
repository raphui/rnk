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
		LIST_FOREACH(task, &m->waiting_tasks, next) {
			if (t->priority > task->priority)
				LIST_INSERT_BEFORE(task, t, next);
		}

	} else {
		LIST_INSERT_HEAD(&m->waiting_tasks, t, next);
	}


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

void mutex_lock_isr(struct mutex *mutex)
{
	int ret;

	ret = __mutex_lock(mutex);
	if (ret < 0) {
		debug_printk("mutex_lock FAILED !\r\n");

		if (mutex->owner)
			if (mutex->owner->state == TASK_RUNNABLE)
				schedule_task(mutex->owner);
			else
				schedule_isr();
		else
			schedule_isr();

	} else {
		debug_printk("mutex (%x) lock\r\n", mutex);
	}
}

void svc_mutex_lock(struct mutex *mutex)
{
	int ret;

	ret = __mutex_lock(mutex);
	if (ret < 0) {
		debug_printk("mutex_lock FAILED !\r\n");

		if (mutex->owner)
			if (mutex->owner->state == TASK_RUNNABLE)
				schedule_task(mutex->owner);
			else
				schedule_task(NULL);
		else
			schedule_task(NULL);

	} else {
		debug_printk("mutex (%x) lock\r\n", mutex);
	}
}

void mutex_lock(struct mutex *mutex)
{
	SVC_ARG(SVC_ACQUIRE_MUTEX, mutex);
}

void mutex_unlock_isr(struct mutex *mutex)
{
	struct task *current_task = get_current_task();
	struct task *task;

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_task) {
		mutex->lock = 0;
		mutex->owner = NULL;

		if (mutex->waiting) {
			task = LIST_FIRST(&mutex->waiting_tasks);
			LIST_REMOVE(task, next);
			task->state = TASK_RUNNABLE;
			mutex->waiting--;

			insert_runnable_task(task);
			schedule_isr();
		}

		debug_printk("mutex (%x) unlock\r\n", mutex);

	} else {
		debug_printk("mutex cannot be unlock, task is not the owner\r\n");
	}
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
			task = LIST_FIRST(&mutex->waiting_tasks);
			LIST_REMOVE(task, next);
			task->state = TASK_RUNNABLE;
			mutex->waiting--;

			insert_runnable_task(task);
			schedule_task(task);
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
