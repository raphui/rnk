#include <stdio.h>
#include <mutex.h>
#include <errno.h>
#include <scheduler.h>

#include <arch/svc.h>

static int __mutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct task *current_task = get_current_task();

	if (mutex->lock) {
		debug_printk("mutex already locked\r\n");
		ret = -EDEADLOCK;

		if (mutex->owner) {
			debug_printk("mutex has owner\r\n");

			if (mutex->waiting) {
				/* Make task waiting if higher priority */
				if (mutex->waiting->priority < current_task->priority) {
					mutex->waiting = current_task;
					current_task->state = TASK_BLOCKED;
				}

			} else {
				mutex->waiting = current_task;
				current_task->state = TASK_BLOCKED;
			}

		} else {
			debug_printk("No owner for mutex (%x)\r\n", mutex);
		}

	} else {
		mutex->lock = 1;
		mutex->owner = current_task;
		mutex->waiting = NULL;
	}

	return ret;
}

void mutex_lock(struct mutex *mutex)
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

void mutex_unlock(struct mutex *mutex)
{
	struct task *current_task = get_current_task();
	struct task *task;

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_task) {
		mutex->lock = 0;
		mutex->owner = NULL;

		if (mutex->waiting) {
			if (mutex->waiting->priority > current_task->priority) {
				task = mutex->waiting;
				task->state = TASK_RUNNABLE;
				mutex->waiting = NULL;

				schedule_task(task);
			} else {
				task = mutex->waiting;
				task->state = TASK_RUNNABLE;
				mutex->waiting = NULL;

				schedule_task(NULL);
			}
		}

		debug_printk("mutex (%x) unlock\r\n", mutex);

	} else {
		debug_printk("mutex cannot be unlock, task is not the owner\r\n");
	}
}
