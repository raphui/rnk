#include <stdio.h>
#include <mutex.h>
#include <errno.h>
#include <scheduler.h>

static int __mutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct task *current_task = get_current_task();

	if (mutex->lock) {
		printk("mutex already locked\r\n");
		ret = -EDEADLOCK;

		if (mutex->owner != NULL) {

			if (mutex->waiting) {
				/* Make task waiting if higher counter */
				if (mutex->waiting->counter < current_task->counter)
					mutex->waiting = current_task;
			} else {
				mutex->waiting = get_current_task();
			}

		} else {
			printk("No owner for mutex (%x)\r\n", mutex);
		}

	} else {
		mutex->lock = 1;
		mutex->owner = get_current_task();
		mutex->waiting = NULL;
	}

	return ret;
}

void mutex_lock(struct mutex *mutex)
{
	int ret;

	ret = __mutex_lock(mutex);
	if (ret < 0) {

		if (mutex->owner->state == TASK_RUNNABLE)
			schedule_task(mutex->owner);
		else
			schedule_task(NULL);

	} else {
		printk("mutex (%x) lock\r\n", mutex);
	}
}

void mutex_unlock(struct mutex *mutex)
{
	struct task *current_task = get_current_task();
	struct task *task;

	if (!mutex->lock) {
		printk("mutex already unlock\r\n");
	}

	mutex->lock = 0;
	mutex->owner = NULL;

	if (mutex->waiting) {
		if (mutex->waiting->counter > current_task->counter) {
			task = mutex->waiting;
			mutex->waiting = NULL;

			schedule_task(task);
		}
	}

	printk("mutex (%x) unlock\r\n", mutex);
}
