#include <stdio.h>
#include <mutex.h>
#include <errno.h>
#include <scheduler.h>
#include <utils.h>

#include <arch/svc.h>

static void insert_waiting_task(struct mutex *m, struct task *t)
{
	struct entry *e = (struct entry *)&(m->waiting_tasks.head);
	struct task *task;

	if (m->waiting) {
		while (e->next) {
			task = (struct task *)container_of(e, struct task, list_entry);

			if (t->priority > task->priority) {
				list_insert_before(&m->waiting_tasks, &task->list_entry, &t->list_entry);
				break;
			}

			e = e->next;
		}
	} else {
		list_insert_head(&m->waiting_tasks, &t->list_entry);
	}


}

static int __mutex_lock(struct mutex *mutex)
{
	int ret = 0;
	struct task *current_task = get_current_task();

	if (mutex->lock) {
		debug_printk("mutex already locked\r\n");
		ret = -EDEADLOCK;

		if (mutex->owner) {
			debug_printk("mutex has owner\r\n");

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

	return ret;
}

void init_mutex(struct mutex *mutex) {
	mutex->lock = 0;
	mutex->owner = NULL;
	mutex->waiting = 0;

	list_init(&mutex->waiting_tasks);
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
	struct entry *e;

	if (!mutex->lock)
		debug_printk("mutex already unlock\r\n");

	if (mutex->owner == current_task) {
		mutex->lock = 0;
		mutex->owner = NULL;

		if (mutex->waiting) {
			e = list_get_head(&(mutex->waiting_tasks));
			task = (struct task *)container_of(e, struct task, list_entry);
			task->state = TASK_RUNNABLE;
			mutex->waiting--;

			insert_runnable_task(task);
			schedule_task(task);
		}

		debug_printk("mutex (%x) unlock\r\n", mutex);

	} else {
		debug_printk("mutex cannot be unlock, task is not the owner\r\n");
	}
}
