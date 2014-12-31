#include <stdio.h>
#include <mutex.h>

void mutex_lock(struct mutex *mutex)
{
	if (mutex->lock) {
		printk("mutex already locked\r\n");
		return;
	}

	mutex->lock = 1;
}

void mutex_unlock(struct mutex *mutex)
{
	if (!mutex->lock) {
		printk("mutex already unlock\r\n");
		return;
	}

	mutex->lock = 0;
}
