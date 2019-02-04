#include <kernel/ksem.h>
#include <kernel/wait.h>
#include <kernel/thread.h>
#include <errno.h>
#include <kernel/scheduler.h>
#include <kernel/spinlock.h>
#include <kernel/printk.h>
#include <kernel/syscall.h>
#include <kernel/ktime.h>
#include <export.h>
#include <trace.h>

int ksem_init(struct semaphore *sem, int value)
{
	int ret = 0;

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->value = value;
	sem->count = 0;

	wait_queue_init(&sem->wait);

	trace_sem_create(sem);

err:
	return ret;
}

int ksem_wait(struct semaphore *sem)
{
	int ret = 0;

	thread_lock(state);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count--;

	trace_sem_wait(sem);

	if (sem->count < 0) {
		debug_printk("unable to got sem (%p)(%d)\r\n", sem, sem->count);

		ret = wait_queue_block_irqstate(&sem->wait, &state);
	}

err:
	thread_unlock(state);
	return ret;
}

int ksem_timedwait(struct semaphore *sem, int timeout)
{
	int ret = 0;

	thread_lock(state);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count--;

	trace_sem_wait(sem);

	if (sem->count < 0) {
		debug_printk("unable to got sem (%p)(%d)\r\n", sem, sem->count);

		ret = wait_queue_block_timed(&sem->wait, timeout, &state);
	}

err:
	thread_unlock(state);
	return ret;
}

int ksem_post(struct semaphore *sem)
{
	int ret = 0;

	thread_lock(state);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count++;

	if (sem->count > sem->value)
		sem->count = sem->value;

	trace_sem_post(sem);

	if (sem->count <= 0)
		ret = wait_queue_wake_irqstate(&sem->wait, &state);

err:
	thread_unlock(state);
	return ret;
}

int ksem_post_isr(struct semaphore *sem)
{
	int ret = 0;

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count++;

	if (sem->count > sem->value)
		sem->count = sem->value;

	trace_sem_post(sem);

	if (sem->count <= 0)
		ret = wait_queue_wake_isr(&sem->wait);

err:
	return ret;
}
