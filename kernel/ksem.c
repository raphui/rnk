#include <ksem.h>
#include <wait.h>
#include <thread.h>
#include <errno.h>
#include <scheduler.h>
#include <spinlock.h>
#include <printk.h>
#include <syscall.h>
#include <export.h>

int ksem_init(struct semaphore *sem, unsigned int value)
{
	int ret = 0;

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->value = value;
	sem->count = 0;

	wait_queue_init(&sem->wait);

err:
	return ret;
}

int ksem_wait(struct semaphore *sem)
{
	int ret = 0;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	if (--sem->count < 0) {
		debug_printk("unable to got sem (%p)(%d)\r\n", sem, sem->count);

		ret = wait_queue_block_irqstate(&sem->wait, &irqstate);
	}

err:
	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	return ret;
}

int ksem_post(struct semaphore *sem)
{
	int ret = 0;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	if (!sem) {
		ret = -EINVAL;
		goto err;
	}

	sem->count++;

	if (sem->count > sem->value)
		sem->count = sem->value;

	if (sem->count <= 0)
		ret = wait_queue_wake_irqstate(&sem->wait, &irqstate);

err:
	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	return ret;
}
