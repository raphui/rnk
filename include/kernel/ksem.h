#ifndef KSEM_H
#define KSEM_H

#include <kernel/wait.h>

struct semaphore {
	int value;
	int count;
	struct wait_queue wait;
};

int ksem_init(struct semaphore *sem, int value);
int ksem_wait(struct semaphore *sem);
int ksem_timedwait(struct semaphore *sem, int timeout);
int ksem_post(struct semaphore *sem);
int ksem_post_isr(struct semaphore *sem);
int ksem_get_count(struct semaphore *sem);
int ksem_reset(struct semaphore *sem);

#endif /* KSEM_H */
