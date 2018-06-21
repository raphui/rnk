#ifndef KSEM_H
#define KSEM_H

#include <wait.h>

struct semaphore {
	unsigned int value;
	int count;
	struct wait_queue wait;
};

int ksem_init(struct semaphore *sem, unsigned int value);
int ksem_wait(struct semaphore *sem);
int ksem_post(struct semaphore *sem);

#endif /* KSEM_H */
