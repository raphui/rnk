#ifndef KSEM_H
#define KSEM_H

#include <list.h>

struct semaphore {
	unsigned int value;
	int count;
	unsigned int waiting;
	struct list_node waiting_threads;
};

int ksem_init(struct semaphore *sem, unsigned int value);
int ksem_wait(struct semaphore *sem);
int ksem_post(struct semaphore *sem);

#endif /* KSEM_H */
