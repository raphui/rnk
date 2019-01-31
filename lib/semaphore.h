#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <kernel/ksem.h>

typedef struct sem {
	struct semaphore ksem;
} sem_t;

int sem_init(sem_t *sem, unsigned int value);
int sem_wait(sem_t *sem);
int sem_post(sem_t *sem);

#endif /* SEMAPHORE_H */
