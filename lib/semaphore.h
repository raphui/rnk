#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <ksem.h>

typedef struct sem {
	struct semaphore ksem;
} sem_t;

void sem_init(sem_t *sem, unsigned int value);
void sem_wait(sem_t *sem);
void sem_post(sem_t *sem);

#endif /* SEMAPHORE_H */
