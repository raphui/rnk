#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <kernel/ksem.h>

typedef struct sem {
	struct semaphore ksem;
} sem_t;

int sem_init(sem_t *sem, unsigned int value);
int sem_wait(sem_t *sem);
int sem_timedwait(sem_t *sem, int timeout);
int sem_post(sem_t *sem);
int sem_get_count(sem_t *sem);
int sem_reset(sem_t *sem);

#endif /* SEMAPHORE_H */
