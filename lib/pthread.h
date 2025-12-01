#ifndef PTHREAD_H
#define PTHREAD_H

#include <stdint.h>

#include <kernel/thread.h>
#include <kernel/kmutex.h>

typedef struct pthread_attr {
	uint32_t stack_size;
} pthread_attr_t;

typedef struct pthread {
	struct thread *thr;
} pthread_t;

typedef struct pthread_mutex {
	struct mutex kmutex;
} pthread_mutex_t;

int pthread_create(pthread_t *thread, pthread_attr_t *attr, void (*start_routine)(void *), void *arg, unsigned int priority);
int pthread_suspend(pthread_t *thread);
int pthread_resume(pthread_t *thread);
int pthread_join(pthread_t *thread, void **retval);
int pthread_mutex_init(pthread_mutex_t *mutex);
int pthread_mutex_lock(pthread_mutex_t *mutex);
int pthread_mutex_unlock(pthread_mutex_t *mutex);

#endif /* PTHREAD_H */
