#ifndef KMUTEX_H
#define KMUTEX_H

#include <thread.h>
#include <wait.h>

struct mutex {
	int lock;
	struct thread *owner;
	unsigned old_prio;
	struct wait_queue wait;
};

int kmutex_init(struct mutex *mutex);
int kmutex_lock(struct mutex *mutex);
int kmutex_unlock(struct mutex *mutex);

#endif /* KMUTEX_H */
