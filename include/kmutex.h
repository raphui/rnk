#ifndef KMUTEX_H
#define KMUTEX_H

#include <thread.h>
#include <list.h>

struct mutex {
	unsigned char lock;
	struct thread *owner;
	unsigned int waiting;
	unsigned old_prio;
	struct list_node waiting_threads;
};

int kmutex_init(struct mutex *mutex);
int kmutex_lock(struct mutex *mutex);
int kmutex_unlock(struct mutex *mutex);

#endif /* KMUTEX_H */
