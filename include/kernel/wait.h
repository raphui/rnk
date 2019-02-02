#ifndef WAIT_H
#define WAIT_H

#include <list.h>

struct wait_queue {
	struct list_node list;
	int count;
};

struct thread;

int wait_queue_init(struct wait_queue *wait);
int wait_queue_block(struct wait_queue *wait);
int wait_queue_block_timed(struct wait_queue *wait, int timeout, unsigned long *irqstate);
int wait_queue_block_thread(struct wait_queue *wait, struct thread *thread);
int wait_queue_block_irqstate(struct wait_queue *wait, unsigned long *irqstate);
int wait_queue_wake(struct wait_queue *wait);
int wait_queue_wake_irqstate(struct wait_queue *wait, unsigned long *irqstate);
int wait_queue_wake_thread(struct thread *thread);
int wait_queue_wake_isr(struct wait_queue *wait);

#endif /* WAIT_H */
