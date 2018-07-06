#ifndef WAIT_H
#define WAIT_H

#include <list.h>

struct wait_queue {
	struct list_node list;
	int count;
};

int wait_queue_init(struct wait_queue *wait);
int wait_queue_block(struct wait_queue *wait);
int wait_queue_block_irqstate(struct wait_queue *wait, unsigned long *irqstate);
int wait_queue_wake(struct wait_queue *wait);
int wait_queue_wake_irqstate(struct wait_queue *wait, unsigned long *irqstate);

#endif /* WAIT_H */
