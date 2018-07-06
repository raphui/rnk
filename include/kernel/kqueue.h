#ifndef QUEUE_H
#define QUEUE_H

#include <kernel/wait.h>

struct queue {
	unsigned int *head;
	unsigned int *tail;
	unsigned int *curr;
	unsigned int *wr;
	unsigned int item_queued;
	unsigned int size;
	unsigned int item_size;
	struct wait_queue wait_receive;
	struct wait_queue wait_post;
};

int kqueue_init(struct queue *queue, unsigned int size, unsigned int item_size);
int kqueue_clear(struct queue *queue);
int kqueue_destroy(struct queue *queue);
int kqueue_post(struct queue *queue, void *item, unsigned int timeout);
int kqueue_receive(struct queue *queue, void *item, unsigned int timeout);
int kqueue_update(struct queue *queue, unsigned int size, unsigned int item_size);

#endif /* QUEUE_H */
