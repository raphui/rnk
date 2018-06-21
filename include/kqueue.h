#ifndef QUEUE_H
#define QUEUE_H

#include <list.h>

struct queue {
	unsigned int *head;
	unsigned int *tail;
	unsigned int *curr;
	unsigned int *wr;
	unsigned int item_queued;
	unsigned int size;
	unsigned int item_size;
	unsigned int waiting_post;
	unsigned int waiting_receive;
	struct list_node waiting_receive_threads;
	struct list_node waiting_post_threads;
};

int kqueue_init(struct queue *queue, unsigned int size, unsigned int item_size);
int kqueue_clear(struct queue *queue);
int kqueue_post(struct queue *queue, void *item, unsigned int timeout);
int kqueue_receive(struct queue *queue, void *item, unsigned int timeout);

#endif /* QUEUE_H */
