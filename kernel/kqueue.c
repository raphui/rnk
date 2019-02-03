#include <kernel/kqueue.h>
#include <kernel/wait.h>
#include <kernel/thread.h>
#include <kernel/scheduler.h>
#include <mm/mm.h>
#include <kernel/syscall.h>
#include <kernel/printk.h>
#include <string.h>
#include <kernel/ktime.h>
#include <kernel/spinlock.h>
#include <export.h>
#include <errno.h>
#include <trace.h>

int kqueue_init(struct queue *queue, unsigned int size, unsigned int item_size)
{
	int ret = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	queue->item_queued = 0;
	queue->size = size;
	queue->item_size = item_size;

	queue->head = (unsigned int *)kmalloc(size * item_size);
	if (!queue->head) {
		ret = -ENOMEM;
		goto err;
	}

	queue->curr = queue->head;
	queue->wr = queue->head;
	queue->tail = queue->head + (size * item_size);

	wait_queue_init(&queue->wait_receive);
	wait_queue_init(&queue->wait_post);

	trace_queue_create(queue, item_size, size);

err:
	return ret;
}

int kqueue_clear(struct queue *queue)
{
	int ret = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	queue->item_queued = 0;
	queue->wr = queue->head;
	queue->curr = queue->head;

	trace_queue_clear(queue);

err:
	return ret;
}

int kqueue_destroy(struct queue *queue)
{
	int ret = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	kfree(queue->head);

err:
	return ret;
}

int kqueue_update(struct queue *queue, unsigned int size, unsigned int item_size)
{
	int ret = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	ret = kqueue_destroy(queue);
	if (ret < 0)
		goto err;

	ret = kqueue_init(queue, size, item_size);

	trace_queue_update(queue, item_size, size);

err:
	return ret;
}

static void _kqueue_post(struct queue *queue, void *item)
{
	thread_lock(state);

	if (queue->item_queued < queue->size) {
		if ((queue->wr + queue->item_size) <= queue->tail) {
			memcpy(queue->wr, item, queue->item_size);
			debug_printk("wr: %x, v: %d\r\n", queue->wr, *(int *)item);
			queue->wr += queue->item_size;
			queue->item_queued++;

			wait_queue_wake_irqstate(&queue->wait_receive, &state);
		}
	}

	thread_unlock(state);
}

int kqueue_post(struct queue *queue, void *item, unsigned int timeout)
{
	int ret = 0;
	int back_from_sleep = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	trace_queue_send(queue, item);

	for (;;) {
		if (back_from_sleep) {
			break;
		} else if (queue->item_queued < queue->size) {
			_kqueue_post(queue, item);
			break;
		} else if (timeout) {
			return -ENOTSUP;
			ktime_usleep(timeout);
			back_from_sleep = 1;
		} else if (!timeout) {
			ret = -EAGAIN;
			break;
		}
	}

err:
	return ret;
}

static void _kqueue_receive(struct queue *queue, void *item)
{
	thread_lock(state);

	if (queue->item_queued) {
		if ((queue->curr + queue->item_size) <= queue->wr) {
			memcpy(item, queue->curr, queue->item_size);
			queue->curr += queue->item_size;
			queue->item_queued--;

			wait_queue_wake_irqstate(&queue->wait_post, &state);
		}

		if (queue->curr == queue->wr)
			kqueue_clear(queue);
	}

	thread_unlock(state);
}

int kqueue_receive(struct queue *queue, void *item, unsigned int timeout)
{
	int ret = 0;
	int back_from_sleep = 0;

	if (!queue) {
		ret = -EINVAL;
		goto err;
	}

	trace_queue_receive(queue, timeout);

	for (;;) {
		if (queue->item_queued) {
			_kqueue_receive(queue, item);
			break;
		} else if (timeout) {
			return -ENOTSUP;
			ktime_usleep(timeout);
			back_from_sleep = 1;
		} else if (back_from_sleep || !timeout) {
			ret = -EAGAIN;
			break;
		}
	}
err:
	return ret;
}
