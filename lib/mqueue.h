#ifndef MQUEUE_H
#define MQUEUE_H

#include <kernel/kqueue.h>
#include <stddef.h>
#include <list.h>

struct mq_attr
{
	unsigned int mq_maxmsg;
	unsigned int mq_msgsize;
	unsigned int mq_flags;
	unsigned int mq_curmsgs;
};

struct mq_priv {
	int handle;
	char *name;
	struct mq_attr attr;
	struct queue q;
	struct list_node node;
};

typedef int mqd_t;

mqd_t mq_open(const char *name, int flags, ...);
int mq_close(mqd_t fd);
int mq_getattr(mqd_t fd, struct mq_attr *attr);
int mq_setattr(mqd_t fd, const struct mq_attr *attr, struct mq_attr *oldattr);
int mq_receive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio);
int mq_timedreceive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio, int timeout);
int mq_send(mqd_t fd, const char *msg, size_t msg_len, unsigned int msg_prio);

#endif /* MQUEUE_H */
