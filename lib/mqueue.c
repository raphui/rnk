#include <mqueue.h>
#include <errno.h>
#include <export.h>
#include <syscall.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <pthread.h>

static int mq_count = 0;

static struct list_node mq_list;

static pthread_mutex_t mutex;

static struct mq_attr default_attr = {
	.mq_maxmsg = 5,
	.mq_msgsize = 1,
	.mq_flags = O_RDWR,
	.mq_curmsgs = 0,
};

static struct mq_priv *mq_get(int handle)
{
	int i = 0;
	int found = 0;
	struct mq_priv *mq = NULL;

	pthread_mutex_lock(&mutex);

	if (handle < 0)
		return NULL;

	list_for_every_entry(&mq_list, mq, struct mq_priv, node) {
		if (i++ == (handle - 1)) {
			found = 1;
			break;
		}
	}

	pthread_mutex_unlock(&mutex);

	if (!found)
		mq = NULL;

	return mq;
}

static struct mq_priv *mq_search(const char *name)
{
	int found = 0;
	struct mq_priv *mq = NULL;

	pthread_mutex_lock(&mutex);

	list_for_every_entry(&mq_list, mq, struct mq_priv, node) {
		if (!strcmp(mq->name, name)) {
			found = 1;
			break;
		}
	}

	pthread_mutex_unlock(&mutex);

	if (!found)
		mq = NULL;

	return mq;
}

mqd_t mq_open(const char *name, int flags, ...)
{
	int ret;
	va_list va;
	int found = 0;
	struct mq_priv *mq = NULL;
	struct mq_attr *attr = NULL;

	va_start(va, flags);

	if (!mq_count) {
		list_initialize(&mq_list);
		pthread_mutex_init(&mutex);
	}

	mq = mq_search(name);
	if (mq)
		found = 1;

	/* we don't support mode_t arg */
	va_arg(va, int);

	attr = va_arg(va, struct mq_attr*);

	switch (flags) {
	case O_CREAT:
		if (found)
			goto out;
		break;
	case O_EXCL:
		goto err_inval;
	case O_CREAT | O_EXCL:
		if (found)
			goto err_inval;
		break;
	default:
		break;
	}


	if (!attr)
		attr = &default_attr;

	mq = malloc(sizeof(struct mq_priv));
	if (!mq)
		goto err_nomem;

	memcpy(&mq->attr, attr, sizeof(struct mq_attr));

	mq->name = malloc(strlen(name) + 1);

	if(!mq->name)
		goto err;

	strcpy(mq->name, name);
	mq->name[strlen(name)] = '\0';

	ret = syscall(SYSCALL_QUEUE_CREATE, &mq->q, mq->attr.mq_maxmsg, mq->attr.mq_msgsize);

	if (ret < 0)
		goto err;

	mq_count++;

	pthread_mutex_lock(&mutex);

	list_add_tail(&mq_list, &mq->node);

	pthread_mutex_unlock(&mutex);
out:
	return mq_count;
err_inval:
	return -EINVAL;
err_nomem:
	return -ENOMEM;
err:
	free(mq);
	return -EIO;
}
EXPORT_SYMBOL(mq_open);

int mq_close(mqd_t fd)
{
	int ret = 0;
	struct mq_priv *mq = NULL;

	mq = mq_get(fd);
	if (!mq)
		goto err;

	ret = syscall(SYSCALL_QUEUE_DESTROY, &mq->q);

	list_delete(&mq->node);

	mq_count--;

err:
	return ret;
}
EXPORT_SYMBOL(mq_close);

int mq_getattr(mqd_t fd, struct mq_attr *attr)
{
	int ret = 0;
	struct mq_priv *mq = NULL;

	if (!attr) {
		ret = -EINVAL;
		goto err;
	}

	mq = mq_get(fd);
	if (!mq) {
		ret = -EBADF;
		goto err;
	}

	memcpy(attr, &mq->attr, sizeof(struct mq_attr));

err:
	return ret;
}
EXPORT_SYMBOL(mq_getattr);

int mq_setattr(mqd_t fd, const struct mq_attr *attr, struct mq_attr *oldattr)
{
	int ret = 0;
	struct mq_priv *mq = NULL;

	if (!attr) {
		ret = -EINVAL;
		goto err;
	}

	mq = mq_get(fd);
	if (!mq)
		goto err;

	if (oldattr)
		memcpy(oldattr, &mq->attr, sizeof(struct mq_attr));

	memcpy(&mq->attr, attr, sizeof(struct mq_attr));

err:
	return ret;
}
EXPORT_SYMBOL(mq_setattr);

int mq_receive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio)
{
	int ret = 0;
	struct mq_priv *mq = NULL;

	mq = mq_get(fd);
	if (!mq)
		goto err;

	if (!(mq->attr.mq_flags & (O_RDWR | O_RDONLY))) {
		ret = -EBADF;
		goto err;
	}

	if (msg_len < mq->attr.mq_msgsize) {
		ret = -EMSGSIZE;
		goto err;
	}

	ret = syscall(SYSCALL_QUEUE_RECEIVE, &mq->q, msg, 0);

err:
	return ret;
}
EXPORT_SYMBOL(mq_receive);

int mq_send(mqd_t fd, const char *msg, size_t msg_len, unsigned int msg_prio)
{
	int ret = 0;
	struct mq_priv *mq = NULL;

	mq = mq_get(fd);
	if (!mq)
		goto err;

	if (!(mq->attr.mq_flags & (O_RDWR | O_WRONLY))) {
		ret = -EBADF;
		goto err;
	}

	if (msg_len > mq->attr.mq_msgsize) {
		ret = -EMSGSIZE;
		goto err;
	}

	ret = syscall(SYSCALL_QUEUE_POST, &mq->q, msg, 0);

err:
	return ret;
}
EXPORT_SYMBOL(mq_send);
