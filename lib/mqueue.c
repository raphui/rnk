/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

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

static mqd_t mq_search(const char *name)
{
	mqd_t mq = NULL;

	pthread_mutex_lock(&mutex);

	list_for_every_entry(&mq_list, mq, struct mq_priv, node) {
		if (!strcmp(mq->name, name))
			break;
	}

	pthread_mutex_unlock(&mutex);

	return mq;
}

static int mq_is_valid(mqd_t fd)
{
	int found = -EBADF;
	mqd_t mq = NULL;

	if (!fd)
		goto err;

	pthread_mutex_lock(&mutex);

	list_for_every_entry(&mq_list, mq, struct mq_priv, node) {
		if (!strcmp(mq->name, fd->name)) {
			found = 1;
			break;
		}
	}

	pthread_mutex_unlock(&mutex);

err:
	return found;
}

mqd_t mq_open(const char *name, int flags, ...)
{
	int ret;
	va_list va;
	int found = 0;
	mqd_t mq = NULL;
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
		goto err_inval;
	}


	if (!attr)
		attr = &default_attr;

	mq = malloc(sizeof(*mq));
	if (!mq)
		goto err_nomem;

	memcpy(&mq->attr, attr, sizeof(*attr));

	mq->name = malloc(strlen(name) + 1);

	if(!mq->name) {
		free(mq);
		goto err;
	}

	strcpy(mq->name, name);
	mq->name[strlen(name)] = '\0';

	ret = syscall(SYSCALL_QUEUE_CREATE, &mq->q);

	if (ret < 0) {
		free(mq);
		goto err;
	}

	mq_count++;

	pthread_mutex_lock(&mutex);

	list_add_tail(&mq_list, &mq->node);

	pthread_mutex_unlock(&mutex);
out:
	return mq;
err_inval:
	return ERR_PTR(-EINVAL);
err_nomem:
	return ERR_PTR(-ENOMEM);
err:
	return ERR_PTR(-EIO);
}
EXPORT_SYMBOL(mq_open);

int mq_close(mqd_t fd)
{
	int ret = 0;

	ret = mq_is_valid(fd);
	if (ret < 0)
		goto err;

	ret = syscall(SYSCALL_QUEUE_DESTROY, &fd->q);

	list_delete(&fd->node);

	mq_count--;

err:
	return ret;
}
EXPORT_SYMBOL(mq_close);

int mq_getattr(mqd_t fd, struct mq_attr *attr)
{
	int ret = 0;

	if (!attr) {
		ret = -EINVAL;
		goto err;
	}

	ret = mq_is_valid(fd);
	if (ret < 0)
		goto err;

	memcpy(attr, &fd->attr, sizeof(struct mq_attr));

err:
	return ret;
}
EXPORT_SYMBOL(mq_getattr);

int mq_setattr(mqd_t fd, const struct mq_attr *attr, struct mq_attr *oldattr)
{
	int ret = 0;

	if (!attr) {
		ret = -EINVAL;
		goto err;
	}

	ret = mq_is_valid(fd);
	if (ret < 0)
		goto err;

	if (oldattr)
		memcpy(oldattr, &fd->attr, sizeof(struct mq_attr));

	memcpy(&fd->attr, attr, sizeof(struct mq_attr));

err:
	return ret;
}
EXPORT_SYMBOL(mq_setattr);

int mq_receive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio)
{
	int ret = 0;

	ret = mq_is_valid(fd);
	if (ret < 0)
		goto err;

	if (!(fd->attr.mq_flags & (O_RDWR | O_RDONLY))) {
		ret = -EBADF;
		goto err;
	}

	if (msg_len < fd->attr.mq_msgsize) {
		ret = -EMSGSIZE;
		goto err;
	}

	ret = syscall(SYSCALL_QUEUE_RECEIVE, &fd->q, msg, 0);

err:
	return ret;
}
EXPORT_SYMBOL(mq_receive);

int mq_send(mqd_t fd, const char *msg, size_t msg_len, unsigned int msg_prio)
{
	int ret = 0;

	ret = mq_is_valid(fd);
	if (ret < 0)
		goto err;

	if (!(fd->attr.mq_flags & (O_RDWR | O_WRONLY))) {
		ret = -EBADF;
		goto err;
	}

	if (msg_len > fd->attr.mq_msgsize) {
		ret = -EMSGSIZE;
		goto err;
	}

	ret = syscall(SYSCALL_QUEUE_POST, &fd->q, msg, 0);

err:
	return ret;
}
EXPORT_SYMBOL(mq_send);
