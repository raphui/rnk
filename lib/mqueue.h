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

#ifndef MQUEUE_H
#define MQUEUE_H

#include <kqueue.h>
#include <stddef.h>

struct mq_attr
{
	unsigned int mq_maxmsg;
	unsigned int mq_msgsize;
	unsigned int mq_flags;
	unsigned int mq_curmsgs;
};

struct mq_priv {
	struct mq_attr attr;
	struct queue q;
};

typedef struct mq_priv* mqd_t;

mqd_t mq_open(const char *name, int flags, ...);
int mq_close(mqd_t fd);
int mq_getattr(mqd_t fd, struct mq_attr *attr);
int mq_setattr(mqd_t fd, const struct mq_attr *attr, struct mq_attr *oldattr);
int mq_receive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio);
int mq_send(mqd_t fd, const char *msg, size_t msg_len, unsigned int msg_prio);

#endif /* MQUEUE_H */
