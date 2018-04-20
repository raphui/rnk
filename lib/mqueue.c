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

mqd_t mq_open(const char *name, int flags, ...)
{
	return ERR_PTR(-ENOTSUP);
}
EXPORT_SYMBOL(mq_open);

int mq_close(mqd_t fd)
{
	return -ENOTSUP;
}
EXPORT_SYMBOL(mq_close);

int mq_getattr(mqd_t fd, struct mq_attr *attr)
{
	return -ENOTSUP;
}
EXPORT_SYMBOL(mq_getattr);

int mq_setattr(mqd_t fd, const struct mq_attr *attr, struct mq_attr *oldattr)
{
	return -ENOTSUP;
}
EXPORT_SYMBOL(mq_setattr);

int mq_receive(mqd_t fd, char *msg, size_t msg_len, unsigned int msg_prio)
{
	return -ENOTSUP;
}
EXPORT_SYMBOL(mq_receive);

int mq_send(mqd_t fd, const char *msg, size_t msg_len, unsigned int msg_prio)
{
	return -ENOTSUP;
}
EXPORT_SYMBOL(mq_send);
