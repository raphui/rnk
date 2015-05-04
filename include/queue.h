/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef QUEUE_H
#define QUEUE_H

struct entry {
	struct entry *prev;
	struct entry *next;
}

struct list {
	struct entry *head;
	struct entry *tail;
};

static inline void list_init(struct list *l)
{
	l->head = NULL;
	l->tail = NULL;
}

static inline void list_insert_tail(struct list *l, struct entry *e)
{
	l->tail->next = e;
	e->prev = l->tail;

	l->tail = e;
}

static inline void list_insert_head(struct list *l, struct entry *e)
{
	l->head->prev = e;
	e->next = l->head;

	l->head = e;
}

static inline void list_insert_before(struct entry *p, struct entry *e)
{
	e->prev = p->prev;
	e->next = p;
	p->prev = e;
}

static inline void list_insert_after(struct entry *p, struct entry *e)
{
	e->prev = p;
	e->next = p->next;
	p->next = e;
}

static inline void list_remove(struct entry *e)
{
	e->prev->next = e->next;
	e->next->prev = e->prev;
}

static inline struct entry *list_get_head(struct list *l)
{
	struct entry *e = l->head;

	l->head = l->head->next;
	l->head->prev = NULL;

	return e;
}

static inline struct entry *list_get_tail(struct list *l)
{
	struct entry *e = l->tail;

	l->tail = l->tail->prev;
	l->tail->next = NULL;

	return e;
}

#endif /* QUEUE_H */
