/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef VECTOR_H
#define VECTOR_H

struct isr_entry {
	void *arg;
	void (*isr)(void *);
};

unsigned int vector_current_irq(void);
void vector_set_isr_wrapper(void *wrapper);
int vector_set_isr_entry(struct isr_entry *entry, int irq);
struct isr_entry *vector_get_isr_entry(int irq);

#endif /* VECTOR_H */
