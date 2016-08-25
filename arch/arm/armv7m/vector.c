/*
 * Copyright (C) 2016 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <armv7m/system.h>

struct isr_entry {
	void *arg;
	void (*isr)(void *);
};

struct isr_entry sw_isr_table[CONFIG_NUM_IRQS];

void isr_wrapper(void) {
	unsigned char irq_num = IPSR();
	struct isr_entry entry;

	/* skip arch specific irq, index 0 has to be the 16th irq (first vendor specific irq) */
	irq_num -= 16;

	entry = sw_isr_table[irq_num];

	if (entry.isr && entry.arg)
		entry.isr(entry.arg);
}

typedef void (*vect)(void);

vect isr_vector_table[CONFIG_NUM_IRQS] = {
	[0 ...(CONFIG_NUM_IRQS - 1)] = isr_wrapper,
};
