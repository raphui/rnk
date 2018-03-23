/*
 * Copyright (C) 2015 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <thread.h>
#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <armv7m/vector.h>

extern void __svc(void);
extern void __pendsv(void);
extern void systick_handler(void);

void arch_init(void)
{
	struct isr_entry entry;

	mpu_init();

	entry.isr = __svc;
	entry.arg = NULL;

	vector_set_isr_entry(&entry, -5);

	entry.isr = __pendsv;
	vector_set_isr_entry(&entry, -2);

	entry.isr = systick_handler;
	vector_set_isr_entry(&entry, -1);
}

void arch_init_tick(void)
{
	systick_init();
}

void arch_request_sched(void)
{
	pendsv_request();
}
