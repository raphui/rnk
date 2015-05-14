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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <timer.h>

int timer_init(struct timer *timer)
{
	tim_ops.init(timer);
}

void timer_set_rate(struct timer *timer, unsigned long rate)
{
	tim_ops.set_rate(timer, rate);
}

void timer_set_counter(struct timer *timer, unsigned short counter)
{
	tim_ops.set_counter(timer, counter);
}

void timer_enable(struct timer *timer)
{
	tim_ops.enable(timer);
}

void timer_disable(struct timer *timer)
{
	tim_ops.disable(timer);
}

void timer_clear_it_flags(struct timer *timer, unsigned int flags)
{
	tim_ops.clear_it_flags(timer, flags);
}
