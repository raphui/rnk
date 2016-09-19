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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef SPINLOCK_H
#define SPINLOCK_H

#include <arch/spinlock.h>
#include <thread.h>

#define SPIN_LOCK_FLAG_INTERRUPTS ARCH_DEFAULT_SPIN_LOCK_FLAG_INTERRUPTS

static inline void spin_lock(unsigned long *lock)
{
	arch_spin_lock(lock);
}

static inline int spin_trylock(unsigned long *lock)
{
	return arch_spin_trylock(lock);
}

static inline void spin_unlock(unsigned long *lock)
{
	arch_spin_unlock(lock);
}

static inline void spin_lock_init(unsigned long *lock)
{
	arch_spin_lock_init(lock);
}

static inline int spin_lock_held(unsigned long *lock)
{
	return arch_spin_lock_held(lock);
}


static inline void spin_lock_save( unsigned long *lock, unsigned long  *statep, unsigned long flags)
{
	arch_interrupt_save(statep, flags);
	spin_lock(lock);
}

static inline void spin_unlock_restore(unsigned long *lock, unsigned long old_state, unsigned long flags)
{
	spin_unlock(lock);
	arch_interrupt_restore(old_state, flags);
}

#define spin_lock_irqsave(lock, statep) spin_lock_save(lock, &(statep), SPIN_LOCK_FLAG_INTERRUPTS)
#define spin_unlock_irqrestore(lock, statep) spin_unlock_restore(lock, statep, SPIN_LOCK_FLAG_INTERRUPTS)

#define thread_lock(state) unsigned long state; spin_lock_irqsave(&thread_lock, state)
#define thread_unlock(state) spin_unlock_irqrestore(&thread_lock, state)

#endif /* SPINLOCK_H */
