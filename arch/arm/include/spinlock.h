#ifndef ARM_SPINLOCK_H
#define ARM_SPINLOCK_H



#ifdef CONFIG_CPU_ARMV7M
#include <armv7m/system.h>
#endif /* CONFIG_CPU_ARMV7M */

#define SPIN_LOCK_INITIAL_VALUE 0

#define SPIN_LOCK_FLAG_IRQ                      0x40000000

#define ARCH_DEFAULT_SPIN_LOCK_FLAG_INTERRUPTS  SPIN_LOCK_FLAG_IRQ


enum {
	SPIN_LOCK_STATE_RESTORE_IRQ = 1,
	SPIN_LOCK_STATE_RESTORE_FIQ = 2,
};


static inline void arch_spin_lock_init(unsigned long *lock)
{
	*lock = SPIN_LOCK_INITIAL_VALUE;
}

static inline int arch_spin_lock_held(unsigned long *lock)
{
	return *lock != 0;
}

static inline void arch_spin_lock(unsigned long *lock)
{
	*lock = 1;
}

static inline int arch_spin_trylock(unsigned long *lock)
{
	return 0;
}

static inline void arch_spin_unlock(unsigned long *lock)
{
	*lock = 0;
}

static inline void arch_interrupt_save(unsigned long *statep, unsigned long flags)
{
	unsigned long state = 0;

	if ((flags & SPIN_LOCK_FLAG_IRQ) && !arch_it_disabled()) {
		state |= SPIN_LOCK_STATE_RESTORE_IRQ;
		__disable_it();
	}	

	*statep = state;
}

static inline void arch_interrupt_restore(unsigned long old_state, unsigned long flags)
{
	if ((flags & SPIN_LOCK_FLAG_IRQ) && (old_state & SPIN_LOCK_STATE_RESTORE_IRQ))
		__enable_it();
}

#endif /* ARM_SPINLOCK_H */
