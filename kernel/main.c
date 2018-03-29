/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
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

#include <stdio.h>
#include <scheduler.h>
#include <thread.h>
#include <mm.h>
#include <time.h>
#include <elfloader.h>
#include <arch/system.h>
#include <arch/spinlock.h>

#ifdef CONFIG_UNWIND
#include <backtrace.h>
#endif /* CONFIG_UNWIND */

#include <init.h>

extern initcall_t __rnk_initcalls_start[], __rnk_initcalls_end[];
extern exitcall_t __rnk_exitcalls_start[], __rnk_exitcalls_end[];

void loading_thread(void)
{
	int ret;

	printk("starting thread K\r\n");

	ret = elf_exec((char *)0x08020000, 220417, 0x08020000);
	if (ret < 0)
		printk("failed to exec elf\r\n");
	else
		printk("efl execution done\r\n");

	while (1)
		;
}

int main(void)
{
	int ret;
	unsigned long irqstate;
	initcall_t *initcall;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	printk("Welcome to rnk\r\n");

	printk("- Initialise architecture...\r\n");

	arch_init();

	for (initcall = __rnk_initcalls_start; initcall < __rnk_initcalls_end; initcall++) {
		debug_printk("initcall-> %pS\n", *initcall);
		ret = (*initcall)();
		if (ret < 0)
			error_printk("initcall %pS failed: %d\n", *initcall, ret);
	}

	printk("- Initialise scheduler...\r\n");

#ifdef CONFIG_UNWIND
	unwind_init();
#endif /* CONFIG_UNWIND */

	printk("- Add thread to scheduler\r\n");

	add_thread(&loading_thread, LOW_PRIORITY);

	printk("- Start scheduling...\r\n");
	start_schedule();

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);

	while(1)
		;

	return 0; //Never reach
}
