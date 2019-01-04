#include <kernel/scheduler.h>
#include <kernel/thread.h>
#include <kernel/printk.h>
#include <arch/system.h>
#include <kernel/ktime.h>
#include <kernel/wait.h>
#include <mm/mm.h>
#include <init.h>
#include <trace.h>

int thread_switching = 0;
unsigned int system_tick = 0;

int schedule_init(void)
{
	int ret = 0;

	thread_init();

	return ret;
}
core_initcall(schedule_init);

void start_schedule(void)
{
#ifndef CONFIG_TICKLESS
	arch_init_tick();
#endif
	arch_request_sched();
}

void schedule(void)
{
	struct thread *t;

	t = find_next_thread();
	switch_thread(t);
	thread_switching = 1;
}

void schedule_thread(struct thread *thread)
{
	struct thread *t;
	int runnable = 0;

	t = get_current_thread();
#if defined(CONFIG_SCHEDULE_ROUND_ROBIN) || defined(CONFIG_SCHEDULE_RR_PRIO)
	if (t)
		t->quantum--;
#endif /* CONFIG_SCHEDULE_ROUND_ROBIN */

	if (t)
		runnable = is_thread_runnable(t);

#ifdef CONFIG_SCHEDULE_ROUND_ROBIN
	if (!t || !t->quantum || !runnable) {

		if (t && runnable)
				insert_runnable_thread(t);

		if (!thread)
			t = find_next_thread();
		else
			t = thread;

		switch_thread(t);
	}
#elif defined(CONFIG_SCHEDULE_PRIORITY) || defined (CONFIG_SCHEDULE_RR_PRIO)
	if (t && runnable)
			insert_runnable_thread(t);

	if (!thread)
		t = find_next_thread();
	else
		t = thread;

	switch_thread(t);
#endif
}

void schedule_thread_stop(struct thread *thread)
{
	struct thread *t;

	if (thread)
		t = thread;
	else
		t = get_current_thread();

	wait_queue_wake(&t->wait_exit);

	remove_runnable_thread(t);
	t->state = THREAD_STOPPED;

	trace_thread_stop(thread);

	schedule_thread(NULL);

	thread_destroy(t);

}

void schedule_yield(void)
{
	arch_request_sched();
}
