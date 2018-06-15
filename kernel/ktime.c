#include <thread.h>
#include <ktime.h>
#include <timer.h>
#include <syscall.h>
#include <printk.h>
#include <scheduler.h>
#include <spinlock.h>
#include <init.h>
#include <export.h>

static struct list_node sleeping_threads;

static void remove_sleeping_thread(struct thread *thread)
{
	list_delete(&thread->node);
}

int time_init(void)
{
	int ret = 0;

	list_initialize(&sleeping_threads);

	return ret;
}
core_initcall(time_init);

void ktime_usleep(unsigned int usec)
{
	struct thread *thread = NULL;
	struct timer timer;

#ifdef CONFIG_HR_TIMER
	timer.num = 2;
	timer.one_pulse = 1;
	timer.count_up = 0;
	timer.counter = usec;
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_BW_DELAY
	int end = system_tick + (usec / 1000);

	do {
		wait_for_interrupt();
	} while (end > system_tick);
#else
	timer.counter = usec / 1000;

	thread = get_current_thread();
	thread->delay = timer.counter + system_tick;

	thread->state = THREAD_BLOCKED;
	remove_runnable_thread(thread);

	list_add_tail(&sleeping_threads, &thread->node);

	schedule_yield();
#endif /* CONFIG_BW_DELAY */
}

void ktime_oneshot(int delay, void (*handler)(void *), void *arg)
{
	int ret;

	ret = timer_oneshot_soft(delay, handler, arg);
	if (ret < 0)
		error_printk("failed to create software timer oneshot\n");
}

void ktime_wakeup_next_delay(void)
{
	int delay;
	struct thread *thread;

	thread = list_peek_head_type(&sleeping_threads, struct thread, node);

	delay = thread->delay;

	list_for_every_entry(&sleeping_threads, thread, struct thread, node)
		if (thread->delay < delay)
			delay = thread->delay;

	timer_wakeup(delay);
}

void decrease_thread_delay(void)
{
	struct thread *thread;
	struct thread *tmp;
	struct thread *curr = get_current_thread();

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, node) {
		if (thread->delay <= system_tick) {
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

#ifdef CONFIG_HR_TIMER
			if (list_is_empty(&sleeping_threads))
				timer_disable(&timer);
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < thread->priority)
				schedule_yield();
#endif /* CONFIG_SCHEDULE_PRIORITY */
		}
	}
}

void decrease_timer_delay(void)
{
	timer_soft_decrease_delay();
}
