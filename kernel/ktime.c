#include <kernel/thread.h>
#include <kernel/ktime.h>
#include <drv/timer.h>
#include <kernel/syscall.h>
#include <kernel/printk.h>
#include <kernel/scheduler.h>
#include <kernel/spinlock.h>
#include <init.h>
#include <export.h>

static struct list_node sleeping_threads;

static void insert_sleeping_thread(struct thread *thread)
{
	struct thread *t;

	list_for_every_entry(&sleeping_threads, t, struct thread, node)
		if (thread->delay < t->delay)
			break;

	list_add_before(&t->node, &thread->node);
}

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

	thread = get_current_thread();

#ifdef CONFIG_TICKLESS
	timer.counter = usec;
#else
	timer.counter = usec / 1000;
#endif

	thread->delay = timer.counter + system_tick;

	thread->state = THREAD_BLOCKED;
	remove_runnable_thread(thread);

	insert_sleeping_thread(thread);

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

#ifdef CONFIG_TICKLESS
void ktime_wakeup_next_delay(void)
{
	struct thread *thread;
	struct thread *next;
	struct thread *curr = get_current_thread();

	thread = list_peek_head_type(&sleeping_threads, struct thread, node);

	while (thread) {
		next = list_next_type(&sleeping_threads, &thread->node, struct thread, node);

		if (thread->delay <= system_tick) {
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

			if (curr->priority < thread->priority)
				schedule_yield();
		}
		else
			break;

		thread = next;
	}

	if (thread)
		timer_wakeup(thread->delay - system_tick, (void (*)(void *))ktime_wakeup_next_delay, NULL);
}
#endif /* CONFIG_TICKLESS */

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
