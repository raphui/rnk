#include <kernel/thread.h>
#include <kernel/ktime.h>
#include <kernel/syscall.h>
#include <kernel/printk.h>
#include <kernel/scheduler.h>
#include <kernel/spinlock.h>
#include <init.h>
#include <export.h>
#include <trace.h>
#include <errno.h>

static struct list_node sleeping_threads;
static struct list_node timer_soft_list;

static void insert_sleeping_thread(struct thread *thread)
{
	struct thread *t;

	thread_lock(state);

	list_for_every_entry(&sleeping_threads, t, struct thread, node)
		if (thread->delay < t->delay)
			break;

	list_add_before(&t->node, &thread->node);

	thread_unlock(state);
}

static void remove_sleeping_thread(struct thread *thread)
{
	thread_lock(state);

	list_delete(&thread->node);

	thread_unlock(state);
}

int time_init(void)
{
	int ret = 0;

	list_initialize(&sleeping_threads);
	list_initialize(&timer_soft_list);

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

	trace_time_usleep(usec);

	thread_lock(state);

	remove_runnable_thread(thread);
	thread->state = THREAD_BLOCKED;

	insert_sleeping_thread(thread);

	thread_unlock(state);

	schedule_yield();
#endif /* CONFIG_BW_DELAY */
}

void ktime_oneshot(struct ktimer *timer, int delay, void (*handler)(void *), void *arg)
{
	struct ktimer *t = NULL;

	timer->delay = delay;
	timer->handler = handler;
	timer->arg = arg;

	list_for_every_entry(&timer_soft_list, t, struct ktimer, node) {
		if (t->delay > timer->delay) {
			list_add_before(&t->node, &timer->node);
			return;
		}
	}

	list_add_tail(&timer_soft_list, &timer->node);
}

int ktime_oneshot_cancel(struct ktimer *timer)
{
	if (!timer)
		return -EINVAL;

	thread_lock(state);

	list_delete(&timer->node);

	thread_unlock(state);

	return 0;
}

#ifdef CONFIG_TICKLESS
void ktime_wakeup_next_delay(void)
{
	struct thread *thread;
	struct thread *next;
	struct thread *tmp;
	struct thread *curr = get_current_thread();

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, node) {
		if (thread->delay <= system_tick) {
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

			if (curr->priority < thread->priority)
				schedule_yield();
		}
	}


	next = list_peek_head_type(&sleeping_threads, struct thread, node);
	if (next)
		timer_wakeup(next->delay - system_tick, (void (*)(void *))ktime_wakeup_next_delay, NULL);
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
				schedule_thread(NULL);
#endif /* CONFIG_SCHEDULE_PRIORITY */
		}
	}
}

void decrease_timer_delay(void)
{
	struct ktimer *t = NULL;
	struct ktimer *tmp = NULL;

	list_for_every_entry_safe(&timer_soft_list, t, tmp, struct ktimer, node) {
		if (!t->delay) {
			t->handler(t->arg);
			list_delete(&t->node);
			continue;
		}

		t->delay--;
	}
}
