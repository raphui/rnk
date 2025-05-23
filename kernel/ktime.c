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

	list_for_every_entry(&sleeping_threads, t, struct thread, state_node)
		if (thread->delay < t->delay)
			break;

	list_add_before(&t->state_node, &thread->state_node);

	thread_unlock(state);
}

static void remove_sleeping_thread(struct thread *thread)
{
	thread_lock(state);

	list_delete(&thread->state_node);

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
	thread->wait_reason = THREAD_WAIT_TIMEOUT;

	insert_sleeping_thread(thread);

	thread_unlock(state);

	schedule_yield();
#endif /* CONFIG_BW_DELAY */
}

void ktime_oneshot(struct ktimer *timer)
{
	struct ktimer *t = NULL;

	thread_lock(state);

#ifdef CONFIG_TICKLESS
	timer->delay = timer->delay + system_tick;
#else
	timer->delay = timer->delay / 1000;
#endif

	list_for_every_entry(&timer_soft_list, t, struct ktimer, node) {
		if (t->delay > timer->delay) {
			list_add_before(&t->node, &timer->node);
			return;
		}
	}

	list_add_tail(&timer_soft_list, &timer->node);

	thread_unlock(state);
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

int ktime_get_ticks(void)
{
#ifdef CONFIG_TICKLESS
	return system_tick / 1000;
#else
	return system_tick;
#endif
}

#ifdef CONFIG_TICKLESS
void ktime_wakeup_next_delay(void)
{
	struct ktimer *t = NULL;
	struct ktimer *tt = NULL;
	struct thread *thread;
	struct thread *next;
	struct thread *tmp;
	struct thread *curr = get_current_thread();
	unsigned int shortest_delay = 0xFFFFFFFF;

	/* XXX: Find the shortest next delay between thread sleeping and software timers */

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, state_node) {
		if (thread->delay <= system_tick) {
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

			if (curr->priority < thread->priority)
				schedule_yield();
		}
	}

	next = list_peek_head_type(&sleeping_threads, struct thread, state_node);
	if (next)
		shortest_delay = next->delay - system_tick;

	list_for_every_entry_safe(&timer_soft_list, t, tt, struct ktimer, node) {
		if (t->delay < shortest_delay) {
			shortest_delay = t->delay - system_tick;
		}
	}

	if (shortest_delay != 0xFFFFFFFF)
		timer_wakeup(shortest_delay, (void (*)(void *))decrease_timer_delay, NULL);

}
#endif /* CONFIG_TICKLESS */

void decrease_thread_delay(void)
{
	struct thread *thread;
	struct thread *tmp;
	struct thread *curr = get_current_thread();

	list_for_every_entry_safe(&sleeping_threads, thread, tmp, struct thread, state_node) {
		if (thread->delay <= system_tick) {
			remove_sleeping_thread(thread);
			insert_runnable_thread(thread);

#ifdef CONFIG_HR_TIMER
			if (list_is_empty(&sleeping_threads))
				timer_disable(&timer);
#endif /* CONFIG_HR_TIMER */

#ifdef CONFIG_SCHEDULE_PRIORITY
			if (curr->priority < thread->priority)
				schedule_thread(thread);
#endif /* CONFIG_SCHEDULE_PRIORITY */
		}
	}
}

void decrease_timer_delay(void)
{
	struct ktimer *t = NULL;
	struct ktimer *tmp = NULL;

	list_for_every_entry_safe(&timer_soft_list, t, tmp, struct ktimer, node) {
#ifdef CONFIG_TICKLESS
		t->delay -= system_tick;
#else
		t->delay--;
#endif
		if (!t->delay) {
			t->handler(t->arg);
			list_delete(&t->node);
			continue;
		}
	}
}
