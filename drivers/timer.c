#include <drv/timer.h>
#include <errno.h>
#include <mm/mm.h>
#include <string.h>
#include <kernel/spinlock.h>
#include <drv/device.h>
#include <init.h>
#include <kernel/printk.h>

static struct list_node timer_list;

#ifdef CONFIG_TICKLESS
static struct list_node timer_lp_list;
#endif

void timer_set_rate(struct timer *timer, unsigned long rate)
{
	timer->tim_ops->set_rate(timer, rate);
}

void timer_set_counter(struct timer *timer, unsigned int counter)
{
	timer->tim_ops->set_counter(timer, counter);
}

void timer_enable(struct timer *timer)
{
	timer->tim_ops->enable(timer);
}

void timer_disable(struct timer *timer)
{
	timer->tim_ops->disable(timer);
}

void timer_clear_it_flags(struct timer *timer, unsigned int flags)
{
	timer->tim_ops->clear_it_flags(timer, flags);
}

static struct timer *timer_request(void)
{
	struct timer *timer = NULL;

	list_for_every_entry(&timer_list, timer, struct timer, node)
		if (!timer->is_used)
			break;

	if (!timer) {
		error_printk("all timers are used\n");
		return NULL;
	}

	timer->is_used = 1;

	return timer;
}

#ifdef CONFIG_TICKLESS
static struct timer *timer_lp_request(void)
{
	struct timer *timer = NULL;

	timer = list_peek_head_type(&timer_lp_list, struct timer, node);

	return timer;
}
#endif /* CONFIG_TICKLESS */

static int timer_release_from_isr(struct timer *timer)
{
	int ret = 0;

	ret = timer->tim_ops->release_irq(timer);
	if (ret < 0)
		error_printk("failed to release timer via hardware IP\n");

	return ret;
}

static void timer_isr(void *arg)
{
	struct timer *timer = (struct timer *)arg;
	void (*callback)(void *) = timer->callback.handler;
	void *callback_arg = timer->callback.arg;

	if (timer->one_shot) {
		timer_disable(timer);
		timer_release_from_isr(timer);
	}

	callback(callback_arg);
}

#ifdef CONFIG_TICKLESS
int timer_wakeup(unsigned int delay, void (*handler)(void *), void *arg)
{
	return timer_oneshot(delay, handler, arg);
}
#endif /* CONFIG_TICKLESS */

int timer_oneshot(unsigned int delay, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct timer *timer = NULL;

	thread_lock(state);

#ifdef CONFIG_TICKLESS
	timer = timer_lp_request();
#else
	timer = timer_request();
#endif
	if (!timer) {
		error_printk("failed to request timer\n");
		return -ENOENT;
	}

#ifdef CONFIG_TICKLESS
	/* if timer is already running */
	timer_disable(timer);
#endif

	timer->one_shot = 1;
	timer->one_pulse = 1;
	timer->count_up = 0;
	timer->counter = delay;
	timer->callback.handler = handler;
	timer->callback.arg = arg;

	timer->tim_ops->request_irq(timer, &timer_isr, timer);

#ifndef CONFIG_TICKLESS
	timer_set_rate(timer, 1000000);
#endif /* CONFIG_TICKLESS */

	timer_set_counter(timer, timer->counter);
	timer_enable(timer);

	thread_unlock(state);

	return ret;
}

struct timer *timer_new(void)
{
	struct timer *timer = NULL;

	timer = (struct timer *)kmalloc(sizeof(struct timer));
	if (!timer) {
		error_printk("cannot allocate timer\n");
		return NULL;
	}

	memset(timer, 0, sizeof(struct timer));

	return timer;
}

int timer_remove(struct timer *timer)
{
	int ret = 0;
	struct timer *timerdev = NULL;

	list_for_every_entry(&timer_list, timerdev, struct timer, node)
		if (timerdev == timer)
			break;

	if (timerdev) {
		list_delete(&timerdev->node);
		kfree(timerdev);
	}
	else
		ret = -ENOENT;

	return ret;
}

#ifdef CONFIG_TICKLESS
int timer_lp_remove(struct timer *timer)
{
	int ret = 0;
	struct timer *timerdev = NULL;

	list_for_every_entry(&timer_lp_list, timerdev, struct timer, node)
		if (timerdev == timer)
			break;

	if (timerdev) {
		list_delete(&timerdev->node);
		kfree(timerdev);
	}
	else
		ret = -ENOENT;

	return ret;
}
#endif /* CONFIG_TICKLESS */

int timer_register(struct timer *timer)
{
	list_add_tail(&timer_list, &timer->node);

	return 0;
}

#ifdef CONFIG_TICKLESS
int timer_lp_register(struct timer *timer)
{
	list_add_tail(&timer_lp_list, &timer->node);

	return 0;
}
#endif /* CONFIG_TICKLESS */

int timer_init(void)
{
	int ret = 0;

	list_initialize(&timer_list);
#ifdef CONFIG_TICKLESS
	list_initialize(&timer_lp_list);
#endif /* CONFIG_TICKLESS */

	return ret;
}
postcore_initcall(timer_init);
