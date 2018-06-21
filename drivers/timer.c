#include <timer.h>
#include <errno.h>
#include <mm.h>
#include <string.h>
#include <kmutex.h>
#include <device.h>
#include <init.h>
#include <printk.h>

static struct mutex timer_mutex;
static struct list_node timer_list;
static struct list_node timer_soft_list;

void timer_set_rate(struct timer *timer, unsigned long rate)
{
	timer->tim_ops->set_rate(timer, rate);
}

void timer_set_counter(struct timer *timer, unsigned short counter)
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

	callback(callback_arg);

	if (timer->one_shot) {
		timer_disable(timer);
		timer_release_from_isr(timer);
	}
}

int timer_oneshot(unsigned int delay, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct timer *timer = NULL;

	kmutex_lock(&timer_mutex);

	timer = timer_request();
	if (!timer) {
		error_printk("failed to request timer\n");
		return -ENOENT;
	}

	timer->one_shot = 1;
	timer->one_pulse = 1;
	timer->count_up = 0;
	timer->counter = delay;
	timer->callback.handler = handler;
	timer->callback.arg = arg;

	timer->tim_ops->request_irq(timer, &timer_isr, timer);

	timer_set_rate(timer, 1000000);
	timer_set_counter(timer, timer->counter);
	timer_enable(timer);

	kmutex_unlock(&timer_mutex);

	return ret;
}


int timer_oneshot_soft(unsigned int delay, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct timer_callback *timer = NULL;
	struct timer_callback *t = NULL;

	timer = (struct timer_callback *)kmalloc(sizeof(struct timer_callback));
	if (!timer) {
		error_printk("cannot allocate soft timer\n");
		return -ENOMEM;
	}

	timer->delay = delay;
	timer->handler = handler;
	timer->arg = arg;

	list_for_every_entry(&timer_soft_list, t, struct timer_callback, node) {
		if (t->delay > timer->delay) {
			list_add_before(&t->node, &timer->node);
			goto out;
		}
	}

	list_add_tail(&timer_soft_list, &timer->node);

out:
	return ret;
}

void timer_soft_decrease_delay(void)
{
	struct timer_callback *t = NULL;
	struct timer_callback *tmp = NULL;

	list_for_every_entry_safe(&timer_soft_list, t, tmp, struct timer_callback, node) {
		if (!t->delay) {
			t->handler(t->arg);
			list_delete(&t->node);
			kfree(t);
			continue;
		}

		t->delay--;
	}
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

int timer_register(struct timer *timer)
{
	list_add_tail(&timer_list, &timer->node);

	return 0;
}

int timer_init(void)
{
	int ret = 0;

	kmutex_init(&timer_mutex);
	list_initialize(&timer_list);
	list_initialize(&timer_soft_list);

	return ret;
}
postcore_initcall(timer_init);
