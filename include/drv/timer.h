#ifndef TIMER_H
#define TIMER_H

#include <drv/device.h>
#include <drv/clk.h>
#include <list.h>

struct timer_device
{
	struct device dev;
};

struct timer_callback {
	int delay;
	void (*handler)(void *);
	void *arg;
	struct list_node node;
};

struct timer
{
	unsigned int num;
	unsigned int base_reg;
	struct clk clock;
	unsigned int one_shot;
	unsigned long rate;
	unsigned int prescaler;
	unsigned int rcc_base;
	unsigned char one_pulse:1;
	unsigned char count_up:1;
	unsigned int counter;
	unsigned int is_used;
	unsigned int irq;
	struct device dev;
	struct timer_callback callback;
	struct timer_operations *tim_ops;
	struct list_node node;
};

struct timer_operations
{
	void (*set_rate)(struct timer *timer, unsigned long rate);
	void (*set_counter)(struct timer *timer, unsigned short counter);
	void (*enable)(struct timer *timer);
	void (*disable)(struct timer *timer);
	void (*clear_it_flags)(struct timer *timer, unsigned int flags);
	int (*request_irq)(struct timer *timer, void (*handler)(void *), void *arg);
	int (*release_irq)(struct timer *timer);
};

int timer_init(void);
int timer_wakeup(unsigned int delay, void (*handler)(void *), void *arg);
int timer_oneshot(unsigned int delay, void (*handler)(void *), void *arg);
int timer_oneshot_soft(unsigned int delay, void (*handler)(void *), void *arg);
void timer_set_rate(struct timer *timer, unsigned long rate);
void timer_set_counter(struct timer *timer, unsigned short counter);
void timer_enable(struct timer *timer);
void timer_disable(struct timer *timer);
void timer_clear_it_flags(struct timer *timer, unsigned int flags);
void timer_soft_decrease_delay(void);
struct timer *timer_new(void);
int timer_remove(struct timer *timer);
int timer_register(struct timer *timer);
int timer_lp_remove(struct timer *timer);
int timer_lp_register(struct timer *timer);


#endif /* TIMER_H*/
