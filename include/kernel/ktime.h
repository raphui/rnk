#ifndef KTIME_H
#define KTIME_H

#include <list.h>
#include <kernel/thread.h>
#include <drv/timer.h>

struct ktimer {
	int delay;
	void (*handler)(void *);
	void *arg;
	struct list_node node;
};


int time_init(void);
void ktime_usleep(unsigned int usec);
void ktime_oneshot(struct ktimer *timer);
int ktime_oneshot_cancel(struct ktimer *timer);
int ktime_get_ticks(void);
void ktime_wakeup_next_delay(void);
void decrease_thread_delay(void);
void decrease_timer_delay(void);

#endif /* KTIME_H */
