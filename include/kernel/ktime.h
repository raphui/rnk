#ifndef KTIME_H
#define KTIME_H

#include <list.h>
#include <kernel/thread.h>
#include <drv/timer.h>

int time_init(void);
void ktime_usleep(unsigned int usec);
void ktime_oneshot(int delay, void (*handler)(void *), void *arg);
void ktime_wakeup_next_delay(void);
void decrease_thread_delay(void);
void decrease_timer_delay(void);

#endif /* KTIME_H */
