#ifndef KTIME_H
#define KTIME_H

#include <list.h>
#include <thread.h>
#include <timer.h>

int time_init(void);
void ktime_usleep(unsigned int usec);
void ktime_oneshot(int delay, void (*handler)(void *), void *arg);
void decrease_thread_delay(void);
void decrease_timer_delay(void);

#endif /* KTIME_H */
