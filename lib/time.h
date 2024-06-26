#ifndef TIME_H
#define TIME_H

#include <kernel/ktime.h>

typedef struct timer_user {
	struct ktimer timer;
} timer_t;

typedef int time_t;

void time_usleep(unsigned int usec);
void time_oneshot(timer_t *timer, int delay, void (*handler)(void *), void *arg);
void time_oneshot_cancel(timer_t *timer);
int time_get_ticks(void);

#endif /* TIME_H */
