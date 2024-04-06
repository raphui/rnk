#include <time.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

void time_usleep(unsigned int usec)
{
	syscall(SYSCALL_TIME_USLEEP, usec);
}
EXPORT_SYMBOL(time_usleep);

void time_oneshot(timer_t *timer, int delay, void (*handler)(void *), void *arg)
{
	timer->timer.delay = delay;
	timer->timer.handler = handler;
	timer->timer.arg = arg;

	syscall(SYSCALL_TIME_ONESHOT, &timer->timer);
}
EXPORT_SYMBOL(time_oneshot);

void time_oneshot_cancel(timer_t *timer)
{
	syscall(SYSCALL_TIME_ONESHOT_CANCEL, &timer->timer);
}
EXPORT_SYMBOL(time_oneshot_cancel);
