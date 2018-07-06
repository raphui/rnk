#include <time.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

void time_usleep(unsigned int usec)
{
	syscall(SYSCALL_TIME_USLEEP, usec);
}
EXPORT_SYMBOL(time_usleep);

void time_oneshot(int delay, void (*handler)(void *), void *arg)
{
	syscall(SYSCALL_TIME_ONESHOT, &delay, handler, arg);
}
EXPORT_SYMBOL(time_oneshot);
