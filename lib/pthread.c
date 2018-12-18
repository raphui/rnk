#include <pthread.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <errno.h>
#include <export.h>

int pthread_create(pthread_t *thread, void (*start_routine)(void *), void *arg, unsigned int priority)
{
	if (!thread)
		return -EINVAL;

	thread->thr = (struct thread *)syscall(SYSCALL_THREAD_CREATE, start_routine, arg, priority);
	if (!thread->thr)
		return -EAGAIN;

	return 0;
}
EXPORT_SYMBOL(pthread_create);

int pthread_join(pthread_t *thread, void **retval)
{
	if (!thread)
		return -EINVAL;
	return syscall(SYSCALL_THREAD_JOIN, thread->thr);
}
EXPORT_SYMBOL(pthread_join);

#ifdef CONFIG_MUTEX
int pthread_mutex_init(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_CREATE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_init);

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_ACQUIRE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_lock);

int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_RELEASE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_unlock);
#endif
