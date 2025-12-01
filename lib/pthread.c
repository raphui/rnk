#include <pthread.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <errno.h>
#include <export.h>

int pthread_create(pthread_t *thread, pthread_attr_t *attr, void (*start_routine)(void *), void *arg, unsigned int priority)
{
	struct thread_attr thr_attr;

	if (!thread)
		return -EINVAL;

	if (attr) {
		thr_attr.stack_size = attr->stack_size;
	} else {
		thr_attr.stack_size = CONFIG_THREAD_STACK_SIZE;
	}
	
	thr_attr.priority = priority;

	thread->thr = (struct thread *)syscall(SYSCALL_THREAD_CREATE, &thr_attr, start_routine, arg);
	if (!thread->thr)
		return -EAGAIN;

	return 0;
}
EXPORT_SYMBOL(pthread_create);

int pthread_suspend(pthread_t *thread)
{
	if (!thread)
		return -EINVAL;
	return syscall(SYSCALL_THREAD_SUSPEND, thread->thr);
}
EXPORT_SYMBOL(pthread_suspend);

int pthread_resume(pthread_t *thread)
{
	if (!thread)
		return -EINVAL;
	return syscall(SYSCALL_THREAD_RESUME, thread->thr);
}
EXPORT_SYMBOL(pthread_resume);

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
